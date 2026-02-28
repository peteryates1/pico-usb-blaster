/*
 * program_fpga.c — Program Altera Cyclone IV FPGA via USB-Blaster (pico-usb-blaster)
 * Minimal C implementation using libusb for maximum USB throughput.
 *
 * Build: gcc -O2 -o program_fpga program_fpga.c -lusb-1.0
 * Usage: sudo ./program_fpga <bitstream.rbf>
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <libusb-1.0/libusb.h>

#define VID 0x09fb
#define PID 0x6001
#define EP_OUT 0x02
#define EP_IN  0x81
#define TIMEOUT_MS 1000

#define IR_LEN 10
#define IR_CONFIG       0x002
#define IR_CHECK_STATUS 0x004
#define IR_STARTUP      0x003
#define IR_BYPASS       0x3FF

static libusb_device_handle *dev;

static double now_sec(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

static void drain(void)
{
    unsigned char buf[64];
    int transferred;
    for (int i = 0; i < 3; i++) {
        if (libusb_bulk_transfer(dev, EP_IN, buf, sizeof(buf), &transferred, 20) != 0)
            break;
    }
}

static int bb_write(const unsigned char *data, int len)
{
    int transferred;
    int rc = libusb_bulk_transfer(dev, EP_OUT, (unsigned char *)data, len, &transferred, TIMEOUT_MS);
    if (rc != 0) {
        fprintf(stderr, "bb_write failed: %s\n", libusb_error_name(rc));
        return -1;
    }
    return transferred;
}

static int bb_read(const unsigned char *cmd, int cmd_len, unsigned char *out, int expect)
{
    int transferred;
    libusb_bulk_transfer(dev, EP_OUT, (unsigned char *)cmd, cmd_len, &transferred, TIMEOUT_MS);

    int got = 0;
    double deadline = now_sec() + 1.0;
    while (now_sec() < deadline && got < expect) {
        unsigned char buf[64];
        int rc = libusb_bulk_transfer(dev, EP_IN, buf, sizeof(buf), &transferred, 200);
        if (rc != 0) break;
        if (transferred >= 2 && buf[0] == 0x31) {
            int payload = transferred - 2;
            if (got + payload > expect) payload = expect - got;
            memcpy(out + got, buf + 2, payload);
            got += payload;
        }
    }
    return got;
}

static void jtag_reset(void)
{
    unsigned char cmds[10];
    for (int i = 0; i < 5; i++) {
        cmds[i*2]   = 0x22;
        cmds[i*2+1] = 0x23;
    }
    bb_write(cmds, 10);
    drain();
}

static void goto_rti(void)
{
    unsigned char cmds[] = {0x20, 0x21};
    bb_write(cmds, 2);
}

static void tck_rti(int n)
{
    while (n > 0) {
        int chunk = n < 30 ? n : 30;
        unsigned char cmds[60];
        for (int i = 0; i < chunk; i++) {
            cmds[i*2]   = 0x20;
            cmds[i*2+1] = 0x21;
        }
        bb_write(cmds, chunk * 2);
        n -= chunk;
    }
    drain();
}

static void scan_ir(int ir_val)
{
    unsigned char cmds[64];
    int p = 0;
    /* RTI -> Select-DR -> Select-IR -> Capture-IR -> Shift-IR */
    cmds[p++] = 0x22; cmds[p++] = 0x23;
    cmds[p++] = 0x22; cmds[p++] = 0x23;
    cmds[p++] = 0x20; cmds[p++] = 0x21;
    cmds[p++] = 0x20; cmds[p++] = 0x21;

    for (int i = 0; i < IR_LEN; i++) {
        unsigned char tdi = (ir_val >> i) & 1 ? 0x10 : 0x00;
        unsigned char tms = (i == IR_LEN - 1) ? 0x02 : 0x00;
        cmds[p++] = 0x20 | tdi | tms;
        cmds[p++] = 0x21 | tdi | tms;
    }
    /* Exit1-IR -> Update-IR -> RTI */
    cmds[p++] = 0x22; cmds[p++] = 0x23;
    cmds[p++] = 0x20; cmds[p++] = 0x21;

    bb_write(cmds, p);
    drain();
}

static void enter_shift_dr(void)
{
    unsigned char cmds[] = {0x22, 0x23, 0x20, 0x21, 0x20, 0x21};
    bb_write(cmds, 6);
    drain();
}

static void exit_shift_dr(void)
{
    unsigned char cmds[] = {0x22, 0x23, 0x20, 0x21};
    bb_write(cmds, 4);
    drain();
}

int main(int argc, char *argv[])
{
    if (argc < 2) {
        fprintf(stderr, "Usage: %s <bitstream.rbf>\n", argv[0]);
        return 1;
    }

    /* Load bitstream */
    FILE *f = fopen(argv[1], "rb");
    if (!f) { perror("fopen"); return 1; }
    fseek(f, 0, SEEK_END);
    long rbf_len = ftell(f);
    fseek(f, 0, SEEK_SET);
    unsigned char *rbf = malloc(rbf_len);
    if (!rbf || fread(rbf, 1, rbf_len, f) != (size_t)rbf_len) {
        fprintf(stderr, "Failed to read bitstream\n");
        return 1;
    }
    fclose(f);
    printf("Bitstream: %ld bytes\n", rbf_len);

    /* Open USB device */
    libusb_init(NULL);
    dev = libusb_open_device_with_vid_pid(NULL, VID, PID);
    if (!dev) {
        fprintf(stderr, "No USB-Blaster found\n");
        return 1;
    }
    libusb_detach_kernel_driver(dev, 0);
    libusb_claim_interface(dev, 0);
    printf("Opened USB-Blaster\n");

    /* JTAG programming sequence */
    printf("JTAG reset...\n");
    jtag_reset();
    goto_rti();

    printf("IR: CONFIG (0x002)...\n");
    scan_ir(IR_CONFIG);

    printf("6000 TCK in RTI...\n");
    tck_rti(6000);

    printf("Shifting bitstream (%ld bytes)...\n", rbf_len);
    double t0 = now_sec();

    enter_shift_dr();

    /* Build command stream: [0x80|len, data...] for each chunk of 63 bytes */
    long total = rbf_len - 1;
    long cmd_len = total + (total + 62) / 63;  /* data + headers */
    unsigned char *cmd_stream = malloc(cmd_len);
    if (!cmd_stream) { fprintf(stderr, "malloc failed\n"); return 1; }

    long pos = 0, cp = 0;
    while (pos < total) {
        int n = (total - pos) < 63 ? (int)(total - pos) : 63;
        cmd_stream[cp++] = 0x80 | n;
        memcpy(cmd_stream + cp, rbf + pos, n);
        cp += n;
        pos += n;
    }

    /* Send in large batches via synchronous bulk OUT */
    int batch = 32768;
    long sent = 0;
    while (sent < cp) {
        int n = (cp - sent) < batch ? (int)(cp - sent) : batch;
        int transferred;
        int rc = libusb_bulk_transfer(dev, EP_OUT, cmd_stream + sent, n, &transferred, 5000);
        if (rc != 0) {
            fprintf(stderr, "bulk write failed at offset %ld: %s\n", sent, libusb_error_name(rc));
            break;
        }
        sent += transferred;

        long data_sent = sent * total / cp;
        if (data_sent > total) data_sent = total;
        double elapsed = now_sec() - t0;
        double speed = elapsed > 0 ? data_sent / elapsed / 1024 : 0;
        printf("  %ld%% (%ld/%ld) %.0f KB/s\r",
               data_sent * 100 / rbf_len, data_sent, rbf_len, speed);
        fflush(stdout);
    }
    printf("\n");

    drain();
    free(cmd_stream);

    /* Last byte: bitbang with TMS=1 on last bit */
    unsigned char last_cmds[16];
    int lp = 0;
    unsigned char last = rbf[rbf_len - 1];
    for (int i = 0; i < 8; i++) {
        unsigned char tdi = (last >> i) & 1 ? 0x10 : 0x00;
        unsigned char tms = (i == 7) ? 0x02 : 0x00;
        last_cmds[lp++] = 0x20 | tdi | tms;
        last_cmds[lp++] = 0x21 | tdi | tms;
    }
    bb_write(last_cmds, lp);

    exit_shift_dr();

    double dt = now_sec() - t0;
    double speed = dt > 0 ? rbf_len / dt / 1024 : 0;
    printf("Done: %.1fs (%.0f KB/s)\n", dt, speed);

    printf("IR: CHECK_STATUS...\n");
    scan_ir(IR_CHECK_STATUS);
    tck_rti(200);

    printf("IR: STARTUP...\n");
    scan_ir(IR_STARTUP);
    tck_rti(200);

    printf("IR: BYPASS...\n");
    scan_ir(IR_BYPASS);

    /* Check CONF_DONE */
    unsigned char rd_cmd[] = {0x62};
    unsigned char rd_buf[1];
    int got = bb_read(rd_cmd, 1, rd_buf, 1);
    int conf_done = got > 0 ? (rd_buf[0] & 1) : -1;
    printf("CONF_DONE: %d\n", conf_done);

    if (conf_done == 1)
        printf("FPGA programmed successfully!\n");
    else
        printf("WARNING: CONF_DONE not asserted\n");

    libusb_release_interface(dev, 0);
    libusb_close(dev);
    libusb_exit(NULL);
    free(rbf);
    return conf_done == 1 ? 0 : 1;
}
