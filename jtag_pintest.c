/*
 * jtag_pintest — pin-level JTAG diagnosis for the pico-usb-blaster.
 *
 * Answers the one question Quartus will not: when jtagconfig says "JTAG chain
 * broken", is TDO coming back at all?
 *
 *   TDO stuck 0 or stuck 1 regardless of TDI  -> the TDO wire is the problem
 *   (open, swapped with TDI, wrong header pin, or no ground reference).
 *   TDO follows TDI                           -> the loop is closed and the
 *   fault is protocol/clocking, not wiring.
 *   A plausible IDCODE                        -> the chain is fine and the
 *   problem is above this layer.
 *
 * Selects the probe by bus:dev, because a genuine Altera cable and this clone
 * share 09fb:6001 and libusb_open_device_with_vid_pid() takes the first match.
 *
 * Firmware pin map (src/blaster.c): GP18 TCK, GP19 TMS, GP16 TDI, GP17 TDO.
 *
 *   gcc -O2 -o jtag_pintest jtag_pintest.c -lusb-1.0
 *   ./jtag_pintest 1:111
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <libusb-1.0/libusb.h>

#define EP_OUT 0x02
#define EP_IN  0x81
#define TIMEOUT_MS 500

/* Bit-bang byte: bit0 TCK, bit1 TMS, bit4 TDI, bit5 OE, bit6 READ */
#define B_OE   0x20
#define B_RD   0x40
#define B_TCK  0x01
#define B_TMS  0x02
#define B_TDI  0x10

static libusb_device_handle *dev;

static int bb(const unsigned char *cmd, int len, unsigned char *out, int want)
{
	int transferred = 0, got = 0;
	if (libusb_bulk_transfer(dev, EP_OUT, (unsigned char *)cmd, len,
				&transferred, TIMEOUT_MS) != 0)
		return -1;
	if (want == 0)
		return 0;
	for (int tries = 0; tries < 20 && got < want; tries++) {
		unsigned char buf[64];
		int n = 0;
		if (libusb_bulk_transfer(dev, EP_IN, buf, sizeof(buf), &n, 200) != 0)
			break;
		/* FTDI-style: first two bytes are modem status, then payload */
		for (int i = 2; i < n && got < want; i++)
			out[got++] = buf[i];
	}
	return got;
}

/* One read sample with the given static levels. */
static int sample(unsigned char lvl)
{
	unsigned char cmd = (unsigned char)(B_OE | B_RD | lvl);
	unsigned char r = 0;
	if (bb(&cmd, 1, &r, 1) != 1)
		return -1;
	return r & 1;                       /* bit0 = TDO */
}

int main(int argc, char **argv)
{
	int want_bus = -1, want_addr = -1;
	const char *want_serial = NULL;
	int auto_pick = (argc <= 1 || !strcmp(argv[1], "auto"));
	if (!auto_pick && sscanf(argv[1], "%d:%d", &want_bus, &want_addr) != 2) {
		/* not bus:dev — treat it as an iSerial substring */
		want_serial = argv[1];
		want_bus = want_addr = -1;
	}

	libusb_init(NULL);
	libusb_device **list;
	ssize_t n = libusb_get_device_list(NULL, &list);
	for (ssize_t i = 0; i < n; i++) {
		struct libusb_device_descriptor d;
		if (libusb_get_device_descriptor(list[i], &d) != 0)
			continue;
		if (d.idVendor != 0x09fb || d.idProduct != 0x6001)
			continue;
		int bus = libusb_get_bus_number(list[i]);
		int addr = libusb_get_device_address(list[i]);
		if (want_bus >= 0 && (bus != want_bus || addr != want_addr))
			continue;
		if (libusb_open(list[i], &dev) != 0) {
			dev = NULL;
			continue;
		}
		if (want_serial) {
			/* iSerial is derived from the RP2040 unique flash ID, so
			 * it is the only selector that stays unambiguous once
			 * there is more than one Pico probe on the bus. */
			unsigned char ser[64] = {0};
			libusb_get_string_descriptor_ascii(dev, d.iSerialNumber,
							   ser, sizeof(ser));
			if (!strstr((char *)ser, want_serial)) {
				libusb_close(dev);
				dev = NULL;
				continue;
			}
		} else if (auto_pick) {
			/* Weakest selector: every Pico running this firmware
			 * reports iManufacturer "Pico" (a genuine cable reports
			 * "Altera"), so this is ambiguous with two Pico probes.
			 * Prefer a serial. */
			unsigned char mfg[64] = {0};
			libusb_get_string_descriptor_ascii(dev, d.iManufacturer,
							   mfg, sizeof(mfg));
			if (strcmp((char *)mfg, "Pico") != 0) {
				libusb_close(dev);
				dev = NULL;
				continue;
			}
		}
		printf("probe at bus %d dev %d\n", bus, addr);
		break;
	}
	libusb_free_device_list(list, 1);
	if (!dev) {
		fprintf(stderr, "no matching probe (did you pass bus:dev?)\n");
		return 1;
	}
	libusb_detach_kernel_driver(dev, 0);
	if (libusb_claim_interface(dev, 0) != 0) {
		fprintf(stderr, "claim interface failed\n");
		return 1;
	}

	printf("\n-- static levels (does TDO respond to anything?) --\n");
	struct { const char *name; unsigned char lvl; } t[] = {
		{"all low      ", 0},
		{"TCK=1        ", B_TCK},
		{"TMS=1        ", B_TMS},
		{"TDI=1        ", B_TDI},
		{"TDI=1 TCK=1  ", B_TDI | B_TCK},
	};
	int seen0 = 0, seen1 = 0;
	for (unsigned i = 0; i < sizeof(t)/sizeof(t[0]); i++) {
		int v = sample(t[i].lvl);
		printf("  %s -> TDO=%d\n", t[i].name, v);
		if (v == 0) seen0 = 1;
		if (v == 1) seen1 = 1;
	}

	printf("\n-- IDCODE (reset, then shift DR) --\n");
	unsigned char cmds[64];
	int c = 0;
	for (int i = 0; i < 5; i++) {           /* >=5 TMS=1 clocks -> Test-Logic-Reset */
		cmds[c++] = B_OE | B_TMS;
		cmds[c++] = B_OE | B_TMS | B_TCK;
	}
	/* TLR -> Run-Test/Idle -> Select-DR -> Capture-DR -> Shift-DR */
	unsigned char nav[] = {B_OE, B_OE|B_TCK,
			       B_OE|B_TMS, B_OE|B_TMS|B_TCK,
			       B_OE, B_OE|B_TCK,
			       B_OE, B_OE|B_TCK};
	memcpy(cmds + c, nav, sizeof(nav));
	c += sizeof(nav);
	bb(cmds, c, NULL, 0);

	unsigned char shift[5] = {0x80 | 0x40 | 4, 0, 0, 0, 0};  /* shift+read, 4 bytes */
	unsigned char idbuf[8];
	memset(idbuf, 0, sizeof(idbuf));
	int got = bb(shift, sizeof(shift), idbuf, 4);
	if (got >= 4) {
		unsigned int id = idbuf[0] | (idbuf[1]<<8) | (idbuf[2]<<16) | (idbuf[3]<<24);
		printf("  IDCODE = 0x%08x  (raw %02x %02x %02x %02x)\n",
		       id, idbuf[0], idbuf[1], idbuf[2], idbuf[3]);
		if (id == 0x00000000 || id == 0xffffffff)
			printf("  -> not a valid IDCODE: TDO is stuck, so the chain is open\n");
		else
			printf("  -> looks like a real IDCODE (EP4CGX150 = 0x028040dd)\n");
	} else {
		printf("  only %d bytes back\n", got);
	}

	printf("\n-- verdict --\n");
	/* The IDCODE is the authoritative signal. TDO is only driven while the TAP
	 * is in a Shift state, so reading 0 at static levels is normal and says
	 * nothing — do not diagnose from it. (seen0/seen1 are reported above purely
	 * as context.) */
	(void)seen0; (void)seen1;
	if (got >= 4) {
		unsigned int id = idbuf[0] | (idbuf[1]<<8) | (idbuf[2]<<16) | (idbuf[3]<<24);
		if (id != 0 && id != 0xffffffff) {
			printf("  Chain is GOOD at the pin level: a real IDCODE came back, so\n"
			       "  TCK/TMS/TDI/TDO and ground are all correct. If Quartus still\n"
			       "  reports \"JTAG chain broken\", the fault is in this firmware's\n"
			       "  emulation of the Blaster protocol, NOT the wiring.\n");
		} else {
			printf("  TDO stuck at 0x%08x during shift: the chain is open. Check\n"
			       "  GP17 (TDO) first, then TDI/TDO swap and common ground.\n", id);
		}
	} else {
		printf("  No shift data returned at all — check the probe firmware.\n");
	}

	libusb_release_interface(dev, 0);
	libusb_close(dev);
	libusb_exit(NULL);
	return 0;
}
