/* The carrier's J2 SWD port as a CMSIS-DAP v2 probe: USB interface 3, beside
 * the Blaster (interface 0) and the J1 serial port (1 and 2).
 *
 * The commands are ARM's CMSIS-DAP firmware (cmsis_dap/DAP.c) with SWD driven
 * by PIO (probe.c, sw_dp_pio.c), as the Raspberry Pi Debug Probe runs them.
 * This file is only the USB side.
 *
 * CMSIS-DAP v2 carries one command per bulk OUT packet and answers each with
 * one bulk IN packet, in order. A host may send several commands before it
 * reads the first answer -- OpenOCD keeps up to the DAP_Info packet count in
 * flight -- so requests and responses each get a ring of DAP_PACKET_COUNT
 * packets.
 *
 * It is a TinyUSB application driver rather than a second instance of the
 * vendor class, because the vendor class pours its OUT endpoint into a byte
 * FIFO and loses the packet boundaries the protocol is made of. TinyUSB calls
 * xfer_cb from tud_task(), in the main loop, and dap_swd_task() runs there
 * too, so nothing here needs a lock. A command holds the loop while it runs,
 * and the Blaster waits that long.
 */
#include "board_config.h"

/* CMakeLists.txt adds this file, and the DAP sources, for the carrier
 * profiles only; board_config.h must agree that this profile has J2 */
#if !HAS_J2_SWD
#error "dap_swd.c is built, but board_config.h gives this profile no J2 probe"
#endif

#include <stdbool.h>

#include "hardware/gpio.h"
#include "tusb.h"
#include "device/usbd_pvt.h"

#include "DAP_config.h"
#include "DAP.h"
#include "dap_swd.h"

/* One DAP packet is one full-speed USB packet */
_Static_assert(DAP_PACKET_SIZE == CFG_TUD_VENDOR_EPSIZE,
               "DAP_PACKET_SIZE must match J2's endpoint size (usb_descriptors.c)");

#define J2_PIN_MASK ((1u << J2_SWCLK_PIN) | (1u << J2_SWDI_PIN) | (1u << J2_SWDIO_PIN) | \
                     (1u << PD_DAP_CONNECTED_LED_PIN) | (1u << PD_DAP_RUNNING_LED_PIN))
_Static_assert((J2_PIN_MASK & ~CARRIER_PICO_DEBUG_PINS) == 0,
               "the J2 probe uses a pin the pico-debug side does not own");

#define N DAP_PACKET_COUNT

static uint8_t req_buf[N][DAP_PACKET_SIZE];
static uint8_t rsp_buf[N][DAP_PACKET_SIZE];
static uint16_t rsp_len[N];

/* Free-running counts; a packet's slot is its count mod N */
static uint32_t req_in, req_done;    /* requests received, executed */
static uint32_t rsp_made, rsp_sent;  /* responses made, sent */

static uint8_t dap_rhport, ep_out, ep_in;
static bool out_armed, in_busy;

static void arm_out(void)
{
    if (ep_out && !out_armed && req_in - req_done < N)
        out_armed = usbd_edpt_xfer(dap_rhport, ep_out, req_buf[req_in % N], DAP_PACKET_SIZE);
}

static void send_next(void)
{
    if (ep_in && !in_busy && rsp_sent != rsp_made)
        in_busy = usbd_edpt_xfer(dap_rhport, ep_in, rsp_buf[rsp_sent % N], rsp_len[rsp_sent % N]);
}

static void dap_driver_init(void)
{
}

static bool dap_driver_deinit(void)
{
    return true;
}

static void dap_driver_reset(uint8_t rhport)
{
    (void)rhport;
    req_in = req_done = rsp_made = rsp_sent = 0;
    ep_out = ep_in = 0;
    out_armed = in_busy = false;
}

/* Claims the CMSIS-DAP interface only. The Blaster is vendor class too, but
 * ff/ff/ff, so it goes on to TinyUSB's vendor driver. */
static uint16_t dap_driver_open(uint8_t rhport, tusb_desc_interface_t const *itf, uint16_t max_len)
{
    TU_VERIFY(itf->bInterfaceClass == TUSB_CLASS_VENDOR_SPECIFIC &&
              itf->bInterfaceSubClass == 0 && itf->bInterfaceProtocol == 0 &&
              itf->bNumEndpoints == 2, 0);
    uint16_t const len = sizeof(tusb_desc_interface_t) + 2 * sizeof(tusb_desc_endpoint_t);
    TU_VERIFY(max_len >= len, 0);
    TU_ASSERT(usbd_open_edpt_pair(rhport, tu_desc_next(itf), 2, TUSB_XFER_BULK, &ep_out, &ep_in), 0);
    dap_rhport = rhport;
    arm_out();
    return len;
}

static bool dap_driver_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const *request)
{
    (void)rhport;
    (void)stage;
    (void)request;
    return false;  /* CMSIS-DAP v2 has no class requests */
}

static bool dap_driver_xfer_cb(uint8_t rhport, uint8_t ep_addr, xfer_result_t result, uint32_t xferred_bytes)
{
    (void)rhport;
    if (ep_addr == ep_out)
    {
        out_armed = false;
        if (result == XFER_RESULT_SUCCESS && xferred_bytes > 0)
            req_in++;
        arm_out();
        return true;
    }
    if (ep_addr == ep_in)
    {
        in_busy = false;
        rsp_sent++;
        send_next();
        return true;
    }
    return false;
}

static usbd_class_driver_t const dap_driver =
{
    .name = "CMSIS-DAP",
    .init = dap_driver_init,
    .deinit = dap_driver_deinit,
    .reset = dap_driver_reset,
    .open = dap_driver_open,
    .control_xfer_cb = dap_driver_control_xfer_cb,
    .xfer_cb = dap_driver_xfer_cb,
    .sof = NULL,
};

usbd_class_driver_t const *usbd_app_driver_get_cb(uint8_t *driver_count)
{
    *driver_count = 1;
    return &dap_driver;
}

void dap_swd_init(void)
{
    /* J2 idles the way a DAP_Disconnect leaves it: inputs with no pulls, so
     * only the target's own pulls set its lines (the pads reset pulled down) */
    gpio_disable_pulls(J2_SWCLK_PIN);
    gpio_disable_pulls(J2_SWDIO_PIN);

    DAP_Setup();  /* default clock and transfer settings; LED3 and LED4 off */
}

/* Run what has arrived, while there is room for the answers.
 *
 * DAP_QueueCommands packets wait for the packet that ends the queue (any other
 * command), then all of them run as DAP_ExecuteCommands, in order and each
 * still answered: queuing only removes the USB round trips between them. If
 * every slot fills with queued packets they run anyway rather than deadlock. */
void dap_swd_task(void)
{
    while (req_done != req_in && rsp_made - rsp_sent < N)
    {
        uint8_t *req = req_buf[req_done % N];
        if (req[0] == ID_DAP_QueueCommands)
        {
            uint32_t i = req_done;
            while (i != req_in && req_buf[i % N][0] == ID_DAP_QueueCommands)
                i++;
            if (i == req_in && req_in - req_done < N)
                return;  /* the queue has not ended yet */
            req[0] = ID_DAP_ExecuteCommands;
        }
        rsp_len[rsp_made % N] = (uint16_t)DAP_ExecuteCommand(req, rsp_buf[rsp_made % N]);
        rsp_made++;
        req_done++;
        arm_out();
        send_next();
    }
}

