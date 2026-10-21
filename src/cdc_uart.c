/* The carrier's J1 UART, bridged to the USB serial port (CDC interface 0).
 *
 * This is the pico-debug half of the carrier, on the Raspberry Pi Debug
 * Probe's pin map: uart1 on GP4 (TX) / GP5 (RX), LED1 lit while bytes arrive
 * from the target, LED2 while bytes go to it. The host sets the baud and the
 * frame format through the port's line coding.
 *
 * RX is interrupt-driven into a ring because the main loop also runs the
 * Blaster: the UART's 32-byte FIFO lasts 160 us at 2 Mbaud, and the loop can
 * be away longer than that. TX needs no buffer of its own -- bytes wait in
 * TinyUSB's FIFO, and the host is NAKed while that is full.
 */
#include "board_config.h"

#if HAS_J1_UART

#include <stdbool.h>

#include "bsp/board_api.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/uart.h"
#include "tusb.h"

#include "cdc_uart.h"

#define J1_PIN_MASK ((1u << J1_UART_TX_PIN) | (1u << J1_UART_RX_PIN) | \
                     (1u << J1_UART_RX_TAP_PIN) | (1u << PD_UART_RX_LED_PIN) | \
                     (1u << PD_UART_TX_LED_PIN))
_Static_assert((J1_PIN_MASK & ~CARRIER_PICO_DEBUG_PINS) == 0,
               "the J1 bridge uses a pin the pico-debug side does not own");

#define RX_RING_SIZE 4096u  /* 20 ms at 2 Mbaud */
_Static_assert((RX_RING_SIZE & (RX_RING_SIZE - 1)) == 0, "RX_RING_SIZE must be a power of two");

static uint8_t rx_ring[RX_RING_SIZE];
static volatile uint32_t rx_head;  /* written by the IRQ only */
static volatile uint32_t rx_tail;  /* written by the task only */

#define LED_HOLD_MS 20
static uint32_t rx_led_ms, tx_led_ms;

static void on_uart_rx(void)
{
    while (uart_is_readable(J1_UART))
    {
        uint8_t c = (uint8_t)uart_get_hw(J1_UART)->dr;
        uint32_t head = rx_head;
        if (head - rx_tail < RX_RING_SIZE)  /* full: drop the new byte */
        {
            rx_ring[head & (RX_RING_SIZE - 1)] = c;
            __compiler_memory_barrier();
            rx_head = head + 1;
        }
    }
}

void cdc_uart_init(void)
{
    /* GP6 is wired straight to J1 RX: an input, pulled up so an unconnected
     * J1 idles high (the reset pull-down would hold it low, a break) */
    gpio_init(J1_UART_RX_TAP_PIN);
    gpio_pull_up(J1_UART_RX_TAP_PIN);

    gpio_init(PD_UART_RX_LED_PIN);
    gpio_set_dir(PD_UART_RX_LED_PIN, GPIO_OUT);
    gpio_init(PD_UART_TX_LED_PIN);
    gpio_set_dir(PD_UART_TX_LED_PIN, GPIO_OUT);

    uart_init(J1_UART, 115200);  /* TinyUSB's line coding until the host sets one */
    gpio_set_function(J1_UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(J1_UART_RX_PIN, GPIO_FUNC_UART);
    uart_set_hw_flow(J1_UART, false, false);
    uart_set_fifo_enabled(J1_UART, true);

    irq_set_exclusive_handler(J1_UART_IRQ, on_uart_rx);
    irq_set_enabled(J1_UART_IRQ, true);
    uart_set_irqs_enabled(J1_UART, true, false);
}

void cdc_uart_task(void)
{
    uint32_t now = board_millis();

    /* target -> host. With nobody listening (DTR low) the bytes are dropped
     * rather than held, so opening the port never replays stale output into
     * a protocol like JOP's download handshake. */
    uint32_t head = rx_head;
    __compiler_memory_barrier();
    uint32_t tail = rx_tail;
    if (head != tail)
    {
        if (tud_cdc_connected())
        {
            uint32_t n = head - tail;
            uint32_t room = tud_cdc_write_available();
            if (n > room)
                n = room;
            while (n)
            {
                uint32_t off = tail & (RX_RING_SIZE - 1);
                uint32_t chunk = RX_RING_SIZE - off;
                if (chunk > n)
                    chunk = n;
                tud_cdc_write(&rx_ring[off], chunk);
                tail += chunk;
                n -= chunk;
            }
            tud_cdc_write_flush();
        }
        else
        {
            tail = head;
        }
        __compiler_memory_barrier();
        rx_tail = tail;
        gpio_put(PD_UART_RX_LED_PIN, 1);
        rx_led_ms = now;
    }

    /* host -> target. An empty TX FIFO takes 32 bytes at once; otherwise
     * feed it a byte at a time while it has room. */
    if (tud_cdc_available() && uart_is_writable(J1_UART))
    {
        if (uart_get_hw(J1_UART)->fr & UART_UARTFR_TXFE_BITS)
        {
            uint8_t buf[32];
            uint32_t n = tud_cdc_read(buf, sizeof(buf));
            for (uint32_t i = 0; i < n; i++)
                uart_get_hw(J1_UART)->dr = buf[i];
        }
        else
        {
            while (uart_is_writable(J1_UART) && tud_cdc_available())
            {
                uint8_t c;
                tud_cdc_read(&c, 1);
                uart_get_hw(J1_UART)->dr = c;
            }
        }
        gpio_put(PD_UART_TX_LED_PIN, 1);
        tx_led_ms = now;
    }

    if (now - rx_led_ms >= LED_HOLD_MS)
        gpio_put(PD_UART_RX_LED_PIN, 0);
    if (now - tx_led_ms >= LED_HOLD_MS)
        gpio_put(PD_UART_TX_LED_PIN, 0);
}

void tud_cdc_line_coding_cb(uint8_t itf, cdc_line_coding_t const *coding)
{
    (void)itf;
    if (coding->bit_rate == 0)  /* B0 means "hang up", not a rate */
        return;
    uint data_bits = (coding->data_bits >= 5 && coding->data_bits <= 8) ? coding->data_bits : 8;
    uint stop_bits = coding->stop_bits == 2 ? 2 : 1;  /* 1.5 is not supported */
    uart_parity_t parity = coding->parity == 1 ? UART_PARITY_ODD
                         : coding->parity == 2 ? UART_PARITY_EVEN
                                               : UART_PARITY_NONE;
    uart_set_baudrate(J1_UART, coding->bit_rate);
    uart_set_format(J1_UART, data_bits, stop_bits, parity);
}

#endif /* HAS_J1_UART */
