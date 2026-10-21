#pragma once

/* Board / harness pin profiles.
 *
 * The PIO shift engine sets its sideset/out/in pins independently, so the
 * JTAG pins do NOT have to be adjacent or in any particular order — any
 * mapping works.
 *
 * Select with -DBOARD_TYPE=... at configure time, or edit the default below.
 */

#define BOARD_FLYING_LEADS   0  /* bare Pico + jumper wires, A-E115FB harness */
#define BOARD_DEBUG_JTAG1    1  /* pico-usb-debug-jtag carrier, JTAG1 channel */
#define BOARD_DEBUG_JTAG0    2  /* pico-usb-debug-jtag carrier, JTAG0 channel */

#ifndef BOARD_TYPE
#define BOARD_TYPE BOARD_DEBUG_JTAG1
#endif


#if BOARD_TYPE == BOARD_FLYING_LEADS
/* GP16=TDI, GP17=TDO, GP18=TCK, GP19=TMS — matches pico-dirtyJtag's
 * BOARD_PICO profile, so the same Pico can be swapped between the two
 * firmwares with no rewiring. Verified on hardware: jtagconfig reads
 * IDCODE 020F70DD (EP4CE115) through this mapping. */
#define TCK_DCLK_PIN        18
#define TMS_nCONFIG_PIN     19
#define TDI_ASDI_PIN        16
#define TDO_CONF_DONE_PIN   17
#define nCE_PIN             6   /* unused for JTAG, keep out of the way */
#define nCS_PIN             7   /* unused for JTAG, keep out of the way */
#define DATAOUT_nSTATUS_PIN 8   /* unused for JTAG, keep out of the way */
#define ACTIVE_LED_PIN      PICO_DEFAULT_LED_PIN

#elif BOARD_TYPE == BOARD_DEBUG_JTAG1
/* ../pico-usb-debug-jtag carrier board, JTAG1 channel (headers J7 / J8).
 * Each of TCK/TMS/TDI goes Pico -> SN74LVC1T45 (A->B) -> 33R -> target;
 * TDO returns through U9, whose DIR is hard-wired to GND (always target->Pico).
 * SHIFTER_DIR_PIN drives the DIR input of U7/U8/U10 and MUST be high before
 * the Pico drives TCK/TMS/TDI — with DIR low the shifter drives its own A pin
 * and a Pico output on the same net would be bus contention.
 * AS/PS modes are not wired on this board: JTAG only. */
#define TCK_DCLK_PIN        27  /* U10 */
#define TMS_nCONFIG_PIN     22  /* U8  */
#define TDI_ASDI_PIN        21  /* U7  */
#define TDO_CONF_DONE_PIN   26  /* U9, input only */
#define SHIFTER_DIR_PIN     3   /* U7/U8/U10 DIR, 10k pulldown R14 */
/* nCE_PIN / nCS_PIN (AS mode) are not wired, so they stay undefined: the
 * GPIOs they used to drive, GP7/GP8, are the pico-debug side's UART LEDs. */
#define DATAOUT_nSTATUS_PIN 9   /* not wired; GP9 is the board's only free pin,
                                 * so its pull-up reads 1 like a real cable */
#define ACTIVE_LED_PIN      28  /* LED6, JTAG1's LED */

#elif BOARD_TYPE == BOARD_DEBUG_JTAG0
/* ../pico-usb-debug-jtag carrier board, JTAG0 channel (headers J5 / J6). */
#define TCK_DCLK_PIN        20  /* U6 */
#define TMS_nCONFIG_PIN     18  /* U4 */
#define TDI_ASDI_PIN        17  /* U3 */
#define TDO_CONF_DONE_PIN   19  /* U5, input only */
#define SHIFTER_DIR_PIN     11  /* U3/U4/U6 DIR, 10k pulldown R15 */
/* nCE_PIN / nCS_PIN: not wired — see the JTAG1 profile */
#define DATAOUT_nSTATUS_PIN 9   /* free pin — see above */
#define ACTIVE_LED_PIN      10  /* LED7, JTAG0's LED */

#else
#error "unknown BOARD_TYPE"
#endif

#if BOARD_TYPE == BOARD_DEBUG_JTAG1 || BOARD_TYPE == BOARD_DEBUG_JTAG0
/* Pins the carrier's pico-debug side owns. They follow the Raspberry Pi Debug
 * Probe's pin map: UART on J1 (GP4 TX, GP5 RX via schmitt, GP6 raw tap), SWD
 * on J2 (GP12 SWCLK, GP13 SWDI via schmitt, GP14 SWDIO), UART C on J4
 * (GP0/GP1), and the probe's five LEDs from pico-sdk boards/debug_probe.h:
 * GP2 LED5 USB connected, GP7 LED1 UART RX, GP8 LED2 UART TX, GP15 LED3 DAP
 * connected, GP16 LED4 DAP running. blaster.c refuses to build if the JTAG
 * side touches any of them. */
#define CARRIER_PICO_DEBUG_PINS \
    ((1u << 0) | (1u << 1) | (1u << 2) | (1u << 4) | (1u << 5) | (1u << 6) | \
     (1u << 7) | (1u << 8) | (1u << 12) | (1u << 13) | (1u << 14) | \
     (1u << 15) | (1u << 16))

/* The pico-debug side's J1 UART, bridged to a USB serial port (cdc_uart.c).
 * J1 RX reaches GP5 through U2A's schmitt buffer and GP6 directly through
 * 100R. GP6 stays an input with its pull-up: the pad's reset pull-down would
 * hold an unconnected J1 RX low, which the UART reads as a continuous break. */
#define J1_UART             uart1
#define J1_UART_IRQ         UART1_IRQ
#define J1_UART_TX_PIN      4   /* J1.1 via R1 */
#define J1_UART_RX_PIN      5   /* J1.3 via U2A */
#define J1_UART_RX_TAP_PIN  6   /* J1.3 via R2, unbuffered */
#define PD_UART_RX_LED_PIN  7   /* LED1: bytes from the target */
#define PD_UART_TX_LED_PIN  8   /* LED2: bytes to the target */
#define PD_USB_LED_PIN      2   /* LED5: USB connected */
#define HAS_J1_UART         1
#endif

#ifndef HAS_J1_UART
#define HAS_J1_UART         0
#endif
