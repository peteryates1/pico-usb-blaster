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
#define nCE_PIN             7   /* not wired to the header; drives LED1 */
#define nCS_PIN             8   /* not wired to the header; drives LED2 */
#define DATAOUT_nSTATUS_PIN 9   /* not wired; GP9 is the board's only free pin,
                                 * so its pull-up reads 1 like a real cable */
#define ACTIVE_LED_PIN      10  /* LED7 */

#elif BOARD_TYPE == BOARD_DEBUG_JTAG0
/* ../pico-usb-debug-jtag carrier board, JTAG0 channel (headers J5 / J6). */
#define TCK_DCLK_PIN        20  /* U6 */
#define TMS_nCONFIG_PIN     18  /* U4 */
#define TDI_ASDI_PIN        17  /* U3 */
#define TDO_CONF_DONE_PIN   19  /* U5, input only */
#define SHIFTER_DIR_PIN     11  /* U3/U4/U6 DIR, 10k pulldown R15 */
#define nCE_PIN             7   /* not wired to the header; drives LED1 */
#define nCS_PIN             8   /* not wired to the header; drives LED2 */
#define DATAOUT_nSTATUS_PIN 9   /* free pin — see above */
#define ACTIVE_LED_PIN      10  /* LED7 */

#else
#error "unknown BOARD_TYPE"
#endif
