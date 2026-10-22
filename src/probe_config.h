#pragma once

/* The carrier's J2 probe in the names debugprobe's SWD layer expects (probe.c,
 * probe.pio, sw_dp_pio.c and DAP_config.h, all from debugprobe-v2.3.1). This
 * replaces debugprobe's own probe_config.h, which also pulls in FreeRTOS:
 * here the probe runs in the main loop beside the Blaster. The pins are
 * board_config.h's. */
#include "board_config.h"

#ifdef ACTIVE_LED_WS2812_PIN
#error "ws2812.c drives pio1 SM0, which the J2 probe uses"
#endif

#define PROBE_PIO           pio1    /* pio0 is the Blaster's */
#define PROBE_SM            0

/* SWDIO is read back on its own pin, through U2B, as on the Debug Probe */
#define PROBE_IO_SWDI
#define PROBE_PIN_OFFSET    J2_SWCLK_PIN
#define PROBE_PIN_SWCLK     J2_SWCLK_PIN
#define PROBE_PIN_SWDI      J2_SWDI_PIN
#define PROBE_PIN_SWDIO     J2_SWDIO_PIN
/* J2 carries no target reset, so PROBE_PIN_RESET stays undefined */

#define PROBE_DAP_CONNECTED_LED PD_DAP_CONNECTED_LED_PIN
#define PROBE_DAP_RUNNING_LED   PD_DAP_RUNNING_LED_PIN

#define probe_info(format, ...)  ((void)0)
#define probe_debug(format, ...) ((void)0)
#define probe_dump(format, ...)  ((void)0)
