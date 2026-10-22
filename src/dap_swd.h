#pragma once

/* The carrier's J2 SWD port as a CMSIS-DAP v2 probe. Only built for the
 * carrier profiles (HAS_J2_SWD in board_config.h). */
void dap_swd_init(void);
void dap_swd_task(void);
