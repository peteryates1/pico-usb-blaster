#pragma once

/* The carrier's J1 UART, bridged to a USB serial port. Only built for the
 * carrier profiles (HAS_J1_UART in board_config.h). */
void cdc_uart_init(void);
void cdc_uart_task(void);
