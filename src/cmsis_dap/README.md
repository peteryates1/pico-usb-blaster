# CMSIS-DAP for the carrier's J2 probe

The command handling behind the carrier's J2 SWD port (`../dap_swd.c`) is
ARM's CMSIS-DAP firmware, taken as the Raspberry Pi Debug Probe ships it:
[raspberrypi/debugprobe](https://github.com/raspberrypi/debugprobe) tag
`debugprobe-v2.3.1`, commit `39e3c8d15ba36cd607a276d7dfe23bde03e88be0`.

| file | from debugprobe | licence | changed here |
|---|---|---|---|
| `cmsis_dap/DAP.c`, `DAP.h`, `DAP_vendor.c` | `CMSIS_DAP/CMSIS/DAP/Firmware/` | Apache-2.0 (ARM) | no |
| `DAP_config.h` | `include/DAP_config.h` | Apache-2.0 (ARM template, Raspberry Pi's settings) | no |
| `probe.h`, `sw_dp_pio.c` | `src/` | MIT (Raspberry Pi) | no |
| `probe.c`, `probe.pio` | `src/` | MIT (Raspberry Pi) | pio1 instead of pio0, which is the Blaster's; `probe.c` claims its state machine |

`DAP.c` is debugprobe's copy of ARM's, which reports two fewer packets than
`DAP_PACKET_COUNT` buffers ("HACK to avoid ring buffer fullness"); that is
upstream's, not ours.

Written here, not copied: `../probe_config.h`, which maps the PROBE_* names
the files above use onto `board_config.h`'s J2 pins and replaces debugprobe's
FreeRTOS-based version, and `../dap_swd.c`, the USB side.

The Apache License 2.0 is in `LICENSE-Apache-2.0.txt`. The MIT notices are in
each file's header.
