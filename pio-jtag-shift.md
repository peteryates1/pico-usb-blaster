# pico-usb-blaster changes — 2026-02-28

Baseline: EP4CE115 (3.57 MB) at ~490 KB/s / 7.5s via `program_fpga.py`.

## 1. PIO JTAG Shift (`src/blaster_jtag.pio`, `src/blaster.c`)

New 2-instruction PIO program replaces the bit-banged `shift()` loop:

```
.wrap_target
    out pins, 1     side 0 [1]  ; TDI output, TCK low  (2 cycles)
    in pins, 1      side 1 [1]  ; Read TDO,   TCK high (2 cycles)
.wrap
```

- 4 SM-clock cycles per bit, 50% TCK duty cycle
- Autopull/autopush at 8 bits, right-shift (LSB-first)
- Stalls on empty TX FIFO with TCK low (safe idle)
- Divider 2: 120 MHz sys / 2 = 60 MHz SM clock = **15 MHz TCK**
- TDO input synchronizer bypassed for minimum latency
- Uses pio0 (ws2812 LED uses pio1, no conflict)

### Firmware integration

**Init** (`blaster_init()`): Load program on pio0, claim SM, configure pins
(OUT=TDI, IN=TDO, side-set=TCK), enable SM, then switch TCK/TDI back to SIO
so bitbang still works. SM stalls immediately on empty TX FIFO.

**Pin switching** on shift mode entry/exit:
- `shift_enter_pio()`: `pio_gpio_init()` on TCK + TDI (2 register writes)
- `shift_exit_pio()`: drain RX FIFO, `gpio_set_function(GPIO_FUNC_SIO)`,
  restore output direction

**`shift_pio()`** — the per-byte PIO interface:
```c
static inline uint8_t shift_pio(uint8_t data) {
    *(io_rw_8 *)&jtag_pio->txf[jtag_sm] = data;
    while (pio_sm_is_rx_fifo_empty(jtag_pio, jtag_sm))
        tight_loop_contents();
    return (uint8_t)(jtag_pio->rxf[jtag_sm] >> 24);
}
```

Original `shift()` renamed to `shift_bitbang()` and kept as fallback if PIO
init fails (`jtag_pio_ok == false`). Bitbang mode (TAP navigation) is
unchanged.

### TDO readback gotcha

PIO right-shift `in` places 8 bits at positions [31:24] of the 32-bit RX
FIFO word. An 8-bit FIFO read (`*(io_rw_8 *)&rxf`) returns bits [7:0] =
always zero. Must read the full 32-bit word and extract with `>> 24`.

TX side is fine: 8-bit writes to the TX FIFO register replicate the byte to
all 4 positions (RP2040 hardware behavior), and right-shift `out` consumes
bits [7:0].

## 2. TinyUSB Vendor Control Callback Fix (`src/main.c`)

The original code defined `tud_control_request_cb()`. **This function does
not exist in TinyUSB** — it was never called. All vendor control requests
(EEPROM reads, FTDI status queries) were silently STALLing on the control
endpoint.

This didn't matter for `program_fpga.py` which only uses bulk endpoints, but
broke `jtagd`/`quartus_pgm` which rely on FTDI vendor control requests.

Fix: replace with the correct callback:
```c
bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage,
                                tusb_control_request_t const *request)
```

The `stage` parameter distinguishes SETUP/DATA/ACK phases; we handle
everything in CONTROL_STAGE_SETUP and return true for other stages.

## 3. FTDI Control Request Emulation (`src/main.c`)

`jtagd` (the Quartus JTAG daemon) talks to USB-Blasters via FTDI-specific
vendor control requests. Added proper handlers for:

| bRequest | Name | Direction | Handling |
|----------|------|-----------|----------|
| 0x00 | SIO_RESET | OUT | Calls `blaster_reset()` |
| 0x01 | SIO_MODEM_CTRL | OUT | ACK silently |
| 0x02 | SIO_SET_FLOW_CTRL | OUT | ACK silently |
| 0x03 | SIO_SET_BAUD_RATE | OUT | ACK silently |
| 0x04 | SIO_SET_DATA | OUT | ACK silently |
| 0x05 | SIO_GET_MODEM_STATUS | IN | Returns `{0x31, 0x60}` (CTS+DSR, THRE+TEMT) |
| 0x09 | SIO_SET_LATENCY_TIMER | OUT | Stores wValue; used for bulk IN interval |
| 0x0A | SIO_GET_LATENCY_TIMER | IN | Returns stored latency value |
| 0x90 | READ_EEPROM | IN | Returns 2 bytes from FT245 EEPROM table (existing) |

The modem/line status bytes `{0x31, 0x60}` match what the real FT245BL
reports and match the 2-byte header prepended to every bulk IN packet.

## 4. TX FIFO Accumulation Fix (`src/main.c`)

`vendor_task()` sends periodic FTDI status packets via
`tud_vendor_write()` + `tud_vendor_write_flush()`. The problem:
`tud_vendor_write()` appends to TinyUSB's internal TX FIFO. If the host
hasn't read the previous packet yet, multiple writes concatenate into a
single USB IN transfer:

```
Expected:  [31 60]  [31 60]  [31 60 xx]
Actual:    [31 60 31 60 31 60 31 60 31 60 xx]
```

`jtagd` strips the first 2-byte FTDI header and treats the rest as payload.
With duplicate headers in the payload, it sees garbage instead of the JTAG
read-back data.

Fix: check that the TX FIFO is fully drained before writing:
```c
if (tud_vendor_write_available() < CFG_TUD_VENDOR_TX_BUFSIZE)
    return;  // previous packet still pending
```

This ensures each USB IN transfer contains exactly one FTDI status header.

## 5. Bulk IN Latency (`src/main.c`)

The bulk IN response interval was hardcoded at 10ms. `jtagd` sets the FTDI
latency timer to 1ms via SET_LATENCY_TIMER. Changed `vendor_task()` to use
the stored `ftdi_latency_timer` value instead of a constant.

Also added immediate send when data is available (`tx_ready > 0`) rather
than waiting for the timer — critical for `jtagd` which expects prompt
read-back responses during chain scanning.

## 6. RX Buffer Size (`src/tusb_config.h`)

`CFG_TUD_VENDOR_RX_BUFSIZE` increased from 64 to 1024. Allows TinyUSB to
accept multiple USB OUT packets between firmware processing, reducing NAK
frequency.

## 7. Process All Available Data (`src/main.c`)

Changed `vendor_task()` from `if (tud_vendor_available())` to
`while (tud_vendor_available())` so all buffered packets are processed
before yielding back to `tud_task()`.

## 8. C Host Programmer (`program_fpga.c`)

New file. Minimal Cyclone IV JTAG programmer using libusb directly. Same
protocol sequence as `program_fpga.py` but with lower per-transfer overhead.

Build: `gcc -O2 -o program_fpga program_fpga.c -lusb-1.0`

Usage: `sudo ./program_fpga <bitstream.rbf>`

## Results

### EP4CE115 programming time (3.57 MB bitstream)

| Host | Time | Throughput |
|------|------|------------|
| `program_fpga.c` (C/libusb) | ~6.5s | ~560 KB/s |
| `program_fpga.py` (Python/pyusb) | ~7s | ~490 KB/s |
| `quartus_pgm` via `jtagd` | ~37s | ~96 KB/s |
| Baseline (bitbang + Python, before) | 7.5s | ~490 KB/s |

### Analysis

The PIO shifts at 15 MHz vs ~6 MHz bitbang (2.5x faster per byte), but only
delivers ~5% end-to-end improvement. The bottleneck is USB Full Speed bulk
scheduling: we achieve ~8 transactions per 1ms frame out of ~22 theoretical
max. The firmware processes packets faster than USB delivers them, so PIO's
speed advantage is mostly wasted waiting for the next USB packet.

`quartus_pgm` is much slower because it performs additional verification
passes and uses smaller shift sequences. But the important outcome is that it
**works at all** — the FTDI emulation fixes (#2-#5) were the real win,
enabling compatibility with Intel's standard programming tools.

### Possible future improvements

To actually benefit from PIO's speed, the USB delivery rate needs to
increase. Options:
- DMA from a staging buffer to PIO (eliminate per-byte CPU involvement)
- USB double-buffering tuning on RP2040
- Larger shift chunks in the host protocol
- Async USB transfers on the host side

## Files Changed

```
src/blaster_jtag.pio   (new)   PIO program + C init helper
src/blaster.c                  PIO shift, pin switching, bitbang fallback
src/main.c                     FTDI emulation, TinyUSB callback, TX FIFO fix, latency
src/tusb_config.h              RX buffer 64 -> 1024
src/CMakeLists.txt             PIO header generation
program_fpga.c         (new)   C host programmer
```
