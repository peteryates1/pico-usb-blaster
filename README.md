# Pico USB Blaster
This firmware turns your Raspberry Pi Pico (or other RP2040/RP2350-based board) 
into an **Altera USB-blaster / Intel FPGA Download Cable**-compatible JTAG/AS/PS programmer usable with Quartus, OpenOCD, etc.

The project relies only on **tinyUSB** stack, so it also should be also easily portable to any **tinyUSB**-capable device

## How-to
![plot](./doc/pic.jpg)

### Required parts
* Raspberry Pi Pico or Pico 2 / another RP2040/RP2350 board
* if non-3.3V operation required - adequate fast level shifter capable of [loading 1k TCK pulldown](https://www.intel.com/content/www/us/en/docs/programmable/683546/current/pull-up-and-pull-down-of-jtag-pins-during.html)
* i tested TXS0108E i had, and while it works, i can't recommend it (if 1k pulldown is present), because TCK quickly drops to ~2v instead of steady 3.3v

### Building
This fork is built with the [Raspberry Pi Pico SDK](https://github.com/raspberrypi/pico-sdk)
2.2.0; point `PICO_SDK_PATH` at your copy. The pin map is one of the profiles
in [src/board_config.h](./src/board_config.h), chosen with `-DBOARD_TYPE` when
you configure ([Pinout](#pinout) has the pins):

| `BOARD_TYPE` | Profile | For | Also on the USB device |
|--------------|---------|-----|------------------------|
| `0` | `BOARD_FLYING_LEADS` | a bare Pico on jumper wires | - |
| `1` | `BOARD_DEBUG_JTAG1` | the [pico-usb-debug-jtag](https://github.com/peteryates1/pico-usb-debug-jtag) carrier's JTAG1 channel (J7/J8) | the [J1 serial port](#the-carriers-j1-serial-port) and the [J2 debug probe](#the-carriers-j2-debug-probe) |
| `2` (default) | `BOARD_DEBUG_JTAG0` | the carrier's JTAG0 channel (J5/J6) | the same |

Use one build directory per variant:

```bash
export PICO_SDK_PATH=/path/to/pico-sdk
cd src

# the carrier's JTAG0
cmake -S . -B build-jtag0 -DPICO_BOARD=pico -DBOARD_TYPE=2
make -C build-jtag0 -j$(nproc)

# the carrier's JTAG1
cmake -S . -B build-jtag1 -DPICO_BOARD=pico -DBOARD_TYPE=1
make -C build-jtag1 -j$(nproc)

# flying leads
cmake -S . -B build-flying -DPICO_BOARD=pico -DBOARD_TYPE=0
make -C build-flying -j$(nproc)
```

Each directory then holds `pico_usb_blaster.uf2`, for BOOTSEL, and
`pico_usb_blaster.elf`, for an SWD probe. After the first `cmake`,
`make -C <dir>` alone rebuilds.

- `BOARD_TYPE` and the two options below are CMake cache variables. Running
  `cmake` on an existing directory without them keeps their old values, so
  give each variant its own directory. The configure step prints them:
  `-- blaster: BOARD_TYPE=2 JTAG_CLKDIV= TDO_DIAG_PULLDOWN=`.
- `-DJTAG_CLKDIV=n` sets TCK to 120 MHz / (4 × n). The default, 5, gives
  6 MHz, a genuine USB-Blaster's rate; 40 gives 750 kHz.
- `-DTDO_DIAG_PULLDOWN=1` is a diagnostic build that inverts TDO's pull, to
  tell a floating TDO from a driven one. `cmake` warns while it is set.
- `make` does not remake a deleted `.uf2`: the SDK writes it only when it
  links the `.elf`. Delete the `.elf` too and `make` writes both again.
- Tested on hardware with an RP2040 Pico. `-DPICO_BOARD=pico2` (RP2350) also
  builds, but no Pico 2 has run it.
- `check_variants.sh`, run from the repository root with `PICO_SDK_PATH` set,
  builds every profile from a fresh configure and checks that the default is
  JTAG0's image.

### Flashing
- **BOOTSEL**: hold BOOTSEL while you plug in the Pico's USB (on the carrier:
  hold BOOTSEL, press and release SW4, then release BOOTSEL). Copy the `.uf2`
  to the drive that appears, or run
  `picotool load -v -x build-jtag0/pico_usb_blaster.uf2`.
- **SWD**, from a CMSIS-DAP probe on the Pico's SWD pins. The carrier brings
  them out on J3 with SWCLK and SWDIO swapped, so J3 needs a crossed cable.
  Name the probe by its serial, because the carrier's J2 is a CMSIS-DAP probe
  too:

  ```bash
  openocd -f interface/cmsis-dap.cfg -c "adapter serial <probe serial>" \
          -c "set USE_CORE 0" -f target/rp2040.cfg -c "adapter speed 1000" \
          -c "program build-jtag0/pico_usb_blaster.elf verify reset exit"
  ```

  A `program` that fails partway leaves the Pico halted with its flash half
  written: retry, or recover it with BOOTSEL.

### Pinout
The PIO engine drives each pin on its own, so a profile can put the signals on
any GPIOs. The profiles in [src/board_config.h](./src/board_config.h):

| Signal | JTAG | AS | PS | Flying leads (`0`) | Carrier JTAG1 (`1`) | Carrier JTAG0 (`2`) |
|--------|------|----|----|--------------------|---------------------|---------------------|
| TCK_DCLK | TCK | DCLK | DCLK | GP18 | GP27 | GP20 |
| TMS_nCONFIG | TMS | nCONFIG | nCONFIG | GP19 | GP22 | GP18 |
| TDI_ASDI | TDI | ASDI | DATA0 | GP16 | GP21 | GP17 |
| TDO_CONF_DONE (in) | TDO | CONF_DONE | CONF_DONE | GP17 | GP26 | GP19 |
| nCE | - | nCE | - | GP6 | not wired | not wired |
| nCS | - | nCS | - | GP7 | not wired | not wired |
| DATAOUT_nSTATUS (in) | - | DATAOUT | nSTATUS | GP8 | GP9, not wired | GP9, not wired |
| level shifters' DIR | | | | - | GP3 | GP11 |
| activity LED | | | | GP25 | GP28 (LED6) | GP10 (LED7) |

- Flying leads: the JTAG pins match pico-dirtyJtag's `BOARD_PICO`, so one Pico
  can switch between the two firmwares without rewiring. GP0 and GP1 are the
  debug UART's TX and RX.
- The carrier is JTAG only: AS and PS are not wired. GP9 is its only free pin,
  and its pull-up reads 1, as on a real cable. The firmware drives DIR high
  before any JTAG pin becomes an output. There is no debug UART: GP0 and GP1
  are the carrier's J4, and `board_config.h` lists the other pins its
  pico-debug side owns.

Upstream's pin map (TCK on GP11, the other signals on GP12-17) is not one of
these profiles.

### Drivers
Follow the 'Installing the Intel FPGA Download Cable Driver' instruction from Intel website

The device replicates the overall USB-Blaster USB descriptor structure and pretends to have the same VID&PID for the drivers to be compatible.
The board-unique serial is provided by the **tinyUSB** bsp:
```
Bus 001 Device 003: ID 09fb:6001 Altera Blaster
Device Descriptor:
  bLength                18
  bDescriptorType         1
  bcdUSB               1.10
  bDeviceClass            0
  bDeviceSubClass         0
  bDeviceProtocol         0
  bMaxPacketSize0         8
  idVendor           0x09fb Altera
  idProduct          0x6001 Blaster
  bcdDevice            4.00
  iManufacturer           1 Pico
  iProduct                2 USB Blaster
  iSerial                 3 e6613893
  bNumConfigurations      1
  Configuration Descriptor:
    bLength                 9
    bDescriptorType         2
    wTotalLength       0x0020
    bNumInterfaces          1
    bConfigurationValue     1
    iConfiguration          0
    bmAttributes         0x80
      (Bus Powered)
    MaxPower              150mA
    Interface Descriptor:
      bLength                 9
      bDescriptorType         4
      bInterfaceNumber        0
      bAlternateSetting       0
      bNumEndpoints           2
      bInterfaceClass       255 Vendor Specific Class
      bInterfaceSubClass    255 Vendor Specific Subclass
      bInterfaceProtocol    255 Vendor Specific Protocol
      iInterface              0
      Endpoint Descriptor:
        bLength                 7
        bDescriptorType         5
        bEndpointAddress     0x81  EP 1 IN
        bmAttributes            2
          Transfer Type            Bulk
          Synch Type               None
          Usage Type               Data
        wMaxPacketSize     0x0040  1x 64 bytes
        bInterval               0
      Endpoint Descriptor:
        bLength                 7
        bDescriptorType         5
        bEndpointAddress     0x02  EP 2 OUT
        bmAttributes            2
          Transfer Type            Bulk
          Synch Type               None
          Usage Type               Data
        wMaxPacketSize     0x0040  1x 64 bytes
        bInterval               0
Device Status:     0x0000
  (Bus Powered)
```

### The carrier's J1 serial port

Built for the pico-usb-debug-jtag carrier (`-DBOARD_TYPE=1` or `2`), the device
also has a USB serial port for the carrier's J1 UART: a CDC-ACM function on
interfaces 1 and 2, named `J1 UART`, driving `uart1` (GP4 TX, GP5 RX through
the carrier's schmitt buffer). On Linux it is
`/dev/serial/by-id/usb-Pico_USB_Blaster_<serial>-if01`. The Blaster stays
interface 0 with the same endpoints, and the device descriptor does not change,
so Quartus sees the same cable. The host sets the baud and frame format. LED1
and LED2 show RX and TX traffic, and LED5 shows USB connected, as on the
Raspberry Pi Debug Probe. The flying-leads build is unchanged, byte for byte.

`test_cdc_uart.py` checks the port end to end through UART1's internal
loopback, which it switches on over SWD (through the Debug Probe on the
carrier's J3). Measured 2026-10-21:
- Loss-free at 115200, 921600, 2 M and 2.5 Mbaud.
- At 3 Mbaud, the UART's maximum on the 48 MHz peripheral clock, a continuous
  stream loses data: the bridge moves about 210 KB/s.
- To test the J1 pins themselves, jumper J1 TX to J1 RX and pass `--external`.

### The carrier's J2 debug probe

Built for the carrier, the device is also a CMSIS-DAP v2 debug probe on J2,
which is the Raspberry Pi Debug Probe's SWD port: interface 3, named
`J2 CMSIS-DAP`, on bulk endpoints 0x05 and 0x85. SWCLK is GP12 and SWDIO GP14,
read back on GP13 through the carrier's schmitt buffer. LED3 lights while a
debugger is connected, and LED4 while it has the target running. The command
handling and the PIO SWD layer are the Debug Probe's own, from
raspberrypi/debugprobe v2.3.1 (`src/cmsis_dap/README.md` lists what came from
where and what changed). SWD runs on pio1, beside the Blaster's pio0. The
flying-leads build is unchanged apart from its build date.

Name it by the device's serial:

```bash
openocd -f interface/cmsis-dap.cfg -c "adapter serial e66440a2" \
        -f target/rp2040.cfg -c "adapter speed 5000"
```

With more than one CMSIS-DAP probe attached, always name one. Without
`adapter serial`, OpenOCD takes the first it finds, and if that is this one, a
`program ... reset` meant for another probe's target reflashes the Pico on J2.

`test_swd_dap.py` checks it against an RP2040 on J2: the interface's shape, the
chip ID, bootrom block reads, a register round trip, halt and resume, and a
64 KB flash read. Measured 2026-10-22:
- Every check passes. Flash reads run at 66 KB/s at 1 MHz SWCLK and 229 KB/s
  at 5 MHz.
- Quartus is unaffected while the probe is idle: the carrier's ten-cycle run
  gave 30/30 single-try chain scans and 10/10 configurations at 74-76 s, as
  before.
- Both at once work, but share the Pico. With J2 reading the target's flash
  nonstop at 5 MHz (82 MB in 391 s, every 16 KB read identical to the first),
  three scan-configure-DoAll cycles all passed, and each configuration took
  90-91 s instead of 74-76 s.

### Usage
Now you can program Intel/Altera JTAG/AS/PS devices with Quartus or use it as a generic JTAG-adapter (where USB-Blaster is supported)

Unfortunately i can't confirm AS/PS operation, because i don't have such devices around, 
but looking at the protocol there should be no issues in theory

## Prettier version
I found RP2040-zero board the perfect size for an assembled device and also added a TXS0108E (see 'Required parts' for concerns regarding this exact level-shifter)

This setup uses onboard ws2812 rgb led (support included), pins 8-14 as data I/O and pin 15 as shifter output enable

![plot](./doc/v2.jpg)

And a 3D-printable enclosure. Instructions, STLs and a Fusion 360 source file are [here](./3d_print/)

![plot](./doc/v2_enclosure.jpg)


## Credits
* Protocol description, specs, FT245 EEPROM taken from
[Teensy_Blaster](https://github.com/Memotech-Bill/Teensy_Blaster/),
[usbd-blaster](https://github.com/sameer/usbd-blaster) and 20-year old forum posts
* WS2812 PIO code taken from [pico-examples](https://github.com/raspberrypi/pico-examples)
