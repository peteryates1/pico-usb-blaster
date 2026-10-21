#!/usr/bin/env python3
"""Check the carrier's J1 serial port: the Pico has a CDC port, and bytes sent
to it come back through UART1 unchanged at each baud rate.

    test_cdc_uart.py [--serial e66440a2] [--adapter E6647C740343412F]
                     [--bauds 115200,921600,2000000] [--seconds 1.0] [--external]

By default the loopback is UART1's own (UARTCR.LBE), switched on over SWD
through the Debug Probe on J3 and switched off again afterwards. That tests USB,
the CDC interface, the bridge and the baud setting, but NOT the J1 pins or the
schmitt buffer. With J1 TX jumpered to J1 RX, pass --external to test those too
(no SWD then).

Exit 0 = pass, 1 = fail, 2 = could not run the test.
"""
import argparse, glob, os, random, subprocess, sys, threading, time

import serial

UART1_CR = 0x40038030
SET, CLR = 0x2000, 0x3000      # RP2040 atomic set/clear register aliases
LBE = 1 << 7


def find_cdc_ttys(usb_serial):
    """(interface dir, tty name) for every tty under the USB device with this serial."""
    found = []
    for dev in glob.glob("/sys/bus/usb/devices/*"):
        try:
            with open(os.path.join(dev, "serial")) as f:
                if f.read().strip().lower() != usb_serial.lower():
                    continue
        except OSError:
            continue
        for itf in sorted(glob.glob(dev + ":1.*")):
            for tty in glob.glob(os.path.join(itf, "tty", "*")):
                found.append((os.path.basename(itf), os.path.basename(tty)))
    return found


def swd(adapter, *cmds):
    """One OpenOCD session; the target keeps running. Returns the output."""
    args = ["openocd", "-f", "interface/cmsis-dap.cfg", "-c", "adapter serial " + adapter,
            "-f", "target/rp2040.cfg", "-c", "adapter speed 1000", "-c", "init"]
    for c in cmds:
        args += ["-c", c]
    args += ["-c", "exit"]
    r = subprocess.run(args, capture_output=True, text=True, timeout=60)
    return r.stdout + r.stderr


def read_word(adapter, addr):
    out = swd(adapter, "mdw 0x%08x" % addr)
    for line in out.splitlines():
        if line.lower().startswith("0x%08x:" % addr):
            return int(line.split()[1], 16)
    raise RuntimeError("SWD read of 0x%08x failed:\n%s" % (addr, out[-800:]))


def loopback(port, baud, seconds):
    """Send `seconds` worth of random bytes at `baud`, read them back, compare."""
    n = max(2000, int(baud / 10 * seconds))
    data = bytes(random.getrandbits(8) for _ in range(n))
    with serial.Serial(port, baud, timeout=0.05) as s:
        time.sleep(0.1)
        s.reset_input_buffer()
        got = bytearray()
        def writer():
            for i in range(0, n, 256):
                s.write(data[i:i + 256])
        t = threading.Thread(target=writer)
        t0 = time.time()
        t.start()
        deadline = t0 + n * 10 / baud * 3 + 2
        while len(got) < n and time.time() < deadline:
            got += s.read(4096)
        t.join()
        dt = time.time() - t0
        got += s.read(4096)              # anything late, to report extra bytes
    bad = next((i for i, (a, b) in enumerate(zip(data, got)) if a != b), None)
    ok = len(got) == n and bad is None
    print("  %8d baud: sent %d, got %d, %s, %.2f s (%.0f B/s)" %
          (baud, n, len(got), "match" if bad is None else "FIRST MISMATCH at byte %d" % bad,
           dt, len(got) / dt if dt else 0))
    return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--serial", default="e66440a2", help="USB serial of the carrier's Pico")
    ap.add_argument("--adapter", default="E6647C740343412F", help="Debug Probe on J3")
    ap.add_argument("--bauds", default="115200,921600,2000000")
    ap.add_argument("--seconds", type=float, default=1.0, help="data per baud, in seconds of line time")
    ap.add_argument("--external", action="store_true", help="J1 TX is jumpered to J1 RX; no SWD")
    a = ap.parse_args()

    ttys = find_cdc_ttys(a.serial)
    if not ttys:
        print("FAIL: the Pico %s has no serial port (no tty under any of its USB interfaces)" % a.serial)
        return 1
    if len(ttys) > 1:
        print("FAIL: expected one serial port, found %s" % ttys)
        return 1
    itf, tty = ttys[0]
    port = "/dev/" + tty
    print("serial port: %s on interface %s" % (port, itf))

    cr = None
    if not a.external:
        try:
            cr = read_word(a.adapter, UART1_CR)
        except Exception as e:
            print("cannot run: %s" % e)
            return 2
        if cr & 0x301 != 0x301:
            print("FAIL: UART1 is not enabled for TX and RX (UARTCR=0x%03x)" % cr)
            return 1
        swd(a.adapter, "mww 0x%08x 0x%x" % (UART1_CR + SET, LBE))
    ok = True
    try:
        if cr is not None:
            now = read_word(a.adapter, UART1_CR)
            if not now & LBE:
                print("cannot run: the SWD write did not set LBE (UARTCR=0x%03x)" % now)
                return 2
            print("loopback: UART1 internal (UARTCR 0x%03x -> 0x%03x)" % (cr, now))
        else:
            print("loopback: external, J1 TX to J1 RX")
        for b in (int(x) for x in a.bauds.split(",")):
            ok &= loopback(port, b, a.seconds)
    finally:
        if cr is not None and not cr & LBE:
            swd(a.adapter, "mww 0x%08x 0x%x" % (UART1_CR + CLR, LBE))
            after = read_word(a.adapter, UART1_CR)
            print("loopback off: UARTCR 0x%03x%s" % (after, "" if after == cr else
                  "  (NOT restored, was 0x%03x)" % cr))
    print("PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
