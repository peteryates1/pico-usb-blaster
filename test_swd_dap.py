#!/usr/bin/env python3
"""Check the carrier's J2 debug probe: the Pico has a CMSIS-DAP v2 interface
beside the Blaster, and OpenOCD can debug an RP2040 wired to J2 through it.

    test_swd_dap.py [--serial e66440a2] [--speed 1000]

The target on J2 must be an RP2040 (the test reads its chip ID). Everything is
read-only except one round trip through WATCHDOG_SCRATCH0, which is put back,
and a halt/resume of core 0.

Exit 0 = pass, 1 = fail, 2 = could not run the test.
"""
import argparse, glob, os, re, subprocess, sys

CHIP_ID = 0x40000000           # SYSINFO_CHIP_ID: REVISION[31:28] PART[27:12] MANUFACTURER[11:0]
ROM_MAGIC = 0x00000010         # 'M', 'u', 0x01, then the bootrom version
SCRATCH0 = 0x4005800c          # WATCHDOG_SCRATCH0; the bootrom uses only 4-7
PATTERN = 0x5a3cc3a5


def find_device(usb_serial):
    for dev in glob.glob("/sys/bus/usb/devices/*"):
        try:
            with open(os.path.join(dev, "serial")) as f:
                if f.read().strip().lower() == usb_serial.lower():
                    return dev
        except OSError:
            continue
    return None


def interfaces(dev):
    """{interface number: {"cls", "sub", "proto", "name", "eps"}}, the endpoints
    in DESCRIPTOR order (OpenOCD takes the first as OUT, the second as IN)."""
    with open(os.path.join(dev, "descriptors"), "rb") as f:
        raw = f.read()
    out, cur, i = {}, None, 0
    while i + 2 <= len(raw):
        n, kind = raw[i], raw[i + 1]
        if n == 0:
            break
        if kind == 4:                                  # interface
            cur = raw[i + 2]
            out.setdefault(cur, {"cls": raw[i + 5], "sub": raw[i + 6], "proto": raw[i + 7],
                                 "name": "", "eps": []})
        elif kind == 5 and cur is not None:            # endpoint
            out[cur]["eps"].append((raw[i + 2], raw[i + 3] & 3))
        i += n
    for num, itf in out.items():
        try:
            with open("%s:1.%d/interface" % (dev, num)) as f:
                itf["name"] = f.read().strip()
        except OSError:
            pass
    return out


def openocd(serial, speed, script):
    # Core 0 only: rp2040.cfg's default SMP pair cannot resume a target whose
    # core 1 never left the bootrom ("core1 not halted"), which is the usual case
    args = ["openocd", "-f", "interface/cmsis-dap.cfg", "-c", "adapter serial " + serial,
            "-c", "set USE_CORE 0",
            "-f", "target/rp2040.cfg", "-c", "adapter speed %d" % speed, "-c", "init",
            "-c", script, "-c", "exit"]
    r = subprocess.run(args, capture_output=True, text=True, timeout=120)
    return r.returncode, r.stdout + r.stderr


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--serial", default="e66440a2", help="USB serial of the carrier's Pico")
    ap.add_argument("--speed", type=int, default=1000, help="SWCLK in kHz")
    a = ap.parse_args()

    dev = find_device(a.serial)
    if not dev:
        print("cannot run: no USB device with serial %s" % a.serial)
        return 2
    itfs = interfaces(dev)
    ok = True

    blaster = itfs.get(0)
    if blaster and (blaster["cls"], blaster["sub"], blaster["proto"]) == (0xff, 0xff, 0xff) \
            and blaster["eps"] == [(0x81, 2), (0x02, 2)]:
        print("PASS interface 0 is still the Blaster (ff/ff/ff, bulk 0x81 IN then 0x02 OUT)")
    else:
        print("FAIL interface 0 is not the Blaster Quartus expects: %s" % blaster)
        ok = False

    dap = [(n, i) for n, i in sorted(itfs.items()) if "CMSIS-DAP" in i["name"]]
    if len(dap) != 1:
        print("FAIL expected one interface named CMSIS-DAP, found %d: %s" %
              (len(dap), {n: i["name"] for n, i in itfs.items()}))
        return 1
    n, i = dap[0]
    shape = (i["cls"], i["sub"], i["proto"]) == (0xff, 0, 0) and len(i["eps"]) >= 2 \
        and i["eps"][0][1] == 2 and not i["eps"][0][0] & 0x80 \
        and i["eps"][1][1] == 2 and bool(i["eps"][1][0] & 0x80)
    print("%s interface %d '%s': class %02x/%02x/%02x, endpoints %s" %
          ("PASS" if shape else "FAIL", n, i["name"], i["cls"], i["sub"], i["proto"],
           " ".join("0x%02x" % e for e, _ in i["eps"])))
    ok &= shape

    script = "; ".join([
        'echo "chip_id [read_memory 0x%08x 32 1]"' % CHIP_ID,
        'echo "rom_magic [read_memory 0x%08x 32 1]"' % ROM_MAGIC,
        'set a [read_memory 0 32 256]', 'set b [read_memory 0 32 256]',
        'echo "rom_reads_equal [expr {$a eq $b}] words [llength $a]"',
        'set orig [read_memory 0x%08x 32 1]' % SCRATCH0,
        'write_memory 0x%08x 32 {0x%08x}' % (SCRATCH0, PATTERN),
        'set got [read_memory 0x%08x 32 1]' % SCRATCH0,
        'write_memory 0x%08x 32 $orig' % SCRATCH0,
        'echo "scratch orig $orig got $got after [read_memory 0x%08x 32 1]"' % SCRATCH0,
        'halt 1000', 'echo "halted [rp2040.core0 curstate]"',
        'resume', 'echo "resumed [rp2040.core0 curstate]"',
        'set t0 [ms]', 'set x [read_memory 0x10000000 32 16384]', 'set t1 [ms]',
        'echo "xip_read words [llength $x] ms [expr {$t1 - $t0}]"',
    ])
    try:
        rc, log = openocd(a.serial, a.speed, script)
    except FileNotFoundError:
        print("cannot run: openocd is not installed")
        return 2
    except subprocess.TimeoutExpired:
        print("FAIL openocd did not finish in 120 s")
        return 1

    def val(key):
        m = re.search(r"^%s (.*)$" % key, log, re.M)
        return m.group(1).split() if m else None

    def num(s):
        return int(s, 0)

    checks = []
    using = re.search(r"Using CMSIS-DAPv2 interface with VID:PID=(\S+), serial=(\S+)", log)
    checks.append((bool(using) and using.group(2) == a.serial,
                   "OpenOCD drives the CMSIS-DAPv2 interface of %s" % a.serial,
                   using.group(0) if using else "no CMSIS-DAPv2 interface opened"))
    dpidr = re.search(r"SWD DPIDR (0x[0-9a-f]+)", log)
    checks.append((bool(dpidr), "the target answers on SWD", dpidr.group(0) if dpidr else "no DPIDR"))
    v = val("chip_id")
    checks.append((v is not None and num(v[0]) & 0x0fffffff == 0x00002927,
                   "SYSINFO_CHIP_ID is an RP2040 (part 2, manufacturer 0x927)", v))
    v = val("rom_magic")
    checks.append((v is not None and num(v[0]) & 0xffffff == 0x01754d,
                   "the bootrom magic reads 'Mu\\x01'", v))
    v = val("rom_reads_equal")
    checks.append((v == ["1", "words", "256"], "two 1 KB block reads of the bootrom agree", v))
    v = val("scratch")
    good = v is not None and len(v) == 6 and num(v[3]) == PATTERN and num(v[5]) == num(v[1])
    checks.append((good, "WATCHDOG_SCRATCH0 takes a write, reads it back, and is put back", v))
    v, w = val("halted"), val("resumed")
    checks.append((v == ["halted"] and w == ["running"], "core 0 halts and resumes", (v, w)))
    v = val("xip_read")
    checks.append((v is not None and v[1] == "16384", "a 64 KB read of the target's flash completes",
                   "%s ms" % v[3] if v else v))

    for good, what, detail in checks:
        print("%s %s: %s" % ("PASS" if good else "FAIL", what, detail))
        ok &= good
    if v:
        print("     64 KB in %s ms = %.0f KB/s at %d kHz SWCLK" %
              (v[3], 64000.0 / max(1, int(v[3])), a.speed))
    if not ok:
        print("---- openocd log (tail) ----")
        print(log[-3000:])
    print("PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
