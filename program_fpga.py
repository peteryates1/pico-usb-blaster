#!/usr/bin/env python3
"""Program Altera Cyclone IV FPGA via USB-Blaster (pico-usb-blaster)"""
import usb.core, usb.util, sys, time

P = lambda *a, **k: print(*a, **k, flush=True)

if len(sys.argv) < 2:
    P(f"Usage: {sys.argv[0]} <bitstream.rbf>")
    sys.exit(1)

rbf_path = sys.argv[1]

# EP4CE115 constants
IR_LEN = 10
IR_CONFIG = 0x002
IR_CHECK_STATUS = 0x004
IR_STARTUP = 0x003
IR_BYPASS = 0x3FF

# Open USB-Blaster (try each device, skip stale entries)
d = None
for dev in usb.core.find(idVendor=0x09fb, idProduct=0x6001, find_all=True):
    try:
        dev.detach_kernel_driver(0)
    except:
        pass
    try:
        dev.set_configuration()
        usb.util.claim_interface(dev, 0)
        # Test if device is actually responsive
        dev.read(0x81, 64, timeout=50)
    except usb.core.USBError as e:
        if e.errno == 19:  # No such device (stale entry)
            continue
        # Timeout is OK — means device is there but no data yet
    d = dev
    P(f"Opened USB-Blaster (dev {d.address})")
    break

if d is None:
    P("No USB-Blaster found")
    sys.exit(1)

def drain(n=3):
    """Read a few status packets to keep USB flowing"""
    for _ in range(n):
        try:
            d.read(0x81, 64, timeout=20)
        except:
            break

def bb_write(cmds):
    """Send bitbang commands"""
    d.write(0x02, bytes(cmds), timeout=1000)

def bb_read(cmds, expect):
    """Send bitbang commands with reads, return payload"""
    d.write(0x02, bytes(cmds), timeout=1000)
    payload = bytearray()
    deadline = time.time() + 1.0
    while time.time() < deadline and len(payload) < expect:
        try:
            r = bytes(d.read(0x81, 64, timeout=200))
        except:
            break
        if len(r) >= 2 and r[0] == 0x31:
            payload.extend(r[2:])
    return payload

def jtag_reset():
    cmds = []
    for _ in range(5):
        cmds += [0x22, 0x23]
    bb_write(cmds)
    drain()

def goto_rti():
    bb_write([0x20, 0x21])

def tck_rti(n):
    """Clock N TCK cycles in RTI"""
    while n > 0:
        chunk = min(30, n)
        cmds = []
        for _ in range(chunk):
            cmds += [0x20, 0x21]
        bb_write(cmds)
        n -= chunk
    drain()

def scan_ir(ir_val):
    # RTI -> Select-DR -> Select-IR -> Capture-IR -> Shift-IR
    cmds = [0x22, 0x23, 0x22, 0x23, 0x20, 0x21, 0x20, 0x21]
    for i in range(IR_LEN):
        tdi = 0x10 if (ir_val >> i) & 1 else 0x00
        tms = 0x02 if i == IR_LEN - 1 else 0x00
        cmds += [0x20 | tdi | tms, 0x21 | tdi | tms]
    # Exit1-IR -> Update-IR -> RTI
    cmds += [0x22, 0x23, 0x20, 0x21]
    bb_write(cmds)
    drain()

def enter_shift_dr():
    """RTI -> Shift-DR"""
    bb_write([0x22, 0x23, 0x20, 0x21, 0x20, 0x21])
    drain()

def exit_shift_dr():
    """Exit Shift-DR -> Update-DR -> RTI (assumes last bit already sent with TMS=1)"""
    bb_write([0x22, 0x23, 0x20, 0x21])
    drain()

# ---- Main ----

P(f"Loading {rbf_path}...")
with open(rbf_path, "rb") as f:
    rbf_jtag = f.read()
P(f"Bitstream: {len(rbf_jtag)} bytes")

P("JTAG reset...")
jtag_reset()
goto_rti()

P("IR: CONFIG (0x002)...")
scan_ir(IR_CONFIG)

P("6000 TCK in RTI...")
tck_rti(6000)

P(f"Shifting bitstream ({len(rbf_jtag)} bytes)...")
t0 = time.time()

enter_shift_dr()

# Build all shift commands upfront: each is [0x80|len, data...]
# Pack multiple shift commands into large USB writes for efficiency
total = len(rbf_jtag) - 1
CHUNK = 63  # max shift mode payload per command

# Pre-build the entire command stream
cmd_stream = bytearray()
pos = 0
while pos < total:
    n = min(CHUNK, total - pos)
    cmd_stream.append(0x80 | n)
    cmd_stream.extend(rbf_jtag[pos:pos+n])
    pos += n

# Send in large batches — larger batch = fewer syscalls, better USB scheduling
BATCH = 32768  # 32KB — best sustained throughput for USB Full Speed
sent = 0
total_cmds = len(cmd_stream)

while sent < total_cmds:
    n = min(BATCH, total_cmds - sent)
    d.write(0x02, bytes(cmd_stream[sent:sent+n]), timeout=5000)
    sent += n
    # Progress every ~64KB of command data
    if (sent // BATCH) % 16 == 0:
        data_sent = min(total, sent * total // total_cmds)
        pct = data_sent * 100 // len(rbf_jtag)
        elapsed = time.time() - t0
        speed = data_sent / elapsed / 1024 if elapsed > 0 else 0
        P(f"  {pct}% ({data_sent}/{len(rbf_jtag)}) {speed:.0f} KB/s")

drain()

# Last byte: bitbang with TMS=1 on last bit
last = rbf_jtag[-1]
cmds = []
for i in range(8):
    tdi = 0x10 if (last >> i) & 1 else 0x00
    tms = 0x02 if i == 7 else 0x00
    cmds += [0x20 | tdi | tms, 0x21 | tdi | tms]
bb_write(cmds)

exit_shift_dr()

t1 = time.time()
dt = t1 - t0
speed = len(rbf_jtag) / dt / 1024 if dt > 0 else 0
P(f"  Done: {dt:.1f}s ({speed:.0f} KB/s)")

P("IR: CHECK_STATUS...")
scan_ir(IR_CHECK_STATUS)
tck_rti(200)

P("IR: STARTUP...")
scan_ir(IR_STARTUP)
tck_rti(200)

P("IR: BYPASS...")
scan_ir(IR_BYPASS)

# Check CONF_DONE
r = bb_read([0x62], 1)
conf_done = (r[0] & 1) if r else -1
P(f"CONF_DONE: {conf_done}")

if conf_done == 1:
    P("FPGA programmed successfully!")
else:
    P("WARNING: CONF_DONE not asserted")

usb.util.release_interface(d, 0)
