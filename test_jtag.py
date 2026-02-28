#!/usr/bin/env python3
"""Test USB-Blaster pin-level verification"""
import usb.core, usb.util, sys, time

d = list(usb.core.find(idVendor=0x09fb, idProduct=0x6001, find_all=True))[-1]
try:
    d.detach_kernel_driver(0)
except:
    pass
d.set_configuration()
usb.util.claim_interface(d, 0)

def send_recv(cmds, expect_reads=0):
    d.write(0x02, bytes(cmds), timeout=1000)
    payload = bytearray()
    deadline = time.time() + 0.5
    while time.time() < deadline:
        try:
            r = bytes(d.read(0x81, 64, timeout=100))
        except:
            break
        if len(r) >= 2 and r[0] == 0x31:
            payload.extend(r[2:])
        if len(payload) >= expect_reads:
            break
    return payload

# Enable output and read TDO/DATAOUT
# Response byte: bit0 = TDO (GP5), bit1 = DATAOUT (GP8)

# Test: toggle each output pin and read back to verify connectivity
print("Pin test - reading TDO(GP5) with various output states:", flush=True)

# All low
r = send_recv([0x60], 1)  # OE+READ, all signals low
if r:
    print(f"  All low:  TDO={r[0]&1} DATAOUT={(r[0]>>1)&1}", flush=True)

# TCK high
r = send_recv([0x61], 1)  # OE+READ+TCK=1
if r:
    print(f"  TCK=1:    TDO={r[0]&1} DATAOUT={(r[0]>>1)&1}", flush=True)

# TMS high
r = send_recv([0x62], 1)
if r:
    print(f"  TMS=1:    TDO={r[0]&1} DATAOUT={(r[0]>>1)&1}", flush=True)

# TDI high
r = send_recv([0x70], 1)  # OE+READ+TDI=1
if r:
    print(f"  TDI=1:    TDO={r[0]&1} DATAOUT={(r[0]>>1)&1}", flush=True)

# Now do JTAG reset and shift mode IDCODE test
print("\nJTAG IDCODE via shift mode:", flush=True)

# Reset
cmds = []
for _ in range(5):
    cmds += [0x22, 0x23]
send_recv(cmds, 0)

# Navigate to Shift-DR
cmds = [0x20, 0x21, 0x22, 0x23, 0x20, 0x21, 0x20, 0x21]
send_recv(cmds, 0)

# Shift mode read 4 bytes: 0xC4 = shift_mode(0x80) + read(0x40) + count(4)
payload = send_recv([0xC4, 0x00, 0x00, 0x00, 0x00], 4)
if len(payload) >= 4:
    idcode = int.from_bytes(payload[:4], 'little')
    print(f"  Shift IDCODE: 0x{idcode:08x} raw: {payload[:4].hex()}", flush=True)
else:
    print(f"  Only got {len(payload)} bytes: {payload.hex()}", flush=True)

# Also try with more data in case there's an off-by-one
print("\nJTAG IDCODE via shift mode (8 bytes):", flush=True)
# Reset + nav again
cmds = []
for _ in range(5):
    cmds += [0x22, 0x23]
cmds += [0x20, 0x21, 0x22, 0x23, 0x20, 0x21, 0x20, 0x21]
send_recv(cmds, 0)

payload = send_recv([0xC8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], 8)
if len(payload) >= 8:
    idcode = int.from_bytes(payload[:4], 'little')
    print(f"  Bytes: {payload[:8].hex()}", flush=True)
    print(f"  IDCODE (first 4): 0x{idcode:08x}", flush=True)
    idcode2 = int.from_bytes(payload[4:8], 'little')
    print(f"  Next 4: 0x{idcode2:08x}", flush=True)
else:
    print(f"  Only got {len(payload)} bytes: {payload.hex()}", flush=True)

# Bitbang IDCODE for comparison
print("\nJTAG IDCODE via bitbang:", flush=True)
cmds = []
for _ in range(5):
    cmds += [0x22, 0x23]
cmds += [0x20, 0x21, 0x22, 0x23, 0x20, 0x21, 0x20, 0x21]
send_recv(cmds, 0)

cmds = []
for _ in range(32):
    cmds += [0x60, 0x21]
payload = send_recv(cmds, 32)
if len(payload) >= 32:
    bits = [(b & 1) for b in payload[:32]]
    idcode = sum(bits[i] << i for i in range(32))
    print(f"  Bits: {''.join(str(b) for b in bits)}", flush=True)
    print(f"  IDCODE: 0x{idcode:08x}", flush=True)

usb.util.release_interface(d, 0)
