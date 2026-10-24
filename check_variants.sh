#!/bin/bash
# check_variants.sh -- build every board profile from a fresh configure, the
# way README.md says, and check the default. Needs PICO_SDK_PATH.
#
#   0 flying leads, 1 carrier JTAG1, 2 carrier JTAG0, and no -DBOARD_TYPE at
#   all, which must build the same flash image as JTAG0 (JTAG1 on our carrier
#   has a hardware fault, so a plain build must not land on it).
#
# Build trees: src/build-check-<name> (ignored by git). Takes a few minutes.
set -u
: "${PICO_SDK_PATH:?set PICO_SDK_PATH to the pico-sdk checkout}"
cd "$(dirname "$0")/src" || exit 1

fail() { echo "FAIL: $*"; exit 1; }

# The UF2's family IDs (a UF2 block's familyID is the word at offset 28).
families() {
    python3 - "$1" <<'EOF'
import struct, sys
d = open(sys.argv[1], 'rb').read()
print(' '.join(sorted({'%08x' % struct.unpack_from('<I', d, i + 28)[0]
                       for i in range(0, len(d), 512)})))
EOF
}

build() {   # build NAME [cmake -D args...]
    local name=$1; shift
    local dir=build-check-$name
    mkdir -p "$dir"
    rm -f "$dir"/pico_usb_blaster.{elf,bin,uf2}
    cmake --fresh -S . -B "$dir" -DPICO_BOARD=pico "$@" > "$dir/check-cmake.log" 2>&1 \
        || fail "$name: cmake failed, see src/$dir/check-cmake.log"
    make -C "$dir" -j"$(nproc)" > "$dir/check-make.log" 2>&1 \
        || fail "$name: make failed, see src/$dir/check-make.log"
    local f
    for f in elf bin uf2; do
        [ -s "$dir/pico_usb_blaster.$f" ] || fail "$name: no pico_usb_blaster.$f"
    done
    [ "$(families "$dir/pico_usb_blaster.uf2")" = e48bff56 ] \
        || fail "$name: UF2 family $(families "$dir/pico_usb_blaster.uf2"), want e48bff56 (rp2040)"
    echo "built $name: $(stat -c %s "$dir/pico_usb_blaster.bin") bytes"
}

build flying  -DBOARD_TYPE=0
build jtag1   -DBOARD_TYPE=1
build jtag0   -DBOARD_TYPE=2
build default

bin() { echo "build-check-$1/pico_usb_blaster.bin"; }

# The profiles must differ, or the default matching JTAG0 proves nothing
cmp -s "$(bin flying)" "$(bin jtag1)" && fail "flying leads and JTAG1 built the same image"
cmp -s "$(bin flying)" "$(bin jtag0)" && fail "flying leads and JTAG0 built the same image"
cmp -s "$(bin jtag1)"  "$(bin jtag0)" && fail "JTAG1 and JTAG0 built the same image"

if ! cmp -s "$(bin default)" "$(bin jtag0)"; then
    for n in flying jtag1; do
        cmp -s "$(bin default)" "$(bin $n)" && fail "the default build is $n's image, not JTAG0's"
    done
    fail "the default build matches no profile's image"
fi
echo "PASS: every profile builds; the default is JTAG0's image"
