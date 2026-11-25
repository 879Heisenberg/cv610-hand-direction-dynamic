#!/usr/bin/env bash
set -Eeuo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SDK_ROOT="${SDK_ROOT:-/root/work/hi_aiot_solution/hi_aiot_solution}"
SDK_BUILD="${SDK_BUILD:-$SDK_ROOT/_build}"
APP_BUILD="${APP_BUILD:-$PROJECT_DIR/build-cv610-live}"
JOBS="${JOBS:-4}"
UBI_FS_SIZE="${UBI_FS_SIZE:-32M}"
OUTPUT_DIR="${OUTPUT_DIR:-$PROJECT_DIR/outputs/gui_hand_direction_display}"

APP="$APP_BUILD/cv610_live_hand"
SOUND_PLAYER="$APP_BUILD/cv610_gesture_sound"
MODEL="$PROJECT_DIR/model/hand_detect_static.om"
SOUND="$PROJECT_DIR/audio/gesture_effect.wav"
START_SCRIPT="$PROJECT_DIR/deploy/S99hand_direction"
NET_SCRIPT="${NET_SCRIPT:-$(cd "$PROJECT_DIR/../.." && pwd)/tools/cv610/S99zz-cv610-net}"
ROOTFS_TMP="$SDK_BUILD/rootfs/gui_rootfs_tmp"
IMAGE_DIR="$SDK_BUILD/rootfs/gui_image"
SDK_LIB_DIR="$SDK_ROOT/sdk/Hi3516CV610/soc/libs/musl"
SDK_SAMPLE_COMM_DIR="$SDK_ROOT/apps/cv610_ev_board/samples/capture_picture/comm"
DRM_ROOT="$SDK_BUILD/open_source/drm/release"
MKUBIIMG="$SDK_ROOT/sdk/Hi3516CV610/boards/dmeb/tools/pc/ubi_sh/mkubiimg.sh"
UBIFS_NAME="rootfs_hi3516cv610_2k_128k_${UBI_FS_SIZE}.ubifs"
BURN_TABLE="$IMAGE_DIR/nand_burn_table.xml"

die()
{
    echo "ERROR: $*" >&2
    exit 1
}

require_file()
{
    [ -f "$1" ] || die "missing file: $1"
}

echo "[1/6] Building the GUI board configuration"
bash "$SDK_ROOT/scripts/build.sh" build \
    --target cv610_ev_board samples gui/square_lcd_video_ui \
    --jobs "$JOBS" \
    "-DUBI_FS_SIZE=$UBI_FS_SIZE"

echo "[2/6] Building cv610_live_hand"
cmake -S "$PROJECT_DIR" -B "$APP_BUILD" \
    -DCMAKE_TOOLCHAIN_FILE="$SDK_ROOT/toolchain.cmake" \
    -DCMAKE_BUILD_TYPE=Release \
    -DHAND_DIRECTION_BUILD_TESTS=OFF \
    -DHAND_DIRECTION_ENABLE_CV610=ON \
    -DHAND_DIRECTION_SDK_ROOT="$SDK_ROOT/sdk/Hi3516CV610/soc" \
    -DHAND_DIRECTION_CV610_SAMPLE_COMM_DIR="$SDK_SAMPLE_COMM_DIR" \
    -DHAND_DIRECTION_DRM_ROOT="$DRM_ROOT"
cmake --build "$APP_BUILD" \
    --target cv610_live_hand cv610_gesture_sound \
    --parallel "$JOBS"

require_file "$APP"
require_file "$SOUND_PLAYER"
require_file "$MODEL"
require_file "$SOUND"
require_file "$START_SCRIPT"
require_file "$NET_SCRIPT"
require_file "$MKUBIIMG"
[ -d "$ROOTFS_TMP" ] || die "GUI rootfs was not created: $ROOTFS_TMP"
[ -d "$IMAGE_DIR" ] || die "GUI image skeleton was not created: $IMAGE_DIR"
[ -f "$BURN_TABLE" ] || die "burn table was not generated: $BURN_TABLE"
[ -d "$SDK_LIB_DIR" ] || die "SDK musl library directory not found: $SDK_LIB_DIR"

echo "[3/6] Injecting application into a fresh GUI rootfs"
cmake --build "$SDK_BUILD" --target rootfs_gui_image --parallel "$JOBS"

install -D -m 0755 "$APP" "$ROOTFS_TMP/root/hand_direction/bin/cv610_live_hand"
install -D -m 0755 "$SOUND_PLAYER" \
    "$ROOTFS_TMP/root/hand_direction/bin/cv610_gesture_sound"
install -D -m 0644 "$MODEL" "$ROOTFS_TMP/root/hand_direction/model/hand_detect_static.om"
install -D -m 0644 "$SOUND" "$ROOTFS_TMP/root/hand_direction/audio/gesture_effect.wav"
for dir_wav in gesture_up.wav gesture_down.wav gesture_left.wav gesture_right.wav; do
    if [ -f "$PROJECT_DIR/audio/$dir_wav" ]; then
        install -D -m 0644 "$PROJECT_DIR/audio/$dir_wav" \
            "$ROOTFS_TMP/root/hand_direction/audio/$dir_wav"
    fi
done
install -D -m 0755 "$START_SCRIPT" "$ROOTFS_TMP/etc/init.d/S99hand_direction"
install -D -m 0755 "$NET_SCRIPT" "$ROOTFS_TMP/etc/init.d/S99zz-cv610-net"

# The vendor GUI sample owns the same DRM screen, so it must not autostart.
rm -f "$ROOTFS_TMP/etc/init.d/S99video_ui"

sh -n "$ROOTFS_TMP/etc/init.d/S99hand_direction"
sh -n "$ROOTFS_TMP/etc/init.d/S99zz-cv610-net"
file "$ROOTFS_TMP/root/hand_direction/bin/cv610_live_hand" | grep -q 'ELF 32-bit' || \
    die "deployed executable is not a 32-bit ARM ELF"
file "$ROOTFS_TMP/root/hand_direction/bin/cv610_gesture_sound" | grep -q 'ELF 32-bit' || \
    die "deployed sound player is not a 32-bit ARM ELF"

rootfs_kib="$(du -sk "$ROOTFS_TMP" | awk '{print $1}')"
limit_kib=$((32 * 1024))
[ "$rootfs_kib" -lt "$limit_kib" ] || die "rootfs is ${rootfs_kib} KiB and does not fit ${UBI_FS_SIZE}"

echo "[4/6] Creating ${UBIFS_NAME}"
rm -f "$IMAGE_DIR/$UBIFS_NAME"

if grep -q '^SDK_TYPE:STRING=hisupport$' "$SDK_BUILD/CMakeCache.txt"; then
    UBI_TOOL_DIR="$SDK_ROOT/sdk/Hi3516CV610/open_source/mtd-utils/out/pc"
else
    UBI_TOOL_DIR=""
fi

(
    cd "$IMAGE_DIR"
    # mkubiimg.sh exits 1 after successful cleanup when the optional keep flag is 0.
    "$MKUBIIMG" hi3516cv610 2k 128k "$ROOTFS_TMP" "$UBI_FS_SIZE" "$UBI_TOOL_DIR" 0 || true
)
require_file "$IMAGE_DIR/$UBIFS_NAME"

selected_rootfs="$(sed -nE 's/.*PartitionName="rootfs"[^>]*SelectFile="([^"]+)".*/\1/p' "$BURN_TABLE")"
[ "$selected_rootfs" = "$UBIFS_NAME" ] || \
    die "burn table selects '$selected_rootfs', expected '$UBIFS_NAME'"

echo "[5/6] Publishing to $OUTPUT_DIR"
mkdir -p "$OUTPUT_DIR"
cp -af "$IMAGE_DIR"/. "$OUTPUT_DIR"/
sha256sum "$OUTPUT_DIR/$UBIFS_NAME" "$OUTPUT_DIR/uImage" "$OUTPUT_DIR/nand_burn_table.xml" \
    >"$OUTPUT_DIR/SHA256SUMS"

echo "[6/6] Validation"
test -x "$ROOTFS_TMP/root/hand_direction/bin/cv610_live_hand"
test -x "$ROOTFS_TMP/root/hand_direction/bin/cv610_gesture_sound"
test -f "$ROOTFS_TMP/root/hand_direction/model/hand_detect_static.om"
test -f "$ROOTFS_TMP/root/hand_direction/audio/gesture_effect.wav"
test -x "$ROOTFS_TMP/etc/init.d/S99hand_direction"
test -x "$ROOTFS_TMP/etc/init.d/S99zz-cv610-net"
test ! -e "$ROOTFS_TMP/etc/init.d/S99video_ui"

echo "Image ready: $OUTPUT_DIR"
echo "Burn table:  $OUTPUT_DIR/nand_burn_table.xml"
ls -lh "$OUTPUT_DIR/$UBIFS_NAME" "$OUTPUT_DIR/nand_burn_table.xml"
