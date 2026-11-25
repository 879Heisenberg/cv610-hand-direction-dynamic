#!/bin/sh
set -eu

ROOT=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)

exec "$ROOT/bin/cv610_smoke" \
    "$ROOT/model/hand_detect_static.om" \
    "$ROOT/frames/palm_640x360_nv21.raw" \
    640 360 640 nv21
