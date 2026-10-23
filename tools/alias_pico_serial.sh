#!/usr/bin/env bash
set -euo pipefail

# The untouched upstream Blitz packer.py hardcodes /dev/ttyUSB0.
# Original Pico USB CDC devices often enumerate as /dev/ttyACM0.
DEVICE="${1:-/dev/ttyACM0}"
TARGET="/dev/ttyUSB0"

if [[ ! -e "$DEVICE" ]]; then
  echo "ERROR: source device does not exist: $DEVICE" >&2
  echo "Check: ls -l /dev/ttyACM* /dev/ttyUSB*" >&2
  exit 1
fi

SOURCE_REAL="$(readlink -f "$DEVICE")"
if [[ -e "$TARGET" && ! -L "$TARGET" ]]; then
  echo "ERROR: $TARGET already exists as a real device; refusing to overwrite it." >&2
  echo "The unchanged Blitz packer requires /dev/ttyUSB0. Resolve the port conflict manually." >&2
  exit 1
fi
if [[ -L "$TARGET" ]]; then
  TARGET_REAL="$(readlink -f "$TARGET" || true)"
  if [[ "$TARGET_REAL" == "$SOURCE_REAL" ]]; then
    echo "$TARGET already points to $SOURCE_REAL"
    exit 0
  fi
  echo "ERROR: $TARGET is already a symlink to ${TARGET_REAL:-a missing target}; refusing to replace it." >&2
  exit 1
fi

sudo ln -s "$SOURCE_REAL" "$TARGET"
echo "Created $TARGET -> $SOURCE_REAL"
echo "This alias is temporary and may need recreating after reconnect/reboot."
echo "Remove it only if you created it and it still points to the Pico: sudo rm $TARGET"
