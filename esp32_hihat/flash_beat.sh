#!/bin/bash
# flash_beat.sh -- compile and upload the motor_beat sketch to the classic ESP32.
#   ./flash_beat.sh       compile + upload
#   ./flash_beat.sh -m    compile + upload, then start the metronome host
#   ./flash_beat.sh -c    compile only
#   PORT=/dev/ttyUSB0 ./flash_beat.sh   override the serial port
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
export PATH="$HERE/bin:$PATH"

SKETCH="$HERE/motor_beat"
FQBN="esp32:esp32:esp32"      # classic ESP32 (two-motor beat)

if [ -z "${PORT:-}" ]; then
  PORT="$(ls /dev/ttyUSB* /dev/ttyACM* /dev/cu.usbserial* /dev/cu.usbmodem* 2>/dev/null | head -1 || true)"
fi
if [ -z "${PORT:-}" ]; then
  echo "No serial port found. Plug in the ESP32, or run: PORT=/dev/ttyUSB0 ./flash_beat.sh" >&2
  exit 1
fi

MONITOR=0
COMPILE_ONLY=0
while getopts "mc" opt; do
  case "$opt" in
    m) MONITOR=1 ;;
    c) COMPILE_ONLY=1 ;;
    *) echo "usage: ./flash_beat.sh [-m] [-c]" >&2; exit 2 ;;
  esac
done

echo ">> Compiling $SKETCH  (board: $FQBN)"
arduino-cli compile --fqbn "$FQBN" "$SKETCH"

if [ "$COMPILE_ONLY" -eq 1 ]; then
  echo ">> Compile OK (upload skipped)."
  exit 0
fi

echo ">> Uploading to $PORT"
arduino-cli upload --fqbn "$FQBN" --port "$PORT" "$SKETCH"
echo ">> Done."

if [ "$MONITOR" -eq 1 ]; then
  echo ">> Starting metronome host (Ctrl-C to stop; motor releases on exit)..."
  PY="$HERE/.venv/bin/python"
  [ -x "$PY" ] || PY="python3"
  exec "$PY" "$HERE/motor_beat.py" "$PORT"
fi
