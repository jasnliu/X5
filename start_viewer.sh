#!/usr/bin/env bash
# Backward-compatible name for the left-arm zone recorder.
exec "$(dirname "$(readlink -f "$0")")/start_left_zone.sh" "$@"
