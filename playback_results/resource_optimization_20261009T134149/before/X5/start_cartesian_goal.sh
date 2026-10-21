#!/usr/bin/env bash
# Backward-compatible name for the left-arm Cartesian program.
exec "$(dirname "$(readlink -f "$0")")/start_left_cartesian.sh" "$@"
