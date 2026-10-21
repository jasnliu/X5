#!/usr/bin/env bash
# Exact pre-left-recording runtime; leave the current start_beat.sh untouched.
set -e
ROOT="$(dirname "$(readlink -f "$0")")"
exec "$ROOT/beatTest_snapshot/X5/start_beat.sh" "$@"
