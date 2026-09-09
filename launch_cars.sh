#!/usr/bin/env bash
# Launch the two vehicle controllers after the synchronizer is already running.

set -euo pipefail

if ! command -v gnome-terminal >/dev/null 2>&1; then
    echo "gnome-terminal is required by this convenience launcher." >&2
    echo "Run both controller commands manually in separate terminals." >&2
    exit 1
fi

PYTHON_BIN="${PYTHON_BIN:-python3}"
if ! "$PYTHON_BIN" -c 'import carla, zmq' >/dev/null 2>&1; then
    echo "Activate the configured project environment before launching." >&2
    exit 1
fi

gnome-terminal --title="Valet parking vehicle 1" -- bash -lc \
    "$PYTHON_BIN automatic_control_main_path_planning_test.py --sync; exec bash"
gnome-terminal --title="Valet parking vehicle 2" -- bash -lc \
    "$PYTHON_BIN automatic_control_main_path_planning_test_reverse.py --sync; exec bash"

echo "Launched two vehicle controllers. Ensure synchroniser/synchroniser.py is running first."
