#!/usr/bin/env bash
# Launch the two vehicle controllers after the synchronizer is already running.

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT_DIR"

if ! command -v gnome-terminal >/dev/null 2>&1; then
    echo "gnome-terminal is required by this convenience launcher." >&2
    echo "Run both controller commands manually in separate terminals." >&2
    exit 1
fi

PYTHON_BIN="${PYTHON_BIN:-$ROOT_DIR/.venv/bin/python}"
if [ ! -x "$PYTHON_BIN" ]; then
    echo "Project interpreter not found: $PYTHON_BIN. Run agents/setup_private_carla.sh first." >&2
    exit 1
fi
if ! PYTHONPATH="$ROOT_DIR${PYTHONPATH:+:$PYTHONPATH}" "$PYTHON_BIN" -c 'import carla, zmq' >/dev/null 2>&1; then
    echo "The configured project environment is incomplete." >&2
    exit 1
fi

gnome-terminal --title="Valet parking vehicle 1" -- bash -lc \
    "cd '$ROOT_DIR' && PYTHONPATH='$ROOT_DIR' '$PYTHON_BIN' automatic_control_main_path_planning_test.py --sync --tm-port 8002; exec bash"
gnome-terminal --title="Valet parking vehicle 2" -- bash -lc \
    "cd '$ROOT_DIR' && PYTHONPATH='$ROOT_DIR' '$PYTHON_BIN' automatic_control_main_path_planning_test_reverse.py --sync --tm-port 8003; exec bash"

echo "Launched two vehicle controllers. Ensure synchroniser/synchroniser.py is running first."
