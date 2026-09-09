#!/usr/bin/env bash
# Launch the two vehicle controllers that actually reroute on conflict
# (test2_car_copy_copy_test.py / test2_car2_copy_copy_test.py, paired with
# synchroniser/synchroniser6.py). Run after run_experiment_with_avoidance.sh
# or start_synchroniser_with_avoidance.sh has the master already running.

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

gnome-terminal --title="Valet parking vehicle 1 (avoidance)" -- bash -lc \
    "cd '$ROOT_DIR' && PYTHONPATH='$ROOT_DIR' '$PYTHON_BIN' test2_car_copy_copy_test.py --sync; exec bash"
gnome-terminal --title="Valet parking vehicle 2 (avoidance)" -- bash -lc \
    "cd '$ROOT_DIR' && PYTHONPATH='$ROOT_DIR' '$PYTHON_BIN' test2_car2_copy_copy_test.py --sync; exec bash"

echo "Launched two vehicle controllers with active conflict avoidance."
echo "Ensure synchroniser/synchroniser6.py is running first."
