#!/usr/bin/env bash
# Launch the overlap_obs_with_decision_maker experiment using the pairing
# that actually reroutes vehicles around a detected conflict, instead of
# only detecting and logging it:
#   synchroniser/synchroniser6.py
#   + test2_car_copy_copy_test.py / test2_car2_copy_copy_test.py
#
# synchroniser/synchroniser.py and automatic_control_main_path_planning_test*.py
# (used by run_experiment.sh) only ever detect conflicts - the line that would
# make a car brake or reroute is commented out there. This script wires up
# the pairing that actually acts on a detected conflict.
#
# CARLA 0.9.15 with Town_Valet_Parking_final must already be running on
# localhost:2000.

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT_DIR"

if ! command -v gnome-terminal >/dev/null 2>&1; then
    echo "gnome-terminal is required by this convenience launcher." >&2
    exit 1
fi

PYTHON_BIN="${PYTHON_BIN:-$ROOT_DIR/.venv/bin/python}"
if [ ! -x "$PYTHON_BIN" ]; then
    echo "Project interpreter not found: $PYTHON_BIN. Run agents/setup_private_carla.sh first." >&2
    exit 1
fi
if ! PYTHONPATH="$ROOT_DIR${PYTHONPATH:+:$PYTHONPATH}" "$PYTHON_BIN" - <<'PY'
import carla
client = carla.Client("127.0.0.1", 2000)
client.set_timeout(5.0)
client.get_world()
PY
then
    echo "CARLA is not ready on localhost:2000. Start CARLA and load Town_Valet_Parking_final first." >&2
    exit 1
fi

echo "Starting synchroniser6 (conflict resolution + rerouting) before vehicle controllers..."
gnome-terminal --title="Valet parking synchroniser (avoidance)" -- bash -lc \
    "cd '$ROOT_DIR' && PYTHONPATH='$ROOT_DIR' '$PYTHON_BIN' synchroniser/synchroniser6.py; exec bash"

# Give the master time to bind the ZeroMQ endpoints before Subscribers connect.
sleep 2
./launch_cars_with_avoidance.sh
