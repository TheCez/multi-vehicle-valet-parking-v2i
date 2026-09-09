#!/usr/bin/env bash
# Launch the working overlap_obs_with_decision_maker experiment in its required order.
# CARLA 0.9.15 with Town_Valet_Parking_final must already be running on localhost:2000.

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT_DIR"

if ! command -v gnome-terminal >/dev/null 2>&1; then
    echo "gnome-terminal is required by this convenience launcher." >&2
    exit 1
fi

PYTHON_BIN="${PYTHON_BIN:-python3}"
if ! "$PYTHON_BIN" - <<'PY'
import carla
client = carla.Client("127.0.0.1", 2000)
client.set_timeout(5.0)
client.get_world()
PY
then
    echo "CARLA is not ready on localhost:2000. Start CARLA and load Town_Valet_Parking_final first." >&2
    exit 1
fi

echo "Starting synchronizer before vehicle controllers..."
gnome-terminal --title="Valet parking synchronizer" -- bash -lc \
    "$PYTHON_BIN synchroniser/synchroniser.py; exec bash"

# Give the master time to bind the ZeroMQ endpoints before Subscribers connect.
sleep 2
./launch_cars.sh
