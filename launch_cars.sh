#!/usr/bin/env bash
# Start the three local processes used by overlap_obs_with_decision_maker.
#
# Prerequisites:
#   1. Activate the project Python 3.10 environment.
#   2. Start CARLA 0.9.15 with Town_Valet_Parking_final on localhost:2000.
#   3. Run this script from the repository root.

set -euo pipefail

if ! command -v gnome-terminal >/dev/null 2>&1; then
    echo "gnome-terminal is required by this convenience launcher." >&2
    echo "Start synchroniser/synchroniser.py, then each controller manually." >&2
    exit 1
fi

PYTHON_BIN="${PYTHON_BIN:-python3}"
if ! command -v "$PYTHON_BIN" >/dev/null 2>&1; then
    echo "Python executable '$PYTHON_BIN' was not found." >&2
    exit 1
fi

if ! "$PYTHON_BIN" -c 'import carla, zmq' >/dev/null 2>&1; then
    echo "Activate the configured project environment before launching." >&2
    exit 1
fi

launch() {
    local title="$1"
    local command="$2"
    gnome-terminal --title="$title" -- bash -lc "$command; exec bash"
}

# The controllers import Subscriber from synchroniser/synchroniser.py, so use
# that module's Master rather than one of the archived synchroniser variants.
launch "Valet parking master" "$PYTHON_BIN synchroniser/synchroniser.py"
sleep 2
launch "Valet parking vehicle 1" "$PYTHON_BIN automatic_control_main_path_planning_test.py --sync"
launch "Valet parking vehicle 2" "$PYTHON_BIN automatic_control_main_path_planning_test_reverse.py --sync"

echo "Launched master and two vehicle controllers. Press Ctrl-C in each terminal to stop."
