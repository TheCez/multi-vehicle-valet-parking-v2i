#!/usr/bin/env bash
# Non-destructive private setup helper for overlap_obs_with_decision_maker.
set -euo pipefail

if [ "$#" -lt 1 ] || [ "$#" -gt 2 ]; then
    echo "Usage: $0 /private/path/carla_0.9.15_perfect_private.zip [install-parent]" >&2
    exit 64
fi

ARCHIVE="$1"
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
INSTALL_PARENT="${2:-$(dirname "$ROOT_DIR")}"
CARLA_DIR="$INSTALL_PARENT/CARLA_0.9.15_perfect"
VENV_DIR="$ROOT_DIR/.venv"
WHEEL="$ROOT_DIR/Carla_module/carla-0.9.15-cp310-cp310-linux_x86_64.whl"

if [ ! -f "$ARCHIVE" ]; then
    echo "Private CARLA archive not found: $ARCHIVE" >&2
    exit 66
fi
if [ ! -f "$WHEEL" ]; then
    echo "Matching CARLA wheel is missing from the checkout: $WHEEL" >&2
    exit 66
fi
if ! command -v unzip >/dev/null 2>&1; then
    echo "Install the 'unzip' package first." >&2
    exit 69
fi
if [ -e "$CARLA_DIR" ]; then
    echo "Refusing to overwrite existing CARLA directory: $CARLA_DIR" >&2
    exit 73
fi

if unzip -Z1 "$ARCHIVE" | grep -Eq '(^/|(^|/)\.\.(/|$))'; then
    echo "Refusing an archive containing absolute or parent-directory paths." >&2
    exit 65
fi

top_level="$(unzip -Z1 "$ARCHIVE" | sed -n '1p' | cut -d/ -f1)"
if [ "$top_level" != "CARLA_0.9.15_perfect" ]; then
    echo "Archive must have CARLA_0.9.15_perfect as its top-level directory." >&2
    exit 65
fi
if ! unzip -Z1 "$ARCHIVE" | grep -qx 'CARLA_0.9.15_perfect/CarlaUE4.sh'; then
    echo "Archive does not contain CARLA_0.9.15_perfect/CarlaUE4.sh." >&2
    exit 65
fi

mkdir -p "$INSTALL_PARENT"
unzip -q "$ARCHIVE" -d "$INSTALL_PARENT"
if [ ! -x "$CARLA_DIR/CarlaUE4.sh" ]; then
    echo "CARLA archive extracted but CarlaUE4.sh is not executable." >&2
    exit 65
fi

python3.10 -m venv "$VENV_DIR"
"$VENV_DIR/bin/python" -m pip install --upgrade pip
"$VENV_DIR/bin/python" -m pip install -r "$ROOT_DIR/requirements.txt"
"$VENV_DIR/bin/python" -m pip install "$WHEEL"

echo "Private CARLA installed at: $CARLA_DIR"
echo "Environment ready at: $VENV_DIR"
echo "Next: source .venv/bin/activate && $CARLA_DIR/CarlaUE4.sh"
