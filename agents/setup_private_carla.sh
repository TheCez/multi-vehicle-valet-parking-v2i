#!/usr/bin/env bash
# Non-destructive private setup helper for overlap_obs_with_decision_maker.
set -euo pipefail

if [ "$#" -lt 1 ] || [ "$#" -gt 2 ]; then
    echo "Usage: $0 /private/path/carla_0.9.15_perfect_private.tar.gz [install-parent]" >&2
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
if ! command -v tar >/dev/null 2>&1; then
    echo "Install the 'tar' package first." >&2
    exit 69
fi
if ! command -v uv >/dev/null 2>&1; then
    echo "Install uv first: https://docs.astral.sh/uv/" >&2
    exit 69
fi
if [ -e "$CARLA_DIR" ]; then
    echo "Refusing to overwrite existing CARLA directory: $CARLA_DIR" >&2
    exit 73
fi

if tar -tzf "$ARCHIVE" | grep -Eq '(^/|(^|/)\.\.(/|$))'; then
    echo "Refusing an archive containing absolute or parent-directory paths." >&2
    exit 65
fi

top_level="$(tar -tzf "$ARCHIVE" | sed -n '1p' | cut -d/ -f1)"
if [ "$top_level" != "CARLA_0.9.15_perfect" ]; then
    echo "Archive must have CARLA_0.9.15_perfect as its top-level directory." >&2
    exit 65
fi
if ! tar -tzf "$ARCHIVE" | grep -qx 'CARLA_0.9.15_perfect/CarlaUE4.sh'; then
    echo "Archive does not contain CARLA_0.9.15_perfect/CarlaUE4.sh." >&2
    exit 65
fi

mkdir -p "$INSTALL_PARENT"
tar -xzf "$ARCHIVE" -C "$INSTALL_PARENT"
if [ ! -x "$CARLA_DIR/CarlaUE4.sh" ]; then
    echo "CARLA archive extracted but CarlaUE4.sh is not executable." >&2
    exit 65
fi

uv venv --python 3.10 "$VENV_DIR"
uv pip install --python "$VENV_DIR/bin/python" -r "$ROOT_DIR/requirements.txt"
uv pip install --python "$VENV_DIR/bin/python" "$WHEEL"

echo "Private CARLA installed at: $CARLA_DIR"
echo "Environment ready at: $VENV_DIR"
echo "Next: source .venv/bin/activate && $CARLA_DIR/CarlaUE4.sh"
