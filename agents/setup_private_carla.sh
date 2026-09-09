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
CRDESIGNER_PATCH="$ROOT_DIR/runtime_patches/commonroad-scenario-designer-0.8.4/crdesigner"

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
    if [ ! -x "$CARLA_DIR/CarlaUE4.sh" ]; then
        echo "Existing CARLA directory is incomplete; refusing to overwrite: $CARLA_DIR" >&2
        exit 73
    fi
    echo "Using existing private CARLA directory: $CARLA_DIR"
else
    ARCHIVE_LISTING="$(mktemp)"
    trap 'unlink "$ARCHIVE_LISTING" 2>/dev/null || true' EXIT
    tar -tzf "$ARCHIVE" > "$ARCHIVE_LISTING"

    if grep -Eq '(^/|(^|/)\.\.(/|$))' "$ARCHIVE_LISTING"; then
        echo "Refusing an archive containing absolute or parent-directory paths." >&2
        exit 65
    fi

    top_level="$(sed -n '1p' "$ARCHIVE_LISTING" | cut -d/ -f1)"
    if [ "$top_level" != "CARLA_0.9.15_perfect" ]; then
        echo "Archive must have CARLA_0.9.15_perfect as its top-level directory." >&2
        exit 65
    fi
    if ! grep -qx 'CARLA_0.9.15_perfect/CarlaUE4.sh' "$ARCHIVE_LISTING"; then
        echo "Archive does not contain CARLA_0.9.15_perfect/CarlaUE4.sh." >&2
        exit 65
    fi

    mkdir -p "$INSTALL_PARENT"
    tar -xzf "$ARCHIVE" -C "$INSTALL_PARENT"
    if [ ! -x "$CARLA_DIR/CarlaUE4.sh" ]; then
        echo "CARLA archive extracted but CarlaUE4.sh is not executable." >&2
        exit 65
    fi
fi

uv venv --python 3.10 "$VENV_DIR"
# requirements.txt records the known working thesis environment. Its Scenario
# Designer is intentionally patched below and has legacy metadata that conflicts
# with the newer CommonRoad packages, so preserve this tested combination rather
# than allowing a resolver to replace it with an incompatible upstream release.
uv pip install --no-deps --python "$VENV_DIR/bin/python" -r "$ROOT_DIR/requirements.txt"
uv pip install --python "$VENV_DIR/bin/python" "$WHEEL"

if [ ! -f "$CRDESIGNER_PATCH/map_conversion/opendrive/cr2odr/elements/road.py" ]; then
    echo "Modified Scenario Designer converter is missing: $CRDESIGNER_PATCH" >&2
    exit 66
fi
SITE_PACKAGES="$("$VENV_DIR/bin/python" -c 'import site; print(site.getsitepackages()[0])')"
rsync -a "$CRDESIGNER_PATCH/" "$SITE_PACKAGES/crdesigner/"
"$VENV_DIR/bin/python" - <<'PY'
import carla
import zmq
from crdesigner.map_conversion.opendrive.cr2odr.elements.road import Road
print("Verified CARLA, ZeroMQ, and patched Scenario Designer:", Road.__module__)
PY

echo "Private CARLA installed at: $CARLA_DIR"
echo "Environment ready at: $VENV_DIR"
echo "Next: source .venv/bin/activate && $CARLA_DIR/CarlaUE4.sh"
