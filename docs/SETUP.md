# Reproducing the thesis stack

This document records the environment used for the archived thesis variants. The project couples a CARLA server, two Python-controlled vehicles, CommonRoad scenario conversion, CommonRoad-Reach reachable sets, occupancy-grid conflict detection, Hybrid A* replanning, and ZeroMQ state exchange.

## Reproduced environment

- Linux Mint 21.3 (Ubuntu 22.04 base)
- Python 3.10.12
- CARLA source branch `ue4-dev`, commit `8e623cb41c1fdbde2f4b85118d2064de57d85384`
- CARLA version string `0.9.15-327-g8e623cb41`
- CARLA Unreal Engine fork 4.26, commit `73bcba55b`
- CommonRoad Scenario Designer 0.8.4
- CommonRoad Reach 2025.1.0
- CommonRoad Route Planner 2025.1.0

CARLA's build is large and hardware-sensitive. The official 0.9.15 guide estimates roughly 130 GB for CARLA plus Unreal Engine and recommends a dedicated GPU with at least 6 GB VRAM. Use the exact 0.9.15 documentation rather than current `latest` instructions.

## 1. Clone a thesis variant

Clone the repository and select one branch from `docs/BRANCHES.md`. Each branch is an independent snapshot; do not merge the experimental branches merely to install the project.

```bash
git clone https://github.com/TheCez/multi-vehicle-valet-parking-v2i.git
cd multi-vehicle-valet-parking-v2i
git switch thesis/original
```

Replace `thesis/original` with the desired branch.

## 2. Build CARLA and Unreal Engine

CARLA must be built from source to import or edit these RoadRunner maps reliably.

1. Clone the CARLA Unreal Engine 4.26 fork. Access to Epic's Unreal Engine source may require linking GitHub and Epic accounts.
2. Check out the recorded Unreal revision and build the editor.
3. Clone CARLA, check out the recorded CARLA revision, download its matching assets, and build the Python API and server.

```bash
git clone https://github.com/CarlaUnreal/UnrealEngine.git UnrealEngine_4.26
git -C UnrealEngine_4.26 checkout 73bcba55b

git clone https://github.com/carla-simulator/carla.git carla
git -C carla checkout 8e623cb41c1fdbde2f4b85118d2064de57d85384

export UE4_ROOT=/absolute/path/to/UnrealEngine_4.26
cd carla
./Update.sh
make PythonAPI
make launch
```

On Linux Mint, apply `runtime_patches/carla-0.9.15/linuxmint-setup.patch` before `make PythonAPI`. The patch adds Linux Mint to CARLA's supported Debian-family distributions. The exact Python 3.10 wheel built for this project is also backed up in `Carla_module/`.

The original workstation enabled MathWorks RoadRunner/Datasmith plugins in `Unreal/CarlaUE4/CarlaUE4.uproject`. Those proprietary plugin binaries are intentionally not redistributed. Install compatible RoadRunner plugins from MathWorks if you need to repeat the editor-based import; the provided source and cooked map assets remain archived in this branch.

## 3. Import the branch's map from source

The reproducible map source is under `carla_map/source/`. Copy the package directory into CARLA's `Import/` directory, then run CARLA's importer:

```bash
cp -a carla_map/source/. /absolute/path/to/carla/Import/
cd /absolute/path/to/carla
make import ARGS="--package=thesis_valet_parking --no-carla-materials"
```

CARLA requires the `.fbx` and `.xodr` files to share the same basename. The archived source folders preserve the RoadRunner textures, metadata, and geometry. If this branch contains cooked assets only, use the next section or regenerate the source package from the archived OpenDRIVE and original RoadRunner project.

## 4. Install the pre-cooked map

The `carla_map/Maps/` tree mirrors the CARLA destination. With the simulator stopped:

```bash
export CARLA_INSTALL=/absolute/path/to/CARLA_0.9.15
cp -a carla_map/Maps/. "$CARLA_INSTALL/CarlaUE4/Content/Carla/Maps/"
```

For a source checkout, the corresponding destination is `$CARLA_INSTALL/Unreal/CarlaUE4/Content/Carla/Maps/`. The `.umap`, `.uexp`, and `OpenDrive/*.xodr` files are a matched set and must remain together. If a standalone packaged build does not register a directly copied cooked map, import/package it through the CARLA source build instead.

## 5. Create the Python environment

```bash
python3.10 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
python -m pip install Carla_module/carla-0.9.15-cp310-cp310-linux_x86_64.whl
```

Branches that contain `runtime_patches/commonroad-scenario-designer-0.8.4/` need the archived CommonRoad change:

```bash
cp -a runtime_patches/commonroad-scenario-designer-0.8.4/crdesigner/. \
  .venv/lib/python3.10/site-packages/crdesigner/
```

This changes the Scenario Designer converter to import `compute_curvature_from_polyline` and `compute_pathlength_from_polyline` from `commonroad_clcs.util`, matching the installed 2025 CommonRoad packages.

## 6. Start and verify CARLA

Start CARLA in terminal 1:

```bash
"$CARLA_INSTALL/CarlaUE4.sh" -quality-level=Low
```

In terminal 2, activate the project environment and verify the server/map:

```bash
source .venv/bin/activate
python - <<'PY'
import carla
client = carla.Client("localhost", 2000)
client.set_timeout(20.0)
print(client.get_server_version())
print("\n".join(client.get_available_maps()))
PY
```

Load the map named in the branch README with CARLA's `PythonAPI/util/config.py --map <map-name>` or `client.load_world("<map-name>")`.

## 7. Run the two-vehicle experiment

From the branch root:

```bash
source .venv/bin/activate
chmod +x launch_cars.sh
./launch_cars.sh
```

`launch_cars.sh` opens the forward and reverse controllers in separate GNOME Terminal windows. Without GNOME Terminal, run these in separate shells:

```bash
python automatic_control_main_path_planning_test.py --sync
python automatic_control_main_path_planning_test_reverse.py --sync
```

The CARLA server must already be running and the matching map must be loaded. ZeroMQ ports used by the selected scripts must be free.

## Troubleshooting

- `ImportError: libcarla...`: use Python 3.10 x86_64 and install the supplied wheel inside the active environment.
- Map absent from `get_available_maps()`: rebuild/import through the source checkout; copying cooked files may not update a packaged build's asset registry.
- OpenDRIVE warning: verify that the `.xodr` basename matches the `.umap`/map name and is in `Maps/OpenDrive/`.
- CommonRoad curvature import error: apply the archived Scenario Designer patch after installing requirements.
- No visualization window: confirm an X11/Wayland display is available and that PyQt6, pygame, OpenCV, and Matplotlib installed successfully.
- Connection timeout: CARLA normally listens on TCP 2000/2001; start it before the clients and check that another server is not using the ports.

## What is and is not archived

Archived: thesis source, branch-specific experiment data, RoadRunner map exports, cooked CARLA map assets, the custom Python 3.10 CARLA wheel, precise source patches, logo, and demo video.

Not archived: full virtual environments, Python bytecode, the 100+ GB Unreal/CARLA build tree, CARLA base assets, or proprietary MathWorks RoadRunner plugins. Those are replaced by pinned revisions, checksums, and build instructions.
