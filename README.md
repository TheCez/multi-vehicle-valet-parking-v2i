# Multi-Vehicle Valet Parking with V2I Coordination

<p align="center">
  <img src="docs/media/master_thesis_logo.png" alt="Autonomous valet parking project logo" width="640">
</p>

<p align="center">
  <strong>CARLA · CommonRoad · Occupancy grids · Hybrid A* · ZeroMQ coordination</strong><br>
  A reproducible research showcase for resolving traffic conflicts in autonomous multi-vehicle valet parking.
</p>

<p align="center">
  <a href="docs/media/master_thesis_final_video.mp4">Watch the demonstration video</a> ·
  <a href="https://ajayc.dev/">Project portfolio</a> ·
  <a href="docs/SETUP.md">Reproduction guide</a> ·
  <a href="docs/V2I_COMMUNICATION.md">V2I coordination notes</a>
</p>

> **Showcase branch:** `overlap_obs_with_decision_maker` — the recommended, final-map snapshot of the thesis work.

## Demo

<video src="docs/media/master_thesis_final_video.mp4" controls muted playsinline width="100%">
  Your browser does not support embedded video. Use the demonstration-video link above.
</video>

The demo shows the stack operating in the custom valet-parking environment: live CARLA state is converted into planning data, vehicle occupancies are compared, conflicts are detected, and updated trajectories are distributed to the simulated vehicles.

## Why this project matters

Multi-vehicle valet parking is a compact but demanding coordination problem: agents share constrained space, their planned trajectories can conflict, and a solution has to be computed quickly enough to remain useful. This project combines a realistic simulator with CommonRoad-based planning tools and a central communication layer to explore that problem end-to-end.

| Contribution | What is implemented here |
|---|---|
| CARLA-to-planning bridge | Captures simulator state and converts it into CommonRoad scenarios. |
| Conflict prediction | Generates reachable sets and rasterized occupancy grids to identify spatiotemporal overlap. |
| Decision making | Applies the branch's overlapping-obstacle strategy and Hybrid A* replanning. |
| Multi-vehicle coordination | Uses a ZeroMQ synchronizer to collect state and distribute decisions. |
| Reproducible custom world | Archives cooked map assets, source map data, OpenDRIVE, textures, and the matching CARLA Python wheel. |

## Architecture

~~~text
CARLA 0.9.15 + Town_Valet_Parking_final
                 │
                 ▼
       scene capture / CommonRoad conversion
                 │
                 ▼
 reachable sets + occupancy-grid generation
                 │
                 ▼
  multi-vehicle conflict detection and decision maker
                 │
                 ▼
        Hybrid A* trajectory update
                 │
                 ▼
 ZeroMQ synchronizer ───────────────► CARLA vehicle controllers
~~~

## Quick start

This is a source-build CARLA research project, not a pip-only package. The exact map assets and CARLA wheel are included, but a compatible CARLA 0.9.15 build is required to load or modify the custom map.

~~~bash
git clone https://github.com/TheCez/multi-vehicle-valet-parking-v2i.git
cd multi-vehicle-valet-parking-v2i
git switch overlap_obs_with_decision_maker

python3.10 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
python -m pip install Carla_module/carla-0.9.15-cp310-cp310-linux_x86_64.whl
~~~

Then follow the map-installation and source-build steps in [docs/SETUP.md](docs/SETUP.md), start CARLA with `Town_Valet_Parking_final`, and launch the complete local stack (master plus two controllers):

~~~bash
./launch_cars.sh
~~~

## Project contents

| Path | Purpose |
|---|---|
| `synchroniser/` | Central multi-vehicle coordination, vehicle state handling, and decision logic. |
| `conflict_solver/` | Occupancy-grid conflict detection and resolution strategies. |
| `occupation_grid/` | Occupancy-grid construction and visualization. |
| `hybid_a_star_agent/` | Hybrid A* planning integration. |
| `carla_map/` | Cooked map files plus the RoadRunner FBX/OpenDRIVE source package. |
| `Carla_module/` | Archived CPython 3.10 CARLA 0.9.15 wheel matching this final-map setup. |
| `runtime_patches/` | Only the modified CARLA and CommonRoad runtime files; no non-portable virtual environment. |
| `docs/` | Setup, branch guide, communication design, media, and limitations. |

## Branches are experiments, not duplicates

The repository deliberately preserves the thesis work as independent branches rather than blending incompatible experiments. `overlap_obs_with_decision_maker` is the default showcase. See [docs/BRANCHES.md](docs/BRANCHES.md) for the map, strategy, and source-folder mapping for every archived variant.

## Reproducibility and limitations

- Pinned target: **CARLA 0.9.15**, **Python 3.10**, Linux x86_64; recorded CARLA source revision `8e623cb41`.
- The custom map can be copied as cooked assets, but editing/reimporting it requires the CARLA source build process. The included FBX, OpenDRIVE, textures, and import manifest are the authoritative archival inputs.
- The central ZeroMQ layer is a research-oriented V2I-style coordination mechanism, not a secured production V2X protocol. Its endpoints and operational limits are documented in [docs/V2I_COMMUNICATION.md](docs/V2I_COMMUNICATION.md).
- The repository intentionally excludes copied virtual environments and proprietary RoadRunner/Unreal plugins. It retains the specific modified library file and a patch instead.

## Thesis context

This repository archives the master's-thesis project **“Computation and Validation of Occupancy Grids for Solving Traffic Conflicts in Multi-vehicle Trajectory Planning.”** It is provided for technical review and reproducibility, not as a production autonomous-driving system.

Project identity, logo, and demonstration media are reproduced from the [project portfolio](https://ajayc.dev/).
