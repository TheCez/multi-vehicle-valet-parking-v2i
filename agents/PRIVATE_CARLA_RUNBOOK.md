# Private CARLA live-run runbook

Use this runbook only for the private, personal reproduction of the `overlap_obs_with_decision_maker` experiment. Do not upload the CARLA ZIP, the custom-map package, or derived real-site information to a public repository, release, issue, artifact store, or external service.

## Required inputs

1. Clone this repository and switch to `overlap_obs_with_decision_maker`.
2. Obtain the private archive named `CARLA_0.9.15_perfect_private.tar.gz` from approved personal storage.
3. Put that archive in the clone root for a self-contained private workspace. It is locally excluded from Git; never add, commit, or upload it:

```text
multi-vehicle-valet-parking-v2i/
├── CARLA_0.9.15_perfect_private.tar.gz
├── agents/
└── ...
```

The archive must contain one top-level directory named `CARLA_0.9.15_perfect/`, including `CarlaUE4.sh`. This is the required compiled distribution for this branch: it matches `Town_Valet_Parking_final` and the bundled CPython 3.10 CARLA wheel.

## Agent procedure

1. Confirm the checkout is on `overlap_obs_with_decision_maker` and that no server is already listening on ports 2000, 2001, 5555, 5556, or 5557.
2. Confirm the archive is private and is **not** inside the Git checkout.
3. Run the setup helper. It validates the archive layout, unpacks the archive beside the repository when absent, and creates `.venv` with **uv**, Python 3.10, the matching CARLA wheel, and the thesis runtime baseline. It also installs the archived modified Scenario Designer converter from `runtime_patches/commonroad-scenario-designer-0.8.4/`. This deliberately reproduces the known working patched package combination instead of silently substituting a newer upstream library. It never overwrites an existing incomplete CARLA directory; a valid prior extraction is reused so a failed dependency install can be resumed safely.

```bash
cd multi-vehicle-valet-parking-v2i
./agents/setup_private_carla.sh ./CARLA_0.9.15_perfect_private.tar.gz
```

4. Activate the environment and start the private CARLA distribution in terminal 1 **with its graphical window visible**. Do not pass `-RenderOffScreen`, `-nullrhi`, or any headless option.

```bash
source .venv/bin/activate
../CARLA_0.9.15_perfect/CarlaUE4.sh
```

5. After CARLA is responsive on `localhost:2000`, load the final map in terminal 2:

```bash
source .venv/bin/activate
python - <<'PY'
import carla
client = carla.Client("127.0.0.1", 2000)
client.set_timeout(30.0)
world = client.load_world("Town_Valet_Parking_final")
print(world.get_map().name)
PY
```

6. Run the ordered launcher from the repository root. It first starts the matching `synchroniser/synchroniser.py` master, then starts the two vehicle controllers.

```bash
./run_experiment.sh
```

**Known limitation of this pipeline: no active collision avoidance.** `synchroniser.py` + `automatic_control_main_path_planning_test.py`/`_reverse.py` detect and log conflicts between the two vehicles, but the line that would make a vehicle brake or reroute on a detected conflict is commented out in every branch of this project (verified against both `overlap_obs_with_decision_maker` and `Working` on `TheCez/commonroad_path_planner`). Both cars always just keep following their independently pre-planned paths, so if those paths cross while both vehicles are present, they can physically collide. This is not a bug introduced by later fixes - it has always been the case on this pipeline. If a run needs to reproduce actual avoidance/rerouting behavior, use the alternate pipeline below instead.

### Alternate pipeline: with active conflict avoidance (rerouting)

`synchroniser/synchroniser6.py` + `test2_car_copy_copy_test.py` / `test2_car2_copy_copy_test.py` is a separate, more advanced pairing that genuinely reroutes a vehicle around a detected conflict (real hybrid-A* replanning through the conflict area, not just a brake). It is also what recorded `simulation_capture.mp4` in this repo. Run it the same way, from the repository root, once CARLA is up and the map is loaded:

```bash
./run_experiment_with_avoidance.sh
```

This does not touch `synchroniser.py` or the default launcher scripts - it is a fully separate entry point (`launch_cars_with_avoidance.sh` launches the two `test2_car*_copy_copy_test.py` controllers).

**Known limitation: conflict resolution is not fully reliable.** In repeated live verification (2026-09-09), this pairing ran cleanly for several minutes at a time, including at least one real conflict detected and successfully resolved with an actual reroute, with no crash on either side. But it has also been observed getting stuck: both vehicle-controller processes pegged at 100%+ CPU indefinitely, with `world.get_actors()` from a fresh client reporting zero actors and the CARLA world snapshot frame stuck at 0 - i.e. the simulation stops advancing - while the controllers' own logs kept printing as if ticks were still arriving. The root cause was not isolated (a `py-spy` stack dump could not be taken without elevated ptrace permissions in that environment). Do not assume a report of "it froze" or "the cars crashed" is a new regression on this pipeline specifically - reproduce it live before diagnosing further, the same way the earlier `synchroniser.py`-pipeline crashes in this file's history were confirmed.

7. Validate that CARLA uses the expected map and that the three ZeroMQ ports are listening:

```bash
ss -ltn '( sport = :5555 or sport = :5556 or sport = :5557 )'
```

## Failure handling

- If `load_world` cannot find `Town_Valet_Parking_final`, stop and confirm that the approved private archive—not a stock CARLA release—was used. Do not substitute a different map silently.
- If the CARLA wheel fails to import, use CPython 3.10 on x86_64 Linux and rerun the setup helper.
- If `run_experiment.sh` says CARLA is unavailable, wait until CARLA fully starts and retry; do not start the controllers before the synchronizer.
- The live run requires a graphical/accelerated CARLA environment. Keep the CARLA window open; headless rendering is not an accepted substitute for this run.
- If the synchronizer or a vehicle controller crashes on `run_experiment.sh`, reproduce it live with output captured to a file (e.g. `PYTHONPATH="$PWD" python -u synchroniser/synchroniser.py > sync.log 2>&1 &`, then the same for each controller) rather than guessing from a stack trace alone - every crash found in this pipeline so far (`AttributeError: 'Vehicle' object has no attribute 'world'` from passing `world.player` instead of `world` to `CommonRoadSceneGenerator`; unpacking bugs on `window.update_visualization()` and `generate_occupation_grid()`; a `synchroniser.py` deadlock from two subscribers racing a shared `REP` socket mid-handshake, fixed by switching to `ROUTER`; a Traffic Manager port collision from both controllers defaulting to `--tm-port 8002`, fixed by passing `8002`/`8003` explicitly in `launch_cars.sh`) was found and fixed this way, not by static reading alone.
- Each controller run leaves the CARLA world in synchronous mode and can leak vehicle/sensor actors if killed uncleanly. Before relaunching after a killed run, reset the world:
  ```python
  import carla
  c = carla.Client("127.0.0.1", 2000); c.set_timeout(10.0)
  w = c.get_world()
  for a in list(w.get_actors().filter('vehicle.*')) + list(w.get_actors().filter('sensor.*')):
      a.destroy()
  s = w.get_settings(); s.synchronous_mode = False; w.apply_settings(s)
  ```

## Archive integrity reference

The expected SHA-256 of the bundled CPython 3.10 wheel is:

```text
651aabb7503db52f12bbd195efac85d6b6b055e5b061a7da0ee03f2edd0a105f
```

Use a private checksum manifest for the full 11 GB extracted distribution. Do not publish a manifest that exposes prohibited site-specific filenames or metadata.

## Required completion report from an agent

After a successful run, tell the user exactly how to repeat it: start the visible CARLA application, load `Town_Valet_Parking_final`, activate `.venv`, and run `./run_experiment.sh` from the repository root (or `./run_experiment_with_avoidance.sh` for the pipeline with active conflict rerouting - see above). Report the CARLA map name, whether all three ZeroMQ ports bound, and any failure without claiming that the simulation ran if it did not.
