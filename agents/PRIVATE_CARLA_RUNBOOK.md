# Private CARLA live-run runbook

Use this runbook only for the private, personal reproduction of the `overlap_obs_with_decision_maker` experiment. Do not upload the CARLA ZIP, the custom-map package, or derived real-site information to a public repository, release, issue, artifact store, or external service.

## Required inputs

1. Clone this repository and switch to `overlap_obs_with_decision_maker`.
2. Obtain the private archive named `CARLA_0.9.15_perfect_private.tar.gz` from approved personal storage.
3. Put that archive **beside** the clone, not inside Git:

```text
workspace/
├── CARLA_0.9.15_perfect_private.tar.gz
└── multi-vehicle-valet-parking-v2i/
```

The archive must contain one top-level directory named `CARLA_0.9.15_perfect/`, including `CarlaUE4.sh`. This is the required compiled distribution for this branch: it matches `Town_Valet_Parking_final` and the bundled CPython 3.10 CARLA wheel.

## Agent procedure

1. Confirm the checkout is on `overlap_obs_with_decision_maker` and that no server is already listening on ports 2000, 2001, 5555, 5556, or 5557.
2. Confirm the archive is private and is **not** inside the Git checkout.
3. Run the setup helper. It validates the archive layout, unpacks the archive beside the repository when absent, and creates `.venv` with **uv**, Python 3.10, the matching dependencies, and the CARLA wheel. It never overwrites an existing incomplete CARLA directory; a valid prior extraction is reused so a failed dependency install can be resumed safely.

```bash
cd multi-vehicle-valet-parking-v2i
./agents/setup_private_carla.sh ../CARLA_0.9.15_perfect_private.tar.gz
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

7. Validate that CARLA uses the expected map and that the three ZeroMQ ports are listening:

```bash
ss -ltn '( sport = :5555 or sport = :5556 or sport = :5557 )'
```

## Failure handling

- If `load_world` cannot find `Town_Valet_Parking_final`, stop and confirm that the approved private archive—not a stock CARLA release—was used. Do not substitute a different map silently.
- If the CARLA wheel fails to import, use CPython 3.10 on x86_64 Linux and rerun the setup helper.
- If `run_experiment.sh` says CARLA is unavailable, wait until CARLA fully starts and retry; do not start the controllers before the synchronizer.
- The live run requires a graphical/accelerated CARLA environment. Keep the CARLA window open; headless rendering is not an accepted substitute for this run.

## Archive integrity reference

The expected SHA-256 of the bundled CPython 3.10 wheel is:

```text
651aabb7503db52f12bbd195efac85d6b6b055e5b061a7da0ee03f2edd0a105f
```

Use a private checksum manifest for the full 11 GB extracted distribution. Do not publish a manifest that exposes prohibited site-specific filenames or metadata.

## Required completion report from an agent

After a successful run, tell the user exactly how to repeat it: start the visible CARLA application, load `Town_Valet_Parking_final`, activate `.venv`, and run `./run_experiment.sh` from the repository root. Report the CARLA map name, whether all three ZeroMQ ports bound, and any failure without claiming that the simulation ran if it did not.
