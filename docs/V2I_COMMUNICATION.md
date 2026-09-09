# V2I-style multi-vehicle coordination

This project uses ZeroMQ to coordinate simulated vehicles through a central synchronizer. It is a research-oriented V2I-style coordination layer: the central process aggregates vehicle state and conflict information, then distributes decisions back to vehicle clients. It is not a production vehicle-to-infrastructure protocol implementation.

## Roles and transport

| Role | ZeroMQ pattern | Default endpoint | Purpose |
|---|---|---|---|
| Synchronizer to vehicles | XPUB / SUB | `tcp://127.0.0.1:5555` | Publishes shared state and conflict decisions. |
| Vehicle to synchronizer | REQ / REP | `tcp://127.0.0.1:5556` | Supports synchronous state/conflict requests. |
| Vehicle liveness | ROUTER / DEALER | `tcp://*:5557` | Heartbeat and reconnect handling. |
| CARLA simulator | CARLA RPC | `localhost:2000` | Simulator control and world state. |

The implementation records pending disconnects, receives vehicle state, builds occupancy grids, identifies spatiotemporal overlap, invokes the selected conflict solver, and broadcasts the decision/updated grid.

## Important limitations

- Endpoints are local by default; changing them to a network interface requires access control and transport security that this research code does not provide.
- The REQ/REP pattern is intentionally strict: every request needs one reply before the next request. Restart clients after a protocol mismatch.
- Only one synchronizer should bind the default ports at a time.
- CARLA must be available before the synchronizer starts because it creates a client connection during initialization.

## Practical test sequence

1. Start CARLA on port 2000 with the branch-specific map loaded.
2. Start the branch's synchronizer/master process.
3. Start the two controller scripts using `launch_cars.sh` or separate terminals.
4. Confirm that ports 5555, 5556, and 5557 are listening and that both clients register.
5. Trigger or replay a conflict scenario and inspect the occupancy-grid decision output.

The final-map decision-maker branch (`thesis/overlap-obstacles-new-map`) is the recommended showcase snapshot because it includes the refined synchronization and conflict-resolution work together with the final custom map.
