# Archived thesis variants

Each local thesis folder was placed on its own branch to preserve experiments that were originally made in separate clones.

| Local folder | Archive branch | Strategy | CARLA map |
|---|---|---|---|
| `commonroad-scene` | `thesis/commonroad-scene` | Early scene-generation snapshot | `valet_parking` |
| `commonroad_generator3` | `thesis/original` | Original implementation | `Town_Valet_Parking` |
| `commonroad_generator2` | `thesis/all-car-reachability` | Reachability for all cars | `Town_Valet_Parking` |
| `commonroad_generator` | `thesis/overlap-obstacles` | Overlapping-obstacle strategy | `Town_Valet_Parking` |
| `commonroad_generator4` | `thesis/all-car-reachability-parking-lines` | All-car reachability with marked parking spaces | `Town_Valet_Parking_Lines` |
| `commonroad_generator5` | `thesis/overlap-obstacles-new-map` | Overlapping-obstacle strategy on final map | `Town_Valet_Parking_final` |
| `commonroad_generator6` | `thesis/simple-strategy-new-map` | Simple conflict strategy on final map | `Town_Valet_Parking_final` |
| `commonroad_generator7` | `thesis/all-car-reachability-new-map` | All-car reachability on final map | `Town_Valet_Parking_final` |
| `commonroad_baseline_config` | `thesis/baseline-config` | Baseline configuration experiments | `Town_Valet_Parking_final` |
| `commonroad_generator_baseline` | `thesis/baseline-final` | Final baseline | `Town_Valet_Parking_final` |

The showcase repository also contains the communication and standalone map-workspace snapshots:

- `thesis/valet-parking-workspace`
- `thesis/communication-2019a`
- `thesis/communication-development`

The branches intentionally remain separate. They capture different code, data, maps, and environment assumptions and are not a linear release sequence.
