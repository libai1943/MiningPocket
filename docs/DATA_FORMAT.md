# Task and result definitions

Each `benchmark/case_NNN.json` contains the original planar metric task. All geometric numeric arrays were converted without changing their floating-point values. `manifest.json` records source and converted-file hashes, map groups, and task provenance. The benchmark combines mining-map-derived tasks with constructed environments and a documented corrected starting pose; it is not a claim that every task is a raw field measurement.

- `start`, `goal`: rear-axle-center `[x, y, heading]`, meters and radians. Start and finish are stationary with zero curvature.
- `boundary`: occupied map samples `[x, y]`. The implementation also restricts the planning footprint to their convex hull. This is the fixed discrete benchmark map definition.
- `parkedTruck`: zero or more poses of static parked bodies. Their interiors are filled at the common 0.25 m pitch.
- `truck`: front/rear overhang distances from the rear axle, body width, wheelbase, and minimum turning radius.
- `minimumStraight`: required zero-curvature terminal reverse straight length.
- `metadata`: task provenance where present. Map-group/provenance metadata also occur in the manifest and are not planner inputs.

The planning footprint adds the configured margin to all four sides of the physical rectangle. Sampling checks add the separate between-sample motion-cover allowance described in the paper. There is no online geographic-coordinate conversion.

A result path is an array of `[length, initial_curvature, final_curvature, gear]` pieces. Curvature varies linearly over each nonzero-length piece. Gear is +1 for forward and -1 for reverse. For unsigned path distance s, the rear-axle pose obeys `dx/ds = gear*cos(heading)`, `dy/ds = gear*sin(heading)`, and `dheading/ds = gear*curvature`. The scene start determines the first pose. Integration of the complete path must meet the exact common terminal tolerances.

`first_feasible_seconds` is null when no complete feasible trajectory was obtained. `planning_seconds` includes release of per-query structures and preparation of the return value; writing the JSON result is excluded. `stages` reports exclusive nested-profiler times and call counts. Successes contain `execution_seconds`, `moving_seconds`, `dwell_seconds`, `extra_stops`, `cusps`, and `J`. Cusps and additional stops are different: a same-direction curvature jump also requires an internal stop. The common start and terminal rest states do not count as additional internal stops. A combined shift and steering event is counted once.

`inspect` exports sampled positions, steering/velocity profiles, and event times for a saved successful result. `validate-results` is a finer-resolution numerical recheck of the same benchmark geometry; it is not a replacement for a continuous-world map-error model.
