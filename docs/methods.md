# BRS construction and execution

BRS plans motion for a truck entering a narrow loading pocket. The fixed map, vehicle geometry, start pose, goal pose, and terminal reverse straight define each task. Every local invocation uses the same three construction branches.

## Candidate construction

1. **Reverse-only branch.** Build the terminal straight outward from the goal. A task-aligned layered graph then connects its endpoint to the start. Edges use continuous-curvature steering. Lazy dynamic programming minimizes length in the finite graph. Refinement concentrates states around the coarse route; the complete coarse and fine routes both receive the common speed profile. Retain the route with smaller execution cost J, including the terminal straight.
2. **Goal-side branch.** Extend straight anchors from the goal and enumerate steering sign, angle, radius, and ramp length. Check each reverse-entry fan before constructing its forward connector from the start. Merge complete candidates and order them by forward length plus weighted reverse length. Profile the first three feasible candidates and retain the smallest J.
3. **Start-side branch.** Enumerate the forward fan from the start and connect each safe cusp pose to the goal approach. Use the same complete-candidate ordering, validation, shortlist size, and J selection.

Direct continuous-curvature connections to the terminal anchor supplement the reverse-only branch and the zero-fan limit of the goal-side branch. They use radius factors 1, 1.3, and 1.7, ramps 4 and 8 m, and the first three feasible curves per direction. These are the same direct-closure scales used by CC-A*. A reverse-only result remains an incumbent while both one-cusp families are evaluated at the first sampling level. Heading differences at most 135 degrees prioritize goal-side construction; larger differences prioritize start-side construction. If no candidate is feasible, broaden the sampling level. A local invocation returns the smallest J among its admitted feasible candidates.

## Recursive continuation

When all local constructions fail, expand prefixes with the four combinations of forward/reverse and left/right steering. Prefix lengths are 4, 8, and 12 m. Each prefix begins and ends with zero curvature. A collision-free endpoint becomes the start of another local invocation, retaining the original goal, map, and vehicle constraints.

The breadth-first queue stores complete prefixes, parent identities, depth, last gear, and cusp count. The configured limits are depth two, 128 tested states, three cusps, and a 900 s wall-clock safeguard. A successful child is concatenated with its full prefix and checked from the original start. Finish the current parent's prefix choices and return the smallest complete execution cost among its successes. State and candidate limits remain finite.

## Continuous-curvature geometry

Connectors use a turn, common tangent, and second turn. Each turn comprises a clothoid ramp, an optional circular portion, and a symmetric return ramp. Scalar closure solves the tangent heading. Special endpoint headings are retained exactly; proximity-based root deduplication preserves root values. Sharpness decreases continuously in the near-zero-angle range, allowing a vanishing turn to approach straight-line execution. Symmetry reduces displacement integration to one ramp and its conjugate, with an exact circular integral.

## Common trajectory evaluation

J = T_f + lambda_s N_s. The experiments set lambda_s = 10 s per extra internal stop. T_f includes moving time and physical stationary dwell. The additional stop term is an operational preference. Internal stops exclude the common initial/final rest states and count simultaneous shifting and steering adjustment as one event.

All methods use forward/reverse limits 4/2 m/s, acceleration/braking 0.5/0.7 m/s^2, lateral acceleration 1 m/s^2, steering rate 5 degrees/s, gear-change duration 2 s, and settling duration 0.5 s. The modeled shared stop lasts the maximum of gear and steering durations, plus settling. These are common simulation settings.

The physical truck is 10.40 by 4.90 m, with wheelbase 6.22 m and minimum turning radius 15 m. Planning adds 0.50 m on all four sides. Planning checks use 0.04 m arc-length spacing and a between-sample motion bound. Independent final checking uses 0.02 m spacing with separate collision arithmetic. Position and heading tolerances are 1e-5 m and 1e-7 rad. The terminal reverse straight is 12.54 m.

## Comparison implementations

HA implements Hybrid A* search with Reeds-Shepp closure. BIAGT implements static bidirectional improved A-search-guided trees. GHA-search implements the generalized Hybrid A* search component associated with C-MINLP. CC-A* combines conventional pose-bin A* exploration with the same continuous-curvature connector used by BRS. The common execution evaluator is applied to each valid geometric result. Navigation Toolbox supplies Dubins/Reeds-Shepp connections for the comparison planners; references and notices are retained in the repository.

Source defaults are defined in dp.options, cm.options, cw.defaults, base.options, and evalcw.options. plan_case and run_benchmark perform fresh planning from benchmark inputs. The main BRS regression is reproduced with run_paper_experiments(1:100,{'BRS'},'results/reproduced_brs').

## Experiment protocol

All five implementations use a 900 s wall-clock safeguard and their documented finite candidate/state limits. Checks occur between complete operations. HA and BIAGT return their first validated candidate. GHA-search evaluates up to 100 candidates. CC-A* evaluates the first successful closure batch. BRS evaluates its admitted reverse-only and one-cusp shortlists at the first feasible sampling level, or the successful siblings at a recursive parent. Every admitted complete result uses the common J evaluator.

Planning runtime includes geometry search and final time parameterization; it excludes file loading, serialization, and independent post-validation. The comparison-planner order rotates across tasks. MATLAB computational threads are limited to one, and numerical connector caches are cleared per query. Repetitions use three predefined cases and three runs per method. The environment report identifies the host and software.

A first admitted complete trajectory is recorded after a method's construction and screening operations. For BRS, admission of the reverse-only candidate follows its coarse/fine comparison. `first_final.json` quantifies the following candidate-selection work; the refinement records separately report all coarse/fine cost comparisons.

Quality and relative-runtime summaries use the same task subset on which all five methods succeed. Absolute planning-time means and success counts use all attempts. Best-tested counts include numerical ties. A separate planning-plus-predicted-execution measure describes a truck waiting for computation before moving.

The component queries keep the same curve representation, parameters, and checks. They retain Branch I plus only the goal-side family, retain Branch I plus only the start-side family, or return the first feasible candidate from each local invocation. The local-first configuration retains the same parent-level prefix evaluation. Root outcomes from the full solver provide the no-prefix comparison. The complete records include counts of generated direct/fan connections and paired costs. Layered-graph edge evaluations are recorded separately in each graph search result.
