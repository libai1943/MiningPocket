# Planner mechanisms

## Shared model and evaluator

All methods plan the rear-axle pose of the same kinematic truck against the fixed benchmark geometry. Each query starts and finishes stationary. The final approach is a zero-curvature reverse straight of at least the scene's prescribed length. The contour adds 0.5 m on all four sides of the physical 10.4 m by 4.9 m body. The common evaluation uses front/rear axle offsets 8.32/2.08 m, wheelbase 6.22 m, and minimum radius 15 m.

Curvature is piecewise linear in unsigned path distance. A continuous-curvature turn ramps from zero to its peak curvature, optionally follows a circle, and ramps to zero. Near zero heading change, sharpness decreases with the square root of heading-change magnitude; exact special roots are retained without rounding. Endpoint connectors solve a scalar tangent-heading closure and verify the integrated endpoint. A bounded Fresnel series evaluates small zero-initial-curvature ramps; general ramps use Gauss–Legendre quadrature. Straight and circular integrations are analytic.

Collision checking uses a spatial bounding-box hierarchy over occupied samples, early rejection on a coarse mesh, and the complete 0.04 m checking mesh. The between-sample motion bound is separate from the operational footprint margin. Exact-input collision results are memoized only within the current query. Final records receive a separate 0.02 m audit.

The common speed profile observes speed, acceleration, braking, lateral acceleration, and steering-rate limits. Same-direction curvature jumps require a stop and stationary steering; gear changes require a stop as well. Simultaneous gear and steering actions use the maximum of their durations plus settling. Each internal event counts once even when both actions occur. Boundary rest states do not count as additional internal stops.

## BRS

1. Reverse-only direct connectors are checked in length order. The first complete validated and time-parameterized result returns immediately.
2. If direct reverse closure fails, a task-aligned layered DAG searches reverse-only connectors. Unknown edges initially use Euclidean lower bounds; a topological shortest-route recurrence requests only the necessary connector generation and collision checks. A fully validated route returns immediately. Optional refinement has a 0.2 s budget and retains the coarse incumbent unless complete execution cost improves.
3. The heading difference orders the goal-side and start-side one-cusp constructions. Differences at most 135 degrees use II then III; larger differences use III then II. Both remain available. The solver broadens the designated fan radii and angular sampling only after the current level's constructions fail.
4. Safe goal-side fans grow outward from the terminal approach and are executed in reverse. Safe start-side fans grow forward from the initial pose. Unresolved fans enter a priority queue with an admissible completion-length bound. A popped fan generates its connectors; complete entries use their actual weighted forward/reverse length. Only popped complete candidates require full feasibility and execution evaluation.
5. The first complete candidate starts a bounded selection period inside that successful construction: at most 30 feasible candidates and 0.5 seconds after the first solution. The incumbent minimizes the complete execution cost among candidates actually evaluated. No further branch or sampling level is opened after success. All limits remain inside the same query deadline.
6. If the local constructions fail, breadth-first prefixes use forward/reverse and left/right steering at lengths 4, 8, and 12 m. Curvature ramps to and from its maximum. Depth is at most two, the tested-state cap is 128, and the total cusp cap is three. Each accepted child restarts local planning with the original goal and shared deadline. Candidates are validated and timed with their retained prefix from the original start. The successful child transfers this complete incumbent directly to its parent.

All state samples and candidate geometry are generated from the queried scene. The implementation does not consult recorded solutions, case labels, or a case-to-successful-branch lookup.

## Comparison strategies

- **HA**: continuous poses in 1 m / 5 degree / gear bins, 3 m circular primitives at five steering levels in both directions, obstacle-grid and analytic RS estimates, and an RS closure at the root and every ten expansions. Returns the first complete feasible trajectory.
- **BIAGT**: static two-tree search with prioritized primitive modes, shared cost information, distance rejection, and RS bridges. It uses the stated pose metric, common primitives, and nearest cross-tree bridge candidates. Returns the first complete feasible trajectory. Moving-obstacle scheduling from the full cited framework is outside this static task.
- **GHA-search**: generalized HA search front end with exchanged endpoint roles, zero heuristic, forward and reverse Dubins closures, a 150,000-node cap, and at most 100 complete feasible candidates under the common query deadline. It retains the minimum common J. Nonlinear optimization from C-MINLP is not part of this search-component comparison.
- **CC-A***: conventional pose-bin search using the same continuous-curvature turn and endpoint connector as BRS. Its state key includes gear and cusp count. Primitives use lengths 4, 8, and 12 m with five steering levels. Direct CC closures and RS-proposed cusp configurations provide terminal connections. It selects the lowest J in its first successful closure batch. This is a shared-representation comparison, not a claimed reproduction of a separately published complete planner.

Each implementation exposes its own stopping rule. All methods share the 10 s complete-query limit and executable-trajectory validity definition. Extended 500 s diagnostics use first-complete-solution return, are reported separately, and never replace main-comparison failures.

## Theoretical scope

Ordered finite-catalogue selection minimizes its geometric ordering cost when all smaller feasible candidates are excluded by completed checks or valid lower bounds. Lazy DAG search gives a minimum over its fixed graph and finite connector lists when the minimizing route is completely validated before budgets expire. Incumbent-preserving selection cannot worsen the execution cost of the candidates it actually compares. Prefix coverage requires exhaustive finite expansion, nonbinding time/state caps, and continuation-equivalent merging. These statements do not establish continuous-space global optimality or resolution completeness for the bounded implementation.
