# Implementations and evaluation

- **BRS**: deterministic Branching and Recursive Sampling. cm.solve invokes dp.plan for the reverse-only, coarse-to-fine zero-cusp branch; cm.local prioritizes the goal-side and start-side one-cusp families. Prefix expansion invokes the same local solver at new configurations. The cc package implements continuous-curvature connectors; cw and s1 supply geometry checks.
- **HA**: Hybrid A* search with Reeds–Shepp connections. The implemented search component excludes nonlinear smoothing in Dolgov et al.'s complete framework.
- **BIAGT**: our static-scene implementation of bidirectional improved A-search guided trees. Dynamic-obstacle scheduling is outside this benchmark.
- **GHA-search**: generalized Hybrid A* search component related to C-MINLP, without subsequent continuous nonlinear optimization.

These are the authors' MATLAB implementations of the stated strategies, not unmodified upstream code. The source of each comparison method and adaptation scope must be retained when reporting results. Navigation Toolbox provides Dubins/Reeds–Shepp connections for the comparison planners. See THIRD_PARTY_LICENSE.txt for attribution of analytic Dubins equations and references.bib for exact bibliographic metadata.

## Fixed parameters

Source defaults are the authority: dp.options, cm.options, cw.defaults, base.options, and evalcw.options. No sensitivity sweep is part of this release. Common trajectory limits are forward/reverse speed 4/2 m/s, acceleration/braking 0.5/0.7 m/s², lateral acceleration 1 m/s², steering rate 5 degrees/s, gear-change dwell 2 s, and settling dwell 0.5 s. Steering and shifting overlap during a shared stop; the larger required time is used, plus settling. Start and finish are stationary. Extra internal stops exclude common initial/final rest states and count a shared steering/gear event once.

J = T_f + lambda_s N_s, with lambda_s = 10 s in the experiments. Actual dwell is already included in T_f; the extra term is an operational penalty, not a second estimate of physical dwell. The per-case best valid tested cost is the reference, not a certified global optimum.

## Reproducibility

results/recorded/ holds the fixed manuscript observations. plan_case and run_benchmark generate new outputs only under results/reproduced/. Fresh wall-clock times depend on hardware, warm-up, and toolbox versions; they are never substituted silently into historical measurements. Scene loading and independent validation lie outside the fresh planning timer. Total planning time adds common time-parameterization to geometric planning. validate_recorded uses the independent 0.02 m checker, including between-sample motion bounds, exact terminal-pose tolerances, and the final reverse-straight condition.

Finite sampling and resource budgets remain explicit. Successful results on the curated benchmark do not establish continuous-space completeness.
