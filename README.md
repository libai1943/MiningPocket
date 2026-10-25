# MiningPocket: branching and recursive motion planning

Reproducibility artifact for **BRS: Branching and Recursive Sampling for Motion Planning of Autonomous Mining Trucks in Narrow Unstructured Loading Pockets**.

This repository supplies BRS, four comparison planners (HA, BIAGT, GHA-search, and CC-A*), the fixed 100-task benchmark, twelve independent synthetic pockets, and a common trajectory evaluator. It includes 500 main comparisons, 60 independent-geometry queries, 300 component queries, and 45 timing repetitions. Every query has an outcome and measured runtime. Main-comparison and independent-geometry trajectories have independent validation records.

## Quick start

MATLAB R2021b or later with Navigation Toolbox supports all five planners. BRS uses the bundled MATLAB packages.

```matlab
miningpocket_setup
r = plan_case(3,'BRS');
view_case(3,'BRS')
run_benchmark                         % fresh 100-task comparison
validate_recorded                     % independently audit supplied paths
addpath('tests'); test_numerics
```

Python 3 can verify the archives and recalculate the metrics without additional packages:

```text
python tools/verify_artifact.py
python tools/recompute_metrics.py
```

`run_paper_experiments` saves individual queries under `results/validated/` and resumes completed queries. `run_holdout_experiments`, `run_component_experiments`, and `run_timing_repeats` reproduce the additional studies. The component runner also exports numerical paper data and checks the source polygons of the independent tasks. Run the studies sequentially when collecting runtimes.

To regenerate the complete paper study, run the following sequence. The final command also performs the timing repetitions, exports the plot inputs, and validates the independent source geometry.

```matlab
run_paper_experiments
run_holdout_experiments
run_component_experiments
```

## Task and execution model

Coordinates are local planar metres. The physical truck is 10.4 by 4.9 m, with wheelbase 6.22 m and minimum turning radius 15 m. Planning adds a 0.5 m buffer on all four sides. All planners share endpoint tolerances, a compulsory final reverse straight, collision rules, and time parameterization. The fixed evaluation is `J = T_f + 10 N_s`; actual stationary dwell is included in `T_f`, and `N_s` counts additional internal stops.

The main benchmark contains mining-map-derived and constructed tasks used in method development. The twelve synthetic pockets vary environmental geometry with fixed parameters and retain every generated valid task. See [benchmark definition](docs/benchmark.md) and [method and evaluation details](docs/methods.md).

## Recorded results

BRS solves 100/100 fixed tasks under independent checking. The common-success comparison across five methods contains 100 tasks. On that shared set, BRS has mean model-predicted execution time 65.61 s, 0.48 additional stops, and mean relative cost gap 1.34%. It obtains the lowest tested cost in 94 of the 100 main tasks, including ties. The supplied metric script derives the complete comparison from individual query records. `python tools/plot_paper_figures.py` recreates the numerical figures from the released data using NumPy, Matplotlib, and an installed Times New Roman font. The shared palette is defined in `tools/palette.py`.

Planning times are sequential MATLAB wall-clock measurements on the platform described in `qa/experiment_environment.json`. Execution times are predictions of the common vehicle model. The manuscript states the finite candidate and graph properties supported by the algorithm; the benchmark results characterize the supplied tasks.

## Files

- `src/`: planners, shared connectors, collision tests, and trajectory evaluator.
- `benchmark/`: 100 fixed tasks, twelve independent pockets, and provenance.
- `results/recorded/`: main query records and compressed MATLAB trajectories.
- `results/holdout/`, `results/components/`, `results/timing/`: additional studies.
- `results/paper_data/`: numerical inputs for the manuscript figures.
- `qa/`: source, input, and result fingerprints and validation reports.
- `tests/`: portable planning and numerical checks.

Archives contain actual MATLAB files; no Git LFS or private data service is needed. Third-party notices appear in `THIRD_PARTY_LICENSE.txt`. Consult the authors before redistribution or commercial reuse of material without a separate licence.
