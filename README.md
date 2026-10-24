# MiningPocket — BRS motion planning for autonomous mining trucks

Reproducibility artifact for **BRS: Branching and Recursive Sampling for Motion Planning of Autonomous Mining Trucks in Narrow Unstructured Loading Pockets**.

Authors: Bai Li, Fan Bu, Guizhen Yu, and Zhijiang Shao. Manuscript prepared for submission to IEEE Transactions on Vehicular Technology.

This repository contains the BRS MATLAB implementation, the fixed **100-case benchmark**, **all three comparison planners used in the experiments** (HA, BIAGT, and GHA-search), a common path-to-trajectory evaluator, and the 400 recorded planning results. Reviewers can inspect the code, rerun the planners, and independently verify the recorded trajectories and reported statistics.

## Quick start

MATLAB R2021b or later with Navigation Toolbox is required for the comparison planners. BRS uses the bundled MATLAB packages. Python 3 is sufficient for archive integrity and metric checks; no third-party Python package is needed.

Clone or download the repository and change MATLAB's current folder to it:

    miningpocket_setup
    r = plan_case(3,'BRS');      % fresh planning from Case #3
    view_case(3,'BRS')
    run_benchmark              % all four planners on all 100 cases

A full rerun can take substantial time. Each result is saved incrementally under results/reproduced/; recorded results are never used to seed a planner.

    addpath('tests'); smoke_release
    validate_recorded          % independently checks all 400 supplied solutions
    run_refinement_ablation    % same 100 cases, refinement enabled/disabled

From a shell:

    python tools/verify_artifact.py
    python tools/recompute_metrics.py

The archives are intentionally part of the repository. miningpocket_setup unpacks the 100 tasks; validate_recorded unpacks the recorded solutions. No external data service, private drive path, or original project folder is required. The archives contain MATLAB .mat files, not Git LFS pointers.

## Contents

| Location | Contents |
|---|---|
| src/ | MATLAB BRS, comparison planners, connectors, collision and timing code |
| benchmark/cases.zip | 100 fixed local-planar benchmark tasks |
| benchmark/manifest.json | Case IDs, map groups, and construction provenance |
| results/recorded/ | All 400 saved planning outputs, metrics, and ablation logs |
| experiment_settings.json | Common evaluation weight |
| qa/ | Recorded validation, environment, and SHA-256 fingerprints |
| tests/ | Portable entry-point and independent-validation smoke tests |
| docs/ | Task definition, parameters, method scope, and reproduction notes |

## Recorded results

All four implementations produced 100/100 valid results under the supplied independent checker. BRS has mean execution cost 77.8113 s with the fixed stop penalty, and the lowest tested cost in 99 tasks. Its mean per-task relative gap to the best tested result is 0.0613%. These results concern the curated benchmark and do not certify continuous global optimality. Historical wall-clock times characterize the stated MATLAB implementations and should not be interpreted as a comparison of equally optimized C++ programs.

See [benchmark definition](docs/benchmark.md) and [implementation/evaluation details](docs/methods.md). Internal label Ours in historical records denotes BRS. Published figures use Case #1--#100.

This repository contains research code. Third-party code notices are retained in THIRD_PARTY_LICENSE.txt. Consult the authors before redistribution or commercial reuse of material for which no separate licence is specified.
