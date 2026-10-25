# MiningPocket

Native C++ implementations and the 100-task benchmark for **BRS: Branching and Recursive Sampling for Motion Planning of Autonomous Mining Trucks in Narrow Unstructured Loading Pockets**.

The repository contains BRS, Hybrid A* (HA), the static BIAGT search, the GHA search front end of C-MINLP, and a continuous-curvature A* comparison (CC-A*). Every planner uses the same vehicle, fixed scenes, collision checker, terminal reverse approach, speed parameterization, and execution-cost evaluator. The planner executable has no MATLAB, MATLAB Engine, MATLAB Runtime, or Navigation Toolbox dependency.

## Build

A C++17 compiler and CMake 3.18 or later are sufficient. JSON support and the analytic Reeds–Shepp implementation are included. The build unpacks `benchmark/cases.zip` once into 100 JSON scene files, preserving their recorded byte hashes.

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

The executable is `build/miningpocket` on a single-configuration Unix build and usually `build/Release/miningpocket.exe` with Visual Studio. The Windows measurement build uses LLVM-MinGW Clang, `-std=c++17 -O3 -DNDEBUG -static`, and one computational thread. `tools/build.ps1 -Compiler <path-to-clang++.exe>` builds the same single executable directly.

## Run one task or all 100

Run from the repository root, replacing the executable path for your build:

```sh
build/miningpocket run . --case 30 --method BRS --out case30.jsonl
build/miningpocket run . --all --method BRS --out brs100.jsonl
build/miningpocket run . --case 3 --method "CC-A*"
```

Method names are `BRS`, `HA`, `BIAGT`, `GHA`, and `CC-A*`. The parameters in `benchmark/parameters.json` apply to every query. Cases #1–#100 are the complete benchmark; no subset is needed.

The default query limit is 10 seconds. It covers map indexing, candidate construction, search, collision checks, time parameterization, and return preparation. Input file loading and JSON serialization are outside that interval. A shared deadline with a 0.2-second return reserve interrupts long operations. A completed incumbent is retained when further bounded selection is interrupted. Calls returned after the deadline are failures.

Branch I returns its first complete feasible reverse-only trajectory. A successful one-cusp construction can evaluate up to 30 feasible candidates within 0.5 seconds after its first complete solution. It retains the lowest complete-trajectory cost among those candidates and returns without opening another branch. Prefix invocations retain the original start for complete-path validation and scoring. `--first-only` disables subsequent candidate improvement for first-solution diagnostics. `--refine` enables the optional bounded graph-refinement component; the default leaves Branch I immediate-return.

## Experiments and validation

`tools/run_study.py` runs methods sequentially, rotating their order over the 100 tasks. It requires Python 3 and `psutil` for orchestration and process monitoring only. Planning and evaluation remain native C++.

```sh
python tools/run_study.py comparison --exe build/miningpocket --cpu 2
python tools/run_study.py components --exe build/miningpocket --cpu 2
python tools/run_study.py selection --exe build/miningpocket --cpu 2
python tools/run_diagnostics.py --exe build/miningpocket --cpu 2
build/miningpocket validate-results . --out results/comparison100.jsonl
build/miningpocket parity . --out parity.json
build/miningpocket selftest .
```

Choose a logical CPU that belongs to a performance core on the measurement machine. All planners must use the same affinity, optimized build, thread setting, and background conditions. Avoid concurrent experiments, compilation, or plotting while collecting times. Omitting `--cpu` preserves the operating system's affinity setting. Hardware, compiler/build metadata, source hashes, executable hash, and sampled process load accompany each study.

The main comparison always uses the 10-second deadline. `run_diagnostics.py` reruns only task–method pairs that failed the main comparison, with a 500-second limit and immediate return after the first complete feasible trajectory. These diagnostics never change the main success counts. A timeout is retained as a failure.

`validate-results` rechecks successful trajectories at 0.02 m spacing and recomputes endpoint, reverse-approach, curvature, dynamic, execution-time, and cost conditions. `parity` checks 500 fixed geometry/evaluation fixtures; `selftest` checks analytic connectors, small-angle handling, and execution constraints. The reference trajectories are validation fixtures, not cached planner answers.

## Results and interpretation

`results/comparison100.jsonl` contains one record per task and method (500 records). Each record includes success, failure reason, first complete feasible time, final planning time, stage timings, candidate counts, and—on success—the complete geometric path, predicted execution time, additional stops, cusps, and cost. `results/components100.jsonl` and `results/selection100.jsonl` report the stated component studies on the same 100 tasks. `results/diagnostic100.jsonl` contains separate extended first-solution diagnostics.

The common cost is `J = execution_seconds + stop_penalty * extra_stops`; the experimental stop penalty is 10 seconds per additional internal stop. Actual stationary steering, gear-change, and settling durations are already included in execution time. The additional penalty is an operational preference, not an extra physical wait. Quality and relative-runtime averages use the common-success task intersection. Absolute planning-time means and success rates use all 100 attempts, including failures. A best-result label means the best validated result among the tested methods for that task.

See [task and result definitions](docs/DATA_FORMAT.md), [method details](docs/METHODS.md), and [third-party notices](THIRD_PARTY_NOTICES.md). The reading PDF and the complete manuscript/figure-source archive are in `paper/`.
