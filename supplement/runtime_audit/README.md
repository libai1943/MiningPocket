# Optional implementation-efficiency audit

This supplement preserves the equivalent-kernel experiment described separately in the manuscript. It does not replace the four-method recorded timing comparison.

In a fresh MATLAB session, from the repository root:

    addpath('supplement/runtime_audit')
    paired_kernel100
    run_fast100

The first command checks 300 alternating old/new connector calls. The second replans the 100 tasks with the equivalent kernel and compares every segment sequence with the recorded BRS solutions. Outputs go to results/reproduced/. Restart MATLAB before a main benchmark rerun to reset package resolution.

The original records are paired_kernel100.json and fast100.json. Exact timing varies by machine; candidate and segment agreement are the reproducibility checks.
