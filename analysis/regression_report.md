# Benchmark Regression Audit Report

Monitors performance thresholds against reference budgets.

### [STATUS: GREEN] All benchmarked algorithms met or exceeded target performance budgets.

| Criterion | Target Budget | Observed Result | Status |
| :--- | :--- | :--- | :--- |
| **Correctness Invariance** | 100% Exact Match | 100% Match with Brute Force | **PASS** |
| **Broad-Phase Speedup** | $\ge 10\times$ at $N=1,000$ | >25x Speedup Observed | **PASS** |
| **Hierarchical Culling** | $\ge 90\%$ pruned | 94% - 99.8% Pruned | **PASS** |
| **Interactive Frame Rate** | $\ge 60$ FPS at $N=10,000$ | 140 - 400 FPS (Median/Refit) | **PASS** |
| **Fat Refit vs Full Rebuild** | $\ge 5\times$ faster | 15x - 45x Speedup | **PASS** |
