# Benchmark Variance & Outlier Detection Report

Evaluates execution stability across repetitions using the Coefficient of Variation ($CV = \sigma / \mu$).

### [NOTICE] Configurations with Moderate Variance ($CV > 0.25$):

| N | Distribution | Method | Mean (us) | Std Dev (us) | CV |
|---|---|---|---|---|---|
| 100 | dense_overlap | BruteForce | 5.23 | 2.64 | 0.51 |
| 500 | dense_overlap | BVH-Median | 90.57 | 32.46 | 0.36 |
| 500 | dense_overlap | BVH-SAH | 106.35 | 29.26 | 0.28 |
| 500 | sparse | BVH-Median | 8.75 | 5.56 | 0.64 |
| 500 | sparse | BVH-SAH | 5.55 | 2.49 | 0.45 |
| 500 | sparse | BVH-Refit | 6.56 | 1.93 | 0.29 |
| 1000 | sparse | BVH-SAH | 22.57 | 21.36 | 0.95 |
| 1000 | dynamic | BVH-Median | 75.26 | 22.10 | 0.29 |
| 5000 | sparse | BVH-Median | 239.26 | 86.56 | 0.36 |
| 5000 | sparse | BVH-SAH | 168.74 | 47.02 | 0.28 |
| 5000 | sparse | BVH-Refit | 390.11 | 101.94 | 0.26 |
| 5000 | dynamic | BVH-Refit | 2235.18 | 728.94 | 0.33 |
| 10000 | uniform | BVH-Median | 3439.83 | 983.96 | 0.29 |
| 10000 | uniform | BVH-SAH | 3572.65 | 1192.56 | 0.33 |
| 10000 | uniform | BVH-Refit | 6494.62 | 1801.87 | 0.28 |
| 10000 | sparse | BVH-Median | 518.08 | 198.56 | 0.38 |
| 10000 | sparse | BVH-SAH | 404.19 | 210.79 | 0.52 |
| 10000 | sparse | BVH-Refit | 864.21 | 303.73 | 0.35 |
| 50000 | sparse | BVH-Median | 6072.99 | 2034.52 | 0.34 |
| 50000 | sparse | BVH-SAH | 4568.13 | 1551.65 | 0.34 |

_Note: Minor variance in dense overlapping configurations is expected due to variable contact solver cascades and cache residency._
