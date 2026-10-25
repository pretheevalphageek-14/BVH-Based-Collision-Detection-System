# Benchmark Results

Benchmarked over 300 frames per entry (fixed timestep dt = 1/60 s).

| n | Method | Build/Refit (us) | Query Time (us) | Pairs Tested | Total Step (us) |
|------|--------------|------------------|-----------------|--------------|-----------------|
| 100 | Median Split | 214.61 | 67.10 | 281 | 281.70 |
| 100 | SAH Split | 305.31 | 52.27 | 229 | 357.58 |
| 100 | Refit (Fat) | 5.65 | 278.79 | 737 | 284.43 |
| 500 | Median Split | 1245.78 | 579.52 | 2100 | 1825.30 |
| 500 | SAH Split | 1581.59 | 440.48 | 1907 | 2022.07 |
| 500 | Refit (Fat) | 31.16 | 6548.19 | 14852 | 6579.35 |
| 1000 | Median Split | 2626.42 | 1479.69 | 5602 | 4106.11 |
| 1000 | SAH Split | 3073.23 | 1187.82 | 5412 | 4261.05 |
| 1000 | Refit (Fat) | 71.87 | 23213.54 | 53182 | 23285.41 |
| 5000 | Refit (Fat) | 348.03 | 558365.01 | 1396448 | 558713.03 |

_Measured with <chrono::high_resolution_clock> on MSVC /O2 /W4._
