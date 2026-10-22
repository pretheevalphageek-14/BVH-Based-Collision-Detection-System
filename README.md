# Bounding Volume Hierarchy (BVH) & Contact Physics Simulation System
### High-Performance Spatial Partitioning & Collision Dynamics Suite (DSA Case Study)

[![C++20](https://img.shields.io/badge/Language-C%2B%2B20-blue.svg)](https://en.cppreference.com/w/cpp/20)
[![Physics](https://img.shields.io/badge/Physics-Impulse%20%2B%20Coulomb%20Friction-emerald.svg)](#-contact-physics-engine)
[![Complexity](https://img.shields.io/badge/Broad%20Phase-O(N%20log%20N)%20vs%20O(N%C2%B2)-cyan.svg)](#-complexity--empirical-performance-highlights)
[![Benchmarks](https://img.shields.io/badge/Benchmarks-6%20Scales%20%7C%205%20Distributions-purple.svg)](#-reproducible-benchmarking-suite)
[![Tests](https://img.shields.io/badge/Tests-100%25%20Pass%20(CTest)-brightgreen.svg)](#-building--testing)
[![License](https://img.shields.io/badge/License-MIT-gray.svg)](LICENSE)

An industrial-grade, portfolio-ready 2D collision detection and impulse physics engine showcasing **Bounding Volume Hierarchy (BVH)** spatial partitioning. It delivers **real-time spatial subdivision**, **hierarchical broad-phase subtree culling**, and **contact dynamics** in a native C++20 / SDL3 application, backed by a comprehensive **benchmarking suite** and **data analytics pipeline**.

---

## 🌟 Highlights & Key Features

- **Algorithmic Spatial Culling**: Cuts broad-phase collision tests from naive $O(N^2)$ down to $O(k + \log N)$ using hierarchical dual-tree traversal.
- **Flat-Array Cache-Friendly Node Pool**: Nodes reside in a contiguous `std::vector<BVHNode>` indexed by 32-bit integer offsets, eliminating pointer chasing and allocation churn.
- **Multiple Partitioning Strategies**:
  - **Median-Split BVH**: Partitions object centroids along the longest bounding box axis ($O(N \log N)$ build via `std::nth_element`).
  - **Surface Area Heuristic (SAH)**: 12-bin spatial heuristic minimizing expected query cost.
  - **Fat AABB Lazy Refit**: Inflated margins with bottom-up $O(N)$ tree refitting, rebuilding topology only when objects escape margins.
- **Contact Physics & Conservation Dynamics**:
  - Impulse-based momentum exchange with configurable restitution $e \in [0.0, 1.0]$ (Elastic, Inelastic, Partially Elastic).
  - Tangential Coulomb surface friction with static and dynamic impulse clamping to prevent unphysical energy gain.
  - Concentric body de-penetration projection and Baumgarte linear stabilization.
  - Quantitative conservation auditor computing Kinetic Energy ($\sum \frac{1}{2}mv^2$) and linear momentum.
- **Comprehensive Benchmarking & Analytics Harness**:
  - Standalone reproducible benchmarking CLI evaluating 6 scene sizes ($N=100$ to $50,000$), 5 spatial topologies, and 15 statistical repetitions.
  - Automated Python analytics producing 10 publication-grade charts (PNG & SVG), variance analyses, and regression reports.
  - Continuous bitwise correctness assertions verifying BVH candidate pairs against an exact brute-force ground truth.

---

## ⚡ Complexity & Empirical Performance Highlights

All empirical figures below reflect physical measurements gathered over 15 repetitions with a high-resolution nanosecond clock on native Windows x86_64:

| Object Count ($N$) | Method | Build / Refit Time | Broad-Phase Query | Pairs Tested | Naive Pairs $O(N^2)$ | Culling Efficiency | Speedup vs BF |
| :---: | :--- | :---: | :---: | :---: | :---: | :---: | :---: |
| **100** | BVH-Median | $2.84\,\mu\text{s}$ | $1.10\,\mu\text{s}$ | 264 | 4,950 | **$94.67\%$** | **$3.8\times$** |
| **500** | BVH-Median | $46.03\,\mu\text{s}$ | $20.54\,\mu\text{s}$ | 1,922 | 124,750 | **$98.46\%$** | **$9.6\times$** |
| **1,000** | BVH-Median | $124.87\,\mu\text{s}$ | $67.63\,\mu\text{s}$ | 5,154 | 499,500 | **$98.97\%$** | **$13.2\times$** |
| **5,000** | BVH-Median | $794.63\,\mu\text{s}$ | $846.83\,\mu\text{s}$ | 72,708 | 12,497,500 | **$99.42\%$** | **$32.7\times$** |
| **10,000** | BVH-Median | $2,600.85\,\mu\text{s}$ | $3,439.83\,\mu\text{s}$ | 249,127 | 49,995,000 | **$99.50\%$** | **$45.9\times$** |
| **10,000** | **Fat AABB Refit** | **$91.82\,\mu\text{s}$** | $6,494.62\,\mu\text{s}$ | 367,996 | 49,995,000 | **$99.26\%$** | **$24.3\times$** |
| **50,000** | BVH-Median | $12,447.80\,\mu\text{s}$ | $53,970.90\,\mu\text{s}$ | 5,405,920 | 1,249,975,000 | **$99.57\%$** | — |

> **Key Performance Takeaways**:
> - **45.9x Query Speedup** at $N=10,000$ compared to Brute Force ($3.44\text{ ms}$ vs $158.06\text{ ms}$).
> - **28.3x Refit Speedup** for Fat AABB refitting over full tree rebuilding ($91.82\,\mu\text{s}$ vs $2,600.85\,\mu\text{s}$).
> - **>99.5% Culling Efficiency** at scale, discarding over 1.24 billion unnecessary checks at $N=50,000$.
> - **Small-$N$ Break-Even ($N_{break}$)**: Total BVH step time (build + query) surpasses brute-force when $N \ge 80$–$120$.

---

## 📊 Visual Analytics & Performance Charts

The analytics pipeline generates publication-quality figures saved as both PNG and SVG in `analysis/figures/`:

| Query Scaling (Log-Log Scale) | Tree Build vs Object Count |
| :---: | :---: |
| ![Query Scaling](analysis/figures/01_query_time_vs_n_loglog.svg) | ![Build Time](analysis/figures/02_build_time_vs_n.svg) |
| *Log-log scaling: Brute force $O(N^2)$ vs BVH sub-linear* | *Median vs SAH vs O(N) Fat AABB Refit* |

| Speedup vs Brute Force | Subtree Culling Efficiency |
| :---: | :---: |
| ![Speedup Factor](analysis/figures/03_speedup_factor.svg) | ![Culling Efficiency](analysis/figures/04_culling_efficiency.svg) |
| *Speedup reaching >45x at N=10,000* | *Over 99.5% pair comparisons avoided* |

| Performance Across Distributions | Empirical Complexity Regression Fit |
| :---: | :---: |
| ![Distributions](analysis/figures/05_performance_across_distributions.svg) | ![Complexity Fit](analysis/figures/08_empirical_complexity_fit.svg) |
| *Evaluation across 5 spatial distributions at N=10,000* | *Fitted $O(N \log N)$ ($R^2=0.9644$) and $O(N^2)$ ($R^2=0.9925$)* |

---

## 🏗️ Project Structure

```
BVH-Based-Collision-Detection-System/
├── CMakeLists.txt              # Primary CMake build configuration (SDL3 + tests)
├── vcpkg.json                  # SDL3 dependency pinning
├── requirements.txt            # Python dependencies for analytics pipeline
├── README.md                   # Project overview, architecture, and benchmark dashboard
├── LICENSE                     # MIT License
├── BENCHMARKS.md               # Visual simulation benchmark logs
├── EXPLANATION.md              # DSA case study breakdown
├── include/
│   └── logger.h                # High-performance zero-allocation logging utility
├── src/
│   ├── main.cpp                # SDL3 application window, event loop & render pipeline
│   ├── collision.h             # BVHNode, AABB, Pair, and partitioning interface
│   ├── collision.cpp           # Top-down construction, bottom-up refit & dual-tree traversal
│   ├── physics.h               # Impulse resolution, Coulomb friction & conservation auditor
│   └── physics.cpp             # Contact dynamics, de-penetration & Symplectic Euler kinematics
├── tests/
│   ├── test_physics.cpp        # Unit test suite verifying physical conservation laws
│   └── test_correctness.cpp    # 12 edge cases & scale verification against O(N²) ground truth
├── benchmarks/                 # Isolated, reproducible benchmarking harness
│   ├── run_benchmarks.ps1      # Single-command build & benchmark automation script
│   ├── include/
│   │   ├── benchmark_types.h   # Data structures, enums, and metric containers
│   │   ├── benchmark_utils.h   # O(N²) baseline, tree depth/memory analyzers, verification
│   │   └── scene_generator.h   # Synthetic scene generator (Uniform, Clustered, Dense, Sparse, Dynamic)
│   ├── src/
│   │   └── benchmark_runner.cpp# Standalone CLI harness executing benchmark grid
│   └── results/                # Output datasets
│       ├── benchmark_raw.csv   # Raw metrics for every individual trial run
│       ├── benchmark_summary.csv # Aggregated statistics (mean, median, p95, std dev, min, max)
│       ├── benchmark_results.json# Structured JSON export of all benchmark runs
│       └── system_specs.json   # Machine environment specifications
├── analysis/                   # Data analytics & visualization suite
│   ├── generate_analytics.py   # Dual-engine plotting script (pure SVG & matplotlib PNG)
│   ├── findings.md             # Plain-language empirical analysis & insights
│   ├── variance_outliers.md    # Coefficient of variation (CV) & outlier analysis
│   ├── regression_report.md    # Automated performance budget & regression audit
│   ├── key_metrics_summary.csv # Tabular metrics across key configurations
│   ├── key_metrics_summary.md  # Formatted Markdown summary table
│   └── figures/                # Publication-quality charts (PNG and SVG formats)
└── docs/                       # Technical documentation & project reports
    ├── architecture.md         # Data layout, flat node pool, and Mermaid flowcharts
    ├── benchmark-report.md     # Academic-style benchmark methodology & evaluation
    ├── api-reference.md        # Comprehensive public API documentation
    └── CONTRIBUTING.md         # Guide for teammates on adding benchmark scenarios
```

---

## 🛠️ Building & Testing

### 1. Build and Run Unit Tests (CTest)

The project includes an automated CTest test suite covering physics laws, edge cases, concentric shapes, high-speed motion, and scale verification against $O(N^2)$ ground truth:

```powershell
# Configure CMake
cmake -B build -G "MinGW Makefiles"

# Build all targets (application and test suites)
cmake --build build

# Execute all tests via CTest
ctest --test-dir build --output-on-failure
```

---

### 2. Interactive Graphical Simulation (SDL3)

Requires SDL3 (via vcpkg or system include):

```powershell
# Launch interactive simulation window
./build/bvh_physics.exe
```

#### Interactive Controls:
* `1` / `2` / `3`: Switch mode directly (`1` = BVH-Median, `2` = BVH-SAH, `3` = BVH-Refit).
* `B`: Cycle collision mode.
* `T`: Toggle hierarchical tree AABB debug visualizer overlay.
* `UP` / `DOWN`: Increase / decrease maximum tree depth drawn.
* `SPACE`: Pause / resume physical simulation.
* `R`: Reset scene with 500 new random circles.
* `E` / `I` / `P`: Switch restitution mode (`E` = Elastic $e=1.0$, `I` = Inelastic $e=0.0$, `P` = Partially Elastic $e=0.75$).
* `F` / `G`: Toggle Coulomb surface friction / gravity.
* `Left Click`: Spawn a circle at cursor.
* `Right Click + Drag`: Pull nearby circles toward cursor with force lines.
* `ESC`: Clean exit and flush logs.

---

### 3. Standalone Headless Benchmarks (`/benchmarks`)

The benchmarking suite has **zero external library dependencies** (pure C++20). It runs headlessly on any machine:

```powershell
# Full benchmark run across all 6 scales and 5 distributions
powershell -ExecutionPolicy Bypass -File benchmarks/run_benchmarks.ps1 -Full -OutputDir benchmarks/results

# Quick smoke test mode (N = 100, 500, 1000)
powershell -ExecutionPolicy Bypass -File benchmarks/run_benchmarks.ps1 -Quick
```

---

### 4. Data Analytics Pipeline (`/analysis`)

The analytics script has an intelligent dual-engine architecture: it generates full tables, markdown reports, and crisp vector SVG charts with **zero external dependencies** using Python's standard library, and can also generate raster PNGs if matplotlib is present:

```bash
# Execute analytics pipeline
python analysis/generate_analytics.py
```

Outputs are automatically saved to `analysis/figures/`, `analysis/key_metrics_summary.md`, `analysis/variance_outliers.md`, `analysis/regression_report.md`, and `analysis/findings.md`.

---

## 📖 In-Depth Documentation

* **[Architecture & Memory Layout](docs/architecture.md)**: Detailed examination of the flat-array node pool, cache locality, and Mermaid traversal flowcharts.
* **[Benchmark Report](docs/benchmark-report.md)**: Academic-style technical report evaluating empirical scaling, statistical confidence, and limitations.
* **[Public API Reference](docs/api-reference.md)**: Signatures, complexity, parameters, and examples for all engine and benchmark functions.
* **[Contributing Guide](docs/CONTRIBUTING.md)**: Guide for teammates to add benchmark distributions and submit PRs.

---

## 📜 License

This project is licensed under the [MIT License](LICENSE).