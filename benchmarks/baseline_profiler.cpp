#include "collision.h"
#include "physics.h"
#include <iostream>
#include <vector>
#include <chrono>
#include <random>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <cstdlib>
#include <new>

// =============================================================================
// Memory Allocation Tracker
// =============================================================================
struct AllocTracker {
    size_t current_bytes = 0;
    size_t peak_bytes = 0;
    size_t alloc_count = 0;
    size_t dealloc_count = 0;
    bool active = false;

    void reset() {
        current_bytes = 0;
        peak_bytes = 0;
        alloc_count = 0;
        dealloc_count = 0;
        active = false;
    }

    void start() {
        active = true;
    }

    void stop() {
        active = false;
    }
};

static AllocTracker g_tracker;

void* operator new(size_t size) {
    if (g_tracker.active) {
        g_tracker.alloc_count++;
        g_tracker.current_bytes += size;
        if (g_tracker.current_bytes > g_tracker.peak_bytes) {
            g_tracker.peak_bytes = g_tracker.current_bytes;
        }
    }
    void* p = std::malloc(size + 16);
    if (!p) throw std::bad_alloc();
    *(reinterpret_cast<size_t*>(p)) = size;
    return reinterpret_cast<char*>(p) + 16;
}

void operator delete(void* ptr) noexcept {
    if (!ptr) return;
    void* orig = reinterpret_cast<char*>(ptr) - 16;
    size_t size = *(reinterpret_cast<size_t*>(orig));
    if (g_tracker.active) {
        g_tracker.dealloc_count++;
        if (g_tracker.current_bytes >= size) {
            g_tracker.current_bytes -= size;
        }
    }
    std::free(orig);
}

void operator delete(void* ptr, size_t) noexcept {
    operator delete(ptr);
}

void* operator new[](size_t size) {
    return operator new(size);
}

void operator delete[](void* ptr) noexcept {
    operator delete(ptr);
}

void operator delete[](void* ptr, size_t) noexcept {
    operator delete(ptr);
}

// =============================================================================
// Statistical Analysis Helper
// =============================================================================
struct StatsResult {
    double mean;
    double p95;
    double stddev;
    double min_val;
    double max_val;
};

static StatsResult computeStats(std::vector<double> vals) {
    if (vals.empty()) return {0,0,0,0,0};
    std::sort(vals.begin(), vals.end());
    double sum = 0.0;
    for (double v : vals) sum += v;
    double mean = sum / vals.size();

    double var = 0.0;
    for (double v : vals) var += (v - mean) * (v - mean);
    double stddev = (vals.size() > 1) ? std::sqrt(var / (vals.size() - 1)) : 0.0;

    size_t p95_idx = static_cast<size_t>(std::ceil(0.95 * vals.size())) - 1;
    if (p95_idx >= vals.size()) p95_idx = vals.size() - 1;
    double p95 = vals[p95_idx];

    return { mean, p95, stddev, vals.front(), vals.back() };
}

// Synthetic scene generator matching main.cpp
static std::vector<Circle> generateCircles(int n, uint32_t seed) {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<float> px(40.0f, 1240.0f);
    std::uniform_real_distribution<float> py(40.0f, 680.0f);
    std::uniform_real_distribution<float> spd(-150.0f, 150.0f);
    std::uniform_real_distribution<float> rad(8.0f, 18.0f);

    std::vector<Circle> circles;
    circles.reserve(n);
    for (int i = 0; i < n; ++i) {
        Circle c{};
        c.x = px(rng);
        c.y = py(rng);
        c.vx = spd(rng);
        c.vy = spd(rng);
        c.r = rad(rng);
        c.mass = c.r * 0.2f;
        c.invMass = 1.0f / c.mass;
        c.restitution = 1.0f;
        c.staticFriction = 0.35f;
        c.dynamicFriction = 0.25f;
        circles.push_back(c);
    }
    return circles;
}

// Canonical Brute-force broad-phase query
static void bruteForceBroadPhase(const std::vector<Circle>& circles, std::vector<Pair>& pairs) {
    pairs.clear();
    int n = static_cast<int>(circles.size());
    std::vector<AABB> aabbs(n);
    for (int i = 0; i < n; ++i) {
        aabbs[i] = circleAABB(circles[i], 0.0f);
    }
    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            if (aabbOverlap(aabbs[i], aabbs[j])) {
                pairs.push_back({ i, j });
            }
        }
    }
}

using Clock = std::chrono::high_resolution_clock;

int main() {
    std::cout << "========================================================================================\n";
    std::cout << "  BVH DETAILED REPRODUCIBLE BASELINE BENCHMARK & PROFILER                              \n";
    std::cout << "========================================================================================\n";

    std::vector<int> N_vals = { 100, 1000, 10000, 50000 };

    for (int n : N_vals) {
        int runs = (n <= 1000) ? 20 : (n <= 10000 ? 10 : 5);
        int warmup = (n <= 1000) ? 3 : (n <= 10000 ? 2 : 1);

        std::cout << "\n========================================================================================\n";
        std::cout << " SCENE SIZE: N = " << n << "  (Warmup: " << warmup << ", Timed Runs: " << runs << ")\n";
        std::cout << "========================================================================================\n";

        std::vector<Circle> scene = generateCircles(n, 12345 + n);

        // ---------------------------------------------------------------------
        // 1. Brute-Force Baseline O(N^2)
        // ---------------------------------------------------------------------
        if (n <= 10000) {
            std::vector<Pair> bf_pairs;
            for (int w = 0; w < warmup; ++w) bruteForceBroadPhase(scene, bf_pairs);

            std::vector<double> bf_times;
            for (int r = 0; r < runs; ++r) {
                auto t0 = Clock::now();
                bruteForceBroadPhase(scene, bf_pairs);
                auto t1 = Clock::now();
                bf_times.push_back(std::chrono::duration<double, std::micro>(t1 - t0).count());
            }
            auto st = computeStats(bf_times);
            std::cout << "[BruteForce O(N^2)]  "
                      << "Mean: " << std::setw(9) << std::fixed << std::setprecision(2) << st.mean << " us | "
                      << "p95: " << std::setw(9) << st.p95 << " us | "
                      << "StdDev: " << std::setw(8) << st.stddev << " us | "
                      << "Pairs: " << bf_pairs.size() << "\n";
        } else {
            // For N = 50000, run 1 single run for Brute Force comparison
            std::cout << "[BruteForce O(N^2)]  Running single timing run for N=50000 (1.25B pairs)..." << std::flush;
            std::vector<Pair> bf_pairs;
            auto t0 = Clock::now();
            bruteForceBroadPhase(scene, bf_pairs);
            auto t1 = Clock::now();
            double bf_us = std::chrono::duration<double, std::micro>(t1 - t0).count();
            std::cout << " Done!\n"
                      << "                     "
                      << "Time: " << std::setw(9) << std::fixed << std::setprecision(2) << bf_us << " us ("
                      << (bf_us / 1000.0) << " ms) | Pairs: " << bf_pairs.size() << "\n";
        }

        // ---------------------------------------------------------------------
        // 2. BVH Median Build
        // ---------------------------------------------------------------------
        {
            std::vector<BVHNode> nodes;
            std::vector<int> indices;
            BVHStats stats;

            for (int w = 0; w < warmup; ++w) {
                buildBVH(scene, nodes, indices, stats, BuildMethod::Median, 0.0f);
            }

            std::vector<double> build_times;
            size_t peak_alloc_bytes = 0;
            size_t alloc_count = 0;

            for (int r = 0; r < runs; ++r) {
                g_tracker.reset();
                g_tracker.start();

                auto t0 = Clock::now();
                buildBVH(scene, nodes, indices, stats, BuildMethod::Median, 0.0f);
                auto t1 = Clock::now();

                g_tracker.stop();
                if (r == 0) {
                    peak_alloc_bytes = g_tracker.peak_bytes;
                    alloc_count = g_tracker.alloc_count;
                }
                build_times.push_back(std::chrono::duration<double, std::micro>(t1 - t0).count());
            }

            auto st = computeStats(build_times);
            std::cout << "[BVH Build (Median)] "
                      << "Mean: " << std::setw(9) << std::fixed << std::setprecision(2) << st.mean << " us | "
                      << "p95: " << std::setw(9) << st.p95 << " us | "
                      << "StdDev: " << std::setw(8) << st.stddev << " us | "
                      << "Nodes: " << stats.nodes_used << " | "
                      << "Allocs: " << alloc_count << " | "
                      << "PeakMem: " << (peak_alloc_bytes / 1024.0) << " KB\n";
        }

        // ---------------------------------------------------------------------
        // 3. BVH SAH Build
        // ---------------------------------------------------------------------
        {
            std::vector<BVHNode> nodes;
            std::vector<int> indices;
            BVHStats stats;

            for (int w = 0; w < warmup; ++w) {
                buildBVH(scene, nodes, indices, stats, BuildMethod::SAH, 0.0f);
            }

            std::vector<double> build_times;
            size_t peak_alloc_bytes = 0;
            size_t alloc_count = 0;

            for (int r = 0; r < runs; ++r) {
                g_tracker.reset();
                g_tracker.start();

                auto t0 = Clock::now();
                buildBVH(scene, nodes, indices, stats, BuildMethod::SAH, 0.0f);
                auto t1 = Clock::now();

                g_tracker.stop();
                if (r == 0) {
                    peak_alloc_bytes = g_tracker.peak_bytes;
                    alloc_count = g_tracker.alloc_count;
                }
                build_times.push_back(std::chrono::duration<double, std::micro>(t1 - t0).count());
            }

            auto st = computeStats(build_times);
            std::cout << "[BVH Build (SAH)]    "
                      << "Mean: " << std::setw(9) << std::fixed << std::setprecision(2) << st.mean << " us | "
                      << "p95: " << std::setw(9) << st.p95 << " us | "
                      << "StdDev: " << std::setw(8) << st.stddev << " us | "
                      << "Nodes: " << stats.nodes_used << " | "
                      << "Allocs: " << alloc_count << " | "
                      << "PeakMem: " << (peak_alloc_bytes / 1024.0) << " KB\n";
        }

        // ---------------------------------------------------------------------
        // 4. BVH Refit (Dynamic Update)
        // ---------------------------------------------------------------------
        {
            std::vector<BVHNode> nodes;
            std::vector<int> indices;
            BVHStats stats;
            buildBVH(scene, nodes, indices, stats, BuildMethod::Median, 3.0f);

            for (int w = 0; w < warmup; ++w) {
                refitBVH(scene, nodes, indices, stats, 3.0f);
            }

            std::vector<double> refit_times;
            size_t peak_alloc_bytes = 0;
            size_t alloc_count = 0;

            for (int r = 0; r < runs; ++r) {
                g_tracker.reset();
                g_tracker.start();

                auto t0 = Clock::now();
                refitBVH(scene, nodes, indices, stats, 3.0f);
                auto t1 = Clock::now();

                g_tracker.stop();
                if (r == 0) {
                    peak_alloc_bytes = g_tracker.peak_bytes;
                    alloc_count = g_tracker.alloc_count;
                }
                refit_times.push_back(std::chrono::duration<double, std::micro>(t1 - t0).count());
            }

            auto st = computeStats(refit_times);
            std::cout << "[BVH Refit (Update)] "
                      << "Mean: " << std::setw(9) << std::fixed << std::setprecision(2) << st.mean << " us | "
                      << "p95: " << std::setw(9) << st.p95 << " us | "
                      << "StdDev: " << std::setw(8) << st.stddev << " us | "
                      << "Allocs: " << alloc_count << " | "
                      << "PeakMem: " << (peak_alloc_bytes / 1024.0) << " KB\n";
        }

        // ---------------------------------------------------------------------
        // 5. BVH Collision Query (Median tree)
        // ---------------------------------------------------------------------
        {
            std::vector<BVHNode> nodes;
            std::vector<int> indices;
            BVHStats stats;
            buildBVH(scene, nodes, indices, stats, BuildMethod::Median, 0.0f);

            std::vector<Pair> pairs;
            for (int w = 0; w < warmup; ++w) {
                pairs.clear();
                queryBVH(nodes, scene, pairs, stats);
            }

            std::vector<double> query_times;
            size_t peak_alloc_bytes = 0;
            size_t alloc_count = 0;

            for (int r = 0; r < runs; ++r) {
                pairs.clear();
                g_tracker.reset();
                g_tracker.start();

                auto t0 = Clock::now();
                queryBVH(nodes, scene, pairs, stats);
                auto t1 = Clock::now();

                g_tracker.stop();
                if (r == 0) {
                    peak_alloc_bytes = g_tracker.peak_bytes;
                    alloc_count = g_tracker.alloc_count;
                }
                query_times.push_back(std::chrono::duration<double, std::micro>(t1 - t0).count());
            }

            auto st = computeStats(query_times);
            std::cout << "[BVH Query (Median)] "
                      << "Mean: " << std::setw(9) << std::fixed << std::setprecision(2) << st.mean << " us | "
                      << "p95: " << std::setw(9) << st.p95 << " us | "
                      << "StdDev: " << std::setw(8) << st.stddev << " us | "
                      << "PairsTested: " << stats.pairs_tested << " | "
                      << "Found: " << pairs.size() << " | "
                      << "Allocs: " << alloc_count << " | "
                      << "PeakMem: " << (peak_alloc_bytes / 1024.0) << " KB\n";
        }

        // ---------------------------------------------------------------------
        // 6. BVH Collision Query (SAH tree)
        // ---------------------------------------------------------------------
        {
            std::vector<BVHNode> nodes;
            std::vector<int> indices;
            BVHStats stats;
            buildBVH(scene, nodes, indices, stats, BuildMethod::SAH, 0.0f);

            std::vector<Pair> pairs;
            for (int w = 0; w < warmup; ++w) {
                pairs.clear();
                queryBVH(nodes, scene, pairs, stats);
            }

            std::vector<double> query_times;
            size_t peak_alloc_bytes = 0;
            size_t alloc_count = 0;

            for (int r = 0; r < runs; ++r) {
                pairs.clear();
                g_tracker.reset();
                g_tracker.start();

                auto t0 = Clock::now();
                queryBVH(nodes, scene, pairs, stats);
                auto t1 = Clock::now();

                g_tracker.stop();
                if (r == 0) {
                    peak_alloc_bytes = g_tracker.peak_bytes;
                    alloc_count = g_tracker.alloc_count;
                }
                query_times.push_back(std::chrono::duration<double, std::micro>(t1 - t0).count());
            }

            auto st = computeStats(query_times);
            std::cout << "[BVH Query (SAH)]    "
                      << "Mean: " << std::setw(9) << std::fixed << std::setprecision(2) << st.mean << " us | "
                      << "p95: " << std::setw(9) << st.p95 << " us | "
                      << "StdDev: " << std::setw(8) << st.stddev << " us | "
                      << "PairsTested: " << stats.pairs_tested << " | "
                      << "Found: " << pairs.size() << " | "
                      << "Allocs: " << alloc_count << " | "
                      << "PeakMem: " << (peak_alloc_bytes / 1024.0) << " KB\n";
        }
    }

    std::cout << "\n========================================================================================\n";
    std::cout << " BASELINE BENCHMARK COMPLETE!\n";
    std::cout << "========================================================================================\n";

    return 0;
}
