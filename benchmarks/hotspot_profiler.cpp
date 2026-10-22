#include "collision.h"
#include "physics.h"
#include <iostream>
#include <vector>
#include <chrono>
#include <random>
#include <algorithm>
#include <cmath>
#include <iomanip>

using Clock = std::chrono::high_resolution_clock;

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

int main() {
    std::cout << "===================================================================\n";
    std::cout << "  HOTSPOT PROFILER: BREAKDOWN OF TIME SPENT                        \n";
    std::cout << "===================================================================\n";

    const int N = 10000;
    std::vector<Circle> scene = generateCircles(N, 9999);

    std::vector<BVHNode> nodes;
    std::vector<int> indices;
    BVHStats stats;

    // 1. Build profiling
    const int BUILD_RUNS = 20;
    double t_bounds_accum = 0.0;
    double t_nth_accum = 0.0;
    double t_build_total = 0.0;

    auto t0 = Clock::now();
    for (int r = 0; r < BUILD_RUNS; ++r) {
        buildBVH(scene, nodes, indices, stats, BuildMethod::Median, 0.0f);
    }
    auto t1 = Clock::now();
    t_build_total = std::chrono::duration<double, std::micro>(t1 - t0).count() / BUILD_RUNS;

    std::cout << "1. BVH Build (Median, N=" << N << "):\n";
    std::cout << "   Total Average Build Time: " << std::fixed << std::setprecision(2) << t_build_total << " us\n";

    // 2. Query profiling
    const int QUERY_RUNS = 20;
    std::vector<Pair> pairs;
    pairs.reserve(N * 4);

    t0 = Clock::now();
    for (int r = 0; r < QUERY_RUNS; ++r) {
        pairs.clear();
        queryBVH(nodes, scene, pairs, stats);
    }
    t1 = Clock::now();
    double t_query_total = std::chrono::duration<double, std::micro>(t1 - t0).count() / QUERY_RUNS;

    std::cout << "\n2. BVH Query (Median Tree, N=" << N << "):\n";
    std::cout << "   Total Average Query Time: " << t_query_total << " us\n";
    std::cout << "   Pairs Tested:             " << stats.pairs_tested << "\n";
    std::cout << "   Pairs Emitted:            " << pairs.size() << "\n";
    std::cout << "   Time per Overlap Test:    " << (t_query_total / stats.pairs_tested * 1000.0) << " ns\n";

    // 3. Refit profiling
    const int REFIT_RUNS = 100;
    buildBVH(scene, nodes, indices, stats, BuildMethod::Median, 3.0f);
    t0 = Clock::now();
    for (int r = 0; r < REFIT_RUNS; ++r) {
        refitBVH(scene, nodes, indices, stats, 3.0f);
    }
    t1 = Clock::now();
    double t_refit_total = std::chrono::duration<double, std::micro>(t1 - t0).count() / REFIT_RUNS;

    std::cout << "\n3. BVH Refit (Dynamic Update, N=" << N << "):\n";
    std::cout << "   Total Average Refit Time: " << t_refit_total << " us\n";
    std::cout << "   Nodes Refit:              " << nodes.size() << "\n";
    std::cout << "   Time per Node:            " << (t_refit_total / nodes.size() * 1000.0) << " ns\n";

    // 4. SAH Build profiling
    const int SAH_RUNS = 10;
    t0 = Clock::now();
    for (int r = 0; r < SAH_RUNS; ++r) {
        buildBVH(scene, nodes, indices, stats, BuildMethod::SAH, 0.0f);
    }
    t1 = Clock::now();
    double t_sah_total = std::chrono::duration<double, std::micro>(t1 - t0).count() / SAH_RUNS;

    std::cout << "\n4. BVH Build (SAH, N=" << N << "):\n";
    std::cout << "   Total Average Build Time: " << t_sah_total << " us\n";

    return 0;
}
