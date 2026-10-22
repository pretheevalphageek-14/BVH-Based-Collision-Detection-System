#pragma once

#include "collision.h"
#include "benchmark_types.h"
#include <vector>
#include <algorithm>
#include <cmath>
#include <numeric>
#include <sstream>
#include <iomanip>
#include <string>

/**
 * @file benchmark_utils.h
 * @brief Utilities for brute-force baseline, verification, tree analysis, and statistics.
 */

namespace BenchmarkUtils {

/**
 * @brief Canonical O(N²) all-pairs brute-force baseline.
 * Performs tight AABB overlap check for every distinct pair (i, j) with i < j.
 */
inline void queryBruteForce(const std::vector<Circle>& circles,
                            std::vector<Pair>& out_pairs,
                            int& pairs_tested) {
    out_pairs.clear();
    pairs_tested = 0;
    const int n = static_cast<int>(circles.size());

    // Precompute AABBs for cache efficiency in brute-force
    std::vector<AABB> aabbs;
    aabbs.reserve(n);
    for (int i = 0; i < n; ++i) {
        aabbs.push_back(circleAABB(circles[i], 0.0f));
    }

    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            pairs_tested++;
            if (aabbOverlap(aabbs[i], aabbs[j])) {
                out_pairs.push_back({ i, j });
            }
        }
    }
}

/**
 * @brief Counts actual circle-circle disk overlaps (narrow phase) for broad-phase candidate pairs.
 */
inline int countNarrowPhaseHits(const std::vector<Circle>& circles, const std::vector<Pair>& pairs) {
    int hits = 0;
    for (const auto& p : pairs) {
        const Circle& a = circles[p.a];
        const Circle& b = circles[p.b];
        float dx = b.x - a.x;
        float dy = b.y - a.y;
        float minDist = a.r + b.r;
        if ((dx * dx + dy * dy) < (minDist * minDist)) {
            hits++;
        }
    }
    return hits;
}

/**
 * @brief Verifies that BVH broad-phase candidate pairs match Brute-Force pairs exactly.
 * Fails loudly if there are any spurious or missing pairs.
 */
inline bool verifyCorrectness(const std::vector<Pair>& bvh_pairs,
                              const std::vector<Pair>& brute_pairs,
                              std::string& err_msg) {
    std::vector<Pair> bvh_sorted = bvh_pairs;
    std::vector<Pair> brute_sorted = brute_pairs;

    // Normalize pair order (a < b)
    for (auto& p : bvh_sorted) {
        if (p.a > p.b) std::swap(p.a, p.b);
    }
    for (auto& p : brute_sorted) {
        if (p.a > p.b) std::swap(p.a, p.b);
    }

    std::sort(bvh_sorted.begin(), bvh_sorted.end());
    std::sort(brute_sorted.begin(), brute_sorted.end());

    if (bvh_sorted.size() != brute_sorted.size()) {
        std::ostringstream ss;
        ss << "Count mismatch: BVH produced " << bvh_sorted.size()
           << " pairs, but Brute Force produced " << brute_sorted.size() << " pairs!";
        err_msg = ss.str();
        return false;
    }

    for (size_t i = 0; i < bvh_sorted.size(); ++i) {
        if (!(bvh_sorted[i] == brute_sorted[i])) {
            std::ostringstream ss;
            ss << "Pair mismatch at index " << i << ": BVH=("
               << bvh_sorted[i].a << "," << bvh_sorted[i].b
               << ") vs Brute=("
               << brute_sorted[i].a << "," << brute_sorted[i].b << ")";
            err_msg = ss.str();
            return false;
        }
    }

    return true;
}

/**
 * @brief Computes maximum tree depth recursively in O(nodes).
 */
inline int calculateTreeDepth(const std::vector<BVHNode>& nodes, int node_idx = 0) {
    if (nodes.empty() || node_idx < 0 || node_idx >= static_cast<int>(nodes.size())) {
        return 0;
    }
    const BVHNode& node = nodes[node_idx];
    if (node.object_index >= 0) {
        return 1; // Leaf node
    }
    int dLeft = calculateTreeDepth(nodes, node.left);
    int dRight = calculateTreeDepth(nodes, node.right);
    return 1 + std::max(dLeft, dRight);
}

/**
 * @brief Computes contiguous memory footprint of the BVH in bytes.
 */
inline size_t calculateBVHMemory(const std::vector<BVHNode>& nodes, const std::vector<int>& indices) {
    return (nodes.capacity() * sizeof(BVHNode)) + (indices.capacity() * sizeof(int));
}

/**
 * @brief Computes total Surface Area Heuristic (SAH) quality cost of the BVH tree.
 * SAH(T) = sum_{interior}(SA(node)/SA(root) * C_trav) + sum_{leaf}(SA(leaf)/SA(root) * C_isect * count)
 * with C_trav = 1.0, C_isect = 1.0.
 */
inline double calculateSAHCost(const std::vector<BVHNode>& nodes) {
    if (nodes.empty()) return 0.0;
    float root_sa = aabbSurfaceArea(nodes[0].bounds);
    if (root_sa <= 1e-7f) return 0.0;

    double total_cost = 0.0;
    for (const auto& node : nodes) {
        float sa = aabbSurfaceArea(node.bounds);
        float rel_sa = sa / root_sa;
        total_cost += static_cast<double>(rel_sa * 1.0f);
    }
    return total_cost;
}

/**
 * @brief Computes summary statistics (mean, median, standard deviation, min, max, p95) for numeric vectors.
 */
inline MetricSummary computeSummary(std::vector<double> values) {
    MetricSummary s{};
    if (values.empty()) return s;

    std::sort(values.begin(), values.end());
    s.min_val = values.front();
    s.max_val = values.back();

    double sum = std::accumulate(values.begin(), values.end(), 0.0);
    s.mean = sum / static_cast<double>(values.size());

    size_t mid = values.size() / 2;
    if (values.size() % 2 == 0) {
        s.median = (values[mid - 1] + values[mid]) * 0.5;
    } else {
        s.median = values[mid];
    }

    size_t p95_idx = static_cast<size_t>(std::ceil(0.95 * values.size())) - 1;
    if (p95_idx >= values.size()) p95_idx = values.size() - 1;
    s.p95 = values[p95_idx];

    double variance = 0.0;
    for (double v : values) {
        variance += (v - s.mean) * (v - s.mean);
    }
    s.std_dev = (values.size() > 1) ? std::sqrt(variance / static_cast<double>(values.size() - 1)) : 0.0;

    return s;
}

} // namespace BenchmarkUtils
