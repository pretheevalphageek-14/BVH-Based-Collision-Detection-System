#pragma once

#include <string>
#include <vector>
#include <cstdint>

/**
 * @file benchmark_types.h
 * @brief Common types, configurations, and result containers for BVH Benchmarks.
 */

enum class SceneDistribution {
    Uniform,
    Clustered,
    DenseOverlap,
    Sparse,
    Dynamic
};

inline const char* distributionToString(SceneDistribution d) {
    switch (d) {
        case SceneDistribution::Uniform:      return "uniform";
        case SceneDistribution::Clustered:    return "clustered";
        case SceneDistribution::DenseOverlap: return "dense_overlap";
        case SceneDistribution::Sparse:       return "sparse";
        case SceneDistribution::Dynamic:      return "dynamic";
        default:                              return "unknown";
    }
}

enum class BenchmarkMethod {
    BruteForce,
    BVHMedian,
    BVHSAH,
    BVHRefit
};

inline const char* methodToString(BenchmarkMethod m) {
    switch (m) {
        case BenchmarkMethod::BruteForce: return "BruteForce";
        case BenchmarkMethod::BVHMedian:  return "BVH-Median";
        case BenchmarkMethod::BVHSAH:     return "BVH-SAH";
        case BenchmarkMethod::BVHRefit:   return "BVH-Refit";
        default:                          return "Unknown";
    }
}

struct BenchmarkConfig {
    int n = 500;
    SceneDistribution distribution = SceneDistribution::Uniform;
    BenchmarkMethod method = BenchmarkMethod::BVHMedian;
    int repetitions = 10;
    int warmup_runs = 3;
    uint32_t random_seed = 42;
    float arena_width = 1280.0f;
    float arena_height = 720.0f;
    float fat_margin = 3.0f;
    int dynamic_frames = 10; // For dynamic scenario physics steps
};

struct SingleRunMetric {
    int run_index = 0;
    double build_time_us = 0.0;
    double refit_time_us = 0.0;
    double query_time_us = 0.0;
    double total_step_us = 0.0;
    double fps = 0.0;
    int pairs_tested = 0;          // Bounding volume overlap tests performed
    int node_visits = 0;           // Node visits during stack traversal
    int narrow_phase_pairs = 0;    // Broad-phase candidate pairs emitted
    int narrow_phase_hits = 0;     // Actual circle-circle intersecting pairs
    double culling_efficiency = 0.0; // % of all-pairs tests avoided
    int tree_depth = 0;
    int nodes_used = 0;
    double sah_cost = 0.0;         // SAH tree quality cost
    size_t memory_bytes = 0;
    bool correctness_passed = true;
};

struct MetricSummary {
    double mean = 0.0;
    double median = 0.0;
    double std_dev = 0.0;
    double min_val = 0.0;
    double max_val = 0.0;
    double p95 = 0.0;
};

struct AggregatedBenchmarkResult {
    int n = 0;
    std::string distribution;
    std::string method;
    int repetitions = 0;
    MetricSummary build_time_us;
    MetricSummary refit_time_us;
    MetricSummary query_time_us;
    MetricSummary total_step_us;
    MetricSummary fps;
    MetricSummary pairs_tested;
    MetricSummary node_visits;
    MetricSummary narrow_phase_pairs;
    MetricSummary narrow_phase_hits;
    MetricSummary culling_efficiency_pct;
    int tree_depth = 0;
    int nodes_used = 0;
    double sah_cost = 0.0;
    size_t memory_bytes = 0;
    bool all_correctness_passed = true;
};
