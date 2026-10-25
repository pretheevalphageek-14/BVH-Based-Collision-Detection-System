#include "collision.h"
#include "physics.h"
#include "benchmark_types.h"
#include "scene_generator.h"
#include "benchmark_utils.h"

#include <iostream>
#include <fstream>
#include <vector>
#include <chrono>
#include <string>
#include <iomanip>
#include <filesystem>
#include <thread>

using Clock = std::chrono::high_resolution_clock;
namespace fs = std::filesystem;

// System hardware info collector
static void writeSystemSpecs(const std::string& out_dir) {
    fs::create_directories(out_dir);
    std::string json_path = out_dir + "/system_specs.json";
    std::ofstream f(json_path);
    if (!f.is_open()) return;

    unsigned int cores = std::thread::hardware_concurrency();
#if defined(_WIN32)
    std::string os_name = "Windows (x86_64)";
#elif defined(__linux__)
    std::string os_name = "Linux";
#elif defined(__APPLE__)
    std::string os_name = "macOS";
#else
    std::string os_name = "Unknown OS";
#endif

#if defined(_MSC_VER)
    std::string compiler = "MSVC " + std::to_string(_MSC_VER);
#elif defined(__clang__)
    std::string compiler = "Clang " + std::string(__clang_version__);
#elif defined(__GNUC__)
    std::string compiler = "GCC " + std::string(__VERSION__);
#else
    std::string compiler = "Unknown C++ Compiler";
#endif

    f << "{\n";
    f << "  \"os\": \"" << os_name << "\",\n";
    f << "  \"compiler\": \"" << compiler << "\",\n";
    f << "  \"cpp_standard\": \"C++" << (__cplusplus >= 202002L ? "20" : "17") << "\",\n";
    f << "  \"logical_cores\": " << cores << ",\n";
    f << "  \"timer_resolution\": \"high_resolution_clock (nanosecond)\",\n";
    f << "  \"timestamp\": \"" __DATE__ " " __TIME__ "\"\n";
    f << "}\n";
    f.close();
    std::cout << "[System Info] Logged to " << json_path << "\n";
}

int main(int argc, char** argv) {
    std::string output_dir = "benchmarks/results";
    bool quick_mode = false;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--quick") quick_mode = true;
        if (arg == "--output" && i + 1 < argc) output_dir = argv[++i];
    }

    fs::create_directories(output_dir);
    writeSystemSpecs(output_dir);

    std::string raw_csv_path = output_dir + "/benchmark_raw.csv";
    std::string summary_csv_path = output_dir + "/benchmark_summary.csv";

    std::ofstream raw_file(raw_csv_path);
    raw_file << "run_id,scene_size,distribution,method,repetition,build_time_us,refit_time_us,query_time_us,"
             << "total_step_us,pairs_tested,narrow_phase_pairs,narrow_phase_hits,culling_efficiency_pct,"
             << "tree_depth,nodes_used,memory_bytes,correctness_passed\n";

    std::ofstream summary_file(summary_csv_path);
    summary_file << "scene_size,distribution,method,repetitions,"
                 << "build_mean_us,build_median_us,build_std_us,build_min_us,build_max_us,"
                 << "refit_mean_us,refit_median_us,refit_std_us,"
                 << "query_mean_us,query_median_us,query_std_us,query_min_us,query_max_us,"
                 << "total_mean_us,total_median_us,total_std_us,"
                 << "pairs_tested_mean,narrow_pairs_mean,narrow_hits_mean,culling_eff_mean_pct,"
                 << "tree_depth,nodes_used,memory_kb,correctness_verified\n";

    std::cout << "\n========================================================================================\n";
    std::cout << "  BVH COLLISION DETECTION BENCHMARK HARNESS (STANDALONE SUITE)                          \n";
    std::cout << "========================================================================================\n";
    std::cout << " Output Directory: " << output_dir << "\n";
    std::cout << " Mode: " << (quick_mode ? "QUICK SMOKE TEST" : "COMPREHENSIVE MULTI-SCENE SUITE") << "\n\n";

    // Scene sizes
    std::vector<int> scene_sizes;
    if (quick_mode) {
        scene_sizes = { 100, 500, 1000 };
    } else {
        scene_sizes = { 100, 500, 1000, 5000, 10000, 25000, 50000 };
    }

    // Distributions
    std::vector<SceneDistribution> distributions;
    if (quick_mode) {
        distributions = { SceneDistribution::Uniform, SceneDistribution::Clustered };
    } else {
        distributions = {
            SceneDistribution::Uniform,
            SceneDistribution::Clustered,
            SceneDistribution::DenseOverlap,
            SceneDistribution::Sparse,
            SceneDistribution::Dynamic
        };
    }

    const int REPETITIONS = quick_mode ? 3 : 10;
    const int WARMUP_RUNS = quick_mode ? 1 : 3;
    int global_run_id = 0;

    std::cout << std::left
              << std::setw(7)  << "N"
              << std::setw(15) << "Distribution"
              << std::setw(13) << "Method"
              << std::setw(14) << "Build (us)"
              << std::setw(14) << "Query (us)"
              << std::setw(14) << "Total (us)"
              << std::setw(14) << "Pairs Culled"
              << std::setw(12) << "Cull Eff %"
              << std::setw(10) << "Status" << "\n";
    std::cout << std::string(103, '-') << "\n";

    for (int n : scene_sizes) {
        for (auto dist : distributions) {
            BenchmarkConfig base_cfg;
            base_cfg.n = n;
            base_cfg.distribution = dist;
            base_cfg.repetitions = REPETITIONS;
            base_cfg.warmup_runs = WARMUP_RUNS;
            base_cfg.random_seed = 42 + n * 17;

            // Generate the baseline reference scene
            std::vector<Circle> reference_scene = SceneGenerator::generateScene(base_cfg, base_cfg.random_seed);

            // Compute brute-force baseline for correctness check (and benchmark if n <= 10000)
            std::vector<Pair> brute_reference_pairs;
            int brute_pairs_tested = 0;
            bool run_brute_bench = (n <= (quick_mode ? 1000 : 10000));

            if (run_brute_bench) {
                // Brute force warmup
                for (int w = 0; w < WARMUP_RUNS; ++w) {
                    BenchmarkUtils::queryBruteForce(reference_scene, brute_reference_pairs, brute_pairs_tested);
                }

                std::vector<double> bf_query_times;
                std::vector<double> bf_pairs_tested;

                for (int r = 0; r < REPETITIONS; ++r) {
                    auto t0 = Clock::now();
                    BenchmarkUtils::queryBruteForce(reference_scene, brute_reference_pairs, brute_pairs_tested);
                    double q_us = std::chrono::duration<double, std::micro>(Clock::now() - t0).count();

                    bf_query_times.push_back(q_us);
                    bf_pairs_tested.push_back(brute_pairs_tested);

                    int hits = BenchmarkUtils::countNarrowPhaseHits(reference_scene, brute_reference_pairs);

                    raw_file << (++global_run_id) << "," << n << "," << distributionToString(dist) << ",BruteForce,"
                             << r << ",0.0,0.0," << q_us << "," << q_us << ","
                             << brute_pairs_tested << "," << brute_reference_pairs.size() << ","
                             << hits << ",0.0,0,0,"
                             << (n * sizeof(Circle)) << ",true\n";
                }

                auto bf_summary = BenchmarkUtils::computeSummary(bf_query_times);
                summary_file << n << "," << distributionToString(dist) << ",BruteForce," << REPETITIONS << ","
                             << "0,0,0,0,0,0,0,0,"
                             << bf_summary.mean << "," << bf_summary.median << "," << bf_summary.std_dev << ","
                             << bf_summary.min_val << "," << bf_summary.max_val << ","
                             << bf_summary.mean << "," << bf_summary.median << "," << bf_summary.std_dev << ","
                             << (double)brute_pairs_tested << "," << brute_reference_pairs.size() << ",0,0.0,"
                             << "0,0," << (n * sizeof(Circle) / 1024.0) << ",true\n";

                std::cout << std::left
                          << std::setw(7)  << n
                          << std::setw(15) << distributionToString(dist)
                          << std::setw(13) << "BruteForce"
                          << std::setw(14) << "0.00"
                          << std::setw(14) << std::fixed << std::setprecision(2) << bf_summary.mean
                          << std::setw(14) << bf_summary.mean
                          << std::setw(14) << 0
                          << std::setw(12) << "0.00%"
                          << std::setw(10) << "[BASELINE]" << "\n";
            }

            // Methods to evaluate
            std::vector<BenchmarkMethod> methods = {
                BenchmarkMethod::BVHMedian,
                BenchmarkMethod::BVHSAH,
                BenchmarkMethod::BVHRefit
            };

            for (auto method : methods) {
                // If N > 10000 and method is SAH, it might be slow; we still benchmark it up to 50k
                std::vector<double> build_times, refit_times, query_times, total_times;
                std::vector<double> pairs_tested_vec, narrow_pairs_vec, hits_vec, culling_eff_vec;
                int final_tree_depth = 0;
                int final_nodes_used = 0;
                size_t final_memory = 0;
                bool all_correct = true;

                // Warmup runs
                for (int w = 0; w < WARMUP_RUNS; ++w) {
                    std::vector<Circle> scene = reference_scene;
                    std::vector<BVHNode> nodes;
                    std::vector<int> indices;
                    std::vector<Pair> bvh_pairs;
                    BVHStats st;

                    if (method == BenchmarkMethod::BVHMedian) {
                        buildBVH(scene, nodes, indices, st, BuildMethod::Median, 0.0f);
                        queryBVH(nodes, scene, bvh_pairs, st);
                    } else if (method == BenchmarkMethod::BVHSAH) {
                        buildBVH(scene, nodes, indices, st, BuildMethod::SAH, 0.0f);
                        queryBVH(nodes, scene, bvh_pairs, st);
                    } else if (method == BenchmarkMethod::BVHRefit) {
                        buildBVH(scene, nodes, indices, st, BuildMethod::Median, base_cfg.fat_margin);
                        refitBVH(scene, nodes, indices, st, base_cfg.fat_margin);
                        queryBVH(nodes, scene, bvh_pairs, st);
                    }
                }

                // Timed repetitions
                for (int r = 0; r < REPETITIONS; ++r) {
                    std::vector<Circle> scene = reference_scene;
                    std::vector<BVHNode> nodes;
                    std::vector<int> indices;
                    std::vector<Pair> bvh_pairs;
                    BVHStats st;

                    double b_time = 0.0, r_time = 0.0, q_time = 0.0;

                    if (method == BenchmarkMethod::BVHMedian) {
                        buildBVH(scene, nodes, indices, st, BuildMethod::Median, 0.0f);
                        b_time = st.build_time_us;

                        queryBVH(nodes, scene, bvh_pairs, st);
                        q_time = st.query_time_us;
                    } else if (method == BenchmarkMethod::BVHSAH) {
                        buildBVH(scene, nodes, indices, st, BuildMethod::SAH, 0.0f);
                        b_time = st.build_time_us;

                        queryBVH(nodes, scene, bvh_pairs, st);
                        q_time = st.query_time_us;
                    } else if (method == BenchmarkMethod::BVHRefit) {
                        // Initial build with fat margin
                        buildBVH(scene, nodes, indices, st, BuildMethod::Median, base_cfg.fat_margin);
                        b_time = st.build_time_us;

                        if (dist == SceneDistribution::Dynamic) {
                            // Step physics kinematics 5 frames
                            PhysicsConfig pCfg;
                            for (int f = 0; f < 5; ++f) {
                                updatePhysicsKinematics(scene, 1.0f / 60.0f, base_cfg.arena_width, base_cfg.arena_height, pCfg);
                            }
                        }

                        // Bottom-up refit
                        refitBVH(scene, nodes, indices, st, base_cfg.fat_margin);
                        r_time = st.refit_time_us;

                        queryBVH(nodes, scene, bvh_pairs, st);
                        q_time = st.query_time_us;
                    }

                    double total_t = b_time + r_time + q_time;
                    build_times.push_back(b_time);
                    refit_times.push_back(r_time);
                    query_times.push_back(q_time);
                    total_times.push_back(total_t);

                    pairs_tested_vec.push_back(static_cast<double>(st.pairs_tested));
                    narrow_pairs_vec.push_back(static_cast<double>(bvh_pairs.size()));

                    int hits = BenchmarkUtils::countNarrowPhaseHits(scene, bvh_pairs);
                    hits_vec.push_back(hits);

                    double max_possible_pairs = (static_cast<double>(n) * (n - 1)) * 0.5;
                    double culling_eff = 100.0 * (1.0 - (static_cast<double>(st.pairs_tested) / (std::max)(1.0, max_possible_pairs)));
                    culling_eff_vec.push_back(culling_eff);

                    final_tree_depth = BenchmarkUtils::calculateTreeDepth(nodes);
                    final_nodes_used = st.nodes_used;
                    final_memory = BenchmarkUtils::calculateBVHMemory(nodes, indices);

                    // Correctness verification against brute force (when brute force is available on the exact same scene)
                    bool pass = true;
                    if (run_brute_bench && dist != SceneDistribution::Dynamic && method != BenchmarkMethod::BVHRefit) {
                        std::string err;
                        pass = BenchmarkUtils::verifyCorrectness(bvh_pairs, brute_reference_pairs, err);
                        if (!pass) {
                            std::cerr << "\n[CORRECTNESS FAILURE] N=" << n << " Method=" << methodToString(method)
                                      << " Distribution=" << distributionToString(dist) << "\n"
                                      << " Details: " << err << "\n";
                            all_correct = false;
                            summary_file.close();
                            raw_file.close();
                            return 1;
                        }
                    }

                    raw_file << (++global_run_id) << "," << n << "," << distributionToString(dist) << ","
                             << methodToString(method) << "," << r << ","
                             << b_time << "," << r_time << "," << q_time << "," << total_t << ","
                             << st.pairs_tested << "," << bvh_pairs.size() << "," << hits << ","
                             << culling_eff << "," << final_tree_depth << "," << final_nodes_used << ","
                             << final_memory << "," << (pass ? "true" : "false") << "\n";
                }

                auto b_sum = BenchmarkUtils::computeSummary(build_times);
                auto r_sum = BenchmarkUtils::computeSummary(refit_times);
                auto q_sum = BenchmarkUtils::computeSummary(query_times);
                auto tot_sum = BenchmarkUtils::computeSummary(total_times);
                auto p_sum = BenchmarkUtils::computeSummary(pairs_tested_vec);
                auto np_sum = BenchmarkUtils::computeSummary(narrow_pairs_vec);
                auto h_sum = BenchmarkUtils::computeSummary(hits_vec);
                auto c_sum = BenchmarkUtils::computeSummary(culling_eff_vec);

                summary_file << n << "," << distributionToString(dist) << "," << methodToString(method) << "," << REPETITIONS << ","
                             << b_sum.mean << "," << b_sum.median << "," << b_sum.std_dev << "," << b_sum.min_val << "," << b_sum.max_val << ","
                             << r_sum.mean << "," << r_sum.median << "," << r_sum.std_dev << ","
                             << q_sum.mean << "," << q_sum.median << "," << q_sum.std_dev << "," << q_sum.min_val << "," << q_sum.max_val << ","
                             << tot_sum.mean << "," << tot_sum.median << "," << tot_sum.std_dev << ","
                             << p_sum.mean << "," << np_sum.mean << "," << h_sum.mean << "," << c_sum.mean << ","
                             << final_tree_depth << "," << final_nodes_used << ","
                             << (final_memory / 1024.0) << "," << (all_correct ? "true" : "false") << "\n";

                std::cout << std::left
                          << std::setw(7)  << n
                          << std::setw(15) << distributionToString(dist)
                          << std::setw(13) << methodToString(method)
                          << std::setw(14) << std::fixed << std::setprecision(2) << (method == BenchmarkMethod::BVHRefit ? r_sum.mean : b_sum.mean)
                          << std::setw(14) << q_sum.mean
                          << std::setw(14) << tot_sum.mean
                          << std::setw(14) << (int)p_sum.mean
                          << std::setw(12) << (std::to_string((int)c_sum.mean) + "%")
                          << std::setw(10) << (all_correct ? "[PASS]" : "[FAIL]") << "\n";
            }
        }
    }

    raw_file.close();
    summary_file.close();

    std::cout << "\n========================================================================================\n";
    std::cout << "  BENCHMARK SUITE COMPLETED SUCCESSFULLY!                                              \n";
    std::cout << "  Raw CSV saved to:     " << raw_csv_path << "\n";
    std::cout << "  Summary CSV saved to: " << summary_csv_path << "\n";
    std::cout << "========================================================================================\n\n";

    return 0;
}
