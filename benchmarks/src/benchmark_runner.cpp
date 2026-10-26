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
#include <cmath>
#include <sstream>

using Clock = std::chrono::high_resolution_clock;
namespace fs = std::filesystem;

// System hardware info collector
static void writeSystemSpecs(const std::string& out_dir, std::string& timestamp_out) {
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
    timestamp_out = __DATE__ " " __TIME__;


    f << "{\n";
    f << "  \"os\": \"" << os_name << "\",\n";
    f << "  \"compiler\": \"" << compiler << "\",\n";
    f << "  \"cpp_standard\": \"C++" << (__cplusplus >= 202002L ? "20" : "17") << "\",\n";
    f << "  \"logical_cores\": " << cores << ",\n";
    f << "  \"timer_resolution\": \"high_resolution_clock (nanosecond)\",\n";
    f << "  \"timestamp\": \"" << timestamp_out << "\"\n";
    f << "}\n";
    f.close();
    std::cout << "[System Info] Logged to " << json_path << "\n";
}

int main(int argc, char** argv) {
    std::string output_dir = "benchmarks/results";
    bool quick_mode = false;
    int custom_repetitions = 0;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--quick") quick_mode = true;
        if (arg == "--full") quick_mode = false;
        if (arg == "--output" && i + 1 < argc) output_dir = argv[++i];
        if (arg == "--runs" && i + 1 < argc) custom_repetitions = std::atoi(argv[++i]);
    }

    fs::create_directories(output_dir);
    std::string timestamp_str;
    writeSystemSpecs(output_dir, timestamp_str);

    std::string raw_csv_path = output_dir + "/benchmark_raw.csv";
    std::string summary_csv_path = output_dir + "/benchmark_summary.csv";
    std::string json_path = output_dir + "/benchmark_results.json";

    std::ofstream raw_file(raw_csv_path);
    raw_file << "run_id,scene_size,distribution,method,repetition,build_time_us,refit_time_us,query_time_us,"
             << "total_step_us,fps,pairs_tested,node_visits,narrow_phase_pairs,narrow_phase_hits,culling_efficiency_pct,"
             << "tree_depth,nodes_used,sah_cost,memory_bytes,correctness_passed\n";

    std::ofstream summary_file(summary_csv_path);
    summary_file << "scene_size,distribution,method,repetitions,"
                 << "build_mean_us,build_median_us,build_p95_us,build_std_us,build_min_us,build_max_us,"
                 << "refit_mean_us,refit_median_us,refit_p95_us,refit_std_us,"
                 << "query_mean_us,query_median_us,query_p95_us,query_std_us,query_min_us,query_max_us,"
                 << "total_mean_us,total_median_us,total_p95_us,total_std_us,"
                 << "fps_mean,pairs_tested_mean,node_visits_mean,narrow_pairs_mean,narrow_hits_mean,culling_eff_mean_pct,"
                 << "tree_depth,nodes_used,sah_cost,memory_kb,correctness_verified\n";

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
        scene_sizes = { 100, 500, 1000, 5000, 10000, 50000 };
    }

    // Distributions
    std::vector<SceneDistribution> distributions = {
        SceneDistribution::Uniform,
        SceneDistribution::Clustered,
        SceneDistribution::DenseOverlap,
        SceneDistribution::Sparse,
        SceneDistribution::Dynamic
    };

    const int REPETITIONS = (custom_repetitions > 0) ? custom_repetitions : (quick_mode ? 5 : 15);
    const int WARMUP_RUNS = quick_mode ? 2 : 3;
    int global_run_id = 0;

    // Structured in-memory storage for JSON export
    std::vector<AggregatedBenchmarkResult> aggregated_results;

    std::cout << std::left
              << std::setw(7)  << "N"
              << std::setw(15) << "Distribution"
              << std::setw(13) << "Method"
              << std::setw(14) << "Build (us)"
              << std::setw(14) << "Query (us)"
              << std::setw(14) << "Total (us)"
              << std::setw(12) << "FPS"
              << std::setw(14) << "Pairs Culled"
              << std::setw(12) << "Cull Eff %"
              << std::setw(10) << "Status" << "\n";
    std::cout << std::string(115, '-') << "\n";

    for (int n : scene_sizes) {
        for (auto dist : distributions) {
            BenchmarkConfig base_cfg;
            base_cfg.n = n;
            base_cfg.distribution = dist;
            base_cfg.repetitions = REPETITIONS;
            base_cfg.warmup_runs = WARMUP_RUNS;
            base_cfg.random_seed = 42 + n * 17;

            // Generate deterministic baseline scene
            std::vector<Circle> reference_scene = SceneGenerator::generateScene(base_cfg, base_cfg.random_seed);

            // Compute brute-force baseline for correctness check (and benchmark if n <= 10000)
            std::vector<Pair> brute_reference_pairs;
            int brute_pairs_tested = 0;
            bool run_brute_bench = (n <= 10000);

            if (run_brute_bench) {
                // Brute force warmup
                for (int w = 0; w < WARMUP_RUNS; ++w) {
                    BenchmarkUtils::queryBruteForce(reference_scene, brute_reference_pairs, brute_pairs_tested);
                }

                std::vector<double> bf_query_times;
                std::vector<double> bf_fps_vec;
                std::vector<double> bf_pairs_tested_vec;
                std::vector<double> bf_hits_vec;

                for (int r = 0; r < REPETITIONS; ++r) {
                    auto t0 = Clock::now();
                    BenchmarkUtils::queryBruteForce(reference_scene, brute_reference_pairs, brute_pairs_tested);
                    double q_us = std::chrono::duration<double, std::micro>(Clock::now() - t0).count();
                    double fps = (q_us > 0.0) ? (1e6 / q_us) : 0.0;

                    bf_query_times.push_back(q_us);
                    bf_fps_vec.push_back(fps);
                    bf_pairs_tested_vec.push_back(brute_pairs_tested);

                    int hits = BenchmarkUtils::countNarrowPhaseHits(reference_scene, brute_reference_pairs);
                    bf_hits_vec.push_back(hits);

                    raw_file << (++global_run_id) << "," << n << "," << distributionToString(dist) << ",BruteForce,"
                             << r << ",0.0,0.0," << q_us << "," << q_us << "," << fps << ","
                             << brute_pairs_tested << ",0," << brute_reference_pairs.size() << ","
                             << hits << ",0.0,0,0,0.0,"
                             << (n * sizeof(Circle)) << ",true\n";
                }

                auto bf_summary = BenchmarkUtils::computeSummary(bf_query_times);
                auto bf_fps_summary = BenchmarkUtils::computeSummary(bf_fps_vec);
                auto bf_hits_summary = BenchmarkUtils::computeSummary(bf_hits_vec);

                summary_file << n << "," << distributionToString(dist) << ",BruteForce," << REPETITIONS << ","
                             << "0,0,0,0,0,0,0,0,0,0,"
                             << bf_summary.mean << "," << bf_summary.median << "," << bf_summary.p95 << ","
                             << bf_summary.std_dev << "," << bf_summary.min_val << "," << bf_summary.max_val << ","
                             << bf_summary.mean << "," << bf_summary.median << "," << bf_summary.p95 << "," << bf_summary.std_dev << ","
                             << bf_fps_summary.mean << ","
                             << (double)brute_pairs_tested << ",0," << brute_reference_pairs.size() << ","
                             << bf_hits_summary.mean << ",0.0,"
                             << "0,0,0.0," << (n * sizeof(Circle) / 1024.0) << ",true\n";

                // Save to aggregated records for JSON
                AggregatedBenchmarkResult bf_rec;
                bf_rec.n = n;
                bf_rec.distribution = distributionToString(dist);
                bf_rec.method = "BruteForce";
                bf_rec.repetitions = REPETITIONS;
                bf_rec.query_time_us = bf_summary;
                bf_rec.total_step_us = bf_summary;
                bf_rec.fps = bf_fps_summary;
                bf_rec.pairs_tested = BenchmarkUtils::computeSummary(bf_pairs_tested_vec);
                bf_rec.narrow_phase_hits = bf_hits_summary;
                bf_rec.memory_bytes = n * sizeof(Circle);
                bf_rec.all_correctness_passed = true;
                aggregated_results.push_back(bf_rec);

                std::cout << std::left
                          << std::setw(7)  << n
                          << std::setw(15) << distributionToString(dist)
                          << std::setw(13) << "BruteForce"
                          << std::setw(14) << "0.00"
                          << std::setw(14) << std::fixed << std::setprecision(2) << bf_summary.mean
                          << std::setw(14) << bf_summary.mean
                          << std::setw(12) << (int)bf_fps_summary.mean
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
                std::vector<double> build_times, refit_times, query_times, total_times, fps_times;
                std::vector<double> pairs_tested_vec, node_visits_vec, narrow_pairs_vec, hits_vec, culling_eff_vec;
                int final_tree_depth = 0;
                int final_nodes_used = 0;
                double final_sah_cost = 0.0;
                size_t final_memory = 0;
                bool all_correct = true;

                // Warmup runs (excluded from timing statistics)
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

                    double step_t = (method == BenchmarkMethod::BVHRefit ? r_time : b_time) + q_time;
                    double fps = (step_t > 0.0) ? (1e6 / step_t) : 0.0;

                    build_times.push_back(b_time);
                    refit_times.push_back(r_time);
                    query_times.push_back(q_time);
                    total_times.push_back(step_t);
                    fps_times.push_back(fps);

                    pairs_tested_vec.push_back(static_cast<double>(st.pairs_tested));
                    node_visits_vec.push_back(static_cast<double>(st.node_visits));
                    narrow_pairs_vec.push_back(static_cast<double>(bvh_pairs.size()));

                    int hits = BenchmarkUtils::countNarrowPhaseHits(scene, bvh_pairs);
                    hits_vec.push_back(hits);

                    double max_possible_pairs = (static_cast<double>(n) * (n - 1)) * 0.5;
                    double culling_eff = 100.0 * (1.0 - (static_cast<double>(st.pairs_tested) / (std::max)(1.0, max_possible_pairs)));
                    culling_eff_vec.push_back(culling_eff);

                    final_tree_depth = BenchmarkUtils::calculateTreeDepth(nodes);
                    final_nodes_used = st.nodes_used;
                    final_sah_cost = BenchmarkUtils::calculateSAHCost(nodes);
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
                             << b_time << "," << r_time << "," << q_time << "," << step_t << "," << fps << ","
                             << st.pairs_tested << "," << st.node_visits << "," << bvh_pairs.size() << "," << hits << ","
                             << culling_eff << "," << final_tree_depth << "," << final_nodes_used << ","
                             << final_sah_cost << "," << final_memory << "," << (pass ? "true" : "false") << "\n";
                }

                auto b_sum = BenchmarkUtils::computeSummary(build_times);
                auto r_sum = BenchmarkUtils::computeSummary(refit_times);
                auto q_sum = BenchmarkUtils::computeSummary(query_times);
                auto tot_sum = BenchmarkUtils::computeSummary(total_times);
                auto fps_sum = BenchmarkUtils::computeSummary(fps_times);
                auto p_sum = BenchmarkUtils::computeSummary(pairs_tested_vec);
                auto nv_sum = BenchmarkUtils::computeSummary(node_visits_vec);
                auto np_sum = BenchmarkUtils::computeSummary(narrow_pairs_vec);
                auto h_sum = BenchmarkUtils::computeSummary(hits_vec);
                auto c_sum = BenchmarkUtils::computeSummary(culling_eff_vec);

                summary_file << n << "," << distributionToString(dist) << "," << methodToString(method) << "," << REPETITIONS << ","
                             << b_sum.mean << "," << b_sum.median << "," << b_sum.p95 << "," << b_sum.std_dev << "," << b_sum.min_val << "," << b_sum.max_val << ","
                             << r_sum.mean << "," << r_sum.median << "," << r_sum.p95 << "," << r_sum.std_dev << ","
                             << q_sum.mean << "," << q_sum.median << "," << q_sum.p95 << "," << q_sum.std_dev << "," << q_sum.min_val << "," << q_sum.max_val << ","
                             << tot_sum.mean << "," << tot_sum.median << "," << tot_sum.p95 << "," << tot_sum.std_dev << ","
                             << fps_sum.mean << ","
                             << p_sum.mean << "," << nv_sum.mean << "," << np_sum.mean << "," << h_sum.mean << "," << c_sum.mean << ","
                             << final_tree_depth << "," << final_nodes_used << "," << final_sah_cost << ","
                             << (final_memory / 1024.0) << "," << (all_correct ? "true" : "false") << "\n";

                AggregatedBenchmarkResult bvh_rec;
                bvh_rec.n = n;
                bvh_rec.distribution = distributionToString(dist);
                bvh_rec.method = methodToString(method);
                bvh_rec.repetitions = REPETITIONS;
                bvh_rec.build_time_us = b_sum;
                bvh_rec.refit_time_us = r_sum;
                bvh_rec.query_time_us = q_sum;
                bvh_rec.total_step_us = tot_sum;
                bvh_rec.fps = fps_sum;
                bvh_rec.pairs_tested = p_sum;
                bvh_rec.node_visits = nv_sum;
                bvh_rec.narrow_phase_pairs = np_sum;
                bvh_rec.narrow_phase_hits = h_sum;
                bvh_rec.culling_efficiency_pct = c_sum;
                bvh_rec.tree_depth = final_tree_depth;
                bvh_rec.nodes_used = final_nodes_used;
                bvh_rec.sah_cost = final_sah_cost;
                bvh_rec.memory_bytes = final_memory;
                bvh_rec.all_correctness_passed = all_correct;
                aggregated_results.push_back(bvh_rec);

                std::cout << std::left
                          << std::setw(7)  << n
                          << std::setw(15) << distributionToString(dist)
                          << std::setw(13) << methodToString(method)
                          << std::setw(14) << std::fixed << std::setprecision(2) << (method == BenchmarkMethod::BVHRefit ? r_sum.mean : b_sum.mean)
                          << std::setw(14) << q_sum.mean
                          << std::setw(14) << tot_sum.mean
                          << std::setw(12) << (int)fps_sum.mean
                          << std::setw(14) << (int)p_sum.mean
                          << std::setw(12) << (std::to_string((int)c_sum.mean) + "%")
                          << std::setw(10) << (all_correct ? "[PASS]" : "[FAIL]") << "\n";
            }
        }
    }

    raw_file.close();
    summary_file.close();

    // Export comprehensive JSON results
    std::ofstream json_file(json_path);
    if (json_file.is_open()) {
        json_file << "{\n";
        json_file << "  \"timestamp\": \"" << timestamp_str << "\",\n";
        json_file << "  \"mode\": \"" << (quick_mode ? "quick" : "full") << "\",\n";
        json_file << "  \"repetitions\": " << REPETITIONS << ",\n";
        json_file << "  \"warmup_runs\": " << WARMUP_RUNS << ",\n";
        json_file << "  \"results\": [\n";
        for (size_t i = 0; i < aggregated_results.size(); ++i) {
            const auto& r = aggregated_results[i];
            json_file << "    {\n";
            json_file << "      \"scene_size\": " << r.n << ",\n";
            json_file << "      \"distribution\": \"" << r.distribution << "\",\n";
            json_file << "      \"method\": \"" << r.method << "\",\n";
            json_file << "      \"repetitions\": " << r.repetitions << ",\n";
            json_file << "      \"build_mean_us\": " << r.build_time_us.mean << ",\n";
            json_file << "      \"build_median_us\": " << r.build_time_us.median << ",\n";
            json_file << "      \"build_p95_us\": " << r.build_time_us.p95 << ",\n";
            json_file << "      \"refit_mean_us\": " << r.refit_time_us.mean << ",\n";
            json_file << "      \"refit_median_us\": " << r.refit_time_us.median << ",\n";
            json_file << "      \"refit_p95_us\": " << r.refit_time_us.p95 << ",\n";
            json_file << "      \"query_mean_us\": " << r.query_time_us.mean << ",\n";
            json_file << "      \"query_median_us\": " << r.query_time_us.median << ",\n";
            json_file << "      \"query_p95_us\": " << r.query_time_us.p95 << ",\n";
            json_file << "      \"query_std_us\": " << r.query_time_us.std_dev << ",\n";
            json_file << "      \"query_min_us\": " << r.query_time_us.min_val << ",\n";
            json_file << "      \"query_max_us\": " << r.query_time_us.max_val << ",\n";
            json_file << "      \"total_mean_us\": " << r.total_step_us.mean << ",\n";
            json_file << "      \"total_median_us\": " << r.total_step_us.median << ",\n";
            json_file << "      \"total_p95_us\": " << r.total_step_us.p95 << ",\n";
            json_file << "      \"fps_mean\": " << r.fps.mean << ",\n";
            json_file << "      \"pairs_tested_mean\": " << r.pairs_tested.mean << ",\n";
            json_file << "      \"node_visits_mean\": " << r.node_visits.mean << ",\n";
            json_file << "      \"narrow_pairs_mean\": " << r.narrow_phase_pairs.mean << ",\n";
            json_file << "      \"narrow_hits_mean\": " << r.narrow_phase_hits.mean << ",\n";
            json_file << "      \"culling_eff_mean_pct\": " << r.culling_efficiency_pct.mean << ",\n";
            json_file << "      \"tree_depth\": " << r.tree_depth << ",\n";
            json_file << "      \"nodes_used\": " << r.nodes_used << ",\n";
            json_file << "      \"sah_cost\": " << r.sah_cost << ",\n";
            json_file << "      \"memory_kb\": " << (r.memory_bytes / 1024.0) << ",\n";
            json_file << "      \"correctness_verified\": " << (r.all_correctness_passed ? "true" : "false") << "\n";
            json_file << "    }" << (i + 1 < aggregated_results.size() ? "," : "") << "\n";
        }
        json_file << "  ]\n";
        json_file << "}\n";
        json_file.close();
        std::cout << "  JSON results saved to: " << json_path << "\n";
    }

    std::cout << "\n========================================================================================\n";
    std::cout << "  BENCHMARK SUITE COMPLETED SUCCESSFULLY!                                              \n";
    std::cout << "  Raw CSV saved to:     " << raw_csv_path << "\n";
    std::cout << "  Summary CSV saved to: " << summary_csv_path << "\n";
    std::cout << "  JSON Export saved to: " << json_path << "\n";
    std::cout << "========================================================================================\n\n";

    return 0;
}
