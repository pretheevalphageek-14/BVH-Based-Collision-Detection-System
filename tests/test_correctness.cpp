#include "collision.h"
#include "physics.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <string>
#include <cassert>
#include <cmath>
#include <iomanip>

struct TestCase {
    std::string name;
    std::vector<Circle> circles;
};

// Canonical Brute-force broad-phase pair generator for ground truth
static std::vector<Pair> bruteForcePairs(const std::vector<Circle>& circles) {
    std::vector<Pair> pairs;
    int n = static_cast<int>(circles.size());
    for (int i = 0; i < n; ++i) {
        AABB a = circleAABB(circles[i], 0.0f);
        for (int j = i + 1; j < n; ++j) {
            AABB b = circleAABB(circles[j], 0.0f);
            if (aabbOverlap(a, b)) {
                pairs.push_back({ i, j });
            }
        }
    }
    return pairs;
}

static void normalizePairs(std::vector<Pair>& pairs) {
    for (auto& p : pairs) {
        if (p.a > p.b) std::swap(p.a, p.b);
    }
    std::sort(pairs.begin(), pairs.end());
}

static bool comparePairs(const std::vector<Pair>& actual, const std::vector<Pair>& expected, std::string& err) {
    std::vector<Pair> a = actual;
    std::vector<Pair> b = expected;
    normalizePairs(a);
    normalizePairs(b);

    if (a.size() != b.size()) {
        err = "Pair count mismatch: actual " + std::to_string(a.size()) + " vs expected " + std::to_string(b.size());
        return false;
    }
    for (size_t i = 0; i < a.size(); ++i) {
        if (a[i].a != b[i].a || a[i].b != b[i].b) {
            err = "Mismatch at [" + std::to_string(i) + "]: actual (" +
                  std::to_string(a[i].a) + "," + std::to_string(a[i].b) + ") vs expected (" +
                  std::to_string(b[i].a) + "," + std::to_string(b[i].b) + ")";
            return false;
        }
    }
    return true;
}

static Circle makeC(float x, float y, float r) {
    Circle c{};
    c.x = x; c.y = y;
    c.vx = 0.0f; c.vy = 0.0f;
    c.r = r;
    c.mass = (r > 0.0f) ? (r * 0.2f) : 1.0f;
    c.invMass = 1.0f / c.mass;
    return c;
}

int main() {
    std::cout << "===================================================================\n";
    std::cout << "  BVH CORRECTNESS TEST SUITE & BASELINE VERIFICATION               \n";
    std::cout << "===================================================================\n";

    std::vector<TestCase> testCases;

    // 1. Empty Scene
    testCases.push_back({ "empty_scene", {} });

    // 2. Single Object
    testCases.push_back({ "single_object", { makeC(100.0f, 100.0f, 10.0f) } });

    // 3. Touching Boxes
    {
        std::vector<Circle> c;
        // X-axis touching: circle 0 at x=10 r=5 (bounds [5, 15]), circle 1 at x=20 r=5 (bounds [15, 25]) -> touch at x=15
        c.push_back(makeC(10.0f, 50.0f, 5.0f));
        c.push_back(makeC(20.0f, 50.0f, 5.0f));
        // Y-axis touching: circle 2 at y=10 r=5, circle 3 at y=20 r=5 -> touch at y=15
        c.push_back(makeC(100.0f, 10.0f, 5.0f));
        c.push_back(makeC(100.0f, 20.0f, 5.0f));
        // Corner touching: circle 4 at (200, 200) r=5 ([195, 205]), circle 5 at (210, 210) r=5 ([205, 215]) -> touch at (205, 205)
        c.push_back(makeC(200.0f, 200.0f, 5.0f));
        c.push_back(makeC(210.0f, 210.0f, 5.0f));
        // Separated by small epsilon (no touch): circle 6 at (300, 300) r=5, circle 7 at (310.01f, 300) r=5 -> disjoint
        c.push_back(makeC(300.0f, 300.0f, 5.0f));
        c.push_back(makeC(310.01f, 300.0f, 5.0f));
        testCases.push_back({ "touching_boxes", c });
    }

    // 4. Identical Positions
    {
        std::vector<Circle> c;
        // 5 concentric circles at exactly (50, 50) with varying radii
        c.push_back(makeC(50.0f, 50.0f, 2.0f));
        c.push_back(makeC(50.0f, 50.0f, 5.0f));
        c.push_back(makeC(50.0f, 50.0f, 8.0f));
        c.push_back(makeC(50.0f, 50.0f, 10.0f));
        c.push_back(makeC(50.0f, 50.0f, 12.0f));
        // 5 identical circles at exactly (200, 200) with identical radius
        for (int i = 0; i < 5; ++i) {
            c.push_back(makeC(200.0f, 200.0f, 15.0f));
        }
        testCases.push_back({ "identical_positions", c });
    }

    // 5. Zero-Size Objects
    {
        std::vector<Circle> c;
        // Concentric zero-size circles (overlap)
        c.push_back(makeC(10.0f, 10.0f, 0.0f));
        c.push_back(makeC(10.0f, 10.0f, 0.0f));
        // Zero-size circle touching standard circle boundary (circle at 20,10 r=10 has bounds [10, 30] in X)
        c.push_back(makeC(20.0f, 10.0f, 10.0f));
        // Disjoint zero-size circle
        c.push_back(makeC(100.0f, 100.0f, 0.0f));
        // Two disjoint zero-size circles
        c.push_back(makeC(200.0f, 200.0f, 0.0f));
        c.push_back(makeC(200.0f, 205.0f, 0.0f));
        testCases.push_back({ "zero_size_objects", c });
    }

    // 6. Many Overlapping Objects (Dense Cluster)
    {
        std::vector<Circle> c;
        // 30 circles tightly packed in a 20x20 box, all overlapping
        for (int i = 0; i < 30; ++i) {
            float ox = (i % 6) * 3.0f;
            float oy = (i / 6) * 3.0f;
            c.push_back(makeC(100.0f + ox, 100.0f + oy, 15.0f));
        }
        testCases.push_back({ "many_overlapping_objects", c });
    }

    // 7. Regular Grid Layout (Touching and Disjoint Neighbors)
    {
        std::vector<Circle> c;
        // 6x6 grid with spacing 20, radius 10 (each touching its 4 orthogonal neighbors)
        for (int y = 0; y < 6; ++y) {
            for (int x = 0; x < 6; ++x) {
                c.push_back(makeC(50.0f + x * 20.0f, 50.0f + y * 20.0f, 10.0f));
            }
        }
        testCases.push_back({ "grid_touching_neighbors", c });
    }

    std::ofstream baselineOut("tests/baseline_test_pairs.txt");
    bool all_passed = true;

    for (const auto& tc : testCases) {
        std::cout << "\n------------------------------------------------------------\n";
        std::cout << " RUNNING TEST SCENE: " << tc.name << " (N = " << tc.circles.size() << ")\n";
        std::cout << "------------------------------------------------------------\n";

        // 1. Ground truth via brute force
        std::vector<Pair> groundTruth = bruteForcePairs(tc.circles);
        normalizePairs(groundTruth);

        baselineOut << "SCENE:" << tc.name << " COUNT:" << groundTruth.size() << "\n";
        for (const auto& p : groundTruth) {
            baselineOut << p.a << "," << p.b << "\n";
        }

        std::cout << " Ground Truth Pairs: " << groundTruth.size() << "\n";

        // 2. Test BVH Median
        {
            std::vector<BVHNode> nodes;
            std::vector<int> indices;
            BVHStats stats;
            std::vector<Pair> bvhPairs;

            buildBVH(tc.circles, nodes, indices, stats, BuildMethod::Median, 0.0f);
            queryBVH(nodes, tc.circles, bvhPairs, stats);
            normalizePairs(bvhPairs);

            std::string err;
            bool ok = comparePairs(bvhPairs, groundTruth, err);
            if (!ok) {
                std::cout << " [FAIL] BVH-Median: " << err << "\n";
                all_passed = false;
            } else {
                std::cout << " [PASS] BVH-Median: Exact match (" << bvhPairs.size() << " pairs)\n";
            }
        }

        // 3. Test BVH SAH
        {
            std::vector<BVHNode> nodes;
            std::vector<int> indices;
            BVHStats stats;
            std::vector<Pair> bvhPairs;

            buildBVH(tc.circles, nodes, indices, stats, BuildMethod::SAH, 0.0f);
            queryBVH(nodes, tc.circles, bvhPairs, stats);
            normalizePairs(bvhPairs);

            std::string err;
            bool ok = comparePairs(bvhPairs, groundTruth, err);
            if (!ok) {
                std::cout << " [FAIL] BVH-SAH: " << err << "\n";
                all_passed = false;
            } else {
                std::cout << " [PASS] BVH-SAH: Exact match (" << bvhPairs.size() << " pairs)\n";
            }
        }

        // 4. Test BVH Refit (Initial build with fat margin, then refit)
        {
            std::vector<BVHNode> nodes;
            std::vector<int> indices;
            BVHStats stats;
            std::vector<Pair> bvhPairs;

            buildBVH(tc.circles, nodes, indices, stats, BuildMethod::Median, 3.0f);
            refitBVH(tc.circles, nodes, indices, stats, 3.0f);
            queryBVH(nodes, tc.circles, bvhPairs, stats);
            normalizePairs(bvhPairs);

            std::string err;
            bool ok = comparePairs(bvhPairs, groundTruth, err);
            if (!ok) {
                std::cout << " [FAIL] BVH-Refit: " << err << "\n";
                all_passed = false;
            } else {
                std::cout << " [PASS] BVH-Refit: Exact match (" << bvhPairs.size() << " pairs)\n";
            }
        }
    }

    baselineOut.close();
    std::cout << "\n===================================================================\n";
    if (all_passed) {
        std::cout << " ALL CORRECTNESS TESTS PASSED SUCCESSFULLY!                        \n";
        std::cout << " Baseline saved to tests/baseline_test_pairs.txt                  \n";
    } else {
        std::cout << " SOME TESTS FAILED!                                                \n";
    }
    std::cout << "===================================================================\n";

    return all_passed ? 0 : 1;
}
