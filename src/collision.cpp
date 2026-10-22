#include "collision.h"
#include <algorithm>
#include <chrono>
#include <cassert>
#include <vector>
#include <cmath>
#include <cfloat>

using Clock = std::chrono::high_resolution_clock;

static int buildNodeMedian(const std::vector<Circle>& circles,
                           std::vector<int>& indices,
                           int start, int end,
                           std::vector<BVHNode>& nodes,
                           float fat_margin) {
    int node_idx = (int)nodes.size();
    nodes.emplace_back();
    BVHNode& node = nodes.back();

    float minx = circles[indices[start]].x - circles[indices[start]].r - fat_margin;
    float miny = circles[indices[start]].y - circles[indices[start]].r - fat_margin;
    float maxx = circles[indices[start]].x + circles[indices[start]].r + fat_margin;
    float maxy = circles[indices[start]].y + circles[indices[start]].r + fat_margin;

    for (int i = start + 1; i < end; ++i) {
        const Circle& c = circles[indices[i]];
        minx = std::min(minx, c.x - c.r - fat_margin);
        miny = std::min(miny, c.y - c.r - fat_margin);
        maxx = std::max(maxx, c.x + c.r + fat_margin);
        maxy = std::max(maxy, c.y + c.r + fat_margin);
    }
    node.bounds.min[0] = minx;
    node.bounds.min[1] = miny;
    node.bounds.max[0] = maxx;
    node.bounds.max[1] = maxy;

    int count = end - start;
    if (count == 1) {
        node.object_index = indices[start];
        return node_idx;
    }

    float cminx = circles[indices[start]].x;
    float cminy = circles[indices[start]].y;
    float cmaxx = circles[indices[start]].x;
    float cmaxy = circles[indices[start]].y;
    for (int i = start + 1; i < end; ++i) {
        const Circle& c = circles[indices[i]];
        cminx = std::min(cminx, c.x);
        cminy = std::min(cminy, c.y);
        cmaxx = std::max(cmaxx, c.x);
        cmaxy = std::max(cmaxy, c.y);
    }
    float extent_x = cmaxx - cminx;
    float extent_y = cmaxy - cminy;
    int axis = (extent_x >= extent_y) ? 0 : 1;

    int mid = start + count / 2;
    std::nth_element(indices.begin() + start, indices.begin() + mid, indices.begin() + end,
        [&](int a, int b) {
            return (axis == 0) ? circles[a].x < circles[b].x : circles[a].y < circles[b].y;
        });

    int left_child = buildNodeMedian(circles, indices, start, mid, nodes, fat_margin);
    int right_child = buildNodeMedian(circles, indices, mid, end, nodes, fat_margin);
    nodes[node_idx].left = left_child;
    nodes[node_idx].right = right_child;
    return node_idx;
}

struct SAHBin {
    int count = 0;
    AABB bounds{{FLT_MAX, FLT_MAX}, {-FLT_MAX, -FLT_MAX}};
};

static int buildNodeSAH(const std::vector<Circle>& circles,
                        std::vector<int>& indices,
                        int start, int end,
                        std::vector<BVHNode>& nodes,
                        float fat_margin) {
    int node_idx = (int)nodes.size();
    nodes.emplace_back();
    BVHNode& node = nodes.back();

    float minx = circles[indices[start]].x - circles[indices[start]].r - fat_margin;
    float miny = circles[indices[start]].y - circles[indices[start]].r - fat_margin;
    float maxx = circles[indices[start]].x + circles[indices[start]].r + fat_margin;
    float maxy = circles[indices[start]].y + circles[indices[start]].r + fat_margin;

    for (int i = start + 1; i < end; ++i) {
        const Circle& c = circles[indices[i]];
        minx = std::min(minx, c.x - c.r - fat_margin);
        miny = std::min(miny, c.y - c.r - fat_margin);
        maxx = std::max(maxx, c.x + c.r + fat_margin);
        maxy = std::max(maxy, c.y + c.r + fat_margin);
    }
    node.bounds.min[0] = minx;
    node.bounds.min[1] = miny;
    node.bounds.max[0] = maxx;
    node.bounds.max[1] = maxy;

    int count = end - start;
    if (count <= 4) {
        if (count == 1) {
            node.object_index = indices[start];
        } else {
            float cminx = circles[indices[start]].x;
            float cminy = circles[indices[start]].y;
            float cmaxx = circles[indices[start]].x;
            float cmaxy = circles[indices[start]].y;
            for (int i = start + 1; i < end; ++i) {
                const Circle& c = circles[indices[i]];
                cminx = std::min(cminx, c.x);
                cminy = std::min(cminy, c.y);
                cmaxx = std::max(cmaxx, c.x);
                cmaxy = std::max(cmaxy, c.y);
            }
            float extent_x = cmaxx - cminx;
            float extent_y = cmaxy - cminy;
            int axis = (extent_x >= extent_y) ? 0 : 1;
            int mid = start + count / 2;
            std::nth_element(indices.begin() + start, indices.begin() + mid, indices.begin() + end,
                [&](int a, int b) {
                    return (axis == 0) ? circles[a].x < circles[b].x : circles[a].y < circles[b].y;
                });
            int left_child = buildNodeSAH(circles, indices, start, mid, nodes, fat_margin);
            int right_child = buildNodeSAH(circles, indices, mid, end, nodes, fat_margin);
            nodes[node_idx].left = left_child;
            nodes[node_idx].right = right_child;
        }
        return node_idx;
    }

    float cminx = circles[indices[start]].x;
    float cminy = circles[indices[start]].y;
    float cmaxx = circles[indices[start]].x;
    float cmaxy = circles[indices[start]].y;
    for (int i = start + 1; i < end; ++i) {
        const Circle& c = circles[indices[i]];
        cminx = std::min(cminx, c.x);
        cminy = std::min(cminy, c.y);
        cmaxx = std::max(cmaxx, c.x);
        cmaxy = std::max(cmaxy, c.y);
    }

    constexpr int NUM_BINS = 12;
    float best_cost = FLT_MAX;
    int best_axis = 0;
    int best_bin = -1;

    for (int axis = 0; axis < 2; ++axis) {
        float cmin = (axis == 0) ? cminx : cminy;
        float cmax = (axis == 0) ? cmaxx : cmaxy;
        float bin_width = (cmax - cmin) / NUM_BINS;
        if (bin_width <= 0) continue;

        SAHBin bins[NUM_BINS];

        for (int i = start; i < end; ++i) {
            const Circle& c = circles[indices[i]];
            float centroid = (axis == 0) ? c.x : c.y;
            int bin = (int)((centroid - cmin) / bin_width);
            if (bin >= NUM_BINS) bin = NUM_BINS - 1;
            if (bin < 0) bin = 0;

            bins[bin].count++;
            bins[bin].bounds.min[0] = std::min(bins[bin].bounds.min[0], c.x - c.r - fat_margin);
            bins[bin].bounds.min[1] = std::min(bins[bin].bounds.min[1], c.y - c.r - fat_margin);
            bins[bin].bounds.max[0] = std::max(bins[bin].bounds.max[0], c.x + c.r + fat_margin);
            bins[bin].bounds.max[1] = std::max(bins[bin].bounds.max[1], c.y + c.r + fat_margin);
        }

        int left_count[NUM_BINS];
        AABB left_bounds[NUM_BINS];
        int running_count = 0;
        AABB running_bounds{{FLT_MAX, FLT_MAX}, {-FLT_MAX, -FLT_MAX}};
        for (int i = 0; i < NUM_BINS; ++i) {
            running_count += bins[i].count;
            left_count[i] = running_count;
            running_bounds.min[0] = std::min(running_bounds.min[0], bins[i].bounds.min[0]);
            running_bounds.min[1] = std::min(running_bounds.min[1], bins[i].bounds.min[1]);
            running_bounds.max[0] = std::max(running_bounds.max[0], bins[i].bounds.max[0]);
            running_bounds.max[1] = std::max(running_bounds.max[1], bins[i].bounds.max[1]);
            left_bounds[i] = running_bounds;
        }

        int right_count[NUM_BINS];
        AABB right_bounds[NUM_BINS];
        running_count = 0;
        running_bounds = {{FLT_MAX, FLT_MAX}, {-FLT_MAX, -FLT_MAX}};
        for (int i = NUM_BINS - 1; i >= 0; --i) {
            running_count += bins[i].count;
            right_count[i] = running_count;
            running_bounds.min[0] = std::min(running_bounds.min[0], bins[i].bounds.min[0]);
            running_bounds.min[1] = std::min(running_bounds.min[1], bins[i].bounds.min[1]);
            running_bounds.max[0] = std::max(running_bounds.max[0], bins[i].bounds.max[0]);
            running_bounds.max[1] = std::max(running_bounds.max[1], bins[i].bounds.max[1]);
            right_bounds[i] = running_bounds;
        }

        float parent_sa = aabbSurfaceArea(node.bounds);
        if (parent_sa > 1e-7f) {
            for (int i = 0; i < NUM_BINS - 1; ++i) {
                if (left_count[i] == 0 || right_count[i + 1] == 0) continue;
                float cost = 1.0f +
                    (aabbSurfaceArea(left_bounds[i]) / parent_sa) * left_count[i] +
                    (aabbSurfaceArea(right_bounds[i + 1]) / parent_sa) * right_count[i + 1];
                if (cost < best_cost) {
                    best_cost = cost;
                    best_axis = axis;
                    best_bin = i;
                }
            }
        }
    }

    if (best_bin >= 0) {
        float cmin = (best_axis == 0) ? cminx : cminy;
        float cmax = (best_axis == 0) ? cmaxx : cmaxy;
        float bin_width = (cmax - cmin) / NUM_BINS;

        int mid = start;
        for (int i = start; i < end; ++i) {
            const Circle& c = circles[indices[i]];
            float centroid = (best_axis == 0) ? c.x : c.y;
            int bin = (int)((centroid - cmin) / bin_width);
            if (bin >= NUM_BINS) bin = NUM_BINS - 1;
            if (bin <= best_bin) {
                std::swap(indices[i], indices[mid]);
                ++mid;
            }
        }
        if (mid == start || mid == end) {
            best_bin = -1;
        } else {
            int left_child = buildNodeSAH(circles, indices, start, mid, nodes, fat_margin);
            int right_child = buildNodeSAH(circles, indices, mid, end, nodes, fat_margin);
            nodes[node_idx].left = left_child;
            nodes[node_idx].right = right_child;
            return node_idx;
        }
    }

    float extent_x = cmaxx - cminx;
    float extent_y = cmaxy - cminy;
    int axis = (extent_x >= extent_y) ? 0 : 1;
    int mid = start + count / 2;
    std::nth_element(indices.begin() + start, indices.begin() + mid, indices.begin() + end,
        [&](int a, int b) {
            return (axis == 0) ? circles[a].x < circles[b].x : circles[a].y < circles[b].y;
        });

    int left_child = buildNodeSAH(circles, indices, start, mid, nodes, fat_margin);
    int right_child = buildNodeSAH(circles, indices, mid, end, nodes, fat_margin);
    nodes[node_idx].left = left_child;
    nodes[node_idx].right = right_child;
    return node_idx;
}

void buildBVH(const std::vector<Circle>& circles,
              std::vector<BVHNode>& nodes,
              std::vector<int>& object_indices,
              BVHStats& stats,
              BuildMethod method,
              float fat_margin) {
    auto t0 = Clock::now();

    nodes.clear();
    nodes.reserve(circles.size() * 2);
    object_indices.resize(circles.size());
    for (size_t i = 0; i < circles.size(); ++i) object_indices[i] = (int)i;

    if (!circles.empty()) {
        if (method == BuildMethod::Median) {
            buildNodeMedian(circles, object_indices, 0, (int)circles.size(), nodes, fat_margin);
        } else {
            buildNodeSAH(circles, object_indices, 0, (int)circles.size(), nodes, fat_margin);
        }
    }

    stats.nodes_used = (int)nodes.size();
    stats.build_time_us = std::chrono::duration<double, std::micro>(Clock::now() - t0).count();
}

static int refitNode(const std::vector<Circle>& circles,
                     std::vector<BVHNode>& nodes,
                     const std::vector<int>& object_indices,
                     int node_idx,
                     float fat_margin) {
    BVHNode& node = nodes[node_idx];

    if (node.object_index >= 0) {
        const Circle& c = circles[node.object_index];
        node.bounds.min[0] = c.x - c.r - fat_margin;
        node.bounds.min[1] = c.y - c.r - fat_margin;
        node.bounds.max[0] = c.x + c.r + fat_margin;
        node.bounds.max[1] = c.y + c.r + fat_margin;
        return node_idx;
    }

    int left_idx = node.left;
    int right_idx = node.right;

    refitNode(circles, nodes, object_indices, left_idx, fat_margin);
    refitNode(circles, nodes, object_indices, right_idx, fat_margin);

    const BVHNode& left = nodes[left_idx];
    const BVHNode& right = nodes[right_idx];

    node.bounds.min[0] = std::min(left.bounds.min[0], right.bounds.min[0]);
    node.bounds.min[1] = std::min(left.bounds.min[1], right.bounds.min[1]);
    node.bounds.max[0] = std::max(left.bounds.max[0], right.bounds.max[0]);
    node.bounds.max[1] = std::max(left.bounds.max[1], right.bounds.max[1]);

    return node_idx;
}

void refitBVH(const std::vector<Circle>& circles,
              std::vector<BVHNode>& nodes,
              const std::vector<int>& object_indices,
              BVHStats& stats,
              float fat_margin) {
    auto t0 = Clock::now();

    if (!nodes.empty()) {
        refitNode(circles, nodes, object_indices, 0, fat_margin);
    }

    stats.refit_time_us = std::chrono::duration<double, std::micro>(Clock::now() - t0).count();
}

bool needsRebuild(const std::vector<Circle>& circles,
                  const std::vector<BVHNode>& nodes,
                  const std::vector<int>& object_indices,
                  float fat_margin) {
    (void)fat_margin;
    if (nodes.empty() || object_indices.size() != circles.size())
        return true;

    // Check if any circle's tight AABB has breached its leaf node's fat bounds
    for (const auto& node : nodes) {
        if (node.object_index >= 0) {
            int idx = node.object_index;
            if (idx >= (int)circles.size()) return true;
            AABB tight = circleAABB(circles[idx], 0.0f);
            if (tight.min[0] < node.bounds.min[0] || tight.max[0] > node.bounds.max[0] ||
                tight.min[1] < node.bounds.min[1] || tight.max[1] > node.bounds.max[1]) {
                return true;
            }
        }
    }
    return false;
}

static void queryNodeSelf(int node_idx,
                          const std::vector<BVHNode>& nodes,
                          const std::vector<Circle>& circles,
                          std::vector<Pair>& out_pairs,
                          int& pairs_tested) {
    struct StackEntry { int a, b; };
    std::vector<StackEntry> stack;
    stack.reserve(256);
    stack.push_back({node_idx, node_idx});

    while (!stack.empty()) {
        auto entry = stack.back();
        stack.pop_back();
        int a = entry.a, b = entry.b;

        if (a < 0 || b < 0) continue;

        const BVHNode& na = nodes[a];
        const BVHNode& nb = nodes[b];

        if (!aabbOverlap(na.bounds, nb.bounds)) continue;
        pairs_tested++;

        bool leaf_a = (na.object_index >= 0);
        bool leaf_b = (nb.object_index >= 0);

        if (leaf_a && leaf_b) {
            if (na.object_index != nb.object_index) {
                int ia = na.object_index, ib = nb.object_index;
                if (ia > ib) std::swap(ia, ib);
                // Exact tight-AABB test: fat leaf boxes are only a broad
                // phase filter to cull distant candidates.
                if (aabbOverlap(circleAABB(circles[ia]), circleAABB(circles[ib])))
                    out_pairs.push_back({ia, ib});
            }
        } else if (leaf_a && !leaf_b) {
            stack.push_back({a, nb.left});
            stack.push_back({a, nb.right});
        } else if (!leaf_a && leaf_b) {
            stack.push_back({na.left, b});
            stack.push_back({na.right, b});
        } else {
            if (a == b) {
                // Self-test: cross pairs + recurse into BOTH subtrees,
                // otherwise pairs sharing a subtree are never tested.
                stack.push_back({na.left, na.right});
                stack.push_back({na.left, na.left});
                stack.push_back({na.right, na.right});
            } else {
                stack.push_back({na.left, nb.left});
                stack.push_back({na.left, nb.right});
                stack.push_back({na.right, nb.left});
                stack.push_back({na.right, nb.right});
            }
        }
    }
}

void queryBVH(const std::vector<BVHNode>& nodes,
              const std::vector<Circle>& circles,
              std::vector<Pair>& out_pairs,
              BVHStats& stats) {
    auto t0 = Clock::now();
    out_pairs.clear();
    stats.pairs_tested = 0;

    if (nodes.empty()) {
        stats.query_time_us = std::chrono::duration<double, std::micro>(Clock::now() - t0).count();
        stats.collisions_found = 0;
        return;
    }

    queryNodeSelf(0, nodes, circles, out_pairs, stats.pairs_tested);
    stats.collisions_found = (int)out_pairs.size();
    stats.query_time_us = std::chrono::duration<double, std::micro>(Clock::now() - t0).count();
}

static void collectBoundsRecursive(const std::vector<BVHNode>& nodes,
                                   int node_idx,
                                   int depth,
                                   int max_depth,
                                   std::vector<TreeBounds>& out_bounds) {
    if (node_idx < 0) return;
    if (max_depth >= 0 && depth > max_depth) return;

    const BVHNode& node = nodes[node_idx];
    out_bounds.push_back({node.bounds, depth, node.object_index >= 0});

    if (node.object_index < 0) {
        collectBoundsRecursive(nodes, node.left, depth + 1, max_depth, out_bounds);
        collectBoundsRecursive(nodes, node.right, depth + 1, max_depth, out_bounds);
    }
}

void collectTreeBounds(const std::vector<BVHNode>& nodes,
                       std::vector<TreeBounds>& out_bounds,
                       int max_depth) {
    out_bounds.clear();
    if (!nodes.empty()) {
        collectBoundsRecursive(nodes, 0, 0, max_depth, out_bounds);
    }
}