#pragma once

#include <vector>
#include <cstdint>

struct Circle {
    float x, y;
    float vx, vy;
    float r;
    uint8_t r_col, g_col, b_col;
    uint8_t flash_timer = 0; // Frames remaining for collision highlight ring
    float mass = 1.0f;
    float invMass = 1.0f;
    float restitution = 1.0f;
    float staticFriction = 0.3f;
    float dynamicFriction = 0.2f;
};

struct AABB {
    float min[2];
    float max[2];
};

struct BVHNode {
    AABB bounds;
    int left = -1;
    int right = -1;
    int object_index = -1;
};

struct Pair {
    int a, b;
    bool operator<(const Pair& other) const {
        return a < other.a || (a == other.a && b < other.b);
    }
    bool operator==(const Pair& other) const {
        return a == other.a && b == other.b;
    }
};

enum class BuildMethod { Median, SAH };

struct BVHStats {
    int nodes_used = 0;
    int pairs_tested = 0;
    int collisions_found = 0;
    double build_time_us = 0.0;
    double query_time_us = 0.0;
    double refit_time_us = 0.0;
    int rebuilds_triggered = 0;
};

inline bool aabbOverlap(const AABB& a, const AABB& b) {
    return a.min[0] <= b.max[0] && a.max[0] >= b.min[0] &&
           a.min[1] <= b.max[1] && a.max[1] >= b.min[1];
}

inline AABB circleAABB(const Circle& c, float fat_margin = 0.0f) {
    return {{c.x - c.r - fat_margin, c.y - c.r - fat_margin},
            {c.x + c.r + fat_margin, c.y + c.r + fat_margin}};
}

inline float aabbSurfaceArea(const AABB& a) {
    float w = std::max(0.0f, a.max[0] - a.min[0]);
    float h = std::max(0.0f, a.max[1] - a.min[1]);
    return 2.0f * (w + h); // Perimeter in 2D = surface area for SAH
}

// Tree visualization data
struct TreeBounds {
    AABB bounds;
    int depth;
    bool leaf;
};

void buildBVH(const std::vector<Circle>& circles,
              std::vector<BVHNode>& nodes,
              std::vector<int>& object_indices,
              BVHStats& stats,
              BuildMethod method = BuildMethod::Median,
              float fat_margin = 0.0f);

void refitBVH(const std::vector<Circle>& circles,
              std::vector<BVHNode>& nodes,
              const std::vector<int>& object_indices,
              BVHStats& stats,
              float fat_margin = 0.0f);

bool needsRebuild(const std::vector<Circle>& circles,
                  const std::vector<BVHNode>& nodes,
                  const std::vector<int>& object_indices,
                  float fat_margin);

void queryBVH(const std::vector<BVHNode>& nodes,
              const std::vector<Circle>& circles,
              std::vector<Pair>& out_pairs,
              BVHStats& stats);

void collectTreeBounds(const std::vector<BVHNode>& nodes,
                       std::vector<TreeBounds>& out_bounds,
                       int max_depth = -1);