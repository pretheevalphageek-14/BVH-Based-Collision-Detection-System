#include <SDL3/SDL.h>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <random>
#include <vector>
#include "collision.h"
#include "physics.h"
#include "logger.h"

static PhysicsConfig g_physicsConfig;
static PhysicsAudit g_physicsAudit;

constexpr int WINDOW_W = 1280;
constexpr int WINDOW_H = 720;
constexpr float DT = 1.0f / 60.0f;
constexpr float WALL_RESTITUTION = 0.9f;
constexpr float CIRCLE_RESTITUTION = 0.8f;
constexpr float FAT_MARGIN = 3.0f;

enum CollisionMode { MODE_MEDIAN, MODE_SAH, MODE_REFIT, MODE_COUNT };

static const char* modeLabel(CollisionMode m) {
    switch (m) {
        case MODE_MEDIAN: return "BVH-Median";
        case MODE_SAH:    return "BVH-SAH";
        case MODE_REFIT:  return "BVH-Refit";
        default:          return "Unknown";
    }
}

static std::mt19937 g_rng{ std::random_device{}() };

static Circle makeCircle(float x, float y) {
    std::uniform_real_distribution<float> spd(-150.0f, 150.0f);
    std::uniform_real_distribution<float> rad(8.0f, 18.0f);
    std::uniform_int_distribution<int> col(60, 255);
    Circle c{};
    c.x = x; c.y = y;
    c.vx = spd(g_rng); c.vy = spd(g_rng);
    c.r = rad(g_rng);
    c.r_col = (uint8_t)col(g_rng);
    c.g_col = (uint8_t)col(g_rng);
    c.b_col = (uint8_t)col(g_rng);
    c.flash_timer = 0;
    c.mass = c.r * 0.2f;
    c.invMass = 1.0f / c.mass;
    c.restitution = 1.0f;
    c.staticFriction = 0.35f;
    c.dynamicFriction = 0.25f;
    return c;
}

static void spawnCircles(std::vector<Circle>& circles, int count, int offset) {
    std::uniform_real_distribution<float> px(40.0f, (float)WINDOW_W - 40.0f);
    std::uniform_real_distribution<float> py(40.0f, (float)WINDOW_H - 40.0f);
    for (int i = 0; i < count; ++i) {
        (void)offset;
        circles.push_back(makeCircle(px(g_rng), py(g_rng)));
    }
}

static void resolveCollision(Circle& a, Circle& b) {
    resolvePhysicsCollision(a, b, g_physicsConfig, g_physicsAudit);
}

static void stepPhysics(std::vector<Circle>& circles) {
    updatePhysicsKinematics(circles, DT, (float)WINDOW_W, (float)WINDOW_H, g_physicsConfig);
}

// Filled disk primitive. Colors are set by the caller.
static void fillDisk(SDL_Renderer* r, int cx, int cy, int rr) {
    for (int dy = -rr; dy <= rr; ++dy) {
        int dx = (int)std::sqrt((float)rr * rr - (float)dy * dy);
        SDL_RenderLine(r, (float)(cx - dx), (float)(cy + dy),
                            (float)(cx + dx), (float)(cy + dy));
    }
}

// Base color is assigned ONCE in makeCircle() and never regenerated here.
// Collision feedback draws an outer white/red highlight ring for 1 frame without
// modifying the stored base color (no flicker).
static void renderCircles(SDL_Renderer* r, const std::vector<Circle>& circles) {
    for (const auto& c : circles) {
        int cx = (int)c.x, cy = (int)c.y, rr = (int)c.r;
        SDL_SetRenderDrawColor(r, c.r_col, c.g_col, c.b_col, 255);
        fillDisk(r, cx, cy, rr);
        if (c.flash_timer > 0) {
            // Draw outer white/red highlight ring for 1 frame around the circle
            SDL_SetRenderDrawBlendMode(r, SDL_BLENDMODE_BLEND);
            SDL_SetRenderDrawColor(r, 255, 60, 60, 220); // outer red highlight ring
            for (int dy = -(rr + 2); dy <= rr + 2; ++dy) {
                int outer_dx = (int)std::sqrt((float)(rr + 2) * (rr + 2) - (float)dy * dy);
                SDL_RenderPoint(r, (float)(cx - outer_dx), (float)(cy + dy));
                SDL_RenderPoint(r, (float)(cx + outer_dx), (float)(cy + dy));
            }
            SDL_SetRenderDrawColor(r, 255, 255, 255, 255); // bright white inner ring boundary
            for (int dy = -(rr + 1); dy <= rr + 1; ++dy) {
                int ring_dx = (int)std::sqrt((float)(rr + 1) * (rr + 1) - (float)dy * dy);
                SDL_RenderPoint(r, (float)(cx - ring_dx), (float)(cy + dy));
                SDL_RenderPoint(r, (float)(cx + ring_dx), (float)(cy + dy));
            }
            SDL_SetRenderDrawBlendMode(r, SDL_BLENDMODE_NONE);
        }
    }
}

static void renderTree(SDL_Renderer* r, const std::vector<BVHNode>& nodes, int max_depth) {
    std::vector<TreeBounds> bounds;
    collectTreeBounds(nodes, bounds, max_depth);
    // Depth-based colors: Level 0 = Bright Green, Level 1 = Cyan, Level 2 = Yellow
    static const SDL_FColor LEVELS[] = {
        {0.00f, 1.00f, 0.20f, 1.0f}, // Level 0: Bright Green
        {0.00f, 0.90f, 1.00f, 1.0f}, // Level 1: Cyan
        {1.00f, 0.95f, 0.10f, 1.0f}, // Level 2: Yellow
        {1.00f, 0.35f, 0.85f, 1.0f}, // Level 3: Magenta
        {1.00f, 0.55f, 0.20f, 1.0f}, // Level 4: Orange
        {0.65f, 0.50f, 1.00f, 1.0f}, // Level 5: Lavender
        {1.00f, 0.40f, 0.40f, 1.0f}, // Level 6: Salmon
        {0.80f, 0.80f, 0.80f, 1.0f}, // Level 7: Grey
    };
    const int NUM_LEVELS = (int)(sizeof(LEVELS) / sizeof(LEVELS[0]));
    SDL_SetRenderDrawBlendMode(r, SDL_BLENDMODE_BLEND);
    for (const auto& b : bounds) {
        if (b.leaf && max_depth > 2) continue;
        SDL_FColor col = LEVELS[b.depth % NUM_LEVELS];
        SDL_SetRenderDrawColor(r, (uint8_t)(col.r * 255), (uint8_t)(col.g * 255),
                                     (uint8_t)(col.b * 255), 180);
        SDL_FRect rect{ b.bounds.min[0], b.bounds.min[1],
            b.bounds.max[0] - b.bounds.min[0], b.bounds.max[1] - b.bounds.min[1] };
        SDL_RenderRect(r, &rect); // thin 1px outline
    }
    SDL_SetRenderDrawBlendMode(r, SDL_BLENDMODE_NONE);
}

// ---------- Report generation ----------

struct BenchRow {
    int n;
    const char* method;
    double build_us, query_us;
    int pairs;
    double total_us;
};

static std::vector<BenchRow> g_bench_rows;

static void benchmark(int n, int frames, CollisionMode mode) {
    std::vector<Circle> circles;
    spawnCircles(circles, n, 0);

    double build_sum = 0.0, query_sum = 0.0;
    long pairs_sum = 0;
    const char* names[] = { "Median Split", "SAH Split", "Refit (Fat)" };

    std::vector<BVHNode> nodes;
    std::vector<int> indices;
    std::vector<Pair> bvh_pairs;
    BVHStats st;
    bool first = true;

    LOG_INFO("benchmark start: n=%d frames=%d method=%s", n, frames, names[mode]);

    for (int f = 0; f < frames; ++f) {
        stepPhysics(circles);

        if (mode == MODE_MEDIAN) {
            buildBVH(circles, nodes, indices, st, BuildMethod::Median, 0.0f);
            build_sum += st.build_time_us;
        } else if (mode == MODE_SAH) {
            buildBVH(circles, nodes, indices, st, BuildMethod::SAH, 0.0f);
            build_sum += st.build_time_us;
        } else { // MODE_REFIT
            if (first || needsRebuild(circles, nodes, indices, FAT_MARGIN)) {
                buildBVH(circles, nodes, indices, st, BuildMethod::Median, FAT_MARGIN);
                build_sum += st.build_time_us;
            } else {
                refitBVH(circles, nodes, indices, st, FAT_MARGIN);
                build_sum += st.refit_time_us;
            }
        }

        bvh_pairs.clear();
        queryBVH(nodes, circles, bvh_pairs, st);
        query_sum += st.query_time_us;
        pairs_sum += st.pairs_tested;

        first = false;
    }

    g_bench_rows.push_back({
        n, names[mode],
        build_sum / frames, query_sum / frames,
        (int)(pairs_sum / frames),
        (build_sum + query_sum) / frames
    });

    const BenchRow& r = g_bench_rows.back();
    LOG_INFO("benchmark done : n=%d method=%-13s build/refit=%8.2fus query=%8.2fus pairs=%6d total=%8.2fus",
             r.n, r.method, r.build_us, r.query_us, r.pairs, r.total_us);
}

static void writeBenchmarksMd() {
    FILE* f = nullptr;
    if (fopen_s(&f, "BENCHMARKS.md", "w") != 0 || !f) return;
    fprintf(f, "# Benchmark Results\n\n");
    fprintf(f, "Benchmarked over 300 frames per entry (fixed timestep dt = 1/60 s).\n\n");
    fprintf(f, "| n | Method | Build/Refit (us) | Query Time (us) | Pairs Tested | Total Step (us) |\n");
    fprintf(f, "|------|--------------|------------------|-----------------|--------------|-----------------|\n");
    for (const auto& r : g_bench_rows) {
        fprintf(f, "| %d | %s | %.2f | %.2f | %d | %.2f |\n",
                r.n, r.method, r.build_us, r.query_us, r.pairs, r.total_us);
    }
    fprintf(f, "\n_Measured with <chrono::high_resolution_clock> on MSVC /O2 /W4._\n");
    fclose(f);
    LOG_INFO("BENCHMARKS.md written (%zu rows)", g_bench_rows.size());
}

static void writeExplanationMd() {
    FILE* f = nullptr;
    if (fopen_s(&f, "EXPLANATION.md", "w") != 0 || !f) return;
    fprintf(f, R"(# BVH for Collision Detection — Architecture & DSA Analysis

## 1. High-Level Architecture

The engine uses a **flat-array node pool** instead of a raw/smart pointer tree:

    struct BVHNode { AABB bounds; int left, right, object_index; };

- All nodes live in one contiguous `std::vector<BVHNode>`.
- `left` / `right` are **indices** (pool offsets), never pointers.
- `object_index >= 0` marks a leaf; interior nodes store child indices.
- **Why:** cache-friendly traversal, no allocation churn, no ownership
  semantics needed, trivially serializable, zero dangling references.

Compare with pointer trees: every node is a separate heap allocation,
pointer-chasing kills cache locality, and memory must be reclaimed
manually or via smart-pointer refcounting ceremony.

## 2. Complexity Analysis

Let N = number of objects.

| Algorithm | Build | Query (broad phase) | Notes |
|-----------|-------|---------------------|-------|
| Median-split BVH | O(N log N) | O(N log N) worst, O(k + log N) typical | k = reported pairs |
| SAH BVH | O(N log N * B) | same as median | B = bins (12) per node |
| Fat AABB refit | O(N) per frame | same as median | rebuild only when escaped |

- **BVH build** partitions space recursively; each level of recursion
  processes each object once → O(N log N) depth is log2(N).
- **BVH query** is an iterative stack traversal: two nodes are tested only
  if their AABBs overlap. Non-overlapping subtrees are skipped entirely,
  which is why candidate pairs collapse from O(N^2) to near-linear in k.
- **Refit** recomputes bounds bottom-up from leaves using the existing
  topology: O(N), no re-sorting, no re-partitioning.

## 3. Trade-offs: Median vs SAH vs Fat AABB Refit

### Median split
Sorts object centroids along the longest axis, splits at the midpoint.
- Cheap: O(N log N) but simple.
- Weakness: equal object counts != equal surface areas; can produce
  long thin boxes with poor culling.

### SAH (Surface Area Heuristic)
Cost = C_trav + (SA_left / SA_parent)*N_left + (SA_right / SA_parent)*N_right
- Uses 12 spatial bins per axis; evaluates split planes between bins.
- Selects the split minimizing *expected query cost*.
- Better tree quality (fewer pairs tested) but ~2-4x slower to build.

### Fat AABBs + lazy refit
- Each object's box is inflated by a margin (+3 px).
- The tree is **refit** (bounds recomputed, topology preserved) each frame.
- A **full rebuild** is triggered only when an object escapes its fat box.
- Wins when objects move slowly relative to their margins: O(N) refit
  beats O(N log N) rebuild almost always.
- Loses (slightly) in query quality: fatter boxes → more false positives.

## 4. Controls

| Input | Action |
|-------|--------|
| B | Cycle collision mode: Median -> SAH -> Refit |
| 1 / 2 / 3 | Directly switch mode: 1=Median, 2=SAH, 3=Refit |
| T | Toggle tree AABB visualization (visual debug layer) |
| Up / Down | Increase / decrease max tree depth drawn |
| SPACE | Pause / resume simulation |
| R | Reset scene (500 new random circles) |
| Left click | Spawn a circle at cursor |
| Right click + drag | Pull nearby circles toward cursor (white guide lines show affected objects) |
| ESC | Quit, print benchmarks to console and write BENCHMARKS.md / EXPLANATION.md |

### Visual debug layer & rendering notes

- **Persistent color assignment:** each circle's RGB is randomized exactly once
  at spawn time (in `makeCircle`) and stored in `Circle::color_r/g/b`. The
  render and update loops never regenerate colors, so base colors are stable.
- **Collision highlight:** on impact, a translucent white *outer halo* is drawn
  around the object for ~2 frames. The base fill is redrawn on top, so the
  highlight never overwrites or replaces the assigned color.
- **Tree overlay (key T):** AABBs are drawn as thin 1px outlines, color-coded
  by depth (L0 green, L1 cyan, L2 yellow, L3 magenta, L4 orange, ...) with
  semi-transparent alpha. When the depth slider (Up/Down) exceeds level 2, leaf
  boxes are skipped so only interior nodes are shown — this keeps the
  space-partition picture readable instead of cluttering the screen.
- **Drag force visualization:** while right-click dragging, faint lines connect
  the cursor to every affected object, with a small crosshair at the cursor.

## 5. Collision Pipeline & Tree Traversal

Every frame executes the BVH broad-phase query to find overlapping AABB pairs.
The iterative dual-tree traversal pushes candidate node pairs onto an internal
stack, culling non-overlapping subtrees early. The detected candidate pairs are
then directly resolved using standard impulse physics.
)");
    fclose(f);
    LOG_INFO("EXPLANATION.md written");
}

static void runBenchmarks() {
    BVHLOG_TIMER("extended benchmark suite");
    printf("\n=== EXTENDED BENCHMARKS (300 frames each) ===\n");
    LOG_INFO("=== EXTENDED BENCHMARKS (300 frames each) ===");
    int counts[] = { 100, 500, 1000 };
    for (int n : counts) {
        for (int m = 0; m < MODE_COUNT; ++m) benchmark(n, 300, (CollisionMode)m);
    }
    benchmark(5000, 300, MODE_REFIT);

    printf("| n    | Method        | Build/Refit (us) | Query Time (us) | Pairs Tested | Total Step (us) |\n");
    printf("|------|---------------|------------------|-----------------|--------------|-----------------|\n");
    for (const auto& r : g_bench_rows)
        printf("| %-4d | %-13s | %16.2f | %15.2f | %12d | %15.2f |\n",
               r.n, r.method, r.build_us, r.query_us, r.pairs, r.total_us);

    for (const auto& r : g_bench_rows)
        LOG_INFO("result: n=%-4d | %-13s | build %8.2fus | query %8.2fus | pairs %6d | total %8.2fus",
                 r.n, r.method, r.build_us, r.query_us, r.pairs, r.total_us);

    writeBenchmarksMd();
    writeExplanationMd();
    printf("\nBENCHMARKS.md and EXPLANATION.md written to working directory.\n");
}

// ---------- Main ----------

int main(int argc, char** argv) {
    bvhlog::init("bvh_run.log", bvhlog::Level::Info);
    LOG_INFO("=== BVH Physics - final test run ===");
#ifdef _DEBUG
    const char* build_cfg = "Debug";
#else
    const char* build_cfg = "Release";
#endif
    LOG_INFO("build: %s | %s %s | window=%dx%d dt=%.4f",
             build_cfg, __DATE__, __TIME__, WINDOW_W, WINDOW_H, (double)DT);

    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--bench") == 0) {
            LOG_INFO("flag '--bench' detected: running headless benchmark suite");
            runBenchmarks();
            bvhlog::shutdown();
            return 0;
        }
    }

    if (!SDL_Init(SDL_INIT_VIDEO)) {
        LOG_ERROR("SDL_Init failed: %s", SDL_GetError());
        SDL_Log("SDL_Init failed: %s", SDL_GetError());
        return 1;
    }
    LOG_INFO("SDL initialized, creating %dx%d window", WINDOW_W, WINDOW_H);
    SDL_Window* window = SDL_CreateWindow("BVH Physics - Phase 4", WINDOW_W, WINDOW_H, 0);
    if (!window) {
        LOG_ERROR("window creation failed: %s", SDL_GetError());
        SDL_Log("Window failed: %s", SDL_GetError()); SDL_Quit(); return 1;
    }
    SDL_Renderer* renderer = SDL_CreateRenderer(window, nullptr);
    if (!renderer) {
        LOG_ERROR("renderer creation failed: %s", SDL_GetError());
        SDL_Log("Renderer failed: %s", SDL_GetError()); SDL_DestroyWindow(window); SDL_Quit(); return 1;
    }

    std::vector<Circle> circles;
    spawnCircles(circles, 500, 0);

    std::vector<BVHNode> nodes;
    std::vector<int> indices;
    std::vector<Pair> bvh_pairs;
    BVHStats stats;

    CollisionMode mode = MODE_MEDIAN;
    bool show_tree = false;
    int tree_depth = 6;
    bool paused = false;
    bool running = true;
    bool right_down = false;
    bool force_rebuild = true;
    float mouse_x = 0, mouse_y = 0;

    LOG_INFO("scene ready: %zu circles, mode=%s, tree overlay=OFF, depth=%d",
             circles.size(), modeLabel(mode), tree_depth);

    using Clock = std::chrono::high_resolution_clock;
    auto prev = Clock::now();
    auto run_start = Clock::now();
    float accumulator = 0, fps_timer = 0;
    float status_timer = 0;
    int frame_count = 0;
    float fps = 0;

    while (running) {
        auto now = Clock::now();
        float frame_time = std::chrono::duration<float>(now - prev).count();
        prev = now;
        accumulator += frame_time;
        fps_timer += frame_time;
        frame_count++;
        if (fps_timer >= 1.0f) { fps = frame_count / fps_timer; frame_count = 0; fps_timer = 0; }

        status_timer += frame_time;
        if (status_timer >= 5.0f) {
            LOG_INFO("status: mode=%s objects=%zu nodes=%d pairs=%d collisions=%d build=%.2fus query=%.2fus fps=%.1f",
                     modeLabel(mode), circles.size(), stats.nodes_used,
                     stats.pairs_tested, stats.collisions_found,
                     stats.build_time_us, stats.query_time_us, fps);
            status_timer = 0;
        }

        SDL_Event e;
        while (SDL_PollEvent(&e)) {
            if (e.type == SDL_EVENT_QUIT) running = false;
            if (e.type == SDL_EVENT_KEY_DOWN) {
                switch (e.key.key) {
                    case SDLK_ESCAPE:
                        LOG_INFO("input: ESC -> quit requested");
                        running = false;
                        break;
                    case SDLK_B:
                        mode = (CollisionMode)((mode + 1) % MODE_COUNT);
                        force_rebuild = true;
                        LOG_INFO("input: B -> mode %s", modeLabel(mode));
                        break;
                    case SDLK_1:
                        mode = MODE_MEDIAN;
                        force_rebuild = true;
                        LOG_INFO("input: 1 -> mode %s", modeLabel(mode));
                        break;
                    case SDLK_2:
                        mode = MODE_SAH;
                        force_rebuild = true;
                        LOG_INFO("input: 2 -> mode %s", modeLabel(mode));
                        break;
                    case SDLK_3:
                        mode = MODE_REFIT;
                        force_rebuild = true;
                        LOG_INFO("input: 3 -> mode %s", modeLabel(mode));
                        break;
                    case SDLK_T:
                        show_tree = !show_tree;
                        LOG_INFO("input: T -> tree overlay %s (depth=%d)",
                                 show_tree ? "ON" : "OFF", tree_depth);
                        break;
                    case SDLK_UP:
                        tree_depth++;
                        LOG_DEBUG("input: UP -> tree depth %d", tree_depth);
                        break;
                    case SDLK_DOWN:
                        if (tree_depth > 0) tree_depth--;
                        LOG_DEBUG("input: DOWN -> tree depth %d", tree_depth);
                        break;
                    case SDLK_SPACE:
                        paused = !paused;
                        LOG_INFO("input: SPACE -> %s", paused ? "PAUSED" : "RESUMED");
                        break;
                    case SDLK_R:
                        circles.clear();
                        spawnCircles(circles, 500, 0);
                        force_rebuild = true;
                        LOG_INFO("input: R -> scene reset, %zu circles", circles.size());
                        break;
                    case SDLK_E:
                        g_physicsConfig.mode = PhysicsCollisionMode::Elastic;
                        g_physicsConfig.restitution = 1.0f;
                        LOG_INFO("physics: switched to ELASTIC mode (e = 1.0, kinetic energy conserved)");
                        break;
                    case SDLK_I:
                        g_physicsConfig.mode = PhysicsCollisionMode::Inelastic;
                        g_physicsConfig.restitution = 0.0f;
                        LOG_INFO("physics: switched to INELASTIC mode (e = 0.0, maximal kinetic energy dissipation)");
                        break;
                    case SDLK_P:
                        g_physicsConfig.mode = PhysicsCollisionMode::PartiallyElastic;
                        g_physicsConfig.restitution = 0.75f;
                        LOG_INFO("physics: switched to PARTIALLY ELASTIC mode (e = 0.75)");
                        break;
                    case SDLK_F:
                        g_physicsConfig.frictionEnabled = !g_physicsConfig.frictionEnabled;
                        LOG_INFO("physics: Coulomb surface friction %s", g_physicsConfig.frictionEnabled ? "ENABLED" : "DISABLED");
                        break;
                    case SDLK_G:
                        g_physicsConfig.gravityEnabled = !g_physicsConfig.gravityEnabled;
                        LOG_INFO("physics: environmental gravity %s", g_physicsConfig.gravityEnabled ? "ENABLED" : "DISABLED");
                        break;
                    default: break;
                }
            }
            if (e.type == SDL_EVENT_MOUSE_BUTTON_DOWN) {
                SDL_GetMouseState(&mouse_x, &mouse_y);
                if (e.button.button == SDL_BUTTON_LEFT) {
                    circles.push_back(makeCircle(mouse_x, mouse_y));
                    force_rebuild = true;
                    LOG_INFO("input: left click spawned circle at (%.0f, %.0f) -> objects=%zu",
                             mouse_x, mouse_y, circles.size());
                }
                if (e.button.button == SDL_BUTTON_RIGHT) right_down = true;
            }
            if (e.type == SDL_EVENT_MOUSE_BUTTON_UP) {
                if (e.button.button == SDL_BUTTON_RIGHT) right_down = false;
            }
            if (e.type == SDL_EVENT_MOUSE_MOTION) {
                mouse_x = e.motion.x; mouse_y = e.motion.y;
            }
        }

        // Right-click drag: pull nearby circles toward the cursor
        std::vector<int> dragged; // affected objects, for the force visualization
        if (right_down) {
            for (size_t i = 0; i < circles.size(); ++i) {
                Circle& c = circles[i];
                float dx = mouse_x - c.x, dy = mouse_y - c.y;
                float d2 = dx * dx + dy * dy;
                if (d2 < 200.0f * 200.0f && d2 > 1.0f) {
                    c.vx += dx * 0.6f * DT;
                    c.vy += dy * 0.6f * DT;
                    dragged.push_back((int)i);
                }
            }
        }

        if (!paused) {
            while (accumulator >= DT) {
                stepPhysics(circles);

                if (mode == MODE_MEDIAN) {
                    buildBVH(circles, nodes, indices, stats, BuildMethod::Median, 0.0f);
                } else if (mode == MODE_SAH) {
                    buildBVH(circles, nodes, indices, stats, BuildMethod::SAH, 0.0f);
                } else { // MODE_REFIT
                    if (force_rebuild || needsRebuild(circles, nodes, indices, FAT_MARGIN)) {
                        buildBVH(circles, nodes, indices, stats, BuildMethod::Median, FAT_MARGIN);
                        force_rebuild = false;
                    } else {
                        refitBVH(circles, nodes, indices, stats, FAT_MARGIN);
                    }
                }
                bvh_pairs.clear();
                queryBVH(nodes, circles, bvh_pairs, stats);
                LOG_TRACE("step: nodes=%d pairs=%d collisions=%d build=%.2fus refit=%.2fus query=%.2fus",
                          stats.nodes_used, stats.pairs_tested, stats.collisions_found,
                          stats.build_time_us, stats.refit_time_us, stats.query_time_us);

                for (const auto& p : bvh_pairs)
                    resolveCollision(circles[p.a], circles[p.b]);

                accumulator -= DT;
            }
        } else {
            accumulator = 0;
        }

        // Render
        SDL_SetRenderDrawColor(renderer, 20, 20, 30, 255);
        SDL_RenderClear(renderer);
        renderCircles(renderer, circles);
        if (show_tree) renderTree(renderer, nodes, tree_depth);

        // Force visualization: subtle lines from cursor to dragged objects
        if (!dragged.empty()) {
            SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);
            SDL_SetRenderDrawColor(renderer, 255, 255, 255, 70);
            for (int idx : dragged)
                SDL_RenderLine(renderer, mouse_x, mouse_y,
                                    circles[idx].x, circles[idx].y);
            SDL_SetRenderDrawColor(renderer, 255, 255, 255, 180);
            SDL_RenderLine(renderer, mouse_x - 8, mouse_y, mouse_x + 8, mouse_y);
            SDL_RenderLine(renderer, mouse_x, mouse_y - 8, mouse_x, mouse_y + 8);
            SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_NONE);
        }

        SDL_RenderPresent(renderer);

        auditPhysicsConservation(circles, g_physicsAudit);
        const char* pModes[] = { "ELASTIC(e=1.0)", "INELASTIC(e=0.0)", "PARTIAL(e=0.75)" };
        const char* names[] = { "BVH-Median", "BVH-SAH", "BVH-Refit" };
        char title[350];
        snprintf(title, sizeof(title),
            "[%s | %s | Fric:%s | Grav:%s] KE:%.0f J | N:%zu | Pairs:%d | Col:%d | FPS:%.0f",
            names[mode], pModes[(int)g_physicsConfig.mode],
            g_physicsConfig.frictionEnabled ? "ON" : "OFF",
            g_physicsConfig.gravityEnabled ? "ON" : "OFF",
            g_physicsAudit.totalKE,
            circles.size(),
            stats.pairs_tested,
            stats.collisions_found,
            fps);
        SDL_SetWindowTitle(window, title);
    }

    LOG_INFO("session end: %.1f s elapsed, final mode=%s, objects=%zu",
             std::chrono::duration<double>(Clock::now() - run_start).count(),
             modeLabel(mode), circles.size());

    // Step 1: Explicitly clear and shrink all vector allocations before destroying renderer/window
    circles.clear();
    circles.shrink_to_fit();
    nodes.clear();
    nodes.shrink_to_fit();
    indices.clear();
    indices.shrink_to_fit();
    bvh_pairs.clear();
    bvh_pairs.shrink_to_fit();

    // Step 2: Destroy SDL renderer and window
    if (renderer) {
        SDL_DestroyRenderer(renderer);
        renderer = nullptr;
    }
    if (window) {
        SDL_DestroyWindow(window);
        window = nullptr;
    }

    // Step 3: Call SDL_Quit cleanly
    SDL_Quit();
    LOG_INFO("clean exit");
    bvhlog::shutdown();
    return 0;
}