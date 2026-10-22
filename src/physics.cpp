#include "physics.h"
#include <cmath>
#include <algorithm>

// =============================================================================
// FEATURE 1: Impulse-Based Elastic & Inelastic Collision Resolution
// FEATURE 2: Coulomb Friction (Tangential Impulses) & Baumgarte De-penetration
// =============================================================================
void resolvePhysicsCollision(Circle& a, Circle& b, const PhysicsConfig& config, PhysicsAudit& audit) {
    float dx = b.x - a.x;
    float dy = b.y - a.y;
    float dist2 = dx * dx + dy * dy;
    float minDist = a.r + b.r;

    // Narrow-phase circle-circle intersection check
    if (dist2 >= minDist * minDist) return;

    float dist = 0.0f;
    float nx = 1.0f;
    float ny = 0.0f;
    if (dist2 > 1e-8f) {
        dist = std::sqrt(dist2);
        nx = dx / dist; // Unit normal pointing from a to b
        ny = dy / dist;
    } else {
        // Degenerate case: concentric bodies (dist = 0). Use deterministic unit normal.
        dist = 0.0f;
        nx = 1.0f;
        ny = 0.0f;
    }

    // Relative velocity: v_rel = v_b - v_a
    float rvx = b.vx - a.vx;
    float rvy = b.vy - a.vy;
    float velAlongNormal = rvx * nx + rvy * ny;

    float invMassSum = a.invMass + b.invMass;
    if (invMassSum <= 1e-7f) return;

    double keBefore = 0.5 * (a.mass * (a.vx * a.vx + a.vy * a.vy) + 
                             b.mass * (b.vx * b.vx + b.vy * b.vy));

    // -------------------------------------------------------------------------
    // FEATURE 1: Normal Impulse with Newtonian Restitution e in [0, 1]
    // J_n = - (1 + e) * (v_rel · n) / (1/m_a + 1/m_b)
    // -------------------------------------------------------------------------
    float jNormal = 0.0f;
    if (velAlongNormal < 0.0f) {
        float e = config.restitution;
        jNormal = -(1.0f + e) * velAlongNormal / invMassSum;

        a.vx -= (jNormal * a.invMass) * nx;
        a.vy -= (jNormal * a.invMass) * ny;
        b.vx += (jNormal * b.invMass) * nx;
        b.vy += (jNormal * b.invMass) * ny;

        // Visual collision highlight ring for rendering
        a.flash_timer = 2;
        b.flash_timer = 2;
    }

    // -------------------------------------------------------------------------
    // FEATURE 2: Coulomb Friction (Tangential Impulse Clamping)
    // -------------------------------------------------------------------------
    if (config.frictionEnabled && jNormal > 0.0f) {
        float nrvx = b.vx - a.vx;
        float nrvy = b.vy - a.vy;
        float nvn = nrvx * nx + nrvy * ny;
        float tx = nrvx - nvn * nx;
        float ty = nrvy - nvn * ny;
        float tlen = std::hypot(tx, ty);

        if (tlen > 1e-4f) {
            tx /= tlen;
            ty /= tlen;

            // Desired tangential impulse to cancel relative slip
            float jTangential = -(nrvx * tx + nrvy * ty) / invMassSum;

            float muStatic = std::sqrt(a.staticFriction * b.staticFriction);
            float muDynamic = std::sqrt(a.dynamicFriction * b.dynamicFriction);

            // Coulomb's Law of Friction Clamping:
            // Dynamic friction magnitude is clamped to jNormal * muDynamic, but never
            // exceeds |jTangential| to prevent tangential velocity inversion / energy gain.
            float frictionImpulse = 0.0f;
            if (std::abs(jTangential) <= jNormal * muStatic) {
                frictionImpulse = jTangential;
            } else {
                float maxDynamic = jNormal * muDynamic;
                float clampedMag = std::min(std::abs(jTangential), maxDynamic);
                frictionImpulse = (jTangential < 0.0f) ? -clampedMag : clampedMag;
            }

            a.vx -= (frictionImpulse * a.invMass) * tx;
            a.vy -= (frictionImpulse * a.invMass) * ty;
            b.vx += (frictionImpulse * b.invMass) * tx;
            b.vy += (frictionImpulse * b.invMass) * ty;
        }
    }

    // -------------------------------------------------------------------------
    // Positional De-penetration (Baumgarte Linear Projection)
    // -------------------------------------------------------------------------
    float overlap = minDist - dist;
    const float slop = config.baumgarteSlop;
    const float percent = config.baumgartePercent;
    float correction = (std::max(overlap - slop, 0.0f) / invMassSum) * percent;

    a.x -= nx * correction * a.invMass;
    a.y -= ny * correction * a.invMass;
    b.x += nx * correction * b.invMass;
    b.y += ny * correction * b.invMass;

    // -------------------------------------------------------------------------
    // Energy Conservation Accounting
    // -------------------------------------------------------------------------
    double keAfter = 0.5 * (a.mass * (a.vx * a.vx + a.vy * a.vy) + 
                            b.mass * (b.vx * b.vx + b.vy * b.vy));
    if (keBefore > keAfter) {
        audit.dissipatedEnergy += (keBefore - keAfter);
    }
    audit.collisionsThisFrame++;
    audit.cumulativeCollisions++;
}

void updatePhysicsKinematics(std::vector<Circle>& circles, float dt, float windowW, float windowH, const PhysicsConfig& config) {
    float e = config.restitution;

    for (auto& c : circles) {
        // Environmental gravity
        if (config.gravityEnabled) {
            c.vy += config.gravity * dt;
        }

        // Viscous air resistance drag
        if (config.airDrag > 0.0f) {
            c.vx *= (1.0f - config.airDrag * dt);
            c.vy *= (1.0f - config.airDrag * dt);
        }

        // Position integration (Symplectic Euler)
        c.x += c.vx * dt;
        c.y += c.vy * dt;

        // Boundary walls with restitution & wall friction
        if (c.x - c.r < 0.0f) {
            c.x = c.r;
            c.vx = -c.vx * e;
            if (config.frictionEnabled) c.vy *= (1.0f - c.dynamicFriction * 0.2f);
        }
        if (c.x + c.r > windowW) {
            c.x = windowW - c.r;
            c.vx = -c.vx * e;
            if (config.frictionEnabled) c.vy *= (1.0f - c.dynamicFriction * 0.2f);
        }
        if (c.y - c.r < 0.0f) {
            c.y = c.r;
            c.vy = -c.vy * e;
            if (config.frictionEnabled) c.vx *= (1.0f - c.dynamicFriction * 0.2f);
        }
        if (c.y + c.r > windowH) {
            c.y = windowH - c.r;
            c.vy = -c.vy * e;
            if (config.frictionEnabled) c.vx *= (1.0f - c.dynamicFriction * 0.2f);
        }

        if (c.flash_timer > 0) c.flash_timer--;
    }
}

void auditPhysicsConservation(const std::vector<Circle>& circles, PhysicsAudit& audit) {
    audit.totalKE = 0.0;
    audit.totalPx = 0.0;
    audit.totalPy = 0.0;

    for (const auto& c : circles) {
        audit.totalKE += 0.5 * c.mass * (c.vx * c.vx + c.vy * c.vy);
        audit.totalPx += c.mass * c.vx;
        audit.totalPy += c.mass * c.vy;
    }
}
