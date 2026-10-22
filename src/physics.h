#pragma once

#include "collision.h"
#include <vector>

/**
 * @file physics.h
 * @brief Contact Physics Engine extension for BVH Collision Detection System.
 * 
 * Individual Contribution:
 *  - Feature 1: Impulse-Based Elastic & Inelastic Collision Resolution (with Restitution e & Baumgarte De-penetration)
 *  - Feature 2: Coulomb Surface Friction Dynamics (Static & Kinetic Tangential Impulses) & Kinematics Auditor
 */

enum class PhysicsCollisionMode {
    Elastic,            // e = 1.0 (Kinetic energy strictly conserved)
    Inelastic,          // e = 0.0 (Completely inelastic / plastic impact, maximal energy loss)
    PartiallyElastic    // 0.0 < e < 1.0 (Realistic bounce, e.g., e = 0.75)
};

struct PhysicsConfig {
    PhysicsCollisionMode mode = PhysicsCollisionMode::Elastic;
    float restitution = 1.0f;       // Active coefficient of restitution
    bool frictionEnabled = true;    // Coulomb friction toggle
    bool gravityEnabled = false;    // Gravity toggle
    float gravity = 300.0f;         // Downward gravity acceleration (px/s^2)
    float airDrag = 0.005f;         // Viscous air resistance damping
    float baumgarteSlop = 0.01f;     // Positional penetration allowance slop (px)
    float baumgartePercent = 0.8f;   // Baumgarte stabilization projection factor (0.2 - 0.8)
};

struct PhysicsAudit {
    double totalKE = 0.0;           // Total kinetic energy sum (0.5 * m * v^2)
    double totalPx = 0.0;           // Total momentum X (m * vx)
    double totalPy = 0.0;           // Total momentum Y (m * vy)
    double dissipatedEnergy = 0.0;  // Cumulative kinetic energy lost to inelastic impact & friction
    int collisionsThisFrame = 0;    // Number of narrow-phase collisions resolved in current step
    int cumulativeCollisions = 0;   // Cumulative collisions since start
};

// FEATURE 1: Elastic & Inelastic Collision Impulse + Baumgarte Positional De-penetration
// FEATURE 2: Coulomb Friction Tangential Impulse Resolution
void resolvePhysicsCollision(Circle& a, Circle& b, const PhysicsConfig& config, PhysicsAudit& audit);

// Kinematics integration (Symplectic Euler) & boundary wall collisions with restitution/friction
void updatePhysicsKinematics(std::vector<Circle>& circles, float dt, float windowW, float windowH, const PhysicsConfig& config);

// Real-time quantitative conservation auditor
void auditPhysicsConservation(const std::vector<Circle>& circles, PhysicsAudit& audit);
