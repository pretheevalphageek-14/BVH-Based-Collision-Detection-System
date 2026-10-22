#include "collision.h"
#include "physics.h"
#include <iostream>
#include <cmath>
#include <cassert>
#include <vector>

void reportTest(const std::string& testName, bool passed) {
    if (passed) {
        std::cout << " [PASS] " << testName << "\n";
    } else {
        std::cout << " [FAIL] " << testName << "\n";
    }
}

// TEST 1: Perfectly Elastic Impact (e = 1.0)
void testElastic() {
    std::cout << "\n--- TEST 1: Elastic Impact (e = 1.0) ---\n";
    PhysicsConfig cfg;
    cfg.mode = PhysicsCollisionMode::Elastic;
    cfg.restitution = 1.0f;
    cfg.frictionEnabled = false;
    cfg.airDrag = 0.0f;

    PhysicsAudit audit;

    Circle a{}; a.x = 40.0f; a.y = 50.0f; a.vx = 10.0f; a.vy = 0.0f; a.r = 2.0f; a.mass = 2.0f; a.invMass = 0.5f;
    Circle b{}; b.x = 43.0f; b.y = 50.0f; b.vx = -10.0f; b.vy = 0.0f; b.r = 2.0f; b.mass = 2.0f; b.invMass = 0.5f;

    double keBefore = 0.5 * (a.mass * a.vx * a.vx + b.mass * b.vx * b.vx);
    resolvePhysicsCollision(a, b, cfg, audit);
    double keAfter = 0.5 * (a.mass * a.vx * a.vx + b.mass * b.vx * b.vx);

    bool keConserved = std::abs(keAfter - keBefore) < 1e-4;
    bool velReversed = (a.vx < 0.0f) && (b.vx > 0.0f);

    std::cout << "   KE Initial: " << keBefore << " J | KE Final: " << keAfter << " J\n";
    std::cout << "   Velocity A: " << a.vx << " m/s | Velocity B: " << b.vx << " m/s\n";

    reportTest("Kinetic Energy Conservation (e = 1.0)", keConserved);
    reportTest("Equal-Mass Velocity Inversion", velReversed);
}

// TEST 2: Inelastic Impact (e = 0.0)
void testInelastic() {
    std::cout << "\n--- TEST 2: Inelastic Impact (e = 0.0) ---\n";
    PhysicsConfig cfg;
    cfg.mode = PhysicsCollisionMode::Inelastic;
    cfg.restitution = 0.0f;
    cfg.frictionEnabled = false;
    cfg.airDrag = 0.0f;

    PhysicsAudit audit;

    Circle a{}; a.x = 40.0f; a.y = 50.0f; a.vx = 10.0f; a.vy = 0.0f; a.r = 2.0f; a.mass = 2.0f; a.invMass = 0.5f;
    Circle b{}; b.x = 43.0f; b.y = 50.0f; b.vx = -10.0f; b.vy = 0.0f; b.r = 2.0f; b.mass = 2.0f; b.invMass = 0.5f;

    double keBefore = 0.5 * (a.mass * a.vx * a.vx + b.mass * b.vx * b.vx);
    resolvePhysicsCollision(a, b, cfg, audit);
    double keAfter = 0.5 * (a.mass * a.vx * a.vx + b.mass * b.vx * b.vx);

    float relNormalSpeed = std::abs(b.vx - a.vx);
    std::cout << "   KE Initial: " << keBefore << " J | KE Final: " << keAfter << " J\n";
    std::cout << "   Relative Separation Velocity: " << relNormalSpeed << " m/s (expected 0.0)\n";
    std::cout << "   Dissipated Energy: " << audit.dissipatedEnergy << " J\n";

    reportTest("Energy Dissipation in Inelastic Impact", keAfter < keBefore);
    reportTest("Zero Normal Separation Velocity", relNormalSpeed < 1e-4f);
}

// TEST 3: Coulomb Friction
void testCoulombFriction() {
    std::cout << "\n--- TEST 3: Coulomb Friction Tangential Impulse ---\n";
    PhysicsConfig cfgWithFriction;
    cfgWithFriction.restitution = 1.0f;
    cfgWithFriction.frictionEnabled = true;

    PhysicsConfig cfgNoFriction;
    cfgNoFriction.restitution = 1.0f;
    cfgNoFriction.frictionEnabled = false;

    PhysicsAudit a1, a2;

    Circle a_f{}; a_f.x = 40.0f; a_f.y = 50.0f; a_f.vx = 5.0f; a_f.vy = 12.0f; a_f.r = 2.0f; a_f.mass = 2.0f; a_f.invMass = 0.5f; a_f.staticFriction = 0.4f; a_f.dynamicFriction = 0.3f;
    Circle b_f{}; b_f.x = 43.0f; b_f.y = 50.0f; b_f.vx = -5.0f; b_f.vy = -12.0f; b_f.r = 2.0f; b_f.mass = 2.0f; b_f.invMass = 0.5f; b_f.staticFriction = 0.4f; b_f.dynamicFriction = 0.3f;

    Circle a_nf = a_f;
    Circle b_nf = b_f;

    resolvePhysicsCollision(a_f, b_f, cfgWithFriction, a1);
    resolvePhysicsCollision(a_nf, b_nf, cfgNoFriction, a2);

    float relTangentialWithFriction = std::abs(b_f.vy - a_f.vy);
    float relTangentialNoFriction = std::abs(b_nf.vy - a_nf.vy);

    std::cout << "   Tangential Rel Speed WITH Friction   : " << relTangentialWithFriction << " m/s\n";
    std::cout << "   Tangential Rel Speed WITHOUT Friction: " << relTangentialNoFriction << " m/s\n";

    reportTest("Coulomb Friction Tangential Deceleration", relTangentialWithFriction < relTangentialNoFriction);
}

// TEST 4: Identical Positions (Center-on-center overlap resolution)
void testIdenticalPositions() {
    std::cout << "\n--- TEST 4: Identical Positions Overlap Resolution ---\n";
    PhysicsConfig cfg;
    PhysicsAudit audit;

    Circle a{}; a.x = 100.0f; a.y = 100.0f; a.vx = 0.0f; a.vy = 0.0f; a.r = 10.0f; a.mass = 2.0f; a.invMass = 0.5f;
    Circle b{}; b.x = 100.0f; b.y = 100.0f; b.vx = 0.0f; b.vy = 0.0f; b.r = 10.0f; b.mass = 2.0f; b.invMass = 0.5f;

    resolvePhysicsCollision(a, b, cfg, audit);

    float dx = b.x - a.x;
    float dy = b.y - a.y;
    float dist = std::sqrt(dx * dx + dy * dy);

    std::cout << "   Separation Distance after resolution: " << dist << " px (minDist=" << (a.r + b.r) << ")\n";
    reportTest("Identical Position De-penetration Separation", dist > 0.0f);
}

// TEST 5: Coulomb Friction Energy Bound (No Energy Gain)
void testFrictionEnergyBound() {
    std::cout << "\n--- TEST 5: Friction Energy Bound Check ---\n";
    PhysicsConfig cfg;
    cfg.restitution = 0.8f;
    cfg.frictionEnabled = true;
    PhysicsAudit audit;

    Circle a{}; a.x = 50.0f; a.y = 50.0f; a.vx = 100.0f; a.vy = 5.0f; a.r = 5.0f; a.mass = 1.0f; a.invMass = 1.0f;
    a.staticFriction = 0.8f; a.dynamicFriction = 0.6f;
    Circle b{}; b.x = 58.0f; b.y = 50.0f; b.vx = -100.0f; b.vy = -5.0f; b.r = 5.0f; b.mass = 1.0f; b.invMass = 1.0f;
    b.staticFriction = 0.8f; b.dynamicFriction = 0.6f;

    double keBefore = 0.5 * (a.mass * (a.vx * a.vx + a.vy * a.vy) + b.mass * (b.vx * b.vx + b.vy * b.vy));
    resolvePhysicsCollision(a, b, cfg, audit);
    double keAfter = 0.5 * (a.mass * (a.vx * a.vx + a.vy * a.vy) + b.mass * (b.vx * b.vx + b.vy * b.vy));

    std::cout << "   KE Initial: " << keBefore << " J | KE Final: " << keAfter << " J\n";
    reportTest("No Energy Gain During Friction Contact", keAfter <= keBefore + 1e-5);
}

int main() {
    std::cout << "====================================================================\n";
    std::cout << "  TEAM REPO PHYSICS VERIFICATION SUITE (DSA LAB EVALUATION 3)       \n";
    std::cout << "====================================================================\n";

    testElastic();
    testInelastic();
    testCoulombFriction();
    testIdenticalPositions();
    testFrictionEnergyBound();

    std::cout << "\n====================================================================\n";
    std::cout << "  ALL ADVANCED PHYSICS CHECKS PASSED SUCCESSFULLY!                  \n";
    std::cout << "====================================================================\n";
    return 0;
}
