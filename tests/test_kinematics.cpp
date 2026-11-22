/// @file test_kinematics.cpp
/// @brief Unit tests for ArmModel, angle helpers, FK, Jacobian, IK and the RNG.
///
/// Each TEST is tagged with "@verifies REQ-..." so tools/check_traceability.py can build
/// the requirement-to-test matrix in docs/TRACEABILITY.md.
#include "amp/angles.hpp"
#include "amp/arm_model.hpp"
#include "amp/kinematics.hpp"
#include "amp/rng.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

namespace amp {
namespace {

constexpr double kTight = 1e-12;

Config makeConfig(std::initializer_list<double> values) {
    Config q(static_cast<Eigen::Index>(values.size()));
    Eigen::Index i = 0;
    for (double v : values) {
        q[i++] = v;
    }
    return q;
}

/// Random configuration strictly inside the joint limits (deterministic via Rng).
Config randomConfig(const ArmModel& arm, Rng& rng, double margin = 0.0) {
    Config q(arm.dof());
    for (int i = 0; i < arm.dof(); ++i) {
        const JointLimits& lim = arm.limits(i);
        q[i] = lim.continuous ? rng.uniform(-kPi, kPi) : rng.uniform(lim.lower + margin, lim.upper - margin);
    }
    return q;
}

// ---------------------------------------------------------------------------
// Arm model
// ---------------------------------------------------------------------------

// @verifies REQ-KIN-001
TEST(ArmModel, AcceptsTwoToSixJoints) {
    for (int dof = ArmModel::kMinDof; dof <= ArmModel::kMaxDof; ++dof) {
        const ArmModel arm = ArmModel::uniform(dof, 0.5);
        EXPECT_EQ(arm.dof(), dof);
        EXPECT_DOUBLE_EQ(arm.reach(), 0.5 * dof);
    }
}

// @verifies REQ-KIN-001
TEST(ArmModel, RejectsInvalidDescriptions) {
    EXPECT_THROW((void)ArmModel::uniform(1, 1.0), std::invalid_argument);
    EXPECT_THROW((void)ArmModel::uniform(7, 1.0), std::invalid_argument);
    EXPECT_THROW(ArmModel({1.0, -1.0}, {JointLimits{}, JointLimits{}}), std::invalid_argument);
    EXPECT_THROW(ArmModel({1.0, 1.0}, {JointLimits{}}), std::invalid_argument);
    EXPECT_THROW(ArmModel({1.0, 1.0}, {JointLimits{1.0, -1.0, false}, JointLimits{}}), std::invalid_argument);
    EXPECT_THROW(ArmModel({1.0, 1.0}, {JointLimits{}, JointLimits{}}, Vec2::Zero(), -0.1), std::invalid_argument);
}

// @verifies REQ-KIN-001
TEST(ArmModel, LimitsCheckAndNormalize) {
    const ArmModel arm({1.0, 1.0}, {JointLimits{-1.0, 1.0, false}, JointLimits{0.0, 0.0, true}});
    EXPECT_TRUE(arm.withinLimits(makeConfig({1.0, 100.0})));  // inclusive bound; continuous joint unbounded
    EXPECT_FALSE(arm.withinLimits(makeConfig({1.01, 0.0})));
    EXPECT_FALSE(arm.withinLimits(makeConfig({0.0})));  // wrong size

    const Config n = arm.normalize(makeConfig({2.0, 3.0 * kPi}));
    EXPECT_DOUBLE_EQ(n[0], 1.0);     // clamped
    EXPECT_NEAR(n[1], kPi, kTight);  // wrapped into (-pi, pi]
}

// @verifies REQ-KIN-008
TEST(Angles, WrapToPiIsHalfOpen) {
    EXPECT_NEAR(wrapToPi(0.0), 0.0, kTight);
    EXPECT_NEAR(wrapToPi(kPi), kPi, kTight);
    EXPECT_NEAR(wrapToPi(-kPi), kPi, kTight);  // -pi maps to +pi
    EXPECT_NEAR(wrapToPi(3.0 * kPi / 2.0), -kPi / 2.0, kTight);
    EXPECT_NEAR(wrapToPi(-7.0 * kPi), kPi, 1e-9);
}

// @verifies REQ-KIN-008
TEST(Angles, AngleDiffTakesShortWay) {
    EXPECT_NEAR(angleDiff(kPi - 0.1, -kPi + 0.1), 0.2, 1e-12);  // across the +/-pi seam
    EXPECT_NEAR(angleDiff(0.5, -0.5), -1.0, 1e-12);
}

// ---------------------------------------------------------------------------
// Forward kinematics (hand-computed values)
// ---------------------------------------------------------------------------

// @verifies REQ-KIN-002
TEST(ForwardKinematics, ZeroConfigurationIsStraightAlongX) {
    const ArmModel arm({1.0, 0.5, 0.25}, std::vector<JointLimits>(3), Vec2(0.1, -0.2));
    const auto pts = forwardKinematics(arm, Config::Zero(3));
    ASSERT_EQ(pts.size(), 4u);
    EXPECT_TRUE(pts[0].isApprox(Vec2(0.1, -0.2)));
    EXPECT_TRUE(pts[1].isApprox(Vec2(1.1, -0.2)));
    EXPECT_TRUE(pts[2].isApprox(Vec2(1.6, -0.2)));
    EXPECT_TRUE(pts[3].isApprox(Vec2(1.85, -0.2)));
}

// @verifies REQ-KIN-002
TEST(ForwardKinematics, HandComputedThreeLink) {
    // q = (90, -90, 90) deg with unit links: up 1, right 1, up 1 => EE at (1, 2).
    const ArmModel arm = ArmModel::uniform(3, 1.0);
    const auto pts = forwardKinematics(arm, makeConfig({kPi / 2, -kPi / 2, kPi / 2}));
    EXPECT_NEAR(pts[1].x(), 0.0, kTight);
    EXPECT_NEAR(pts[1].y(), 1.0, kTight);
    EXPECT_NEAR(pts[2].x(), 1.0, kTight);
    EXPECT_NEAR(pts[2].y(), 1.0, kTight);
    EXPECT_NEAR(pts[3].x(), 1.0, kTight);
    EXPECT_NEAR(pts[3].y(), 2.0, kTight);
}

// @verifies REQ-KIN-002
TEST(ForwardKinematics, HandComputedTwoLinkThirtySixty) {
    // l = (2, 1), q = (30, 60) deg: joint1 = 2(cos30, sin30); EE adds 1*(cos90, sin90).
    const ArmModel arm({2.0, 1.0}, std::vector<JointLimits>(2));
    const Vec2 ee = endEffectorPosition(arm, makeConfig({kPi / 6, kPi / 3}));
    EXPECT_NEAR(ee.x(), std::sqrt(3.0), kTight);
    EXPECT_NEAR(ee.y(), 1.0 + 1.0, kTight);
}

// ---------------------------------------------------------------------------
// Jacobian
// ---------------------------------------------------------------------------

// @verifies REQ-KIN-003
TEST(Jacobian, MatchesCentralFiniteDifferences) {
    Rng rng(42);
    for (int dof = 2; dof <= 6; ++dof) {
        const ArmModel arm = ArmModel::uniform(dof, 1.0 / dof);
        for (int trial = 0; trial < 20; ++trial) {
            const Config q = randomConfig(arm, rng);
            const Jacobian jac = jacobian(arm, q);
            constexpr double h = 1e-6;
            for (int j = 0; j < dof; ++j) {
                Config qp = q, qm = q;
                qp[j] += h;
                qm[j] -= h;
                const Vec2 fd = (endEffectorPosition(arm, qp) - endEffectorPosition(arm, qm)) / (2 * h);
                EXPECT_NEAR(jac(0, j), fd.x(), 1e-8) << "dof=" << dof << " joint=" << j;
                EXPECT_NEAR(jac(1, j), fd.y(), 1e-8) << "dof=" << dof << " joint=" << j;
            }
        }
    }
}

// @verifies REQ-KIN-003
TEST(Jacobian, ManipulabilityVanishesAtSingularities) {
    const ArmModel arm = ArmModel::uniform(2, 1.0);
    // Fully stretched. det(J J^T) is ~1e-16 from round-off and the sqrt amplifies it to ~1e-8.
    EXPECT_NEAR(manipulability(arm, makeConfig({0.3, 0.0})), 0.0, 1e-7);
    // For a 2-link arm w = l1 l2 |sin q2|, maximal at a right-angle elbow.
    EXPECT_NEAR(manipulability(arm, makeConfig({0.3, kPi / 2})), 1.0, 1e-12);
    EXPECT_NEAR(manipulability(arm, makeConfig({-1.0, 0.5})), std::sin(0.5), 1e-12);
}

// ---------------------------------------------------------------------------
// Analytic 2-link IK
// ---------------------------------------------------------------------------

// @verifies REQ-KIN-004
TEST(IkAnalytic, RoundTripBothElbows) {
    const ArmModel arm({0.7, 0.5}, std::vector<JointLimits>(2), Vec2(0.3, -0.1));
    Rng rng(7);
    for (int trial = 0; trial < 200; ++trial) {
        const Vec2 target = endEffectorPosition(arm, randomConfig(arm, rng));
        for (ElbowConfig elbow : {ElbowConfig::Up, ElbowConfig::Down}) {
            const IkResult r = solveIkAnalytic2Link(arm, target, elbow);
            ASSERT_TRUE(r.ok()) << toString(r.status);
            EXPECT_LT((endEffectorPosition(arm, r.q) - target).norm(), 1e-9);
            if (elbow == ElbowConfig::Down) {
                EXPECT_GE(r.q[1], 0.0);
            } else {
                EXPECT_LE(r.q[1], 0.0);
            }
        }
    }
}

// @verifies REQ-KIN-004
TEST(IkAnalytic, ElbowSolutionsAreMirrorImages) {
    const ArmModel arm = ArmModel::uniform(2, 1.0);
    const Vec2 target(1.0, 1.0);  // q = (0, 90deg) or (90deg, -90deg)
    const IkResult down = solveIkAnalytic2Link(arm, target, ElbowConfig::Down);
    const IkResult up = solveIkAnalytic2Link(arm, target, ElbowConfig::Up);
    EXPECT_NEAR(down.q[0], 0.0, 1e-12);
    EXPECT_NEAR(down.q[1], kPi / 2, 1e-12);
    EXPECT_NEAR(up.q[0], kPi / 2, 1e-12);
    EXPECT_NEAR(up.q[1], -kPi / 2, 1e-12);
}

// @verifies REQ-KIN-005
TEST(IkAnalytic, ReportsUnreachableTargets) {
    const ArmModel arm({1.0, 0.5}, std::vector<JointLimits>(2));
    EXPECT_EQ(solveIkAnalytic2Link(arm, Vec2(2.0, 0.0), ElbowConfig::Up).status, IkStatus::Unreachable);  // too far
    EXPECT_EQ(solveIkAnalytic2Link(arm, Vec2(0.2, 0.0), ElbowConfig::Up).status, IkStatus::Unreachable);  // too close
    // Exactly at full extension is reachable.
    EXPECT_TRUE(solveIkAnalytic2Link(arm, Vec2(1.5, 0.0), ElbowConfig::Up).ok());
}

// @verifies REQ-KIN-005
TEST(IkAnalytic, ReportsJointLimitViolationAndWrongDof) {
    const ArmModel arm({1.0, 1.0}, {JointLimits{-0.1, 0.1, false}, JointLimits{-kPi, kPi, false}});
    EXPECT_EQ(solveIkAnalytic2Link(arm, Vec2(0.0, 1.5), ElbowConfig::Up).status, IkStatus::JointLimitViolation);
    EXPECT_EQ(solveIkAnalytic2Link(ArmModel::uniform(3, 1.0), Vec2(1.0, 0.0), ElbowConfig::Up).status,
              IkStatus::InvalidInput);
}

// ---------------------------------------------------------------------------
// Damped least squares IK
// ---------------------------------------------------------------------------

// @verifies REQ-KIN-006
TEST(IkDls, RoundTripForTwoToSixLinks) {
    Rng rng(2024);
    for (int dof = 2; dof <= 6; ++dof) {
        const ArmModel arm = ArmModel::uniform(dof, 1.0 / dof, 0.9 * kPi);
        int singleSeedSuccesses = 0;
        int restartSuccesses = 0;
        constexpr int kTrials = 100;
        DlsParams withRestarts;
        withRestarts.restarts = 10;
        for (int trial = 0; trial < kTrials; ++trial) {
            const Vec2 target = endEffectorPosition(arm, randomConfig(arm, rng, 0.1));
            // Seed with a mildly bent arm: the solver should find *a* solution, not the original one.
            const Config seed = Config::Constant(dof, 0.3);
            const IkResult single = solveIkDls(arm, target, seed);
            const IkResult multi = solveIkDls(arm, target, seed, withRestarts);
            for (const IkResult* r : {&single, &multi}) {
                if (r->ok()) {
                    EXPECT_LT((endEffectorPosition(arm, r->q) - target).norm(), 1e-5);
                    EXPECT_TRUE(arm.withinLimits(r->q));
                }
            }
            singleSeedSuccesses += single.ok() ? 1 : 0;
            restartSuccesses += multi.ok() ? 1 : 0;
        }
        // DLS is a local method: from a single poor seed it can stall against a joint limit.
        // Deterministic restarts recover essentially all reachable targets.
        EXPECT_GE(singleSeedSuccesses, 80) << "dof=" << dof;
        EXPECT_GE(restartSuccesses, 99) << "dof=" << dof;
    }
}

// @verifies REQ-KIN-006
TEST(IkDls, UnreachableTargetConvergesToClosestPoint) {
    const ArmModel arm = ArmModel::uniform(3, 1.0);
    const IkResult r = solveIkDls(arm, Vec2(5.0, 0.0), Config::Constant(3, 0.2));
    EXPECT_EQ(r.status, IkStatus::NotConverged);
    // Best possible is full extension toward the target (residual 5 - 3 = 2). Convergence onto
    // the boundary is slow because full extension is a singularity (see ALGORITHMS.md), so we
    // accept 1 cm here and check that the arm points at the target.
    EXPECT_NEAR(r.residual, 2.0, 1e-2);
    EXPECT_NEAR(endEffectorPosition(arm, r.q).y(), 0.0, 1e-2);
}

// @verifies REQ-KIN-007
TEST(IkDls, IsDeterministic) {
    const ArmModel arm = ArmModel::uniform(4, 0.3);
    const Config seed = Config::Constant(4, 0.1);
    const IkResult a = solveIkDls(arm, Vec2(0.4, 0.6), seed);
    const IkResult b = solveIkDls(arm, Vec2(0.4, 0.6), seed);
    ASSERT_TRUE(a.ok());
    EXPECT_EQ(a.iterations, b.iterations);
    for (int i = 0; i < 4; ++i) {
        EXPECT_EQ(a.q[i], b.q[i]);  // bit-identical, not just close
    }
}

// @verifies REQ-KIN-006
TEST(IkDls, RejectsWrongSeedSize) {
    EXPECT_EQ(solveIkDls(ArmModel::uniform(3, 1.0), Vec2(1.0, 0.0), Config::Zero(2)).status, IkStatus::InvalidInput);
}

// ---------------------------------------------------------------------------
// Random number generator
// ---------------------------------------------------------------------------

// @verifies REQ-GEN-001
TEST(Rng, SameSeedSameSequence) {
    Rng a(123), b(123), c(124);
    bool anyDifferent = false;
    for (int i = 0; i < 1000; ++i) {
        const double x = a.uniform01();
        EXPECT_EQ(x, b.uniform01());
        anyDifferent = anyDifferent || (x != c.uniform01());
        EXPECT_GE(x, 0.0);
        EXPECT_LT(x, 1.0);
    }
    EXPECT_TRUE(anyDifferent);
}

// @verifies REQ-GEN-001
TEST(Rng, MatchesReferenceSequenceOnEveryPlatform) {
    // mt19937_64 is specified bit-exactly by the C++ standard: with the default seed 5489
    // the 10000th output is 9981545732273789042. Our uniform01() maps the top 53 bits, so
    // the expected double is fully determined and identical on MSVC, GCC and Clang.
    Rng rng(5489);
    for (int i = 0; i < 9999; ++i) {
        (void)rng.uniform01();
    }
    const double expected = static_cast<double>(9981545732273789042ULL >> 11) / 9007199254740992.0;
    EXPECT_EQ(rng.uniform01(), expected);
}

// @verifies REQ-GEN-001
TEST(Rng, BelowStaysInRange) {
    Rng rng(9);
    for (int i = 0; i < 1000; ++i) {
        EXPECT_LT(rng.below(7), 7u);
    }
}

}  // namespace
}  // namespace amp
