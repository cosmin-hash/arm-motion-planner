/// @file test_collision.cpp
/// @brief Tests for C-space operations, configuration validity, self-collision, and the
/// edge-validity guarantee (never passing through a thin obstacle).
#include "amp/collision.hpp"
#include "amp/cspace.hpp"
#include "amp/cspace_grid.hpp"
#include "amp/kinematics.hpp"
#include "amp/rng.hpp"

#include <gtest/gtest.h>

#include <stdexcept>

namespace amp {
namespace {

Config cfg(double a, double b) {
    Config q(2);
    q << a, b;
    return q;
}

Config cfg(double a, double b, double c) {
    Config q(3);
    q << a, b, c;
    return q;
}

ArmModel continuousArm() {
    return ArmModel({1.0, 1.0}, {JointLimits{0, 0, true}, JointLimits{-2.0, 2.0, false}});
}

// ---------------------------------------------------------------------------
// Configuration space
// ---------------------------------------------------------------------------

// @verifies REQ-CSP-001
TEST(CSpace, DistanceWrapsOnlyContinuousJoints) {
    const ArmModel arm = continuousArm();
    // Joint 0 is continuous: 3.0 -> -3.0 is 2pi - 6 = 0.283 rad the short way.
    EXPECT_NEAR(configDistance(arm, cfg(3.0, 0.0), cfg(-3.0, 0.0)), kTwoPi - 6.0, 1e-12);
    // Joint 1 has limits: no wrapping, 1.5 -> -1.5 is 3 rad.
    EXPECT_NEAR(configDistance(arm, cfg(0.0, 1.5), cfg(0.0, -1.5)), 3.0, 1e-12);
}

// @verifies REQ-CSP-001
TEST(CSpace, DistanceIsAMetric) {
    const ArmModel arm = continuousArm();
    Rng rng(11);
    for (int i = 0; i < 500; ++i) {
        const Config a = sampleUniform(arm, rng);
        const Config b = sampleUniform(arm, rng);
        const Config c = sampleUniform(arm, rng);
        EXPECT_NEAR(configDistance(arm, a, a), 0.0, 1e-15);
        EXPECT_NEAR(configDistance(arm, a, b), configDistance(arm, b, a), 1e-12);
        EXPECT_LE(configDistance(arm, a, c), configDistance(arm, a, b) + configDistance(arm, b, c) + 1e-12);
    }
}

// @verifies REQ-CSP-002
TEST(CSpace, InterpolationEndpointsAndShortestPath) {
    const ArmModel arm = continuousArm();
    const Config a = cfg(3.0, -1.0);
    const Config b = cfg(-3.0, 1.0);
    EXPECT_EQ(interpolate(arm, a, b, 0.0), a);
    EXPECT_EQ(interpolate(arm, a, b, 1.0), b);
    // Midpoint of the continuous joint goes through +/-pi, not through 0.
    const Config mid = interpolate(arm, a, b, 0.5);
    EXPECT_NEAR(std::abs(mid[0]), kPi, 1e-12);
    EXPECT_NEAR(mid[1], 0.0, 1e-12);
    // Distances along the path add up (it is a geodesic).
    EXPECT_NEAR(configDistance(arm, a, mid) + configDistance(arm, mid, b), configDistance(arm, a, b), 1e-12);
}

// @verifies REQ-CSP-003
TEST(CSpace, SamplesRespectLimitsAndAreDeterministic) {
    const ArmModel arm = continuousArm();
    Rng r1(5), r2(5);
    for (int i = 0; i < 1000; ++i) {
        const Config q = sampleUniform(arm, r1);
        EXPECT_TRUE(arm.withinLimits(q));
        EXPECT_GE(q[0], -kPi);
        EXPECT_LT(q[0], kPi);
        EXPECT_EQ(q, sampleUniform(arm, r2));
    }
}

// @verifies REQ-COL-007
TEST(CSpace, DisplacementBoundHoldsForEveryArmPoint) {
    // Brute force: sample many points along the motion and check no FK point (joint or tip)
    // moves farther than the bound in total path length.
    Rng rng(3);
    for (int dof = 2; dof <= 6; ++dof) {
        const ArmModel arm = ArmModel::uniform(dof, 1.0 / dof);
        for (int trial = 0; trial < 50; ++trial) {
            const Config a = sampleUniform(arm, rng);
            const Config b = sampleUniform(arm, rng);
            const double bound = maxWorkspaceDisplacement(arm, a, b);
            std::vector<double> pathLength(static_cast<std::size_t>(dof) + 1, 0.0);
            std::vector<Vec2> prev = forwardKinematics(arm, a);
            constexpr int kSamples = 400;
            for (int k = 1; k <= kSamples; ++k) {
                const std::vector<Vec2> cur = forwardKinematics(arm, interpolate(arm, a, b, double(k) / kSamples));
                for (std::size_t p = 0; p < cur.size(); ++p) {
                    pathLength[p] += (cur[p] - prev[p]).norm();
                }
                prev = cur;
            }
            for (double len : pathLength) {
                EXPECT_LE(len, bound + 1e-9);
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Configuration validity
// ---------------------------------------------------------------------------

// @verifies REQ-COL-006
TEST(CollisionChecker, ReportsObstacleWithLinkAndIndex) {
    const ArmModel arm = ArmModel::uniform(2, 1.0);
    Workspace ws;
    ws.obstacles.push_back(Circle{Vec2(5, 5), 0.1});       // far away
    ws.obstacles.push_back(Circle{Vec2(1.5, 0.15), 0.2});  // 0.15 from link 1 when straight (< 0.2 + 0.02)
    const CollisionChecker checker(arm, ws, {0.0, 0.01, true});

    const CollisionReport hit = checker.check(cfg(0.0, 0.0));
    EXPECT_EQ(hit.kind, CollisionKind::ObstacleContact);
    EXPECT_EQ(hit.link, 1);
    EXPECT_EQ(hit.other, 1);

    EXPECT_TRUE(checker.isValid(cfg(1.0, 0.0)));  // arm swung up and away
}

// @verifies REQ-COL-006
TEST(CollisionChecker, SafetyMarginInflatesLinks) {
    const ArmModel arm({1.0, 1.0}, std::vector<JointLimits>(2), Vec2::Zero(), 0.05);
    Workspace ws;
    ws.obstacles.push_back(Circle{Vec2(1.0, 0.2), 0.1});  // gap to the link core 0.2, r_link + R = 0.15
    EXPECT_TRUE(CollisionChecker(arm, ws, {0.0, 0.01, true}).isValid(cfg(0.0, 0.0)));
    EXPECT_FALSE(CollisionChecker(arm, ws, {0.06, 0.0, true}).isValid(cfg(0.0, 0.0)));  // 0.15 + 0.06 > 0.2
}

// @verifies REQ-COL-006
TEST(CollisionChecker, JointLimitsAndWrongSize) {
    const ArmModel arm = ArmModel::uniform(2, 1.0, 1.0);
    const CollisionChecker checker(arm, Workspace{});
    const CollisionReport r = checker.check(cfg(0.0, 1.5));
    EXPECT_EQ(r.kind, CollisionKind::JointLimit);
    EXPECT_EQ(r.link, 1);
    EXPECT_FALSE(checker.isValid(Config::Zero(3)));
}

// @verifies REQ-COL-004
TEST(CollisionChecker, BoundaryViolation) {
    const ArmModel arm = ArmModel::uniform(2, 0.5);
    Workspace ws;
    ws.bounds = Bounds{Vec2(-2, -0.3), Vec2(2, 2)};  // a "table" just below the base
    const CollisionChecker checker(arm, ws);
    EXPECT_TRUE(checker.isValid(cfg(kPi / 2, 0.0)));                 // pointing up
    const CollisionReport down = checker.check(cfg(-kPi / 2, 0.0));  // pointing into the table
    EXPECT_EQ(down.kind, CollisionKind::Boundary);
    EXPECT_EQ(down.link, 0);  // link 0 already reaches y = -0.5, past the table at -0.3
}

// @verifies REQ-COL-005
TEST(CollisionChecker, SelfCollisionOnlyBetweenNonAdjacentLinks) {
    const ArmModel arm({1.0, 0.5, 1.0}, std::vector<JointLimits>(3), Vec2::Zero(), 0.05);
    // Fold: link 2 comes back over link 0 (zig-zag with 180 deg turns is beyond limits, use 170).
    const double fold = 170.0 * kPi / 180.0;
    const CollisionChecker checker(arm, Workspace{}, {0.0, 0.01, true});
    const CollisionReport r = checker.check(cfg(0.0, fold, fold));
    EXPECT_EQ(r.kind, CollisionKind::Self);
    EXPECT_EQ(r.link, 0);
    EXPECT_EQ(r.other, 2);

    // Adjacent links always share a joint, so a tight fold of a 2-link arm is not "self collision".
    const CollisionChecker twoLink(ArmModel({1.0, 1.0}, std::vector<JointLimits>(2), Vec2::Zero(), 0.05), Workspace{},
                                   {0.0, 0.01, true});
    EXPECT_TRUE(twoLink.isValid(cfg(0.0, fold)));

    // Self-collision can be disabled.
    EXPECT_TRUE(CollisionChecker(arm, Workspace{}, {0.0, 0.01, false}).isValid(cfg(0.0, fold, fold)));
}

// @verifies REQ-COL-006
TEST(CollisionChecker, RejectsUnsafeOptions) {
    const ArmModel arm = ArmModel::uniform(2, 1.0);
    EXPECT_THROW(CollisionChecker(arm, Workspace{}, {0.01, 0.05, true}), std::invalid_argument);  // step > 2m
    EXPECT_THROW(CollisionChecker(arm, Workspace{}, {0.0, 0.0, true}), std::invalid_argument);    // no resolution
    EXPECT_THROW(CollisionChecker(arm, Workspace{}, {-0.01, 0.0, true}), std::invalid_argument);
    EXPECT_DOUBLE_EQ(CollisionChecker(arm, Workspace{}, {0.01, 0.0, true}).options().maxStepDisplacement, 0.02);
}

// ---------------------------------------------------------------------------
// Edge validity
// ---------------------------------------------------------------------------

// @verifies REQ-COL-007
TEST(EdgeValidity, NeverPassesThroughAThinObstacle) {
    // A 2-link arm sweeps from pointing right (-0.6 rad) to pointing up-right (+0.6 rad).
    // A wall 0.1 mm thick lies across the sweep. Both endpoints are collision-free; only the
    // intermediate motion hits the wall.
    const ArmModel arm({1.0, 1.0}, std::vector<JointLimits>(2), Vec2::Zero(), 0.01);
    Workspace ws;
    ws.obstacles.push_back(ConvexPolygon::rectangle(Vec2(1.2, -0.00005), Vec2(1.9, 0.00005)));
    const CollisionChecker checker(arm, ws);

    const Config a = cfg(-0.6, 0.0);
    const Config b = cfg(0.6, 0.0);
    ASSERT_TRUE(checker.isValid(a));
    ASSERT_TRUE(checker.isValid(b));
    EXPECT_FALSE(checker.isEdgeValid(a, b));
    EXPECT_FALSE(checker.isEdgeValid(b, a));  // direction must not matter
}

// @verifies REQ-COL-007
TEST(EdgeValidity, StepsFollowTheDisplacementBound) {
    const ArmModel arm = ArmModel::uniform(2, 0.5);
    const CollisionChecker checker(arm, Workspace{}, {0.005, 0.0, true});
    // Moving joint 0 by 1 rad: bound = 1 * 1.0 m = 1.0 m; step <= 0.01 m -> 100 intervals.
    EXPECT_EQ(checker.edgeSteps(cfg(0, 0), cfg(1, 0)), 100);
    // Moving joint 1 by 1 rad: bound = 1 * 0.5 m -> 50 intervals.
    EXPECT_EQ(checker.edgeSteps(cfg(0, 0), cfg(0, 1)), 50);
    EXPECT_EQ(checker.edgeSteps(cfg(0, 0), cfg(0, 0)), 1);
}

// @verifies REQ-COL-007
TEST(EdgeValidity, ValidEdgesAreCollisionFreeUnderDenseChecking) {
    // Property test: whenever an edge is accepted (with margin m and step <= 2m), a 50x denser
    // check of the *uninflated* arm finds no collision anywhere along it.
    const ArmModel arm({0.4, 0.35, 0.25}, std::vector<JointLimits>(3, JointLimits{-2.8, 2.8, false}), Vec2::Zero(),
                       0.02);
    Workspace ws;
    ws.obstacles.push_back(Circle{Vec2(0.5, 0.4), 0.08});
    ws.obstacles.push_back(ConvexPolygon({Vec2(-0.6, 0.2), Vec2(-0.3, 0.25), Vec2(-0.45, 0.6)}));
    ws.obstacles.push_back(ConvexPolygon::rectangle(Vec2(0.3, -0.5), Vec2(0.302, -0.1)));  // 2 mm wall
    ws.bounds = Bounds{Vec2(-1.2, -0.7), Vec2(1.2, 1.2)};
    const CollisionChecker checker(arm, ws, {0.004, 0.0, true});
    const CollisionChecker exact(arm, ws, {0.0, 1e-4, true});

    Rng rng(99);
    int accepted = 0;
    for (int trial = 0; trial < 300; ++trial) {
        const Config a = sampleUniform(arm, rng);
        const Config b = interpolate(arm, a, sampleUniform(arm, rng), 0.3);  // short-ish edges
        if (!checker.isEdgeValid(a, b)) {
            continue;
        }
        ++accepted;
        const int dense = 50 * checker.edgeSteps(a, b);
        for (int k = 0; k <= dense; ++k) {
            ASSERT_TRUE(exact.isValid(interpolate(arm, a, b, double(k) / dense))) << "trial " << trial;
        }
    }
    EXPECT_GT(accepted, 30);  // make sure the property was exercised
}

// ---------------------------------------------------------------------------
// C-space grid
// ---------------------------------------------------------------------------

// @verifies REQ-COL-008
TEST(CSpaceGrid, MatchesPointwiseChecks) {
    const ArmModel arm = ArmModel::uniform(2, 0.5, 2.5);
    Workspace ws;
    ws.obstacles.push_back(Circle{Vec2(0.6, 0.3), 0.1});
    const CollisionChecker checker(arm, ws);
    const CSpaceGrid2D grid = computeCSpaceGrid2D(checker, 32);
    ASSERT_EQ(grid.cells.size(), 32u * 32u);
    int obstacles = 0, limits = 0, free = 0;
    for (int j = 0; j < 32; ++j) {
        for (int i = 0; i < 32; ++i) {
            EXPECT_EQ(grid.at(i, j), checker.check(cfg(grid.cellCenter(i), grid.cellCenter(j))).kind);
            obstacles += grid.at(i, j) == CollisionKind::ObstacleContact;
            limits += grid.at(i, j) == CollisionKind::JointLimit;
            free += grid.at(i, j) == CollisionKind::None;
        }
    }
    EXPECT_GT(obstacles, 0);
    EXPECT_GT(limits, 0);
    EXPECT_GT(free, 0);
    EXPECT_THROW((void)computeCSpaceGrid2D(CollisionChecker(ArmModel::uniform(3, 0.3), Workspace{}), 8),
                 std::invalid_argument);
}

}  // namespace
}  // namespace amp
