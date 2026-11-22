/// @file test_postprocessing.cpp
/// @brief Shortcut smoothing and trapezoidal time parameterization.
#include "amp/cspace.hpp"
#include "amp/planner.hpp"
#include "amp/rrt.hpp"
#include "amp/smoothing.hpp"
#include "amp/trajectory.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace amp {
namespace {

Config cfg(double a, double b) {
    Config q(2);
    q << a, b;
    return q;
}

/// Same detour problem as the planner tests: a wall the forearm must fold past.
struct WallProblem {
    ArmModel arm{{0.5, 0.4}, std::vector<JointLimits>(2, JointLimits{-2.9, 2.9, false}), Vec2::Zero(), 0.02};
    Workspace ws = [] {
        Workspace w;
        w.obstacles.push_back(ConvexPolygon::rectangle(Vec2(0.6, -0.05), Vec2(1.0, 0.05)));
        return w;
    }();
    CollisionChecker checker{arm, ws};
    Config start = cfg(-0.8, 0.3);
    Config goal = cfg(0.9, -0.3);
};

// ---------------------------------------------------------------------------
// Shortcut smoothing
// ---------------------------------------------------------------------------

// @verifies REQ-PST-001
TEST(Smoothing, ShortensRrtPathsAndKeepsThemValid) {
    const WallProblem p;
    for (std::uint64_t seed = 1; seed <= 10; ++seed) {
        PlannerParams params;
        params.seed = seed;
        const PlanResult plan = RrtPlanner(p.checker).plan(p.start, p.goal, params);
        ASSERT_TRUE(plan.success);
        SmoothingParams sp;
        sp.seed = seed;
        const SmoothingResult s = shortcutSmooth(p.checker, plan.path, sp);
        EXPECT_EQ(s.path.front(), p.start);
        EXPECT_EQ(s.path.back(), p.goal);
        EXPECT_TRUE(isPathValid(p.checker, s.path));
        EXPECT_NEAR(s.costBefore, plan.pathCost, 1e-9);
        EXPECT_LE(s.costAfter, s.costBefore + 1e-12);
        EXPECT_NEAR(s.costAfter, pathLength(p.arm, s.path), 1e-9);
        EXPECT_LT(s.costAfter, 0.9 * s.costBefore) << "seed " << seed;  // RRT paths are jagged
    }
}

// @verifies REQ-PST-001
TEST(Smoothing, FreeSpacePathBecomesStraight) {
    const ArmModel arm = ArmModel::uniform(2, 0.5, 2.9);
    const CollisionChecker checker(arm, Workspace{});
    const std::vector<Config> zigzag = {cfg(-1, -1), cfg(0.5, -1.2), cfg(-0.3, 0.4), cfg(1, 1)};
    const SmoothingResult s = shortcutSmooth(checker, zigzag);
    ASSERT_EQ(s.path.size(), 2u);
    EXPECT_NEAR(s.costAfter, configDistance(arm, cfg(-1, -1), cfg(1, 1)), 1e-12);
}

// @verifies REQ-PST-001
TEST(Smoothing, IsDeterministic) {
    const WallProblem p;
    const PlanResult plan = RrtPlanner(p.checker).plan(p.start, p.goal, {});
    ASSERT_TRUE(plan.success);
    const SmoothingResult a = shortcutSmooth(p.checker, plan.path);
    const SmoothingResult b = shortcutSmooth(p.checker, plan.path);
    ASSERT_EQ(a.path.size(), b.path.size());
    for (std::size_t i = 0; i < a.path.size(); ++i) {
        EXPECT_EQ(a.path[i], b.path[i]);
    }
}

// ---------------------------------------------------------------------------
// Time parameterization
// ---------------------------------------------------------------------------

// @verifies REQ-PST-002
TEST(Trapezoid, TrapezoidAndTriangleCases) {
    // Long move: reaches the cruise rate. v = 1, a = 4: each ramp takes 0.25 s, and together
    // they cover v^2 / a = 0.25; the remaining 0.75 at rate 1 takes 0.75 s. Total 1.25 s.
    const SegmentTiming trap = Trajectory::trapezoid(1.0, 4.0);
    EXPECT_DOUBLE_EQ(trap.accelTime, 0.25);
    EXPECT_DOUBLE_EQ(trap.peakRate, 1.0);
    EXPECT_DOUBLE_EQ(trap.duration, 1.25);
    // Boundary: v = 2, a = 4 ramps cover exactly the whole distance (v^2 / a = 1), no cruise.
    EXPECT_DOUBLE_EQ(Trajectory::trapezoid(2.0, 4.0).duration, 1.0);
    // Short move: v^2 / a = 25 > 1, so it never reaches v. Triangle with t_a = sqrt(1/a) = 0.5.
    const SegmentTiming tri = Trajectory::trapezoid(10.0, 4.0);
    EXPECT_DOUBLE_EQ(tri.accelTime, 0.5);
    EXPECT_DOUBLE_EQ(tri.peakRate, 2.0);
    EXPECT_DOUBLE_EQ(tri.duration, 1.0);
}

// @verifies REQ-PST-002
TEST(TrajectoryTest, RespectsLimitsAndSynchronizesJoints) {
    const ArmModel arm = ArmModel::uniform(3, 0.3, 3.0);
    const std::vector<Config> path = {Config::Zero(3), (Config(3) << 1.0, -0.5, 0.2).finished(),
                                      (Config(3) << 1.2, 1.5, -1.0).finished(),
                                      (Config(3) << 1.25, 1.5, -1.0).finished()};  // a very short last segment
    MotionLimits limits;
    limits.maxVelocity = (Eigen::VectorXd(3) << 1.0, 1.5, 2.0).finished();
    limits.maxAcceleration = (Eigen::VectorXd(3) << 2.0, 3.0, 4.0).finished();
    const Trajectory traj(arm, path, limits);

    ASSERT_EQ(traj.segments().size(), 3u);
    EXPECT_EQ(traj.sample(0.0).q, path.front());
    EXPECT_TRUE(traj.sample(traj.duration()).q.isApprox(path.back(), 1e-12));
    // At and after the end the reference is at rest (a non-zero acceleration here would make
    // acceleration feedforward push a constant torque forever).
    for (double t : {traj.duration(), traj.duration() + 1.0}) {
        EXPECT_EQ(traj.sample(t).qd.norm(), 0.0);
        EXPECT_EQ(traj.sample(t).qdd.norm(), 0.0);
    }

    bool someJointAtVelocityLimit = false;
    constexpr int kSamples = 4000;
    for (int k = 0; k <= kSamples; ++k) {
        const TrajectoryPoint p = traj.sample(traj.duration() * k / kSamples);
        for (int j = 0; j < 3; ++j) {
            EXPECT_LE(std::abs(p.qd[j]), limits.maxVelocity[j] * (1 + 1e-9));
            EXPECT_LE(std::abs(p.qdd[j]), limits.maxAcceleration[j] * (1 + 1e-9));
            someJointAtVelocityLimit =
                someJointAtVelocityLimit || std::abs(p.qd[j]) > limits.maxVelocity[j] * (1 - 1e-6);
        }
    }
    EXPECT_TRUE(someJointAtVelocityLimit);  // time-optimal: the binding joint reaches its limit

    // Every waypoint is reached at a segment boundary with all joints at rest (synchronized).
    for (std::size_t i = 0; i < traj.segments().size(); ++i) {
        const TrajectoryPoint p = traj.sample(traj.segments()[i].startTime);
        EXPECT_TRUE(p.q.isApprox(path[i], 1e-12)) << "waypoint " << i;
        EXPECT_NEAR(p.qd.norm(), 0.0, 1e-12);
    }
}

// @verifies REQ-PST-002
TEST(TrajectoryTest, StaysOnTheStraightSegmentThatWasCollisionChecked) {
    // q(t) must be a point of the straight C-space segment, i.e. a pure interpolation.
    const ArmModel arm = ArmModel::uniform(2, 0.5, 3.0);
    const Config a = cfg(-1.0, 0.5);
    const Config b = cfg(0.8, -0.7);
    const Trajectory traj(arm, {a, b}, MotionLimits::uniform(2, 1.0, 2.0));
    for (int k = 0; k <= 100; ++k) {
        const Config q = traj.sample(traj.duration() * k / 100.0).q;
        const double along = configDistance(arm, a, q) / configDistance(arm, a, b);
        EXPECT_TRUE(q.isApprox(interpolate(arm, a, b, along), 1e-9));
    }
}

// @verifies REQ-PST-002
TEST(TrajectoryTest, RejectsBadLimits) {
    const ArmModel arm = ArmModel::uniform(2, 0.5);
    EXPECT_THROW(Trajectory(arm, {cfg(0, 0), cfg(1, 1)}, MotionLimits::uniform(3, 1.0, 1.0)), std::invalid_argument);
    EXPECT_THROW(Trajectory(arm, {cfg(0, 0), cfg(1, 1)}, MotionLimits::uniform(2, 0.0, 1.0)), std::invalid_argument);
}

}  // namespace
}  // namespace amp
