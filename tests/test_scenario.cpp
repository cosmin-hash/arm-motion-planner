/// @file test_scenario.cpp
/// @brief Scenario JSON parsing, error reporting, and checks on every shipped scenario.
#include "amp/astar.hpp"
#include "amp/dynamics.hpp"
#include "amp/kinematics.hpp"
#include "amp/rrt.hpp"
#include "amp/scenario.hpp"
#include "amp/simulation.hpp"
#include "amp/smoothing.hpp"

#include <gtest/gtest.h>

#include <string>

namespace amp {
namespace {

const char* kMinimal = R"({
  "name": "Minimal",
  "arm": { "link_radius": 0.01,
           "joints": [ { "length": 0.5, "lower_deg": -90, "upper_deg": 90 },
                       { "length": 0.5, "continuous": true } ] },
  "obstacles": [ { "type": "circle", "center": [0.0, 0.8], "radius": 0.1 },
                 { "type": "rectangle", "min": [0.6, 0.6], "max": [0.7, 0.7] },
                 { "type": "polygon", "vertices": [[-0.8, 0.0], [-0.6, 0.0], [-0.7, 0.2]] } ],
  "bounds": { "min": [-1.5, -1.5], "max": [1.5, 1.5] },
  "collision": { "safety_margin": 0.002, "self_collision": false },
  "start": { "joints_deg": [0, 0] },
  "goal":  { "joints_deg": [45, -90] },
  "planner": { "step_size": 0.2, "max_iterations": 1234, "seed": 9, "grid_resolution": 20 }
})";

std::string withGoal(const std::string& goalJson) {
    std::string s = kMinimal;
    const auto pos = s.find(R"("goal":)");
    const auto end = s.find('\n', pos);
    return s.substr(0, pos) + R"("goal": )" + goalJson + "," + s.substr(end);
}

// @verifies REQ-SCN-001
TEST(Scenario, ParsesAllFieldsAndConvertsDegrees) {
    const Scenario s = parseScenario(kMinimal);
    EXPECT_EQ(s.name, "Minimal");
    EXPECT_EQ(s.arm.dof(), 2);
    EXPECT_NEAR(s.arm.limits(0).upper, kPi / 2, 1e-15);
    EXPECT_TRUE(s.arm.limits(1).continuous);
    EXPECT_DOUBLE_EQ(s.arm.linkRadius(), 0.01);
    EXPECT_EQ(s.workspace.obstacles.size(), 3u);
    ASSERT_TRUE(s.workspace.bounds.has_value());
    EXPECT_DOUBLE_EQ(s.workspace.bounds->max.x(), 1.5);
    EXPECT_DOUBLE_EQ(s.collision.safetyMargin, 0.002);
    EXPECT_FALSE(s.collision.selfCollision);
    EXPECT_NEAR(s.goal[0], kPi / 4, 1e-15);
    EXPECT_NEAR(s.goal[1], -kPi / 2, 1e-15);
    EXPECT_FALSE(s.goalTarget.has_value());
    EXPECT_DOUBLE_EQ(s.planner.stepSize, 0.2);
    EXPECT_EQ(s.planner.maxIterations, 1234);
    EXPECT_EQ(s.planner.seed, 9u);
    EXPECT_EQ(s.planner.gridResolution, 20);
    EXPECT_DOUBLE_EQ(s.planner.goalBias, PlannerParams{}.goalBias);  // default kept
}

// @verifies REQ-SCN-001
TEST(Scenario, ResolvesEndEffectorGoalThroughIk) {
    const Scenario up = parseScenario(withGoal(R"({ "end_effector": [0.5, 0.5], "elbow": "up" })"));
    ASSERT_TRUE(up.goalTarget.has_value());
    EXPECT_LT((endEffectorPosition(up.arm, up.goal) - Vec2(0.5, 0.5)).norm(), 1e-9);
    EXPECT_LE(up.goal[1], 0.0);
    const Scenario down = parseScenario(withGoal(R"({ "end_effector": [0.5, 0.5], "elbow": "down" })"));
    EXPECT_GE(down.goal[1], 0.0);
    const Scenario dls = parseScenario(withGoal(R"({ "end_effector": [0.3, -0.4], "ik_seed_deg": [-30, -30] })"));
    EXPECT_LT((endEffectorPosition(dls.arm, dls.goal) - Vec2(0.3, -0.4)).norm(), 1e-5);
}

// @verifies REQ-SCN-001
TEST(Scenario, ReportsMalformedInputClearly) {
    const auto expectError = [](const std::string& json, const std::string& fragment) {
        try {
            (void)parseScenario(json);
            ADD_FAILURE() << "expected ScenarioError containing '" << fragment << "'";
        } catch (const ScenarioError& e) {
            EXPECT_NE(std::string(e.what()).find(fragment), std::string::npos) << e.what();
        }
    };
    expectError("{ not json", "JSON syntax error");
    expectError(R"({ "start": {}, "goal": {} })", "arm");
    expectError(withGoal(R"({ "joints_deg": [1, 2, 3] })"), "goal.joints_deg");
    expectError(withGoal(R"({ "end_effector": [5.0, 0.0] })"), "inverse kinematics failed");
    expectError(withGoal(R"({ "end_effector": [0.5, 0.5], "elbow": "sideways" })"), "goal.elbow");
    expectError(withGoal(R"({ })"), "joints_deg' or 'end_effector");

    std::string badObstacle = kMinimal;
    badObstacle.replace(badObstacle.find(R"("circle")"), 8, R"("hexagon")");
    expectError(badObstacle, "unknown obstacle type 'hexagon'");

    std::string concave = kMinimal;
    concave.replace(concave.find("[-0.7, 0.2]"), 11, "[-0.7, 0.2], [-0.7, 0.05]");
    expectError(concave, "not convex");

    EXPECT_THROW((void)loadScenario("does/not/exist.json"), ScenarioError);
}

// @verifies REQ-SCN-002
TEST(Scenario, ShippedScenariosLoadWithValidStartAndGoal) {
    const auto files = listScenarioFiles(AMP_SCENARIO_DIR);
    ASSERT_GE(files.size(), 5u) << "expected at least five scenarios in " << AMP_SCENARIO_DIR;
    for (const auto& file : files) {
        SCOPED_TRACE(file.filename().string());
        const Scenario s = loadScenario(file);
        EXPECT_FALSE(s.description.empty());
        const CollisionChecker checker(s.arm, s.workspace, s.collision);
        EXPECT_TRUE(checker.isValid(s.start)) << toString(checker.check(s.start).kind);
        EXPECT_TRUE(checker.isValid(s.goal)) << toString(checker.check(s.goal).kind);
        // Every shipped scenario must be solvable: RRT with the file's own parameters succeeds.
        const PlanResult r = RrtPlanner(checker).plan(s.start, s.goal, s.planner);
        EXPECT_TRUE(r.success) << r.message;
    }
}

// @verifies REQ-SCN-002
// @verifies REQ-PLN-006
TEST(Scenario, CoarseGridScenarioDefeatsAStarButNotRrt) {
    // The scenario exists to show resolution incompleteness: the passage is narrower than a
    // grid cell, so A* reports no path although one exists (RRT finds it).
    const Scenario s = loadScenario(std::string(AMP_SCENARIO_DIR) + "/04_astar_coarse_grid_fails.json");
    const CollisionChecker checker(s.arm, s.workspace, s.collision);
    const PlanResult astar = AStarPlanner(checker).plan(s.start, s.goal, s.planner);
    EXPECT_FALSE(astar.success);
    const PlanResult rrt = RrtPlanner(checker).plan(s.start, s.goal, s.planner);
    EXPECT_TRUE(rrt.success) << rrt.message;
    PlannerParams fine = s.planner;
    fine.gridResolution = 4 * s.planner.gridResolution;  // a fine enough grid does find it
    EXPECT_TRUE(AStarPlanner(checker).plan(s.start, s.goal, fine).success);
}

// @verifies REQ-CTL-008
TEST(Scenario, ExecutedMotionWithFeedforwardStaysCollisionFree) {
    // End-to-end: plan, smooth, time-parameterize and execute every shipped scenario with the
    // default PID gains plus acceleration feedforward, then check every logged *actual* pose of
    // the uninflated arm. The planner only guarantees the reference; this checks that the
    // tracking error stays inside the safety margin.
    for (const auto& file : listScenarioFiles(AMP_SCENARIO_DIR)) {
        SCOPED_TRACE(file.filename().string());
        const Scenario s = loadScenario(file);
        const CollisionChecker checker(s.arm, s.workspace, s.collision);
        const PlanResult plan = RrtPlanner(checker).plan(s.start, s.goal, s.planner);
        ASSERT_TRUE(plan.success);
        const SmoothingResult smooth = shortcutSmooth(checker, plan.path);
        const Trajectory traj(s.arm, smooth.path, MotionLimits::uniform(s.arm.dof(), 1.5, 3.0));
        TrackingOptions options;
        options.accelerationFeedforward = true;
        const TrackingLog log = simulateTracking(s.arm, JointDynamics::defaultsFor(s.arm), traj, options);

        CollisionOptions exact = s.collision;
        exact.safetyMargin = 0.0;
        exact.maxStepDisplacement = 0.001;
        const CollisionChecker exactChecker(s.arm, s.workspace, exact);
        int collisions = 0;
        for (const TrackingSample& sample : log.samples) {
            collisions += exactChecker.isValid(sample.actual) ? 0 : 1;
        }
        EXPECT_EQ(collisions, 0);
        EXPECT_LT(log.maxAbsError.maxCoeff(), 0.005);  // 5 mrad
    }
}

}  // namespace
}  // namespace amp
