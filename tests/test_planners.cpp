/// @file test_planners.cpp
/// @brief Tests for RRT, RRT* and A*: valid paths, determinism, RRT* cost monotonicity and
/// convergence, A* optimality on a grid with a known answer, refusal above 3 DOF,
/// endpoint validation, cancellation and progress.
#include "amp/astar.hpp"
#include "amp/cspace.hpp"
#include "amp/planner.hpp"
#include "amp/rrt.hpp"
#include "amp/rrt_star.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <string>

namespace amp {
namespace {

Config cfg(double a, double b) {
    Config q(2);
    q << a, b;
    return q;
}

/// 2-DOF arm with a wall that blocks the direct sweep of the stretched arm. The upper arm
/// (0.5 m) passes under the wall's near end (x = 0.6 m), so the planner must fold the forearm
/// to get past: a short detour in C-space.
struct WallProblem {
    ArmModel arm{{0.5, 0.4}, std::vector<JointLimits>(2, JointLimits{-2.9, 2.9, false}), Vec2::Zero(), 0.02};
    Workspace ws = [] {
        Workspace w;
        w.obstacles.push_back(ConvexPolygon::rectangle(Vec2(0.6, -0.05), Vec2(1.0, 0.05)));
        return w;
    }();
    CollisionChecker checker{arm, ws};
    Config start = cfg(-0.8, 0.3);  // below the wall
    Config goal = cfg(0.9, -0.3);   // above the wall
};

/// Every node's parent chain must reach the root without cycles.
void expectTreeIsAcyclic(const std::vector<TreeNode>& tree) {
    for (std::size_t i = 0; i < tree.size(); ++i) {
        int node = static_cast<int>(i);
        std::size_t steps = 0;
        while (node >= 0 && steps <= tree.size()) {
            node = tree[static_cast<std::size_t>(node)].parent;
            ++steps;
        }
        ASSERT_LE(steps, tree.size()) << "cycle at node " << i;
    }
}

void expectValidPlan(const WallProblem& p, const PlanResult& r) {
    ASSERT_TRUE(r.success) << r.message;
    ASSERT_GE(r.path.size(), 2u);
    EXPECT_EQ(r.path.front(), p.start);
    EXPECT_EQ(r.path.back(), p.goal);  // ends exactly at the goal, not just near it
    EXPECT_TRUE(isPathValid(p.checker, r.path));
    EXPECT_NEAR(r.pathCost, pathLength(p.arm, r.path), 1e-9);
    EXPECT_EQ(r.nodes, r.tree.size());
    EXPECT_GT(r.iterations, 0);
    EXPECT_GE(r.planningTimeMs, 0.0);
    EXPECT_FALSE(r.costHistory.empty());
    expectTreeIsAcyclic(r.tree);
}

// ---------------------------------------------------------------------------
// Common behaviour
// ---------------------------------------------------------------------------

// @verifies REQ-PLN-001
// @verifies REQ-PLN-002
TEST(Planners, AllPlannersReturnValidPathsAroundAWall) {
    const WallProblem p;
    PlannerParams params;
    params.maxIterations = 3000;
    for (PlannerType type : {PlannerType::Rrt, PlannerType::RrtStar, PlannerType::AStar}) {
        SCOPED_TRACE(toString(type));
        const auto planner = makePlanner(type, p.checker);
        EXPECT_EQ(planner->name(), toString(type));
        expectValidPlan(p, planner->plan(p.start, p.goal, params));
    }
}

// @verifies REQ-PLN-008
TEST(Planners, RejectInvalidStartOrGoalWithoutPlanning) {
    const WallProblem p;
    const Config inWall = cfg(0.0, 0.0);  // arm lies along the wall
    ASSERT_FALSE(p.checker.isValid(inWall));
    for (PlannerType type : {PlannerType::Rrt, PlannerType::RrtStar, PlannerType::AStar}) {
        SCOPED_TRACE(toString(type));
        const auto planner = makePlanner(type, p.checker);
        const PlanResult a = planner->plan(inWall, p.goal, {});
        EXPECT_FALSE(a.success);
        EXPECT_NE(a.message.find("Start"), std::string::npos) << a.message;
        EXPECT_EQ(a.iterations, 0);
        const PlanResult b = planner->plan(p.start, inWall, {});
        EXPECT_NE(b.message.find("Goal"), std::string::npos) << b.message;
        const PlanResult c = planner->plan(Config::Zero(3), p.goal, {});
        EXPECT_FALSE(c.success);
    }
}

// @verifies REQ-PLN-003
TEST(Planners, SameSeedGivesIdenticalResults) {
    const WallProblem p;
    PlannerParams params;
    params.maxIterations = 1500;
    params.seed = 42;
    for (PlannerType type : {PlannerType::Rrt, PlannerType::RrtStar, PlannerType::AStar}) {
        SCOPED_TRACE(toString(type));
        const auto planner = makePlanner(type, p.checker);
        const PlanResult a = planner->plan(p.start, p.goal, params);
        const PlanResult b = planner->plan(p.start, p.goal, params);
        ASSERT_TRUE(a.success);
        EXPECT_EQ(a.iterations, b.iterations);
        EXPECT_EQ(a.tree.size(), b.tree.size());
        ASSERT_EQ(a.path.size(), b.path.size());
        for (std::size_t i = 0; i < a.path.size(); ++i) {
            EXPECT_EQ(a.path[i], b.path[i]);  // bit-identical
        }
        EXPECT_EQ(a.pathCost, b.pathCost);
    }
}

// @verifies REQ-PLN-003
TEST(Planners, DifferentSeedsExploreDifferently) {
    const WallProblem p;
    PlannerParams params;
    params.seed = 1;
    const RrtPlanner rrt(p.checker);
    const PlanResult a = rrt.plan(p.start, p.goal, params);
    params.seed = 2;
    const PlanResult b = rrt.plan(p.start, p.goal, params);
    EXPECT_FALSE(a.tree.size() == b.tree.size() && a.pathCost == b.pathCost);
}

// @verifies REQ-PLN-003
// @verifies REQ-NN-001
TEST(Planners, NearestNeighbourStrategyDoesNotChangeTheResult) {
    // Brute force and k-d tree return identical neighbours (including ties), so the whole
    // plan must be bit-identical: the strategy is a pure performance choice.
    const WallProblem p;
    PlannerParams params;
    params.maxIterations = 1500;
    for (PlannerType type : {PlannerType::Rrt, PlannerType::RrtStar}) {
        SCOPED_TRACE(toString(type));
        const auto planner = makePlanner(type, p.checker);
        params.nearestNeighbor = NnStrategy::BruteForce;
        const PlanResult a = planner->plan(p.start, p.goal, params);
        params.nearestNeighbor = NnStrategy::KdTree;
        const PlanResult b = planner->plan(p.start, p.goal, params);
        ASSERT_EQ(a.path.size(), b.path.size());
        for (std::size_t i = 0; i < a.path.size(); ++i) {
            EXPECT_EQ(a.path[i], b.path[i]);
        }
        EXPECT_EQ(a.tree.size(), b.tree.size());
    }
}

// @verifies REQ-PLN-009
TEST(Planners, CancellationStopsAndProgressIsReported) {
    const WallProblem p;
    PlannerParams params;
    params.maxIterations = 100000;
    params.stopAtFirstSolution = false;
    std::atomic<bool> cancel{false};
    int calls = 0;
    PlannerCallbacks cb;
    cb.cancel = &cancel;
    cb.progressInterval = 10;
    cb.onProgress = [&](const PlanProgress& pr) {
        ++calls;
        if (pr.iteration >= 200) {
            cancel = true;  // cancel from "another thread" (here: the callback)
        }
    };
    const PlanResult r = RrtStarPlanner(p.checker).plan(p.start, p.goal, params, cb);
    EXPECT_GE(calls, 20);
    EXPECT_LE(r.iterations, 201);
    EXPECT_EQ(r.message, "Cancelled by user");
}

// @verifies REQ-PLN-009
TEST(Planners, TreeSnapshotsGrowDuringPlanning) {
    const WallProblem p;
    PlannerParams params;
    params.maxIterations = 2000;
    for (PlannerType type : {PlannerType::Rrt, PlannerType::RrtStar, PlannerType::AStar}) {
        SCOPED_TRACE(toString(type));
        std::vector<std::size_t> sizes;
        PlannerCallbacks cb;
        cb.treeInterval = 50;
        cb.onTree = [&](const std::vector<TreeNode>& tree) { sizes.push_back(tree.size()); };
        const PlanResult r = makePlanner(type, p.checker)->plan(p.start, p.goal, params, cb);
        ASSERT_TRUE(r.success);
        ASSERT_GE(sizes.size(), 2u);
        EXPECT_TRUE(std::is_sorted(sizes.begin(), sizes.end()));  // trees only grow
        EXPECT_LE(sizes.back(), r.tree.size());
    }
}

// ---------------------------------------------------------------------------
// RRT
// ---------------------------------------------------------------------------

// @verifies REQ-PLN-004
TEST(Rrt, ExtensionsRespectStepSizeAndCostsAccumulate) {
    const WallProblem p;
    PlannerParams params;
    params.stepSize = 0.1;
    params.goalTolerance = 0.1;
    const PlanResult r = RrtPlanner(p.checker).plan(p.start, p.goal, params);
    ASSERT_TRUE(r.success);
    for (std::size_t i = 1; i < r.tree.size(); ++i) {
        const TreeNode& n = r.tree[i];
        const TreeNode& parent = r.tree[static_cast<std::size_t>(n.parent)];
        const double d = configDistance(p.arm, parent.q, n.q);
        EXPECT_LE(d, params.stepSize + 1e-12);
        EXPECT_NEAR(n.cost, parent.cost + d, 1e-9);
        EXPECT_LT(n.parent, static_cast<int>(i));  // RRT only ever appends leaves
    }
}

// @verifies REQ-PLN-004
TEST(Rrt, ReportsFailureWhenGoalIsEnclosed) {
    // The arm lies in a narrow slot. The folded goal pose fits inside it, but folding the
    // forearm back sweeps it out of the slot, so start and goal are in disconnected free regions.
    const ArmModel arm({0.5, 0.4}, std::vector<JointLimits>(2, JointLimits{-2.9, 2.9, false}), Vec2::Zero(), 0.02);
    Workspace ws;
    ws.bounds = Bounds{Vec2(-0.2, -0.2), Vec2(0.95, 0.2)};  // the arm can barely move inside this slot
    const CollisionChecker checker(arm, ws);
    PlannerParams params;
    params.maxIterations = 500;
    const Config start = cfg(0.0, 0.0);
    const Config goal = cfg(0.0, 2.85);
    ASSERT_TRUE(checker.isValid(start));
    ASSERT_TRUE(checker.isValid(goal));  // the failure must come from planning, not from validation
    const PlanResult r = RrtPlanner(checker).plan(start, goal, params);
    EXPECT_FALSE(r.success);
    EXPECT_EQ(r.iterations, 500);
    EXPECT_NE(r.message.find("iteration budget"), std::string::npos) << r.message;
}

// ---------------------------------------------------------------------------
// RRT*
// ---------------------------------------------------------------------------

// @verifies REQ-PLN-005
TEST(RrtStar, CostIsNonIncreasingAndTreeCostsAreConsistent) {
    const WallProblem p;
    PlannerParams params;
    params.maxIterations = 4000;
    const PlanResult r = RrtStarPlanner(p.checker).plan(p.start, p.goal, params);
    ASSERT_TRUE(r.success);
    ASSERT_FALSE(r.costHistory.empty());
    for (std::size_t i = 1; i < r.costHistory.size(); ++i) {
        EXPECT_LE(r.costHistory[i].second, r.costHistory[i - 1].second);
        EXPECT_GE(r.costHistory[i].first, r.costHistory[i - 1].first);
    }
    EXPECT_NEAR(r.costHistory.back().second, r.pathCost, 1e-9);
    // After rewiring, every node's cost must still equal parent cost + edge length.
    for (std::size_t i = 1; i < r.tree.size(); ++i) {
        const TreeNode& n = r.tree[i];
        const TreeNode& parent = r.tree[static_cast<std::size_t>(n.parent)];
        EXPECT_NEAR(n.cost, parent.cost + configDistance(p.arm, parent.q, n.q), 1e-9);
    }
}

// @verifies REQ-PLN-005
TEST(RrtStar, MoreIterationsNeverWorsenTheSolution) {
    // Determinism makes a shorter run an exact prefix of a longer one with the same seed.
    const WallProblem p;
    PlannerParams params;
    double previous = std::numeric_limits<double>::infinity();
    for (int iters : {1000, 2000, 4000}) {
        params.maxIterations = iters;
        const PlanResult r = RrtStarPlanner(p.checker).plan(p.start, p.goal, params);
        ASSERT_TRUE(r.success);
        EXPECT_LE(r.pathCost, previous + 1e-12);
        previous = r.pathCost;
    }
}

// @verifies REQ-PLN-005
TEST(RrtStar, ConvergesTowardTheStraightLineInFreeSpace) {
    const ArmModel arm = ArmModel::uniform(2, 0.5, 2.9);
    const CollisionChecker checker(arm, Workspace{});
    const Config start = cfg(-1.5, -1.0);
    const Config goal = cfg(1.5, 1.2);
    const double optimum = configDistance(arm, start, goal);
    PlannerParams params;
    params.goalBias = 0.0;  // make RRT* earn it rather than connecting straight to the goal
    params.maxIterations = 3000;
    const PlanResult rrt = RrtPlanner(checker).plan(start, goal, params);
    const PlanResult star = RrtStarPlanner(checker).plan(start, goal, params);
    ASSERT_TRUE(rrt.success);
    ASSERT_TRUE(star.success);
    EXPECT_LT(star.pathCost, optimum * 1.05);
    EXPECT_LE(star.pathCost, rrt.pathCost);
}

// @verifies REQ-PLN-005
TEST(RrtStar, NeighbourRadiusFollowsTheShrinkingFormula) {
    const ArmModel arm = ArmModel::uniform(2, 0.5, kPi);
    const double gamma = RrtStarPlanner::minimumGamma(arm);
    // d = 2, mu = (2pi)^2, zeta_2 = pi: gamma* = 2 sqrt(1.5) sqrt(4 pi^2 / pi) = 2 sqrt(6 pi).
    EXPECT_NEAR(gamma, 2.0 * std::sqrt(6.0 * kPi), 1e-12);
    const double r1000 = RrtStarPlanner::neighborRadius(gamma, 1000, 2, 100.0);
    EXPECT_NEAR(r1000, gamma * std::sqrt(std::log(1000.0) / 1000.0), 1e-12);
    EXPECT_LT(RrtStarPlanner::neighborRadius(gamma, 10000, 2, 100.0), r1000);  // shrinks
    EXPECT_EQ(RrtStarPlanner::neighborRadius(gamma, 1000, 2, 0.1), 0.1);       // capped
}

// ---------------------------------------------------------------------------
// A*
// ---------------------------------------------------------------------------

// @verifies REQ-PLN-006
TEST(AStar, OptimalOnFreeGridWithKnownAnswer) {
    // Limits [-2, 2] with 8 cells per axis: width h = 0.5, centres at -1.75, -1.25, ...
    const ArmModel arm({0.5, 0.5}, std::vector<JointLimits>(2, JointLimits{-2.0, 2.0, false}), Vec2::Zero(), 0.01);
    const CollisionChecker checker(arm, Workspace{}, {0.0, 0.01, false});
    PlannerParams params;
    params.gridResolution = 8;
    // From cell (0, 1) to cell (6, 3): 2 diagonal moves + 4 straight moves.
    const Config start = cfg(-1.75, -1.25);
    const Config goal = cfg(1.25, -0.25);
    const PlanResult r = AStarPlanner(checker).plan(start, goal, params);
    ASSERT_TRUE(r.success) << r.message;
    EXPECT_NEAR(r.pathCost, 0.5 * (2.0 * std::sqrt(2.0) + 4.0), 1e-12);
    EXPECT_EQ(r.path.size(), 7u);  // 6 moves through 7 cell centres
}

// @verifies REQ-PLN-006
TEST(AStar, MatchesDijkstraCostWithObstaclesButExpandsLess) {
    const WallProblem p;
    PlannerParams params;
    params.gridResolution = 40;
    const PlanResult astar = AStarPlanner(p.checker).plan(p.start, p.goal, params);
    params.heuristicWeight = 0.0;  // Dijkstra: no heuristic, guaranteed optimal
    const PlanResult dijkstra = AStarPlanner(p.checker).plan(p.start, p.goal, params);
    ASSERT_TRUE(astar.success);
    ASSERT_TRUE(dijkstra.success);
    EXPECT_NEAR(astar.pathCost, dijkstra.pathCost, 1e-9);
    EXPECT_LT(astar.iterations, dijkstra.iterations);
}

// @verifies REQ-PLN-006
TEST(AStar, ThreeDofUsesTwentySixConnectivity) {
    // One diagonal step in all three joints is a single grid move of length h * sqrt(3).
    const ArmModel arm({0.3, 0.3, 0.3}, std::vector<JointLimits>(3, JointLimits{-2.0, 2.0, false}), Vec2::Zero(), 0.01);
    const CollisionChecker checker(arm, Workspace{}, {0.0, 0.01, false});
    PlannerParams params;
    params.gridResolution = 8;
    Config start(3), goal(3);
    start << -0.25, -0.25, -0.25;
    goal << 0.25, 0.25, 0.25;
    const PlanResult r = AStarPlanner(checker).plan(start, goal, params);
    ASSERT_TRUE(r.success);
    EXPECT_EQ(r.path.size(), 2u);
    EXPECT_NEAR(r.pathCost, 0.5 * std::sqrt(3.0), 1e-12);
}

// @verifies REQ-PLN-007
TEST(AStar, RefusesMoreThanThreeDofWithAnExplanation) {
    const ArmModel arm = ArmModel::uniform(4, 0.25);
    const CollisionChecker checker(arm, Workspace{});
    const PlanResult r = AStarPlanner(checker).plan(Config::Zero(4), Config::Constant(4, 0.1), {});
    EXPECT_FALSE(r.success);
    EXPECT_NE(r.message.find("curse of dimensionality"), std::string::npos) << r.message;
    EXPECT_TRUE(r.tree.empty());
}

}  // namespace
}  // namespace amp
