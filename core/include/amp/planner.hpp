/// @file planner.hpp
/// @brief Common interface, parameters and result type for all motion planners.
///
/// Every planner answers the same question: given a start and a goal configuration and a
/// collision checker, find a sequence of configurations whose consecutive straight-line
/// segments are all valid edges (CollisionChecker::isEdgeValid).
///
/// Determinism: a planner's output depends only on (checker, start, goal, params). The seed
/// is part of params. The one opt-in exception is `timeLimitSeconds`, which makes the
/// amount of work depend on machine speed.
#pragma once

#include "amp/collision.hpp"
#include "amp/nearest_neighbor.hpp"
#include "amp/types.hpp"

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace amp {

/// Tuning parameters shared by the planners (each planner documents which it uses).
struct PlannerParams {
    std::uint64_t seed = 1;         ///< Random seed (RRT, RRT*). Same seed => same result.
    double stepSize = 0.15;         ///< Max C-space distance of one tree extension (rad).
    int maxIterations = 5000;       ///< Iteration budget (samples drawn / nodes expanded).
    double goalBias = 0.05;         ///< Probability of sampling the goal itself (RRT, RRT*).
    double goalTolerance = 0.15;    ///< Try connecting to the goal from nodes this close (rad).
    double timeLimitSeconds = 0.0;  ///< Optional wall-clock budget; 0 = none (keeps determinism).
    NnStrategy nearestNeighbor = NnStrategy::KdTree;

    // RRT*
    bool stopAtFirstSolution = false;  ///< RRT*: stop as soon as any path is found.
    double gammaScale = 1.0;           ///< RRT*: multiplier on the theoretical minimum gamma.
    double maxRewireRadius = 1.0;      ///< RRT*: cap on the shrinking neighbour radius (rad).

    // A*
    int gridResolution = 48;       ///< A*: cells per joint axis.
    double heuristicWeight = 1.0;  ///< A*: 1 = A*, 0 = Dijkstra, > 1 = weighted (not optimal).
};

/// One node of a search tree, for visualisation and path extraction.
struct TreeNode {
    Config q;
    int parent = -1;    ///< Index of the parent node, -1 for the root.
    double cost = 0.0;  ///< Cost-to-come from the start (C-space path length, rad).
};

/// Everything a planner reports.
struct PlanResult {
    bool success = false;
    std::string message;                              ///< Why it failed (or a note on success).
    std::vector<Config> path;                         ///< start, ..., goal (empty on failure).
    double pathCost = 0.0;                            ///< Sum of configDistance along the path (rad).
    int iterations = 0;                               ///< Iterations actually run.
    std::size_t nodes = 0;                            ///< Nodes in the tree (A*: cells expanded).
    double planningTimeMs = 0.0;                      ///< Wall-clock time.
    std::vector<TreeNode> tree;                       ///< The search tree (A*: the expanded cells).
    std::vector<std::pair<int, double>> costHistory;  ///< (iteration, best cost) each time it improves.
};

/// Progress snapshot passed to the progress callback.
struct PlanProgress {
    int iteration = 0;
    std::size_t nodes = 0;
    double bestCost = -1.0;  ///< -1 while no solution has been found.
};

/// Optional hooks for long-running planning (used by the GUI's worker thread).
struct PlannerCallbacks {
    /// If non-null and set to true, the planner stops at the next iteration and returns.
    const std::atomic<bool>* cancel = nullptr;
    /// Called every `progressInterval` iterations from the planning thread.
    std::function<void(const PlanProgress&)> onProgress;
    int progressInterval = 250;
    /// Called every `treeInterval` iterations with the current search tree (for live
    /// visualisation). The reference is only valid during the call: copy what you need.
    std::function<void(const std::vector<TreeNode>&)> onTree;
    int treeInterval = 0;  ///< 0 disables tree snapshots.
};

/// Interface implemented by RRT, RRT* and A*.
class Planner {
public:
    virtual ~Planner() = default;
    /// Short display name ("RRT", "RRT*", "A*").
    [[nodiscard]] virtual std::string name() const = 0;
    /// Plan from `start` to `goal`. Never throws for planning failures: inspect `success` and `message`.
    [[nodiscard]] virtual PlanResult plan(const Config& start, const Config& goal, const PlannerParams& params,
                                          const PlannerCallbacks& callbacks = {}) const = 0;
};

/// The available planners.
enum class PlannerType { Rrt, RrtStar, AStar };

/// Construct a planner bound to `checker` (which must outlive the planner).
[[nodiscard]] std::unique_ptr<Planner> makePlanner(PlannerType type, const CollisionChecker& checker);

/// Display name of a planner type.
[[nodiscard]] const char* toString(PlannerType type);

/// Sum of configDistance between consecutive configurations.
[[nodiscard]] double pathLength(const ArmModel& arm, const std::vector<Config>& path);

/// True if `path` is non-empty and every consecutive pair is a valid edge.
[[nodiscard]] bool isPathValid(const CollisionChecker& checker, const std::vector<Config>& path);

/// Move from `from` toward `to` by at most `stepSize` (C-space distance).
[[nodiscard]] Config steer(const ArmModel& arm, const Config& from, const Config& to, double stepSize);

}  // namespace amp
