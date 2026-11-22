/// @file rrt.hpp
/// @brief Rapidly-exploring Random Tree (LaValle, 1998).
///
/// See docs/ALGORITHMS.md#rrt.
#pragma once

#include "amp/planner.hpp"

namespace amp {

/// Basic RRT with goal bias.
///
/// Uses: seed, stepSize, maxIterations, goalBias, goalTolerance, timeLimitSeconds,
/// nearestNeighbor. Returns the first path found. Probabilistically complete, not optimal.
class RrtPlanner final : public Planner {
public:
    /// Bind to a collision checker, which must outlive the planner.
    explicit RrtPlanner(const CollisionChecker& checker) : checker_(checker) {}
    /// Display name of the algorithm.
    [[nodiscard]] std::string name() const override { return "RRT"; }
    /// Plan from `start` to `goal`; see Planner::plan and the class description.
    [[nodiscard]] PlanResult plan(const Config& start, const Config& goal, const PlannerParams& params,
                                  const PlannerCallbacks& callbacks = {}) const override;

private:
    const CollisionChecker& checker_;
};

}  // namespace amp
