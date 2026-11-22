/// @file astar.hpp
/// @brief A* search on a discretised configuration-space grid.
///
/// See docs/ALGORITHMS.md#a-star.
#pragma once

#include "amp/planner.hpp"

namespace amp {

/// Grid A* over joint space.
///
/// The C-space box (joint limits, or the full circle for continuous joints) is divided into
/// gridResolution cells per joint. Cells are connected to all 3^d - 1 neighbours (8 for
/// 2 DOF, 26 for 3 DOF). Moving between neighbouring cells is allowed if that straight edge is
/// valid. Edge cost and heuristic are both configDistance, so the heuristic is admissible
/// and consistent, and A* returns the cheapest path *on this grid*.
///
/// Refuses arms with more than 3 joints: the grid has resolution^d cells, which is the curse
/// of dimensionality in its plainest form.
///
/// Uses: gridResolution, heuristicWeight, timeLimitSeconds. maxIterations is ignored: the
/// grid itself bounds the work (at most resolution^d expansions). The seed is irrelevant:
/// A* is deterministic by construction.
class AStarPlanner final : public Planner {
public:
    static constexpr int kMaxDof = 3;  ///< Largest DOF the grid planner accepts.

    /// Bind to a collision checker, which must outlive the planner.
    explicit AStarPlanner(const CollisionChecker& checker) : checker_(checker) {}
    /// Display name of the algorithm.
    [[nodiscard]] std::string name() const override { return "A*"; }
    /// Plan from `start` to `goal`; see Planner::plan and the class description.
    [[nodiscard]] PlanResult plan(const Config& start, const Config& goal, const PlannerParams& params,
                                  const PlannerCallbacks& callbacks = {}) const override;

private:
    const CollisionChecker& checker_;
};

}  // namespace amp
