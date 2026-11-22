/// @file rrt_star.hpp
/// @brief RRT* (Karaman and Frazzoli, 2011): asymptotically optimal RRT.
///
/// See docs/ALGORITHMS.md#rrt-star.
#pragma once

#include "amp/planner.hpp"

namespace amp {

/// RRT* with choose-parent and rewire steps over a shrinking neighbour radius
///   r(n) = min( gamma * (log n / n)^(1/d), maxRewireRadius ).
///
/// Keeps improving after the first solution until the iteration (or time) budget is spent,
/// unless stopAtFirstSolution is set. Records every improvement in PlanResult::costHistory.
class RrtStarPlanner final : public Planner {
public:
    /// Bind to a collision checker, which must outlive the planner.
    explicit RrtStarPlanner(const CollisionChecker& checker) : checker_(checker) {}
    /// Display name of the algorithm.
    [[nodiscard]] std::string name() const override { return "RRT*"; }
    /// Plan from `start` to `goal`; see Planner::plan and the class description.
    [[nodiscard]] PlanResult plan(const Config& start, const Config& goal, const PlannerParams& params,
                                  const PlannerCallbacks& callbacks = {}) const override;

    /// The theoretical lower bound on gamma for asymptotic optimality, using the volume of the
    /// whole joint-limit box as an upper bound on the free-space volume:
    ///   gamma* = 2 (1 + 1/d)^(1/d) (mu(C) / zeta_d)^(1/d)
    /// where zeta_d is the volume of the d-dimensional unit ball.
    [[nodiscard]] static double minimumGamma(const ArmModel& arm);

    /// Neighbour radius for a tree of n nodes.
    [[nodiscard]] static double neighborRadius(double gamma, std::size_t n, int dof, double maxRadius);

private:
    const CollisionChecker& checker_;
};

}  // namespace amp
