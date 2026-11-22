/// @file planner.cpp
/// @brief Planner factory and helpers shared by all planners.
#include "amp/planner.hpp"

#include "amp/astar.hpp"
#include "amp/cspace.hpp"
#include "amp/rrt.hpp"
#include "amp/rrt_star.hpp"

#include <algorithm>

#include "planner_internal.hpp"

namespace amp {

std::unique_ptr<Planner> makePlanner(PlannerType type, const CollisionChecker& checker) {
    switch (type) {
        case PlannerType::Rrt:
            return std::make_unique<RrtPlanner>(checker);
        case PlannerType::RrtStar:
            return std::make_unique<RrtStarPlanner>(checker);
        case PlannerType::AStar:
            return std::make_unique<AStarPlanner>(checker);
    }
    return nullptr;
}

const char* toString(PlannerType type) {
    switch (type) {
        case PlannerType::Rrt:
            return "RRT";
        case PlannerType::RrtStar:
            return "RRT*";
        case PlannerType::AStar:
            return "A*";
    }
    return "Unknown";
}

double pathLength(const ArmModel& arm, const std::vector<Config>& path) {
    double total = 0.0;
    for (std::size_t i = 1; i < path.size(); ++i) {
        total += configDistance(arm, path[i - 1], path[i]);
    }
    return total;
}

bool isPathValid(const CollisionChecker& checker, const std::vector<Config>& path) {
    if (path.empty()) {
        return false;
    }
    if (path.size() == 1) {
        return checker.isValid(path.front());
    }
    for (std::size_t i = 1; i < path.size(); ++i) {
        if (!checker.isEdgeValid(path[i - 1], path[i])) {
            return false;
        }
    }
    return true;
}

Config steer(const ArmModel& arm, const Config& from, const Config& to, double stepSize) {
    const double d = configDistance(arm, from, to);
    if (d <= stepSize) {
        return to;
    }
    return interpolate(arm, from, to, stepSize / d);
}

namespace detail {

bool checkEndpoints(const CollisionChecker& checker, const Config& start, const Config& goal, PlanResult& result) {
    const int dof = checker.arm().dof();
    if (start.size() != dof || goal.size() != dof) {
        result.message = "Start or goal has the wrong number of joints";
        return false;
    }
    const CollisionReport s = checker.check(start);
    if (!s.ok()) {
        result.message = std::string("Start configuration is invalid: ") + toString(s.kind);
        return false;
    }
    const CollisionReport g = checker.check(goal);
    if (!g.ok()) {
        result.message = std::string("Goal configuration is invalid: ") + toString(g.kind);
        return false;
    }
    return true;
}

std::vector<Config> extractPath(const std::vector<TreeNode>& tree, int node) {
    std::vector<Config> path;
    for (int i = node; i >= 0; i = tree[static_cast<std::size_t>(i)].parent) {
        path.push_back(tree[static_cast<std::size_t>(i)].q);
    }
    std::reverse(path.begin(), path.end());
    return path;
}

}  // namespace detail
}  // namespace amp
