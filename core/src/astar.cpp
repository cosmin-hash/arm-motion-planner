/// @file astar.cpp
/// @brief A* over a uniform joint-space grid with lazy validity checks.
///
/// Algorithm summary
/// -----------------
/// Snap start and goal to their grid cells. Expand cells in order of f = g + w h, where g is
/// the cost-to-come along grid edges and h = configDistance(cell, goal). Because h never
/// overestimates the remaining grid path cost (straight lines are shortest) and satisfies
/// the triangle inequality, A* with w = 1 expands each cell at most once and returns the
/// cheapest grid path. Cell and edge validity are evaluated lazily, only when the search
/// reaches them, which is what makes grid search usable at all with an expensive checker.
///
/// Details: docs/ALGORITHMS.md#a-star
#include "amp/astar.hpp"

#include "amp/cspace.hpp"

#include <queue>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "planner_internal.hpp"

namespace amp {

namespace {

/// Geometry of the grid along one joint axis.
struct Axis {
    double lower = 0.0;
    double width = 0.0;  ///< Cell width (rad).
    bool wraps = false;  ///< Continuous joint: the last cell neighbours the first.
};

struct OpenEntry {
    double f;
    double g;
    int cell;
};

/// Priority order: lowest f first; on ties prefer larger g (closer to the goal), then lower
/// cell index. The explicit tie-break makes the expansion order, and hence the result, deterministic.
struct OpenOrder {
    bool operator()(const OpenEntry& a, const OpenEntry& b) const {
        if (a.f != b.f) {
            return a.f > b.f;
        }
        if (a.g != b.g) {
            return a.g < b.g;
        }
        return a.cell > b.cell;
    }
};

}  // namespace

PlanResult AStarPlanner::plan(const Config& start, const Config& goal, const PlannerParams& params,
                              const PlannerCallbacks& callbacks) const {
    const detail::PlanRun run(params, callbacks);
    PlanResult result;
    const ArmModel& arm = checker_.arm();
    const int dof = arm.dof();
    const int res = params.gridResolution;

    if (dof > kMaxDof) {
        result.message = "A* grid search refused for a " + std::to_string(dof) + "-DOF arm: a grid with " +
                         std::to_string(res) + " cells per joint has " + std::to_string(res) + "^" +
                         std::to_string(dof) +
                         " cells, which grows exponentially with the number of joints (curse of "
                         "dimensionality). Use RRT or RRT* for more than 3 joints.";
        return result;
    }
    if (res < 2) {
        result.message = "A* grid resolution must be at least 2";
        return result;
    }
    if (!detail::checkEndpoints(checker_, start, goal, result)) {
        result.planningTimeMs = run.elapsedMs();
        return result;
    }

    // ----- Grid geometry -------------------------------------------------------------
    std::array<Axis, kMaxDof> axes{};
    for (int i = 0; i < dof; ++i) {
        const JointLimits& lim = arm.limits(i);
        Axis& ax = axes[static_cast<std::size_t>(i)];
        ax.wraps = lim.continuous;
        ax.lower = lim.continuous ? -kPi : lim.lower;
        ax.width = (lim.continuous ? kTwoPi : lim.upper - lim.lower) / res;
    }
    std::size_t cellCount = 1;
    for (int i = 0; i < dof; ++i) {
        cellCount *= static_cast<std::size_t>(res);
    }

    const auto coordsOf = [&](int cell) {
        std::array<int, kMaxDof> c{};
        for (int i = 0; i < dof; ++i) {
            c[static_cast<std::size_t>(i)] = cell % res;
            cell /= res;
        }
        return c;
    };
    const auto indexOf = [&](const std::array<int, kMaxDof>& c) {
        int idx = 0;
        for (int i = dof - 1; i >= 0; --i) {
            idx = idx * res + c[static_cast<std::size_t>(i)];
        }
        return idx;
    };
    const auto centerOf = [&](int cell) {
        const auto c = coordsOf(cell);
        Config q(dof);
        for (int i = 0; i < dof; ++i) {
            const Axis& ax = axes[static_cast<std::size_t>(i)];
            q[i] = ax.lower + (c[static_cast<std::size_t>(i)] + 0.5) * ax.width;
        }
        return q;
    };
    const auto cellOf = [&](const Config& q) {
        std::array<int, kMaxDof> c{};
        for (int i = 0; i < dof; ++i) {
            const Axis& ax = axes[static_cast<std::size_t>(i)];
            const int k = static_cast<int>(std::floor((q[i] - ax.lower) / ax.width));
            c[static_cast<std::size_t>(i)] = std::clamp(k, 0, res - 1);
        }
        return indexOf(c);
    };

    // All 3^d - 1 neighbour offsets: 8-connected in 2D, 26-connected in 3D.
    std::vector<std::array<int, kMaxDof>> offsets;
    {
        int total = 1;
        for (int i = 0; i < dof; ++i) {
            total *= 3;
        }
        for (int k = 0; k < total; ++k) {
            std::array<int, kMaxDof> o{};
            bool zero = true;
            int rem = k;
            for (int i = 0; i < dof; ++i) {
                o[static_cast<std::size_t>(i)] = rem % 3 - 1;
                zero = zero && o[static_cast<std::size_t>(i)] == 0;
                rem /= 3;
            }
            if (!zero) {
                offsets.push_back(o);
            }
        }
    }

    // ----- Connect start and goal to the grid ----------------------------------------
    const int startCell = cellOf(start);
    const int goalCell = cellOf(goal);
    const Config startCenter = centerOf(startCell);
    const Config goalCenter = centerOf(goalCell);
    if (!checker_.isEdgeValid(start, startCenter) || !checker_.isEdgeValid(goalCenter, goal)) {
        result.message = "Start or goal cannot be connected to the centre of its grid cell at resolution " +
                         std::to_string(res) + " (cell too coarse for the free space around it)";
        result.planningTimeMs = run.elapsedMs();
        return result;
    }

    // ----- Search ---------------------------------------------------------------------
    constexpr double kInf = std::numeric_limits<double>::infinity();
    std::vector<double> g(cellCount, kInf);
    std::vector<int> parent(cellCount, -1);
    std::vector<int> treeIndex(cellCount, -1);
    std::vector<char> closed(cellCount, 0);
    std::vector<signed char> validity(cellCount, -1);  // -1 unknown, 0 invalid, 1 valid (lazy cache)
    const double w = params.heuristicWeight;

    std::priority_queue<OpenEntry, std::vector<OpenEntry>, OpenOrder> open;
    g[static_cast<std::size_t>(startCell)] = 0.0;
    open.push({w * configDistance(arm, startCenter, goalCenter), 0.0, startCell});

    int expansions = 0;
    bool found = false;
    while (!open.empty()) {
        if (run.shouldStop(result.message)) {
            break;
        }
        const OpenEntry top = open.top();
        open.pop();
        const auto cur = static_cast<std::size_t>(top.cell);
        if (closed[cur] != 0) {
            continue;  // stale queue entry (a cheaper one was already expanded)
        }
        closed[cur] = 1;
        ++expansions;

        const Config q = centerOf(top.cell);
        const int parentTree = parent[cur] >= 0 ? treeIndex[static_cast<std::size_t>(parent[cur])] : -1;
        treeIndex[cur] = static_cast<int>(result.tree.size());
        result.tree.push_back(TreeNode{q, parentTree, g[cur]});
        run.progress(expansions, result.tree.size(), -1.0);
        run.tree(expansions, result.tree);

        if (top.cell == goalCell) {
            found = true;
            break;
        }

        const auto c = coordsOf(top.cell);
        for (const auto& o : offsets) {
            std::array<int, kMaxDof> nc{};
            bool inside = true;
            for (int i = 0; i < dof; ++i) {
                const auto ui = static_cast<std::size_t>(i);
                int v = c[ui] + o[ui];
                if (axes[ui].wraps) {
                    v = (v + res) % res;
                } else if (v < 0 || v >= res) {
                    inside = false;
                }
                nc[ui] = v;
            }
            if (!inside) {
                continue;
            }
            const int next = indexOf(nc);
            const auto un = static_cast<std::size_t>(next);
            if (closed[un] != 0) {
                continue;
            }
            const Config qn = centerOf(next);
            if (validity[un] < 0) {
                validity[un] = checker_.isValid(qn) ? 1 : 0;
            }
            if (validity[un] == 0) {
                continue;
            }
            const double tentative = g[cur] + configDistance(arm, q, qn);
            // Only pay for the edge check when this route would actually improve the neighbour.
            if (tentative < g[un] && checker_.isEdgeValid(q, qn)) {
                g[un] = tentative;
                parent[un] = top.cell;
                open.push({tentative + w * configDistance(arm, qn, goalCenter), tentative, next});
            }
        }
    }

    result.iterations = expansions;
    result.nodes = result.tree.size();
    if (found) {
        // start -> start cell centre -> ... -> goal cell centre -> goal (skipping duplicates).
        std::vector<Config> cells;
        for (int cell = goalCell; cell >= 0; cell = parent[static_cast<std::size_t>(cell)]) {
            cells.push_back(centerOf(cell));
        }
        std::reverse(cells.begin(), cells.end());
        result.path.push_back(start);
        for (const Config& qc : cells) {
            if (configDistance(arm, result.path.back(), qc) > 0.0) {
                result.path.push_back(qc);
            }
        }
        if (configDistance(arm, result.path.back(), goal) > 0.0) {
            result.path.push_back(goal);
        }
        result.success = true;
        result.pathCost = pathLength(arm, result.path);
        result.costHistory.emplace_back(expansions, result.pathCost);
        result.message = "Path found (optimal on the grid)";
    } else if (result.message.empty()) {
        result.message = "No path exists on the grid at resolution " + std::to_string(res) +
                         " (the free space may be narrower than one cell)";
    }
    result.planningTimeMs = run.elapsedMs();
    return result;
}

}  // namespace amp
