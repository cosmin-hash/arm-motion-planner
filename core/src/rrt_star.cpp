/// @file rrt_star.cpp
/// @brief RRT*: RRT plus choose-parent and rewiring over a shrinking neighbourhood.
///
/// Algorithm summary
/// -----------------
/// Like RRT, but each new node:
///  1. choose-parent: connects to whichever neighbour within radius r(n) gives the cheapest
///     collision-free cost-to-come (not just the nearest node);
///  2. rewire: offers itself as a cheaper parent to every neighbour, and reparents those it
///     improves (propagating the saving to their subtrees).
/// With r(n) = gamma (log n / n)^(1/d) and gamma above a threshold, the best path converges
/// to the optimum as samples go to infinity (asymptotic optimality), while each iteration
/// examines only O(log n) neighbours on average.
///
/// Details: docs/ALGORITHMS.md#rrt-star
#include "amp/rrt_star.hpp"

#include "amp/cspace.hpp"
#include "amp/rng.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "planner_internal.hpp"

namespace amp {

double RrtStarPlanner::minimumGamma(const ArmModel& arm) {
    const int d = arm.dof();
    double volume = 1.0;  // mu(C): product of joint ranges, an upper bound on mu(C_free)
    for (int i = 0; i < d; ++i) {
        const JointLimits& lim = arm.limits(i);
        volume *= lim.continuous ? kTwoPi : (lim.upper - lim.lower);
    }
    // Volume of the unit d-ball: pi^(d/2) / Gamma(d/2 + 1).
    const double dd = static_cast<double>(d);
    const double unitBall = std::pow(kPi, dd / 2.0) / std::tgamma(dd / 2.0 + 1.0);
    return 2.0 * std::pow(1.0 + 1.0 / dd, 1.0 / dd) * std::pow(volume / unitBall, 1.0 / dd);
}

double RrtStarPlanner::neighborRadius(double gamma, std::size_t n, int dof, double maxRadius) {
    if (n < 2) {
        return maxRadius;
    }
    const double nn = static_cast<double>(n);
    return std::min(gamma * std::pow(std::log(nn) / nn, 1.0 / dof), maxRadius);
}

PlanResult RrtStarPlanner::plan(const Config& start, const Config& goal, const PlannerParams& params,
                                const PlannerCallbacks& callbacks) const {
    const detail::PlanRun run(params, callbacks);
    PlanResult result;
    if (!detail::checkEndpoints(checker_, start, goal, result)) {
        result.planningTimeMs = run.elapsedMs();
        return result;
    }

    const ArmModel& arm = checker_.arm();
    const int dof = arm.dof();
    const double gamma = params.gammaScale * minimumGamma(arm);
    Rng rng(params.seed);
    const auto nn = makeNearestNeighbor(params.nearestNeighbor, arm);

    std::vector<TreeNode>& tree = result.tree;
    std::vector<std::vector<int>> children;  // needed to propagate cost changes after rewiring
    tree.push_back(TreeNode{start, -1, 0.0});
    children.emplace_back();
    nn->add(start);

    std::vector<int> goalParents;  // nodes with a valid edge to the goal
    int bestParent = -1;
    double bestCost = std::numeric_limits<double>::infinity();

    const auto node = [&](int i) -> TreeNode& { return tree[static_cast<std::size_t>(i)]; };
    const auto childrenOf = [&](int i) -> std::vector<int>& { return children[static_cast<std::size_t>(i)]; };

    // Reduce the cost of `root` and its whole subtree by `delta` (iterative DFS).
    const auto propagateCostDecrease = [&](int root, double delta) {
        std::vector<int> stack{root};
        while (!stack.empty()) {
            const int i = stack.back();
            stack.pop_back();
            node(i).cost -= delta;
            for (int c : childrenOf(i)) {
                stack.push_back(c);
            }
        }
    };

    const auto considerGoal = [&](int i) {
        const double d = configDistance(arm, node(i).q, goal);
        if (d <= params.goalTolerance && (d == 0.0 || checker_.isEdgeValid(node(i).q, goal))) {
            goalParents.push_back(i);
        }
    };
    considerGoal(0);

    int iteration = 0;
    while (iteration < params.maxIterations) {
        // Best solution so far: costs can drop through rewiring, so re-evaluate every candidate.
        for (int g : goalParents) {
            const double c = node(g).cost + configDistance(arm, node(g).q, goal);
            if (c < bestCost - 1e-12) {
                bestCost = c;
                bestParent = g;
                result.costHistory.emplace_back(iteration, bestCost);
            }
        }
        if (bestParent >= 0 && params.stopAtFirstSolution) {
            break;
        }
        if (run.shouldStop(result.message)) {
            break;
        }
        ++iteration;
        run.progress(iteration, tree.size(), bestParent >= 0 ? bestCost : -1.0);
        run.tree(iteration, tree);

        // 1. Sample and steer, exactly as in RRT.
        const Config target = rng.uniform01() < params.goalBias ? goal : sampleUniform(arm, rng);
        const auto nearest = static_cast<int>(nn->nearest(target));
        const Config qNew = steer(arm, node(nearest).q, target, params.stepSize);
        if (configDistance(arm, node(nearest).q, qNew) == 0.0 || !checker_.isValid(qNew)) {
            continue;  // cheap rejection before any edge checks
        }

        // 2. Neighbourhood: radius shrinks as the tree grows. Always include the nearest node so
        //    the tree can extend even when the radius is smaller than the step size.
        const double radius = neighborRadius(gamma, tree.size(), dof, params.maxRewireRadius);
        std::vector<std::size_t> near = nn->withinRadius(qNew, radius);
        if (std::find(near.begin(), near.end(), static_cast<std::size_t>(nearest)) == near.end()) {
            near.insert(std::upper_bound(near.begin(), near.end(), static_cast<std::size_t>(nearest)),
                        static_cast<std::size_t>(nearest));
        }

        // 3. Choose parent: try candidates in order of resulting cost, accept the first whose
        //    edge is valid. Sorting first means we usually check only one or two edges.
        struct Candidate {
            int index;
            double cost;
        };
        std::vector<Candidate> candidates;
        candidates.reserve(near.size());
        for (std::size_t n : near) {
            const int i = static_cast<int>(n);
            candidates.push_back({i, node(i).cost + configDistance(arm, node(i).q, qNew)});
        }
        std::stable_sort(candidates.begin(), candidates.end(),
                         [](const Candidate& a, const Candidate& b) { return a.cost < b.cost; });
        int parent = -1;
        double newCost = 0.0;
        for (const Candidate& c : candidates) {
            if (checker_.isEdgeValid(node(c.index).q, qNew)) {
                parent = c.index;
                newCost = c.cost;
                break;
            }
        }
        if (parent < 0) {
            continue;
        }

        tree.push_back(TreeNode{qNew, parent, newCost});
        children.emplace_back();
        const int newIndex = static_cast<int>(tree.size()) - 1;
        childrenOf(parent).push_back(newIndex);
        nn->add(qNew);

        // 4. Rewire: can the new node offer any neighbour a cheaper route?
        for (std::size_t n : near) {
            const int x = static_cast<int>(n);
            if (x == parent) {
                continue;
            }
            const double viaNew = newCost + configDistance(arm, qNew, node(x).q);
            if (viaNew < node(x).cost - 1e-12 && checker_.isEdgeValid(qNew, node(x).q)) {
                std::vector<int>& siblings = childrenOf(node(x).parent);
                siblings.erase(std::find(siblings.begin(), siblings.end(), x));
                node(x).parent = newIndex;
                childrenOf(newIndex).push_back(x);
                propagateCostDecrease(x, node(x).cost - viaNew);
            }
        }

        considerGoal(newIndex);
    }

    // Final evaluation (the loop may have exited right after an improvement).
    for (int g : goalParents) {
        const double c = node(g).cost + configDistance(arm, node(g).q, goal);
        if (c < bestCost - 1e-12) {
            bestCost = c;
            bestParent = g;
            result.costHistory.emplace_back(iteration, bestCost);
        }
    }

    result.iterations = iteration;
    result.nodes = tree.size();
    if (bestParent >= 0) {
        result.success = true;
        result.path = detail::extractPath(tree, bestParent);
        if (configDistance(arm, result.path.back(), goal) > 0.0) {
            result.path.push_back(goal);
        }
        result.pathCost = pathLength(arm, result.path);
        if (result.message.empty()) {
            result.message = "Path found";
        }
    } else if (result.message.empty()) {
        result.message = "No path found within the iteration budget";
    }
    result.planningTimeMs = run.elapsedMs();
    return result;
}

}  // namespace amp
