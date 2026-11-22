/// @file rrt.cpp
/// @brief Rapidly-exploring Random Tree.
///
/// Algorithm summary
/// -----------------
/// Grow a tree from the start. Each iteration samples a random configuration (or, with
/// probability goalBias, the goal itself), finds the nearest tree node, and steps from it
/// toward the sample by at most stepSize. If that short edge is collision-free, the new
/// configuration joins the tree. Nearest-node extension pulls the tree toward unexplored
/// space (the "Voronoi bias"). Whenever a new node lands within goalTolerance of the goal
/// and can see it, the path is complete.
///
/// Details: docs/ALGORITHMS.md#rrt
#include "amp/rrt.hpp"

#include "amp/cspace.hpp"
#include "amp/rng.hpp"

#include "planner_internal.hpp"

namespace amp {

PlanResult RrtPlanner::plan(const Config& start, const Config& goal, const PlannerParams& params,
                            const PlannerCallbacks& callbacks) const {
    const detail::PlanRun run(params, callbacks);
    PlanResult result;
    if (!detail::checkEndpoints(checker_, start, goal, result)) {
        result.planningTimeMs = run.elapsedMs();
        return result;
    }

    const ArmModel& arm = checker_.arm();
    Rng rng(params.seed);
    const auto nn = makeNearestNeighbor(params.nearestNeighbor, arm);
    std::vector<TreeNode>& tree = result.tree;
    tree.push_back(TreeNode{start, -1, 0.0});
    nn->add(start);

    // Close enough to see the goal? Then finish the path through it.
    const auto tryConnectGoal = [&](int node) {
        const Config& q = tree[static_cast<std::size_t>(node)].q;
        const double d = configDistance(arm, q, goal);
        if (d > params.goalTolerance) {
            return false;
        }
        if (d == 0.0) {  // the node *is* the goal
            result.path = detail::extractPath(tree, node);
            return true;
        }
        if (!checker_.isEdgeValid(q, goal)) {
            return false;
        }
        tree.push_back(TreeNode{goal, node, tree[static_cast<std::size_t>(node)].cost + d});
        result.path = detail::extractPath(tree, static_cast<int>(tree.size()) - 1);
        return true;
    };

    int iteration = 0;
    bool found = tryConnectGoal(0);
    while (!found && iteration < params.maxIterations) {
        if (run.shouldStop(result.message)) {
            break;
        }
        ++iteration;

        // 1. Sample (goal-biased).
        const Config target = rng.uniform01() < params.goalBias ? goal : sampleUniform(arm, rng);
        // 2. Nearest node, 3. steer toward the sample.
        const auto nearest = static_cast<int>(nn->nearest(target));
        const TreeNode& from = tree[static_cast<std::size_t>(nearest)];
        const Config qNew = steer(arm, from.q, target, params.stepSize);
        const double edge = configDistance(arm, from.q, qNew);
        // 4. Keep the extension only if the motion is collision-free.
        if (edge > 0.0 && checker_.isEdgeValid(from.q, qNew)) {
            const double cost = from.cost + edge;  // read before push_back: `from` may dangle after
            tree.push_back(TreeNode{qNew, nearest, cost});
            nn->add(qNew);
            found = tryConnectGoal(static_cast<int>(tree.size()) - 1);
        }
        run.progress(iteration, tree.size(), -1.0);
        run.tree(iteration, tree);
    }

    result.success = found;
    result.iterations = iteration;
    result.nodes = tree.size();
    if (found) {
        result.pathCost = pathLength(arm, result.path);
        result.costHistory.emplace_back(iteration, result.pathCost);
        result.message = "Path found";
    } else if (result.message.empty()) {
        result.message = "No path found within the iteration budget";
    }
    result.planningTimeMs = run.elapsedMs();
    return result;
}

}  // namespace amp
