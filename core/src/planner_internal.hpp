/// @file planner_internal.hpp
/// @brief Helpers shared by the planner implementations (not part of the public API).
#pragma once

#include "amp/planner.hpp"

#include <chrono>
#include <string>

namespace amp::detail {

/// Tracks elapsed time, cancellation, the time budget and progress reporting for one plan() call.
class PlanRun {
public:
    PlanRun(const PlannerParams& params, const PlannerCallbacks& callbacks)
        : params_(params), callbacks_(callbacks), start_(std::chrono::steady_clock::now()) {}

    /// Milliseconds since construction.
    [[nodiscard]] double elapsedMs() const {
        return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start_).count();
    }

    /// True if planning must stop now; `why` receives the reason.
    [[nodiscard]] bool shouldStop(std::string& why) const {
        if (callbacks_.cancel != nullptr && callbacks_.cancel->load(std::memory_order_relaxed)) {
            why = "Cancelled by user";
            return true;
        }
        if (params_.timeLimitSeconds > 0.0 && elapsedMs() > params_.timeLimitSeconds * 1000.0) {
            why = "Time limit reached";
            return true;
        }
        return false;
    }

    /// Call once per iteration; forwards the tree every treeInterval iterations.
    void tree(int iteration, const std::vector<TreeNode>& nodes) const {
        if (callbacks_.onTree && callbacks_.treeInterval > 0 && iteration % callbacks_.treeInterval == 0) {
            callbacks_.onTree(nodes);
        }
    }

    /// Call once per iteration; forwards to the progress callback every progressInterval iterations.
    void progress(int iteration, std::size_t nodes, double bestCost) const {
        if (callbacks_.onProgress && callbacks_.progressInterval > 0 && iteration % callbacks_.progressInterval == 0) {
            callbacks_.onProgress(PlanProgress{iteration, nodes, bestCost});
        }
    }

private:
    const PlannerParams& params_;
    const PlannerCallbacks& callbacks_;
    std::chrono::steady_clock::time_point start_;
};

/// Validate start and goal before planning. On failure fills result.message and returns false.
[[nodiscard]] bool checkEndpoints(const CollisionChecker& checker, const Config& start, const Config& goal,
                                  PlanResult& result);

/// Follow parent links from `node` back to the root and return root..node configurations.
[[nodiscard]] std::vector<Config> extractPath(const std::vector<TreeNode>& tree, int node);

}  // namespace amp::detail
