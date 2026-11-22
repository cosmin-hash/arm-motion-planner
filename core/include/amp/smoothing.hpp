/// @file smoothing.hpp
/// @brief Random shortcut smoothing of a planned path.
///
/// Sampling planners return jagged paths. Shortcutting repeatedly picks two points on the
/// path and, if the straight edge between them is valid, replaces everything in between
/// with that edge. A straight segment is never longer than the path it replaces (triangle
/// inequality), so the cost never increases.
///
/// See docs/ALGORITHMS.md#shortcut-smoothing.
#pragma once

#include "amp/collision.hpp"
#include "amp/types.hpp"

#include <cstdint>
#include <vector>

namespace amp {

/// Parameters of the shortcut smoother.
struct SmoothingParams {
    int iterations = 200;    ///< Shortcut attempts.
    std::uint64_t seed = 1;  ///< Random seed (deterministic like the planners).
};

/// Statistics reported by the smoother.
struct SmoothingResult {
    std::vector<Config> path;
    double costBefore = 0.0;  ///< C-space length of the input path (rad).
    double costAfter = 0.0;   ///< C-space length of the output path (rad).
    int shortcutsApplied = 0;
};

/// Shorten `path` by random shortcuts, each validated with CollisionChecker::isEdgeValid.
///
/// Guarantees: the result starts and ends at the same configurations, every edge is valid,
/// and costAfter <= costBefore. Shortcut endpoints are chosen uniformly by arc length, so
/// shortcuts can start and end in the middle of segments, not only at waypoints.
/// @pre isPathValid(checker, path)
[[nodiscard]] SmoothingResult shortcutSmooth(const CollisionChecker& checker, const std::vector<Config>& path,
                                             const SmoothingParams& params = {});

}  // namespace amp
