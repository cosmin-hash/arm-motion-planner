/// @file smoothing.cpp
/// @brief Random shortcut smoothing.
///
/// Algorithm summary
/// -----------------
/// First try the single straight edge from start to goal (often valid after planning around
/// one obstacle). Then repeat: pick arc-length positions s1 < s2 uniformly along the path,
/// interpolate the configurations a and b there, and if the edge a -> b is valid, replace the
/// part of the path between them with it. Each successful shortcut strictly shortens the
/// path (unless it was already straight there).
///
/// Details: docs/ALGORITHMS.md#shortcut-smoothing
#include "amp/smoothing.hpp"

#include "amp/cspace.hpp"
#include "amp/planner.hpp"
#include "amp/rng.hpp"

#include <algorithm>

namespace amp {

namespace {

/// Where a given arc length falls on the path: segment index and fraction along it.
struct PathPoint {
    std::size_t segment = 0;  // between path[segment] and path[segment + 1]
    double fraction = 0.0;
};

PathPoint locate(const ArmModel& arm, const std::vector<Config>& path, double s) {
    for (std::size_t i = 0; i + 1 < path.size(); ++i) {
        const double len = configDistance(arm, path[i], path[i + 1]);
        if (s <= len || i + 2 == path.size()) {
            return {i, len > 0.0 ? std::clamp(s / len, 0.0, 1.0) : 0.0};
        }
        s -= len;
    }
    return {};
}

}  // namespace

SmoothingResult shortcutSmooth(const CollisionChecker& checker, const std::vector<Config>& path,
                               const SmoothingParams& params) {
    const ArmModel& arm = checker.arm();
    SmoothingResult result;
    result.path = path;
    result.costBefore = pathLength(arm, path);
    if (path.size() < 3) {
        result.costAfter = result.costBefore;
        return result;  // nothing to shortcut
    }

    // Cheapest possible win first: the whole path as one straight edge.
    if (checker.isEdgeValid(path.front(), path.back())) {
        result.path = {path.front(), path.back()};
        result.costAfter = pathLength(arm, result.path);
        result.shortcutsApplied = 1;
        return result;
    }

    Rng rng(params.seed);
    std::vector<Config>& p = result.path;
    for (int it = 0; it < params.iterations && p.size() > 2; ++it) {
        const double total = pathLength(arm, p);
        double s1 = rng.uniform(0.0, total);
        double s2 = rng.uniform(0.0, total);
        if (s1 > s2) {
            std::swap(s1, s2);
        }
        const PathPoint a = locate(arm, p, s1);
        const PathPoint b = locate(arm, p, s2);
        if (a.segment == b.segment) {
            continue;  // both on one straight segment: nothing to gain
        }
        const Config qa = interpolate(arm, p[a.segment], p[a.segment + 1], a.fraction);
        const Config qb = interpolate(arm, p[b.segment], p[b.segment + 1], b.fraction);
        if (!checker.isEdgeValid(qa, qb)) {
            continue;
        }
        // New path: p[0..a.segment], qa, qb, p[b.segment+1..end].
        std::vector<Config> shorter(p.begin(), p.begin() + static_cast<std::ptrdiff_t>(a.segment) + 1);
        if (configDistance(arm, shorter.back(), qa) > 0.0) {
            shorter.push_back(qa);
        }
        if (configDistance(arm, shorter.back(), qb) > 0.0) {
            shorter.push_back(qb);
        }
        for (std::size_t i = b.segment + 1; i < p.size(); ++i) {
            if (configDistance(arm, shorter.back(), p[i]) > 0.0) {
                shorter.push_back(p[i]);
            }
        }
        p = std::move(shorter);
        ++result.shortcutsApplied;
    }
    result.costAfter = pathLength(arm, p);
    return result;
}

}  // namespace amp
