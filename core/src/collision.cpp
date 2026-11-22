/// @file collision.cpp
/// @brief Configuration and edge validity checking.
///
/// Algorithm summary
/// -----------------
/// check(q): FK gives the joint positions; each link is a capsule of radius
/// linkRadius + safetyMargin. Test each capsule against each obstacle and the bounds, then
/// each pair of non-adjacent links. Cost O(N * M + N^2) for N links and M obstacles.
///
/// isEdgeValid(a, b): bound the workspace travel D of any arm point along the C-space
/// segment (maxWorkspaceDisplacement), split it into n = ceil(D / delta) steps, and check
/// every sample. With delta <= 2 * margin this is conservative for the uninflated arm.
///
/// Details: docs/ALGORITHMS.md#collision-checking and #edge-validity
#include "amp/collision.hpp"

#include "amp/cspace.hpp"
#include "amp/kinematics.hpp"

#include <cmath>
#include <stdexcept>
#include <type_traits>
#include <utility>

namespace amp {

void translateObstacle(Obstacle& obstacle, const Vec2& delta) {
    std::visit(
        [&](auto& shape) {
            using T = std::decay_t<decltype(shape)>;
            if constexpr (std::is_same_v<T, Circle>) {
                shape.center += delta;
            } else {
                shape.translate(delta);
            }
        },
        obstacle);
}

bool obstacleContains(const Obstacle& obstacle, const Vec2& p) {
    return std::visit(
        [&](const auto& shape) {
            using T = std::decay_t<decltype(shape)>;
            if constexpr (std::is_same_v<T, Circle>) {
                return (p - shape.center).norm() <= shape.radius;
            } else {
                return pointInConvexPolygon(p, shape);
            }
        },
        obstacle);
}

CollisionChecker::CollisionChecker(ArmModel arm, Workspace workspace, CollisionOptions options)
    : arm_(std::move(arm)), workspace_(std::move(workspace)), options_(options) {
    if (options_.safetyMargin < 0.0 || options_.maxStepDisplacement < 0.0) {
        throw std::invalid_argument("CollisionChecker: margin and step displacement must be non-negative");
    }
    if (options_.maxStepDisplacement == 0.0) {
        options_.maxStepDisplacement = 2.0 * options_.safetyMargin;  // largest step that keeps the guarantee
    }
    if (options_.maxStepDisplacement <= 0.0) {
        throw std::invalid_argument(
            "CollisionChecker: with zero safety margin an explicit maxStepDisplacement is required, "
            "and continuous safety cannot be guaranteed");
    }
    if (options_.maxStepDisplacement > 2.0 * options_.safetyMargin && options_.safetyMargin > 0.0) {
        throw std::invalid_argument(
            "CollisionChecker: maxStepDisplacement must be <= 2 * safetyMargin for continuous safety");
    }
}

CollisionReport CollisionChecker::check(const Config& q) const {
    CollisionReport report;
    if (q.size() != arm_.dof()) {
        report.kind = CollisionKind::JointLimit;
        return report;
    }
    for (int i = 0; i < arm_.dof(); ++i) {
        const JointLimits& lim = arm_.limits(i);
        if (!lim.continuous && (q[i] < lim.lower || q[i] > lim.upper)) {
            report.kind = CollisionKind::JointLimit;
            report.link = i;
            return report;
        }
    }

    const std::vector<Vec2> pts = forwardKinematics(arm_, q);
    const double r = effectiveRadius();
    const int links = arm_.dof();

    for (int k = 0; k < links; ++k) {
        const Vec2& a = pts[static_cast<std::size_t>(k)];
        const Vec2& b = pts[static_cast<std::size_t>(k) + 1];

        for (std::size_t o = 0; o < workspace_.obstacles.size(); ++o) {
            const bool hit = std::visit(
                [&](const auto& shape) {
                    using T = std::decay_t<decltype(shape)>;
                    if constexpr (std::is_same_v<T, Circle>) {
                        return capsuleIntersectsCircle(a, b, r, shape);
                    } else {
                        return capsuleIntersectsPolygon(a, b, r, shape);
                    }
                },
                workspace_.obstacles[o]);
            if (hit) {
                report.kind = CollisionKind::ObstacleContact;
                report.link = k;
                report.other = static_cast<int>(o);
                return report;
            }
        }

        if (workspace_.bounds && !capsuleInsideBounds(a, b, r, *workspace_.bounds)) {
            report.kind = CollisionKind::Boundary;
            report.link = k;
            return report;
        }
    }

    if (options_.selfCollision) {
        // Adjacent links share a joint and always touch, so only pairs at least two apart count.
        for (int i = 0; i < links; ++i) {
            for (int j = i + 2; j < links; ++j) {
                const auto si = static_cast<std::size_t>(i);
                const auto sj = static_cast<std::size_t>(j);
                if (capsulesIntersect(pts[si], pts[si + 1], r, pts[sj], pts[sj + 1], r)) {
                    report.kind = CollisionKind::Self;
                    report.link = i;
                    report.other = j;
                    return report;
                }
            }
        }
    }
    return report;
}

int CollisionChecker::edgeSteps(const Config& a, const Config& b) const {
    const double travel = maxWorkspaceDisplacement(arm_, a, b);
    // ceil(travel / delta) intervals guarantee each interval moves any point by <= delta.
    return std::max(1, static_cast<int>(std::ceil(travel / options_.maxStepDisplacement)));
}

bool CollisionChecker::isEdgeValid(const Config& a, const Config& b) const {
    if (!isValid(a) || !isValid(b)) {
        return false;
    }
    const int steps = edgeSteps(a, b);
    for (int k = 1; k < steps; ++k) {
        const double t = static_cast<double>(k) / steps;
        if (!isValid(interpolate(arm_, a, b, t))) {
            return false;
        }
    }
    return true;
}

const char* toString(CollisionKind kind) {
    switch (kind) {
        case CollisionKind::None:
            return "None";
        case CollisionKind::JointLimit:
            return "JointLimit";
        case CollisionKind::ObstacleContact:
            return "ObstacleContact";
        case CollisionKind::Boundary:
            return "Boundary";
        case CollisionKind::Self:
            return "Self";
    }
    return "Unknown";
}

}  // namespace amp
