/// @file collision.hpp
/// @brief Configuration validity and edge (motion) validity for a planar arm among obstacles.
///
/// Safety model
/// ------------
/// Every link is a capsule of radius `linkRadius`. All checks use capsules inflated by a
/// safety margin m. Edges (straight lines in C-space) are discretised so that no point of
/// the arm moves more than `maxStepDisplacement` (<= 2m) between two checked samples. Under
/// that condition, if every sample of the inflated arm is collision-free, the *real* arm is
/// collision-free along the entire continuous motion, no matter how thin an obstacle is.
/// The proof is in docs/ALGORITHMS.md#edge-validity.
#pragma once

#include "amp/arm_model.hpp"
#include "amp/geometry.hpp"
#include "amp/types.hpp"

#include <optional>
#include <variant>
#include <vector>

namespace amp {

/// A workspace obstacle: a disc or a convex polygon.
using Obstacle = std::variant<Circle, ConvexPolygon>;

/// Move an obstacle by `delta` (used by interactive editing).
void translateObstacle(Obstacle& obstacle, const Vec2& delta);

/// True if `p` lies inside or on the obstacle (used for mouse picking).
[[nodiscard]] bool obstacleContains(const Obstacle& obstacle, const Vec2& p);

/// Everything the arm can collide with.
struct Workspace {
    std::vector<Obstacle> obstacles;
    std::optional<Bounds> bounds;  ///< If set, the arm must stay strictly inside this rectangle.
};

/// Tuning of the collision checker.
struct CollisionOptions {
    /// Extra clearance added to every link's radius (metres).
    double safetyMargin = 0.005;
    /// Largest allowed travel of any arm point between consecutive edge samples (metres).
    /// 0 selects the largest value that keeps the continuous-safety guarantee: 2 * safetyMargin.
    double maxStepDisplacement = 0.0;
    /// Check collisions between non-adjacent links.
    bool selfCollision = true;
};

/// Why a configuration is invalid.
enum class CollisionKind {
    None,             ///< Valid configuration.
    JointLimit,       ///< A joint is outside its limits.
    ObstacleContact,  ///< A link touches an obstacle.
    Boundary,         ///< A link leaves the workspace bounds.
    Self,             ///< Two non-adjacent links touch.
};

/// Details of a validity check, for tests, logging and highlighting in the GUI.
struct CollisionReport {
    CollisionKind kind = CollisionKind::None;
    int link = -1;   ///< Offending link (or joint for JointLimit), -1 if none.
    int other = -1;  ///< Obstacle index (Obstacle) or second link (Self), -1 otherwise.

    /// True if the configuration is valid.
    [[nodiscard]] bool ok() const { return kind == CollisionKind::None; }
};

/// Collision checker for one arm in one workspace.
///
/// Immutable after construction and holds no mutable state, so a single instance can be
/// shared by several threads.
class CollisionChecker {
public:
    /// @throws std::invalid_argument for negative values, for a step displacement larger
    ///         than 2 * margin, or for both zero. A zero margin with an explicit step is
    ///         accepted (exact-contact checks, as used by tests), but then edge checking is
    ///         only as good as the step size and the continuous guarantee does not hold.
    CollisionChecker(ArmModel arm, Workspace workspace, CollisionOptions options = {});

    /// Full validity check with a reason. Joint limits are checked first, then obstacles,
    /// bounds and self-collision.
    [[nodiscard]] CollisionReport check(const Config& q) const;

    /// Shorthand for check(q).ok().
    [[nodiscard]] bool isValid(const Config& q) const { return check(q).ok(); }

    /// True if the straight C-space motion from `a` to `b` is collision-free, with the
    /// continuous-safety guarantee described at the top of this file. Both endpoints are checked.
    [[nodiscard]] bool isEdgeValid(const Config& a, const Config& b) const;

    /// Number of intervals the edge a -> b is split into (samples = steps + 1).
    [[nodiscard]] int edgeSteps(const Config& a, const Config& b) const;

    /// The arm being checked.
    [[nodiscard]] const ArmModel& arm() const { return arm_; }
    /// The obstacles and bounds.
    [[nodiscard]] const Workspace& workspace() const { return workspace_; }
    /// The options in effect (with maxStepDisplacement resolved).
    [[nodiscard]] const CollisionOptions& options() const { return options_; }
    /// Link radius plus safety margin: the capsule radius actually checked.
    [[nodiscard]] double effectiveRadius() const { return arm_.linkRadius() + options_.safetyMargin; }

private:
    ArmModel arm_;
    Workspace workspace_;
    CollisionOptions options_;
};

/// Human-readable name of a CollisionKind.
[[nodiscard]] const char* toString(CollisionKind kind);

}  // namespace amp
