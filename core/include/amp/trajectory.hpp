/// @file trajectory.hpp
/// @brief Time parameterization: turning a geometric path into q(t), q'(t), q''(t).
///
/// Each straight C-space segment of the path gets a synchronized trapezoidal velocity
/// profile: accelerate, cruise, decelerate, with all joints sharing one normalized profile
/// s(t) from 0 to 1, so q(t) = q_i + s(t) (q_{i+1} - q_i). Because every joint moves
/// proportionally, the arm stays exactly on the straight segment that the collision checker
/// validated, and all joints start and stop together. The profile is limited by whichever
/// joint is most constrained. The arm comes to rest at every waypoint (rest-to-rest segments).
///
/// See docs/ALGORITHMS.md#time-parameterization.
#pragma once

#include "amp/arm_model.hpp"
#include "amp/types.hpp"

#include <vector>

namespace amp {

/// Per-joint kinematic limits for time parameterization.
struct MotionLimits {
    Eigen::VectorXd maxVelocity;      ///< rad/s, one per joint, > 0
    Eigen::VectorXd maxAcceleration;  ///< rad/s^2, one per joint, > 0

    /// The same limits on every joint.
    [[nodiscard]] static MotionLimits uniform(int dof, double velocity, double acceleration);
};

/// Reference state at one instant.
struct TrajectoryPoint {
    double time = 0.0;
    Config q;    ///< Position.
    Config qd;   ///< Velocity.
    Config qdd;  ///< Acceleration.
};

/// Timing of one segment's normalized trapezoid s(t) on [0, duration].
struct SegmentTiming {
    double startTime = 0.0;
    double duration = 0.0;
    double accelTime = 0.0;  ///< Time spent accelerating (and, symmetrically, decelerating).
    double peakRate = 0.0;   ///< Maximum of s'(t) (1/s).
    double accel = 0.0;      ///< |s''(t)| during the ramps (1/s^2).
};

/// A time-parameterized path. Immutable after construction.
class Trajectory {
public:
    /// Empty trajectory (no waypoints, zero duration).
    Trajectory() = default;

    /// Build from a path (at least one configuration) and per-joint limits.
    /// @throws std::invalid_argument if the limits have the wrong size or are not positive.
    Trajectory(const ArmModel& arm, std::vector<Config> path, const MotionLimits& limits);

    /// Total duration in seconds.
    [[nodiscard]] double duration() const;
    /// Reference state at time t (clamped to [0, duration()]).
    [[nodiscard]] TrajectoryPoint sample(double t) const;
    /// True if the trajectory has no waypoints.
    [[nodiscard]] bool empty() const { return path_.empty(); }
    /// The waypoints.
    [[nodiscard]] const std::vector<Config>& path() const { return path_; }
    /// Per-segment timing, for inspection and tests.
    [[nodiscard]] const std::vector<SegmentTiming>& segments() const { return timings_; }

    /// Normalized trapezoid with peak rate limit `v` and acceleration limit `a` covering 0 -> 1.
    [[nodiscard]] static SegmentTiming trapezoid(double v, double a);

private:
    std::vector<bool> continuous_;  // per joint: wrap angles when sampling
    std::vector<Config> path_;
    std::vector<Config> deltas_;  // per segment: configDifference(path[i], path[i+1])
    std::vector<SegmentTiming> timings_;
};

}  // namespace amp
