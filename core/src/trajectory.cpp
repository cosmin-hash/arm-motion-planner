/// @file trajectory.cpp
/// @brief Synchronized trapezoidal time parameterization.
///
/// Algorithm summary
/// -----------------
/// For a segment with joint displacements d_j, joint j moves d_j * s(t). Its velocity is
/// d_j s' and its acceleration d_j s'', so the limits translate into limits on s:
///     s' <= V = min_j vmax_j / |d_j|,     |s''| <= A = min_j amax_j / |d_j|.
/// The fastest rest-to-rest s from 0 to 1 under those limits is a trapezoid (or a triangle if
/// the segment is too short to reach V). The most constrained joint hits its limit; the
/// others move proportionally slower and arrive at the same time.
///
/// Details: docs/ALGORITHMS.md#time-parameterization
#include "amp/trajectory.hpp"

#include "amp/angles.hpp"
#include "amp/cspace.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace amp {

MotionLimits MotionLimits::uniform(int dof, double velocity, double acceleration) {
    return MotionLimits{Eigen::VectorXd::Constant(dof, velocity), Eigen::VectorXd::Constant(dof, acceleration)};
}

SegmentTiming Trajectory::trapezoid(double v, double a) {
    SegmentTiming t;
    t.accel = a;
    // Distance covered while accelerating to v and decelerating back: v^2 / a.
    if (v * v / a >= 1.0) {
        // Triangle: never reaches v. Accelerate over the first half: 1/2 = 1/2 a t^2.
        t.accelTime = std::sqrt(1.0 / a);
        t.peakRate = a * t.accelTime;
        t.duration = 2.0 * t.accelTime;
    } else {
        // Trapezoid: ramps cover v^2/a in total, cruise covers the rest at rate v.
        t.accelTime = v / a;
        t.peakRate = v;
        t.duration = 2.0 * t.accelTime + (1.0 - v * v / a) / v;
    }
    return t;
}

Trajectory::Trajectory(const ArmModel& arm, std::vector<Config> path, const MotionLimits& limits)
    : path_(std::move(path)) {
    const int dof = arm.dof();
    if (limits.maxVelocity.size() != dof || limits.maxAcceleration.size() != dof ||
        (limits.maxVelocity.array() <= 0.0).any() || (limits.maxAcceleration.array() <= 0.0).any()) {
        throw std::invalid_argument("Trajectory: need one positive velocity and acceleration limit per joint");
    }
    for (int i = 0; i < dof; ++i) {
        continuous_.push_back(arm.limits(i).continuous);
    }

    double t = 0.0;
    for (std::size_t i = 0; i + 1 < path_.size(); ++i) {
        const Config d = configDifference(arm, path_[i], path_[i + 1]);
        double v = std::numeric_limits<double>::infinity();
        double a = std::numeric_limits<double>::infinity();
        for (int j = 0; j < dof; ++j) {
            const double mag = std::abs(d[j]);
            if (mag > 0.0) {
                v = std::min(v, limits.maxVelocity[j] / mag);
                a = std::min(a, limits.maxAcceleration[j] / mag);
            }
        }
        SegmentTiming timing;
        if (std::isfinite(v)) {  // a zero-length segment takes zero time
            timing = trapezoid(v, a);
        }
        timing.startTime = t;
        t += timing.duration;
        deltas_.push_back(d);
        timings_.push_back(timing);
    }
}

double Trajectory::duration() const {
    return timings_.empty() ? 0.0 : timings_.back().startTime + timings_.back().duration;
}

TrajectoryPoint Trajectory::sample(double t) const {
    TrajectoryPoint p;
    if (path_.empty()) {
        return p;
    }
    const auto dof = path_.front().size();
    p.time = std::clamp(t, 0.0, duration());
    p.qd = Config::Zero(dof);
    p.qdd = Config::Zero(dof);
    if (timings_.empty()) {
        p.q = path_.front();
        return p;
    }
    if (p.time >= duration()) {
        // Explicit end state. Recovering it from (startTime + duration) - startTime could land
        // one ulp short of the segment end, i.e. back on the deceleration ramp.
        p.q = path_.back();
        return p;
    }

    // Find the segment containing p.time (the last one if t is at the very end).
    std::size_t seg = timings_.size() - 1;
    for (std::size_t i = 0; i < timings_.size(); ++i) {
        if (p.time < timings_[i].startTime + timings_[i].duration) {
            seg = i;
            break;
        }
    }
    const SegmentTiming& s = timings_[seg];
    const double tau = std::clamp(p.time - s.startTime, 0.0, s.duration);  // time within the segment

    // Normalized profile s(tau), s'(tau), s''(tau).
    double pos = 1.0, rate = 0.0, acc = 0.0;  // at (or past) the end of the segment: at rest
    if (s.duration > 0.0 && tau < s.duration) {
        const double ta = s.accelTime;
        if (tau < ta) {  // accelerating
            pos = 0.5 * s.accel * tau * tau;
            rate = s.accel * tau;
            acc = s.accel;
        } else if (tau <= s.duration - ta) {  // cruising
            pos = 0.5 * s.accel * ta * ta + s.peakRate * (tau - ta);
            rate = s.peakRate;
        } else {  // decelerating (mirror image of the ramp-up)
            const double r = s.duration - tau;
            pos = 1.0 - 0.5 * s.accel * r * r;
            rate = s.accel * r;
            acc = -s.accel;
        }
    }

    const Config& d = deltas_[seg];
    p.q = path_[seg] + pos * d;
    for (Eigen::Index j = 0; j < p.q.size(); ++j) {
        if (continuous_[static_cast<std::size_t>(j)]) {
            p.q[j] = wrapToPi(p.q[j]);
        }
    }
    p.qd = rate * d;
    p.qdd = acc * d;
    return p;
}

}  // namespace amp
