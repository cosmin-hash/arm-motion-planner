/// @file cspace.hpp
/// @brief Configuration-space operations: difference, distance, interpolation, sampling.
///
/// A configuration is a point in the space of joint angles. Joints with limits live on an
/// interval; continuous joints live on a circle, where the shortest way from 170 deg to
/// -170 deg is +20 deg, not -340 deg. Every planner goes through these functions, so
/// wraparound is handled in exactly one place.
///
/// See docs/ALGORITHMS.md#configuration-space.
#pragma once

#include "amp/arm_model.hpp"
#include "amp/rng.hpp"
#include "amp/types.hpp"

namespace amp {

/// Per-joint displacement from `a` to `b`: the shortest signed angle for continuous joints,
/// the plain difference for joints with limits (which cannot wrap).
[[nodiscard]] Config configDifference(const ArmModel& arm, const Config& a, const Config& b);

/// Euclidean length of configDifference(a, b), in radians. Symmetric, zero iff a == b
/// (modulo 2*pi on continuous joints), and satisfies the triangle inequality.
[[nodiscard]] double configDistance(const ArmModel& arm, const Config& a, const Config& b);

/// Point at fraction `t` in [0, 1] along the shortest path from `a` to `b`:
/// t = 0 returns `a`, t = 1 returns `b` (continuous joints wrapped to (-pi, pi]).
[[nodiscard]] Config interpolate(const ArmModel& arm, const Config& a, const Config& b, double t);

/// Uniform random configuration within the joint limits (full circle for continuous joints).
[[nodiscard]] Config sampleUniform(const ArmModel& arm, Rng& rng);

/// Upper bound on how far (metres) any point of the arm can travel when the joints move
/// along the straight line from `a` to `b` in C-space.
///
/// Rotating joint j by |dq_j| moves every point beyond it along an arc of radius at most
/// R_j (the sum of the link lengths from joint j to the tip), so the travel of any point
/// is at most sum_j |dq_j| * R_j. This bound sets the edge-checking resolution.
[[nodiscard]] double maxWorkspaceDisplacement(const ArmModel& arm, const Config& a, const Config& b);

}  // namespace amp
