/// @file cspace.cpp
/// @brief Configuration-space metric, interpolation, sampling and displacement bound.
///
/// Details: docs/ALGORITHMS.md#configuration-space
#include "amp/cspace.hpp"

#include "amp/angles.hpp"

#include <cmath>

namespace amp {

Config configDifference(const ArmModel& arm, const Config& a, const Config& b) {
    Config d(arm.dof());
    for (int i = 0; i < arm.dof(); ++i) {
        d[i] = arm.limits(i).continuous ? angleDiff(a[i], b[i]) : b[i] - a[i];
    }
    return d;
}

double configDistance(const ArmModel& arm, const Config& a, const Config& b) {
    // Hot path (called for every nearest-neighbour candidate): same arithmetic as
    // configDifference(...).norm(), but without allocating a temporary vector.
    double sum = 0.0;
    for (int i = 0; i < arm.dof(); ++i) {
        const double d = arm.limits(i).continuous ? angleDiff(a[i], b[i]) : b[i] - a[i];
        sum += d * d;
    }
    return std::sqrt(sum);
}

Config interpolate(const ArmModel& arm, const Config& a, const Config& b, double t) {
    if (t <= 0.0) {
        return a;
    }
    if (t >= 1.0) {
        return b;  // exact endpoint: avoids round-off making the last sample differ from b
    }
    Config q = a + t * configDifference(arm, a, b);
    for (int i = 0; i < arm.dof(); ++i) {
        if (arm.limits(i).continuous) {
            q[i] = wrapToPi(q[i]);
        }
    }
    return q;
}

Config sampleUniform(const ArmModel& arm, Rng& rng) {
    Config q(arm.dof());
    for (int i = 0; i < arm.dof(); ++i) {
        const JointLimits& lim = arm.limits(i);
        q[i] = lim.continuous ? rng.uniform(-kPi, kPi) : rng.uniform(lim.lower, lim.upper);
    }
    return q;
}

double maxWorkspaceDisplacement(const ArmModel& arm, const Config& a, const Config& b) {
    const Config d = configDifference(arm, a, b);
    double total = 0.0;
    double reachBeyond = arm.reach();  // R_0: distance bound from joint 0 to the tip
    for (int j = 0; j < arm.dof(); ++j) {
        total += std::abs(d[j]) * reachBeyond;
        reachBeyond -= arm.linkLength(j);  // R_{j+1} = R_j - l_j
    }
    return total;
}

}  // namespace amp
