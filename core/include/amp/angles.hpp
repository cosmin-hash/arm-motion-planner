/// @file angles.hpp
/// @brief Small helpers for working with angles on the circle.
#pragma once

#include "amp/types.hpp"

#include <cmath>

namespace amp {

/// Wrap an angle into the half-open interval (-pi, pi].
/// @param angle Any finite angle in radians.
/// @return The equivalent angle in (-pi, pi].
[[nodiscard]] inline double wrapToPi(double angle) {
    // std::remainder returns a value in [-pi, pi]; map -pi to +pi for a unique representative.
    double wrapped = std::remainder(angle, kTwoPi);
    if (wrapped <= -kPi) {
        wrapped += kTwoPi;
    }
    return wrapped;
}

/// Signed shortest angular difference `to - from`, in (-pi, pi].
/// Moving from `from` by the returned amount reaches `to` the short way round the circle.
[[nodiscard]] inline double angleDiff(double from, double to) {
    return wrapToPi(to - from);
}

}  // namespace amp
