/// @file pid.cpp
/// @brief PID with derivative on measurement, derivative low-pass filter, clamping
/// anti-windup and torque saturation.
///
/// Details: docs/ALGORITHMS.md#pid-control
#include "amp/pid.hpp"

#include "amp/types.hpp"

#include <algorithm>

namespace amp {

double PidController::update(double setpoint, double measurement, double dt, double setpointRate, double feedforward) {
    const double error = setpoint - measurement;

    // Measured rate by finite difference, then a first-order low-pass filter:
    //   y'_f += alpha (y'_raw - y'_f),  alpha = dt / (dt + 1 / (2 pi f_c)).
    // On the very first sample there is no previous measurement, so the rate is taken as 0.
    const double rawRate = first_ ? 0.0 : (measurement - previousMeasurement_) / dt;
    if (gains_.derivativeCutoffHz > 0.0 && !first_) {
        const double tau = 1.0 / (kTwoPi * gains_.derivativeCutoffHz);
        const double alpha = dt / (dt + tau);
        filteredRate_ += alpha * (rawRate - filteredRate_);
    } else {
        filteredRate_ = rawRate;
    }
    previousMeasurement_ = measurement;
    first_ = false;

    terms_.p = gains_.kp * error;
    terms_.d = gains_.kd * (setpointRate - filteredRate_);  // derivative on measurement (+ reference rate)
    terms_.feedforward = feedforward;

    // Tentatively integrate, then decide whether to keep it (conditional integration).
    const double candidateIntegral = integral_ + gains_.ki * error * dt;
    const double unsaturated = terms_.p + candidateIntegral + terms_.d + feedforward;
    const double output = std::clamp(unsaturated, -gains_.torqueLimit, gains_.torqueLimit);
    terms_.saturated = output != unsaturated;

    // Freeze the integrator if the output is saturated and the error pushes further into the limit.
    const bool deepening = terms_.saturated && ((unsaturated > 0.0) == (error > 0.0));
    if (!(gains_.antiWindup && deepening)) {
        integral_ = candidateIntegral;
    }
    terms_.i = integral_;
    terms_.output = std::clamp(terms_.p + integral_ + terms_.d + feedforward, -gains_.torqueLimit, gains_.torqueLimit);
    return terms_.output;
}

void PidController::reset() {
    integral_ = 0.0;
    filteredRate_ = 0.0;
    previousMeasurement_ = 0.0;
    first_ = true;
    terms_ = PidTerms{};
}

}  // namespace amp
