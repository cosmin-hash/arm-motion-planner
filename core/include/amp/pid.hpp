/// @file pid.hpp
/// @brief Single-joint PID controller for trajectory tracking.
///
///   u = Kp e  +  I  +  Kd (r' - y'_filtered)  +  feedforward,   clamped to +/- torqueLimit
///   e = r - y,   I += Ki e dt  (only when doing so does not deepen saturation)
///
/// * Derivative on measurement: the derivative term never differentiates the setpoint r, so
///   a setpoint step does not produce a "derivative kick". When tracking a trajectory, its
///   analytic velocity r' is supplied separately (it is smooth by construction); with r' = 0
///   this is the textbook derivative-on-measurement form.
/// * The measured rate y' comes from finite differences of the measurement, passed through a
///   first-order low-pass filter to keep sensor noise from being amplified by Kd.
/// * Anti-windup by clamping (conditional integration): while the output is saturated and
///   the error would push it further into saturation, the integrator is frozen.
///
/// See docs/ALGORITHMS.md#pid-control.
#pragma once

namespace amp {

/// Gains and limits of one PID loop.
struct PidGains {
    double kp = 40.0;                  ///< N m / rad
    double ki = 5.0;                   ///< N m / (rad s)
    double kd = 6.0;                   ///< N m s / rad
    double derivativeCutoffHz = 50.0;  ///< Low-pass cutoff on the measured rate; 0 disables the filter.
    double torqueLimit = 20.0;         ///< |u| <= torqueLimit (N m); must be > 0
    bool antiWindup = true;            ///< Conditional integration while saturated.
};

/// Diagnostic breakdown of the last output, for logging and plots.
struct PidTerms {
    double p = 0.0;
    double i = 0.0;
    double d = 0.0;
    double feedforward = 0.0;
    double output = 0.0;  ///< After saturation.
    bool saturated = false;
};

/// Discrete-time PID loop. One instance per joint.
class PidController {
public:
    /// Create a controller with the given gains and an empty integrator.
    explicit PidController(PidGains gains = {}) : gains_(gains) {}

    /// Compute the control output for one sample period.
    /// @param setpoint      reference position r (rad)
    /// @param measurement   measured position y (rad)
    /// @param dt            sample period (s), > 0
    /// @param setpointRate  reference velocity r' (rad/s); 0 for pure derivative-on-measurement
    /// @param feedforward   torque added after the PID terms (before saturation)
    /// @return the saturated torque command
    double update(double setpoint, double measurement, double dt, double setpointRate = 0.0, double feedforward = 0.0);

    /// Clear the integrator and derivative filter (e.g. before a new motion).
    void reset();

    /// Change gains without resetting state (live tuning).
    void setGains(const PidGains& gains) { gains_ = gains; }
    /// Current gains.
    [[nodiscard]] const PidGains& gains() const { return gains_; }
    /// Breakdown of the most recent update().
    [[nodiscard]] const PidTerms& lastTerms() const { return terms_; }
    /// Current integrator contribution (N m).
    [[nodiscard]] double integral() const { return integral_; }

private:
    PidGains gains_;
    PidTerms terms_;
    double integral_ = 0.0;
    double previousMeasurement_ = 0.0;
    double filteredRate_ = 0.0;
    bool first_ = true;
};

}  // namespace amp
