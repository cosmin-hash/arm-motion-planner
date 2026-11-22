/// @file simulation.hpp
/// @brief Closed-loop simulation: PID joint controllers tracking a trajectory on the
/// simulated joint dynamics.
///
///   reference r(t) --> [PID per joint] --tau--> [joint dynamics] --q--> (measurement)
///                         ^  optional feedforward tau_ff = I r'' + b r'
///
/// The controller runs at a fixed rate (default 1 kHz) and the plant is integrated with the
/// same step, with torque held constant in between (zero-order hold).
#pragma once

#include "amp/dynamics.hpp"
#include "amp/pid.hpp"
#include "amp/trajectory.hpp"

#include <vector>

namespace amp {

/// Settings of a tracking simulation.
struct TrackingOptions {
    PidGains gains;                        ///< Same gains on every joint.
    bool accelerationFeedforward = false;  ///< Add tau_ff = I r'' + b r' from the nominal model.
    double dt = 0.001;                     ///< Control and integration period (s).
    double settleTime = 0.5;               ///< Keep simulating after the trajectory ends (s).
    double logInterval = 0.01;             ///< Store one log sample every this many seconds.
};

/// One logged sample.
struct TrackingSample {
    double time = 0.0;
    Config reference;  ///< r(t)
    Config actual;     ///< q(t)
    Config error;      ///< r(t) - q(t)
    Config torque;     ///< Commanded (saturated) torque.
};

/// Result of a tracking simulation.
struct TrackingLog {
    std::vector<TrackingSample> samples;
    Eigen::VectorXd maxAbsError;  ///< Per joint, over the whole run (rad).
    Eigen::VectorXd rmsError;     ///< Per joint (rad).
    Eigen::VectorXd finalError;   ///< Per joint, at the end of the settle time (rad).
    int saturatedSteps = 0;       ///< Control steps in which any joint saturated.
    int totalSteps = 0;
};

/// Simulate the arm starting at rest at the trajectory's first configuration.
[[nodiscard]] TrackingLog simulateTracking(const ArmModel& arm, const JointDynamics& dynamics,
                                           const Trajectory& trajectory, const TrackingOptions& options);

}  // namespace amp
