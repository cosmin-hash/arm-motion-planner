/// @file simulation.cpp
/// @brief Fixed-step closed-loop tracking simulation.
#include "amp/simulation.hpp"

#include <algorithm>
#include <cmath>

namespace amp {

TrackingLog simulateTracking(const ArmModel& arm, const JointDynamics& dynamics, const Trajectory& trajectory,
                             const TrackingOptions& options) {
    TrackingLog log;
    const int n = arm.dof();
    log.maxAbsError = Eigen::VectorXd::Zero(n);
    log.rmsError = Eigen::VectorXd::Zero(n);
    log.finalError = Eigen::VectorXd::Zero(n);
    if (trajectory.empty()) {
        return log;
    }

    std::vector<PidController> pids(static_cast<std::size_t>(n), PidController(options.gains));
    JointState state{trajectory.sample(0.0).q, Config::Zero(n)};

    const double endTime = trajectory.duration() + options.settleTime;
    const int steps = static_cast<int>(std::ceil(endTime / options.dt));
    const int logEvery = std::max(1, static_cast<int>(std::lround(options.logInterval / options.dt)));
    Eigen::VectorXd sumSquares = Eigen::VectorXd::Zero(n);

    for (int k = 0; k <= steps; ++k) {
        const double t = k * options.dt;
        const TrajectoryPoint ref = trajectory.sample(t);  // holds the final pose after the end

        // Controller: one PID per joint, fed the measured position (no sensor noise here).
        Config torque(n);
        bool anySaturated = false;
        for (int j = 0; j < n; ++j) {
            const double ff = options.accelerationFeedforward
                                  ? dynamics.inertia[j] * ref.qdd[j] + dynamics.damping[j] * ref.qd[j]
                                  : 0.0;
            PidController& pid = pids[static_cast<std::size_t>(j)];
            torque[j] = pid.update(ref.q[j], state.q[j], options.dt, ref.qd[j], ff);
            anySaturated = anySaturated || pid.lastTerms().saturated;
        }

        const Config error = ref.q - state.q;
        log.maxAbsError = log.maxAbsError.cwiseMax(error.cwiseAbs());
        sumSquares += error.cwiseProduct(error);
        log.saturatedSteps += anySaturated ? 1 : 0;
        if (k % logEvery == 0 || k == steps) {
            log.samples.push_back(TrackingSample{t, ref.q, state.q, error, torque});
        }
        log.finalError = error;

        // Plant: integrate one step with the torque held constant.
        state = stepDynamics(arm, dynamics, state, torque, options.dt);
    }
    log.totalSteps = steps + 1;
    log.rmsError = (sumSquares / log.totalSteps).cwiseSqrt();
    return log;
}

}  // namespace amp
