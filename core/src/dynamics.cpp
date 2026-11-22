/// @file dynamics.cpp
/// @brief Decoupled joint dynamics, optional gravity, semi-implicit Euler integration.
///
/// Details: docs/ALGORITHMS.md#joint-dynamics
#include "amp/dynamics.hpp"

#include "amp/kinematics.hpp"

namespace amp {

JointDynamics JointDynamics::defaultsFor(const ArmModel& arm) {
    const int n = arm.dof();
    constexpr double kDensity = 2.0;        // kg per metre of link
    constexpr double kRotorInertia = 0.01;  // kg m^2 reflected motor + gearbox inertia
    constexpr double kDamping = 0.5;        // N m s / rad
    JointDynamics d;
    d.inertia.resize(n);
    d.damping = Eigen::VectorXd::Constant(n, kDamping);
    d.linkMass.resize(n);
    for (int k = 0; k < n; ++k) {
        d.linkMass[k] = kDensity * arm.linkLength(k);
    }
    for (int j = 0; j < n; ++j) {
        // Stretched arm: link k (k >= j) is a rod whose centre is at distance c from joint j.
        double inertia = kRotorInertia;
        double offset = 0.0;  // distance from joint j to the start of link k
        for (int k = j; k < n; ++k) {
            const double l = arm.linkLength(k);
            const double c = offset + 0.5 * l;
            inertia += d.linkMass[k] * (l * l / 12.0 + c * c);  // rod about its centre + parallel axis
            offset += l;
        }
        d.inertia[j] = inertia;
    }
    return d;
}

Config gravityTorque(const ArmModel& arm, const JointDynamics& dyn, const Config& q) {
    const int n = arm.dof();
    Config tau = Config::Zero(n);
    if (!dyn.gravity) {
        return tau;
    }
    const std::vector<Vec2> pts = forwardKinematics(arm, q);
    for (int j = 0; j < n; ++j) {
        for (int k = j; k < n; ++k) {
            const double xCom = 0.5 * (pts[static_cast<std::size_t>(k)].x() + pts[static_cast<std::size_t>(k) + 1].x());
            // Torque about joint j of the weight -m g (in y) applied at x_com: r_x * F_y.
            tau[j] -= dyn.g * dyn.linkMass[k] * (xCom - pts[static_cast<std::size_t>(j)].x());
        }
    }
    return tau;
}

Config forwardDynamics(const ArmModel& arm, const JointDynamics& dyn, const JointState& state, const Config& torque) {
    const Config net = torque - dyn.damping.cwiseProduct(state.qd) + gravityTorque(arm, dyn, state.q);
    return net.cwiseQuotient(dyn.inertia);
}

JointState stepDynamics(const ArmModel& arm, const JointDynamics& dyn, const JointState& state, const Config& torque,
                        double dt) {
    JointState next;
    next.qd = state.qd + dt * forwardDynamics(arm, dyn, state, torque);
    next.q = state.q + dt * next.qd;  // uses the updated velocity: this is what makes it semi-implicit
    return next;
}

}  // namespace amp
