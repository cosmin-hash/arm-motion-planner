/// @file dynamics.hpp
/// @brief Simple joint-space dynamics for the simulated arm.
///
/// Each joint is modelled as an independent rotational inertia with viscous friction:
///     I_j q''_j + b_j q'_j = tau_j + tau_gravity,j(q)
/// The inertia is configuration-independent (the real mass matrix of a serial arm is
/// coupled and varies with the pose); this keeps the control story clear while still
/// giving each joint realistic, different dynamics. Gravity is off by default (the arm
/// lies flat on a table, seen from above); turning it on adds a configuration-dependent
/// load torque that a pure PD controller cannot fully reject, which is where the
/// integral term earns its place.
///
/// Integration: semi-implicit (symplectic) Euler at a fixed step. See
/// docs/ALGORITHMS.md#joint-dynamics for why this was chosen over explicit Euler and RK4.
#pragma once

#include "amp/arm_model.hpp"
#include "amp/types.hpp"

namespace amp {

/// Physical parameters of the simulated joints.
struct JointDynamics {
    Eigen::VectorXd inertia;   ///< kg m^2 per joint, > 0
    Eigen::VectorXd damping;   ///< N m s / rad per joint (viscous friction), >= 0
    Eigen::VectorXd linkMass;  ///< kg per link (used for gravity only)
    bool gravity = false;      ///< true: arm in a vertical plane with g along -y
    double g = 9.81;           ///< m/s^2

    /// Plausible defaults derived from the link lengths: aluminium-like links of 2 kg/m,
    /// inertia of everything beyond each joint with the arm stretched out (the worst case),
    /// plus a small motor/gearbox rotor inertia, and light viscous friction.
    [[nodiscard]] static JointDynamics defaultsFor(const ArmModel& arm);
};

/// Joint positions and velocities.
struct JointState {
    Config q;
    Config qd;
};

/// Gravity torque on every joint at configuration q (zero vector when gravity is off).
/// tau_j = -g * sum_{k >= j} m_k * (x_com,k - x_j), with each link's centre of mass at its midpoint.
[[nodiscard]] Config gravityTorque(const ArmModel& arm, const JointDynamics& dyn, const Config& q);

/// Joint accelerations for the given state and applied motor torques.
[[nodiscard]] Config forwardDynamics(const ArmModel& arm, const JointDynamics& dyn, const JointState& state,
                                     const Config& torque);

/// Advance the state by dt with semi-implicit Euler: velocity first, then position with the
/// *new* velocity. Torque is held constant over the step (zero-order hold, as in a digital controller).
[[nodiscard]] JointState stepDynamics(const ArmModel& arm, const JointDynamics& dyn, const JointState& state,
                                      const Config& torque, double dt);

}  // namespace amp
