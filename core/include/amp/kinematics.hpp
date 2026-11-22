/// @file kinematics.hpp
/// @brief Forward kinematics, Jacobian and inverse kinematics for a planar serial arm.
///
/// See docs/ALGORITHMS.md, sections "Forward kinematics", "Jacobian" and
/// "Inverse kinematics" for the derivations.
#pragma once

#include "amp/arm_model.hpp"
#include "amp/types.hpp"

#include <cstdint>
#include <vector>

namespace amp {

/// World positions of every joint and the end effector for configuration `q`.
///
/// @return A vector of dof()+1 points: element 0 is the base (joint 0), element i is
///         joint i (the start of link i), and the last element is the end effector.
/// @pre q.size() == arm.dof()
[[nodiscard]] std::vector<Vec2> forwardKinematics(const ArmModel& arm, const Config& q);

/// World position of the end effector for configuration `q`.
[[nodiscard]] Vec2 endEffectorPosition(const ArmModel& arm, const Config& q);

/// Analytic 2 x N Jacobian of the end-effector position: J(:, j) = d p_ee / d q_j.
[[nodiscard]] Jacobian jacobian(const ArmModel& arm, const Config& q);

/// Yoshikawa manipulability measure w = sqrt(det(J J^T)) of the position Jacobian.
///
/// w is zero at singular configurations (arm fully stretched or folded), where the end
/// effector cannot move in some direction, and grows as the arm becomes more dexterous.
[[nodiscard]] double manipulability(const ArmModel& arm, const Config& q);

/// Which of the two analytic 2-link solutions to return.
///
/// With q2 the elbow angle: ElbowDown gives q2 >= 0 (the elbow lies clockwise of the
/// base-to-target line, i.e. "below" it when reaching to the right); ElbowUp gives q2 <= 0.
enum class ElbowConfig { Up, Down };

/// Outcome of an inverse kinematics query.
enum class IkStatus {
    Success,              ///< Solution found within tolerance and inside joint limits.
    Unreachable,          ///< Target outside the annulus the arm can reach.
    JointLimitViolation,  ///< A geometric solution exists but violates joint limits.
    NotConverged,         ///< Iterative solver hit its iteration budget (best effort returned).
    InvalidInput,         ///< Wrong DOF for the chosen solver or wrong seed size.
};

/// Result of an inverse kinematics query.
struct IkResult {
    IkStatus status = IkStatus::InvalidInput;
    Config q;               ///< Joint angles (best effort when status != Success).
    double residual = 0.0;  ///< |FK(q) - target| in metres.
    int iterations = 0;     ///< Iterations used (0 for the analytic solver).

    /// True when status == IkStatus::Success.
    [[nodiscard]] bool ok() const { return status == IkStatus::Success; }
};

/// Closed-form inverse kinematics for a 2-link arm (law of cosines).
/// @param arm A 2-DOF arm; any other DOF returns IkStatus::InvalidInput.
/// @param target Desired end-effector position in world coordinates.
/// @param elbow Which of the two mirror-image solutions to return.
[[nodiscard]] IkResult solveIkAnalytic2Link(const ArmModel& arm, const Vec2& target, ElbowConfig elbow);

/// Tuning parameters for the damped least squares IK solver.
struct DlsParams {
    int maxIterations = 300;                ///< Iteration budget per attempt.
    double tolerance = 1e-5;                ///< Stop when the position error is below this (metres).
    double maxDamping = 0.05;               ///< lambda_max (metres): damping applied exactly at a singularity.
    double manipulabilityThreshold = 0.05;  ///< w0 as a fraction of reach^2; damping ramps in below it.
    double maxStep = 0.25;                  ///< Clamp on the per-iteration joint update norm (radians).
    int restarts = 0;                       ///< Extra attempts from pseudo-random seeds if the first fails.
    std::uint64_t restartSeed = 1;          ///< Seed for the restart configurations (keeps results deterministic).
};

/// Iterative inverse kinematics for an N-link arm using damped least squares
/// (Levenberg-Marquardt style): dq = J^T (J J^T + lambda^2 I)^-1 e.
///
/// Damping is adaptive (Nakamura & Hanafusa): zero far from singularities, so the solver
/// behaves like the fast pseudo-inverse method, and ramping up to `maxDamping` as the
/// manipulability measure w = sqrt(det(J J^T)) falls below a threshold.
///
/// The solver is deterministic: the same inputs always produce the same answer, including
/// the restart configurations, which come from a seeded Rng. Joint limits are enforced
/// after every step by clamping.
/// @param seed Initial guess (also selects which of many redundant solutions is found).
[[nodiscard]] IkResult solveIkDls(const ArmModel& arm, const Vec2& target, const Config& seed,
                                  const DlsParams& params = {});

/// Human-readable name of an IkStatus (for logs and the GUI).
[[nodiscard]] const char* toString(IkStatus status);

}  // namespace amp
