/// @file kinematics.cpp
/// @brief Forward kinematics, Jacobian, analytic 2-link IK and damped least squares IK.
///
/// Algorithm summary
/// -----------------
/// * FK: accumulate absolute link angles phi_i = q_0 + ... + q_i and walk along the chain,
///   p_{i+1} = p_i + l_i [cos phi_i, sin phi_i].
/// * Jacobian: joint j rotates everything after it about p_j, so column j is the
///   end-effector position relative to p_j rotated by 90 degrees.
/// * Analytic IK (2 links): law of cosines for the elbow, then atan2 for the shoulder.
/// * DLS IK (N links): repeated damped pseudo-inverse steps toward the target.
///
/// Details and derivations: docs/ALGORITHMS.md#kinematics
#include "amp/kinematics.hpp"

#include "amp/angles.hpp"
#include "amp/rng.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>

namespace amp {

std::vector<Vec2> forwardKinematics(const ArmModel& arm, const Config& q) {
    const int n = arm.dof();
    std::vector<Vec2> points;
    points.reserve(static_cast<std::size_t>(n) + 1);
    points.push_back(arm.base());

    double phi = 0.0;  // absolute orientation of the current link
    Vec2 p = arm.base();
    for (int i = 0; i < n; ++i) {
        phi += q[i];
        p += arm.linkLength(i) * Vec2(std::cos(phi), std::sin(phi));
        points.push_back(p);
    }
    return points;
}

Vec2 endEffectorPosition(const ArmModel& arm, const Config& q) {
    return forwardKinematics(arm, q).back();
}

Jacobian jacobian(const ArmModel& arm, const Config& q) {
    const int n = arm.dof();
    const std::vector<Vec2> pts = forwardKinematics(arm, q);
    const Vec2& ee = pts.back();

    Jacobian jac(2, n);
    for (int j = 0; j < n; ++j) {
        // Rotating joint j by dq moves the end effector on a circle around joint j:
        // d p_ee = dq * z x (p_ee - p_j) = dq * [-(y_ee - y_j), x_ee - x_j].
        const Vec2 r = ee - pts[static_cast<std::size_t>(j)];
        jac(0, j) = -r.y();
        jac(1, j) = r.x();
    }
    return jac;
}

double manipulability(const ArmModel& arm, const Config& q) {
    const Jacobian jac = jacobian(arm, q);
    // det can come out as a tiny negative number from round-off at a singularity.
    return std::sqrt(std::max(0.0, (jac * jac.transpose()).determinant()));
}

IkResult solveIkAnalytic2Link(const ArmModel& arm, const Vec2& target, ElbowConfig elbow) {
    IkResult result;
    if (arm.dof() != 2) {
        result.status = IkStatus::InvalidInput;
        return result;
    }

    const double l1 = arm.linkLength(0);
    const double l2 = arm.linkLength(1);
    const Vec2 d = target - arm.base();
    const double r2 = d.squaredNorm();

    // Law of cosines: r^2 = l1^2 + l2^2 + 2 l1 l2 cos(q2).
    const double c2 = (r2 - l1 * l1 - l2 * l2) / (2.0 * l1 * l2);
    constexpr double kSlack = 1e-12;  // absorb round-off for targets exactly on the boundary
    if (c2 > 1.0 + kSlack || c2 < -1.0 - kSlack) {
        result.status = IkStatus::Unreachable;
        result.q = Config::Zero(2);
        result.residual = std::abs(std::sqrt(r2) - std::clamp(std::sqrt(r2), std::abs(l1 - l2), l1 + l2));
        return result;
    }

    const double s2Mag = std::sqrt(std::max(0.0, 1.0 - c2 * c2));
    const double s2 = (elbow == ElbowConfig::Down) ? s2Mag : -s2Mag;
    const double q2 = std::atan2(s2, std::clamp(c2, -1.0, 1.0));
    // The shoulder points at the target, corrected by the angle the forearm adds.
    const double q1 = wrapToPi(std::atan2(d.y(), d.x()) - std::atan2(l2 * s2, l1 + l2 * c2));

    result.q = Config(2);
    result.q << q1, q2;
    result.residual = (endEffectorPosition(arm, result.q) - target).norm();
    result.status = arm.withinLimits(result.q) ? IkStatus::Success : IkStatus::JointLimitViolation;
    return result;
}

namespace {

/// One damped least squares attempt from a single initial configuration.
IkResult dlsAttempt(const ArmModel& arm, const Vec2& target, const Config& seed, const DlsParams& params) {
    IkResult result;
    Config q = arm.normalize(seed);
    // Manipulability threshold w0 scales with reach^2 so the same parameter works for any arm size.
    const double w0 = params.manipulabilityThreshold * arm.reach() * arm.reach();
    const double lambdaMax2 = params.maxDamping * params.maxDamping;

    for (int iter = 0; iter < params.maxIterations; ++iter) {
        const Vec2 error = target - endEffectorPosition(arm, q);
        if (error.norm() < params.tolerance) {
            result.status = IkStatus::Success;
            result.q = q;
            result.residual = error.norm();
            result.iterations = iter;
            return result;
        }

        const Jacobian jac = jacobian(arm, q);
        const Eigen::Matrix2d jjt = jac * jac.transpose();

        // Adaptive damping: w = sqrt(det(J J^T)) measures how far we are from a singularity
        // (w = 0 when the arm is fully stretched or folded and J loses rank). Far from
        // singularities lambda = 0 (pure pseudo-inverse, fast quadratic-like convergence);
        // near them lambda ramps up so joint steps stay bounded.
        const double w = std::sqrt(std::max(0.0, jjt.determinant()));
        const double ratio = w < w0 ? 1.0 - w / w0 : 0.0;
        const double lambda2 = lambdaMax2 * ratio * ratio;

        // Solve the small 2x2 system (J J^T + lambda^2 I) y = e, then dq = J^T y.
        // A tiny floor keeps the system solvable even when lambda = 0 and J is singular.
        const Eigen::Matrix2d a = jjt + (lambda2 + 1e-12) * Eigen::Matrix2d::Identity();
        const Eigen::Vector2d y = a.ldlt().solve(error);
        Config dq = jac.transpose() * y;

        const double stepNorm = dq.norm();
        if (stepNorm > params.maxStep) {
            dq *= params.maxStep / stepNorm;  // trust-region style clamp for robustness
        }
        q = arm.normalize(q + dq);
    }

    result.q = q;
    result.residual = (target - endEffectorPosition(arm, q)).norm();
    result.iterations = params.maxIterations;
    result.status = result.residual < params.tolerance ? IkStatus::Success : IkStatus::NotConverged;
    return result;
}

}  // namespace

IkResult solveIkDls(const ArmModel& arm, const Vec2& target, const Config& seed, const DlsParams& params) {
    if (seed.size() != arm.dof()) {
        IkResult invalid;
        invalid.status = IkStatus::InvalidInput;
        return invalid;
    }

    IkResult best = dlsAttempt(arm, target, seed, params);
    int totalIterations = best.iterations;

    // DLS is a local method and can stall against a joint limit. Restarting from other
    // configurations is the standard remedy; a seeded Rng keeps the result reproducible.
    Rng rng(params.restartSeed);
    for (int attempt = 0; attempt < params.restarts && !best.ok(); ++attempt) {
        Config start(arm.dof());
        for (int i = 0; i < arm.dof(); ++i) {
            const JointLimits& lim = arm.limits(i);
            start[i] = lim.continuous ? rng.uniform(-kPi, kPi) : rng.uniform(lim.lower, lim.upper);
        }
        IkResult candidate = dlsAttempt(arm, target, start, params);
        totalIterations += candidate.iterations;
        if (candidate.residual < best.residual) {
            best = std::move(candidate);
        }
    }
    best.iterations = totalIterations;
    return best;
}

const char* toString(IkStatus status) {
    switch (status) {
        case IkStatus::Success:
            return "Success";
        case IkStatus::Unreachable:
            return "Unreachable";
        case IkStatus::JointLimitViolation:
            return "JointLimitViolation";
        case IkStatus::NotConverged:
            return "NotConverged";
        case IkStatus::InvalidInput:
            return "InvalidInput";
    }
    return "Unknown";
}

}  // namespace amp
