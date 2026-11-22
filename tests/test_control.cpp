/// @file test_control.cpp
/// @brief Joint dynamics, PID behaviour (step response, anti-windup, derivative kick,
/// derivative filter) and closed-loop trajectory tracking.
#include "amp/dynamics.hpp"
#include "amp/pid.hpp"
#include "amp/rng.hpp"
#include "amp/simulation.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

namespace amp {
namespace {

/// Single-joint plant I q'' + b q' = u, integrated with semi-implicit Euler.
struct SingleJoint {
    double inertia = 0.5;
    double damping = 0.5;
    double q = 0.0;
    double qd = 0.0;
    void step(double u, double dt) {
        qd += dt * (u - damping * qd) / inertia;
        q += dt * qd;
    }
};

struct StepResponse {
    double overshoot = 0.0;    ///< max(q) - target, as a fraction of the step
    double settlingTime = -1;  ///< last time |error| > 2% of the step
    double finalError = 0.0;
    double maxIntegral = 0.0;
};

StepResponse runStep(const PidGains& gains, double target, double duration, double dt = 0.001) {
    SingleJoint plant;
    PidController pid(gains);
    StepResponse r;
    double peak = 0.0;
    for (int k = 0; k * dt < duration; ++k) {
        const double u = pid.update(target, plant.q, dt);
        plant.step(u, dt);
        peak = std::max(peak, plant.q);
        r.maxIntegral = std::max(r.maxIntegral, std::abs(pid.integral()));
        if (std::abs(target - plant.q) > 0.02 * target) {
            r.settlingTime = (k + 1) * dt;
        }
    }
    r.overshoot = (peak - target) / target;
    r.finalError = target - plant.q;
    return r;
}

// ---------------------------------------------------------------------------
// Dynamics
// ---------------------------------------------------------------------------

// @verifies REQ-CTL-001
TEST(Dynamics, FreeDecayMatchesAnalyticSolution) {
    // No torque: q'(t) = q'0 exp(-b t / I), q(t) = q'0 I / b (1 - exp(-b t / I)).
    const ArmModel arm = ArmModel::uniform(2, 0.5);
    JointDynamics dyn = JointDynamics::defaultsFor(arm);
    JointState s{Config::Zero(2), Config::Constant(2, 1.0)};
    const double dt = 1e-4;
    const double tEnd = 1.0;
    for (int k = 0; k < static_cast<int>(tEnd / dt); ++k) {
        s = stepDynamics(arm, dyn, s, Config::Zero(2), dt);
    }
    for (int j = 0; j < 2; ++j) {
        const double rate = dyn.damping[j] / dyn.inertia[j];
        EXPECT_NEAR(s.qd[j], std::exp(-rate * tEnd), 1e-3);
        EXPECT_NEAR(s.q[j], (1.0 - std::exp(-rate * tEnd)) / rate, 1e-3);
    }
}

// @verifies REQ-CTL-001
TEST(Dynamics, DefaultInertiaDecreasesTowardTheTip) {
    const ArmModel arm({0.4, 0.35, 0.25}, std::vector<JointLimits>(3));
    const JointDynamics d = JointDynamics::defaultsFor(arm);
    EXPECT_GT(d.inertia[0], d.inertia[1]);
    EXPECT_GT(d.inertia[1], d.inertia[2]);
    // Last joint: one rod of mass 0.5 kg, length 0.25, about its end: m l^2 / 3, plus rotor inertia.
    EXPECT_NEAR(d.inertia[2], 0.5 * 0.25 * 0.25 / 3.0 + 0.01, 1e-12);
}

// @verifies REQ-CTL-001
TEST(Dynamics, GravityTorqueIsOffByDefaultAndCorrectWhenOn) {
    const ArmModel arm = ArmModel::uniform(2, 1.0);
    JointDynamics d = JointDynamics::defaultsFor(arm);
    const Config horizontal = Config::Zero(2);
    EXPECT_EQ(gravityTorque(arm, d, horizontal), Config::Zero(2));
    d.gravity = true;
    // Stretched horizontally: link masses 2 kg, centres at 0.5 m and 1.5 m from the base.
    const Config tau = gravityTorque(arm, d, horizontal);
    EXPECT_NEAR(tau[0], -9.81 * (2.0 * 0.5 + 2.0 * 1.5), 1e-9);
    EXPECT_NEAR(tau[1], -9.81 * 2.0 * 0.5, 1e-9);
    Config up(2);
    up << kPi / 2, 0.0;  // pointing straight up: no lever arm
    EXPECT_NEAR(gravityTorque(arm, d, up).norm(), 0.0, 1e-9);
}

// ---------------------------------------------------------------------------
// PID
// ---------------------------------------------------------------------------

// @verifies REQ-CTL-002
TEST(Pid, StepResponseSettlesWithLimitedOvershoot) {
    PidGains g;  // defaults: Kp 40, Ki 5, Kd 6 on a 0.5 kg m^2 joint
    const StepResponse r = runStep(g, 0.5, 5.0);
    EXPECT_LT(r.overshoot, 0.15);
    EXPECT_GT(r.settlingTime, 0.0);
    EXPECT_LT(r.settlingTime, 2.5);
    // Within the same 2% band used for the settling time. (The integrator, charged during the
    // transient, unwinds with a slow time constant of about Kp / Ki, so the last 1% takes long.)
    EXPECT_LT(std::abs(r.finalError), 0.02 * 0.5);
}

// @verifies REQ-CTL-003
TEST(Pid, AntiWindupPreventsOvershootGrowthUnderSaturation) {
    // A large step with a low torque limit saturates the actuator for a long time. Without
    // anti-windup the integrator keeps charging during saturation and the response overshoots
    // badly; with conditional integration it does not.
    PidGains g;
    g.ki = 20.0;
    g.torqueLimit = 3.0;
    g.antiWindup = false;
    const StepResponse wound = runStep(g, 2.0, 15.0);
    g.antiWindup = true;
    const StepResponse clamped = runStep(g, 2.0, 15.0);
    EXPECT_GT(wound.overshoot, 0.15);
    EXPECT_LT(clamped.overshoot, 0.5 * wound.overshoot);
    EXPECT_LT(clamped.maxIntegral, 0.5 * wound.maxIntegral);
    EXPECT_LT(std::abs(clamped.finalError), 1e-2);
}

// @verifies REQ-CTL-004
TEST(Pid, NoDerivativeKickOnSetpointStep) {
    // Derivative on measurement: a setpoint step at rest changes P (and I) only.
    PidGains g;
    g.kd = 50.0;
    g.torqueLimit = 1e9;
    PidController pid(g);
    (void)pid.update(0.0, 0.0, 0.001);             // at rest
    const double u = pid.update(1.0, 0.0, 0.001);  // setpoint jumps by 1 rad
    EXPECT_DOUBLE_EQ(pid.lastTerms().d, 0.0);
    EXPECT_NEAR(u, g.kp * 1.0 + g.ki * 1.0 * 0.001, 1e-12);
    // A naive derivative on error would have produced Kd * 1 / dt = 50,000 N m here.
}

// @verifies REQ-CTL-004
TEST(Pid, DerivativeFilterAttenuatesMeasurementNoise) {
    // Constant setpoint, measurement = 0 plus 1 mrad uniform noise. Compare D-term spread.
    const auto dSpread = [](double cutoffHz) {
        PidGains g;
        g.derivativeCutoffHz = cutoffHz;
        g.torqueLimit = 1e9;
        PidController pid(g);
        Rng rng(3);
        double maxAbs = 0.0;
        for (int k = 0; k < 2000; ++k) {
            (void)pid.update(0.0, rng.uniform(-1e-3, 1e-3), 0.001);
            if (k > 100) {
                maxAbs = std::max(maxAbs, std::abs(pid.lastTerms().d));
            }
        }
        return maxAbs;
    };
    const double unfiltered = dSpread(0.0);
    const double filtered = dSpread(20.0);
    EXPECT_LT(filtered, 0.25 * unfiltered);
}

// @verifies REQ-CTL-005
TEST(Pid, OutputIsSaturated) {
    PidGains g;
    g.torqueLimit = 2.0;
    PidController pid(g);
    EXPECT_DOUBLE_EQ(pid.update(10.0, 0.0, 0.001), 2.0);
    EXPECT_TRUE(pid.lastTerms().saturated);
    EXPECT_DOUBLE_EQ(pid.update(-10.0, 0.0, 0.001), -2.0);
}

// ---------------------------------------------------------------------------
// Closed-loop tracking
// ---------------------------------------------------------------------------

Trajectory demoTrajectory(const ArmModel& arm) {
    Config a(3), b(3), c(3);
    a << 0.0, 0.3, -0.2;
    b << 1.2, -0.4, 0.6;
    c << 0.5, 0.9, 0.1;
    return Trajectory(arm, {a, b, c}, MotionLimits::uniform(3, 1.5, 3.0));
}

// @verifies REQ-CTL-006
TEST(Tracking, FeedforwardReducesTrackingError) {
    const ArmModel arm({0.4, 0.35, 0.25}, std::vector<JointLimits>(3));
    const JointDynamics dyn = JointDynamics::defaultsFor(arm);
    const Trajectory traj = demoTrajectory(arm);
    TrackingOptions opt;
    const TrackingLog feedback = simulateTracking(arm, dyn, traj, opt);
    opt.accelerationFeedforward = true;
    const TrackingLog withFf = simulateTracking(arm, dyn, traj, opt);
    for (int j = 0; j < 3; ++j) {
        EXPECT_LT(withFf.maxAbsError[j], 0.5 * feedback.maxAbsError[j]) << "joint " << j;
        EXPECT_LT(std::abs(withFf.finalError[j]), 1e-3);
        EXPECT_LT(std::abs(feedback.finalError[j]), 5e-3);
    }
}

// @verifies REQ-CTL-007
TEST(Tracking, LogContainsReferenceActualErrorAndTorque) {
    const ArmModel arm({0.4, 0.35, 0.25}, std::vector<JointLimits>(3));
    const Trajectory traj = demoTrajectory(arm);
    TrackingOptions opt;
    opt.logInterval = 0.01;
    const TrackingLog log = simulateTracking(arm, JointDynamics::defaultsFor(arm), traj, opt);
    ASSERT_FALSE(log.samples.empty());
    const double expectedSamples = (traj.duration() + opt.settleTime) / opt.logInterval;
    EXPECT_NEAR(static_cast<double>(log.samples.size()), expectedSamples, 2.0);
    for (const TrackingSample& s : log.samples) {
        EXPECT_TRUE(s.error.isApprox(s.reference - s.actual, 1e-12) || s.error.norm() < 1e-12);
        EXPECT_LE(s.torque.cwiseAbs().maxCoeff(), opt.gains.torqueLimit + 1e-12);
    }
    EXPECT_EQ(log.samples.front().time, 0.0);
    EXPECT_GE(log.rmsError.minCoeff(), 0.0);
    EXPECT_LE((log.rmsError.array() <= log.maxAbsError.array() + 1e-12).all(), true);
}

// @verifies REQ-CTL-003
TEST(Tracking, IntegralTermRejectsGravityLoad) {
    // With gravity on, PD alone leaves a steady-state droop; the integral term removes it.
    const ArmModel arm({0.4, 0.35, 0.25}, std::vector<JointLimits>(3));
    JointDynamics dyn = JointDynamics::defaultsFor(arm);
    dyn.gravity = true;
    const Trajectory traj = demoTrajectory(arm);
    TrackingOptions opt;
    opt.settleTime = 4.0;
    opt.gains.torqueLimit = 50.0;
    opt.gains.ki = 0.0;
    const TrackingLog pd = simulateTracking(arm, dyn, traj, opt);
    opt.gains.ki = 30.0;
    const TrackingLog pid = simulateTracking(arm, dyn, traj, opt);
    EXPECT_GT(pd.finalError.cwiseAbs().maxCoeff(), 0.01);
    EXPECT_LT(pid.finalError.cwiseAbs().maxCoeff(), 0.1 * pd.finalError.cwiseAbs().maxCoeff());
}

}  // namespace
}  // namespace amp
