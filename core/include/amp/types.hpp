/// @file types.hpp
/// @brief Fundamental value types shared by every module of the core library.
#pragma once

#include <Eigen/Core>

#include <numbers>

namespace amp {

/// A point or vector in the 2D workspace, in metres.
using Vec2 = Eigen::Vector2d;

/// A configuration of the arm: one joint angle (radians) per revolute joint.
/// Dynamic size so the same code handles 2 to 6 DOF without templates.
using Config = Eigen::VectorXd;

/// The Jacobian of the end-effector position with respect to the joint angles (2 x N).
using Jacobian = Eigen::Matrix<double, 2, Eigen::Dynamic>;

inline constexpr double kPi = std::numbers::pi;
inline constexpr double kTwoPi = 2.0 * std::numbers::pi;

}  // namespace amp
