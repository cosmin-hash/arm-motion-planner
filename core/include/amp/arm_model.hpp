/// @file arm_model.hpp
/// @brief Geometric description of a planar serial arm with revolute joints.
///
/// The arm is a chain base -> joint 0 -> link 0 -> joint 1 -> link 1 -> ... -> end effector.
/// Every joint rotates about the axis perpendicular to the plane. Joint angles are
/// *relative*: q[i] is the angle of link i measured from the direction of link i-1
/// (link 0 is measured from the world +x axis).
#pragma once

#include "amp/types.hpp"

#include <vector>

namespace amp {

/// Range of motion for a single revolute joint.
struct JointLimits {
    double lower = -kPi;      ///< Minimum angle in radians (ignored if continuous).
    double upper = kPi;       ///< Maximum angle in radians (ignored if continuous).
    bool continuous = false;  ///< True if the joint can spin freely (angle wraps around).
};

/// Immutable description of a planar serial arm.
///
/// The model is validated on construction, so every other module can assume it is
/// well-formed (2..6 joints, positive link lengths, lower < upper limits).
class ArmModel {
public:
    static constexpr int kMinDof = 2;  ///< Smallest supported number of joints.
    static constexpr int kMaxDof = 6;  ///< Largest supported number of joints.

    /// Build an arm model.
    /// @param linkLengths Length of each link in metres; its size defines the DOF.
    /// @param limits Joint limits, one per link.
    /// @param base World position of joint 0.
    /// @param linkRadius Half-thickness of every link (links are capsules for collision).
    /// @throws std::invalid_argument if the description is inconsistent.
    ArmModel(std::vector<double> linkLengths, std::vector<JointLimits> limits, Vec2 base = Vec2::Zero(),
             double linkRadius = 0.02);

    /// Convenience factory: `dof` equal links of length `linkLength`, limits of +/- `limit`.
    [[nodiscard]] static ArmModel uniform(int dof, double linkLength, double limit = kPi);

    /// Number of joints (degrees of freedom).
    [[nodiscard]] int dof() const { return static_cast<int>(linkLengths_.size()); }
    /// Length of link `i` in metres.
    [[nodiscard]] double linkLength(int i) const { return linkLengths_[static_cast<std::size_t>(i)]; }
    /// All link lengths.
    [[nodiscard]] const std::vector<double>& linkLengths() const { return linkLengths_; }
    /// Limits of joint `i`.
    [[nodiscard]] const JointLimits& limits(int i) const { return limits_[static_cast<std::size_t>(i)]; }
    /// All joint limits.
    [[nodiscard]] const std::vector<JointLimits>& limits() const { return limits_; }
    /// World position of the base (joint 0).
    [[nodiscard]] const Vec2& base() const { return base_; }
    /// Half-thickness of each link capsule.
    [[nodiscard]] double linkRadius() const { return linkRadius_; }
    /// Maximum reach: the sum of the link lengths.
    [[nodiscard]] double reach() const;

    /// True if every non-continuous joint of `q` lies within its limits (inclusive)
    /// and `q` has exactly dof() entries.
    [[nodiscard]] bool withinLimits(const Config& q) const;

    /// Clamp non-continuous joints to their limits and wrap continuous joints to (-pi, pi].
    [[nodiscard]] Config normalize(const Config& q) const;

private:
    std::vector<double> linkLengths_;
    std::vector<JointLimits> limits_;
    Vec2 base_;
    double linkRadius_;
};

}  // namespace amp
