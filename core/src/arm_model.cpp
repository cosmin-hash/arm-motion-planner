/// @file arm_model.cpp
/// @brief Validation and helpers for ArmModel.
#include "amp/arm_model.hpp"

#include "amp/angles.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <stdexcept>
#include <string>
#include <utility>

namespace amp {

ArmModel::ArmModel(std::vector<double> linkLengths, std::vector<JointLimits> limits, Vec2 base, double linkRadius)
    : linkLengths_(std::move(linkLengths)), limits_(std::move(limits)), base_(base), linkRadius_(linkRadius) {
    const int n = static_cast<int>(linkLengths_.size());
    if (n < kMinDof || n > kMaxDof) {
        throw std::invalid_argument("ArmModel: DOF must be between " + std::to_string(kMinDof) + " and " +
                                    std::to_string(kMaxDof) + ", got " + std::to_string(n));
    }
    if (limits_.size() != linkLengths_.size()) {
        throw std::invalid_argument("ArmModel: need exactly one JointLimits per link");
    }
    for (std::size_t i = 0; i < linkLengths_.size(); ++i) {
        if (!(linkLengths_[i] > 0.0) || !std::isfinite(linkLengths_[i])) {
            throw std::invalid_argument("ArmModel: link lengths must be positive and finite");
        }
        const JointLimits& lim = limits_[i];
        if (!lim.continuous && !(lim.lower < lim.upper)) {
            throw std::invalid_argument("ArmModel: joint lower limit must be below upper limit");
        }
    }
    if (!(linkRadius_ >= 0.0)) {
        throw std::invalid_argument("ArmModel: link radius must be non-negative");
    }
}

ArmModel ArmModel::uniform(int dof, double linkLength, double limit) {
    const auto n = static_cast<std::size_t>(std::max(dof, 0));
    return ArmModel(std::vector<double>(n, linkLength), std::vector<JointLimits>(n, JointLimits{-limit, limit, false}));
}

double ArmModel::reach() const {
    return std::accumulate(linkLengths_.begin(), linkLengths_.end(), 0.0);
}

bool ArmModel::withinLimits(const Config& q) const {
    if (q.size() != dof()) {
        return false;
    }
    for (int i = 0; i < dof(); ++i) {
        const JointLimits& lim = limits(i);
        if (!lim.continuous && (q[i] < lim.lower || q[i] > lim.upper)) {
            return false;
        }
    }
    return true;
}

Config ArmModel::normalize(const Config& q) const {
    Config out = q;
    for (int i = 0; i < dof(); ++i) {
        const JointLimits& lim = limits(i);
        out[i] = lim.continuous ? wrapToPi(q[i]) : std::clamp(q[i], lim.lower, lim.upper);
    }
    return out;
}

}  // namespace amp
