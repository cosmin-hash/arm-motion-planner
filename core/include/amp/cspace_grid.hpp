/// @file cspace_grid.hpp
/// @brief Dense sampling of a 2-DOF configuration space into an occupancy image.
///
/// Used by the GUI to draw the C-obstacle region: the set of joint angles where the arm
/// touches something. It is the clearest picture of why planning happens in C-space.
#pragma once

#include "amp/collision.hpp"

#include <vector>

namespace amp {

/// Occupancy of a 2-DOF C-space sampled on a regular grid over [-pi, pi)^2.
struct CSpaceGrid2D {
    int resolution = 0;                ///< Cells per axis.
    std::vector<CollisionKind> cells;  ///< Row-major: cells[j * resolution + i] is (theta1_i, theta2_j).

    /// Joint angle at the centre of cell index `i` along either axis.
    [[nodiscard]] double cellCenter(int i) const;
    /// Occupancy of cell (i, j).
    [[nodiscard]] CollisionKind at(int i, int j) const {
        return cells[static_cast<std::size_t>(j) * static_cast<std::size_t>(resolution) + static_cast<std::size_t>(i)];
    }
};

/// Check every cell centre of a `resolution` x `resolution` grid.
/// @pre checker.arm().dof() == 2. @throws std::invalid_argument otherwise.
[[nodiscard]] CSpaceGrid2D computeCSpaceGrid2D(const CollisionChecker& checker, int resolution);

}  // namespace amp
