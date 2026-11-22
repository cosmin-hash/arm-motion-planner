/// @file cspace_grid.cpp
/// @brief Brute-force 2-DOF C-space occupancy sampling.
#include "amp/cspace_grid.hpp"

#include <stdexcept>

namespace amp {

double CSpaceGrid2D::cellCenter(int i) const {
    return -kPi + (i + 0.5) * kTwoPi / resolution;
}

CSpaceGrid2D computeCSpaceGrid2D(const CollisionChecker& checker, int resolution) {
    if (checker.arm().dof() != 2) {
        throw std::invalid_argument("computeCSpaceGrid2D: only defined for 2-DOF arms");
    }
    if (resolution < 1) {
        throw std::invalid_argument("computeCSpaceGrid2D: resolution must be positive");
    }
    CSpaceGrid2D grid;
    grid.resolution = resolution;
    grid.cells.resize(static_cast<std::size_t>(resolution) * static_cast<std::size_t>(resolution));

    Config q(2);
    for (int j = 0; j < resolution; ++j) {
        q[1] = grid.cellCenter(j);
        for (int i = 0; i < resolution; ++i) {
            q[0] = grid.cellCenter(i);
            grid.cells[static_cast<std::size_t>(j) * static_cast<std::size_t>(resolution) +
                       static_cast<std::size_t>(i)] = checker.check(q).kind;
        }
    }
    return grid;
}

}  // namespace amp
