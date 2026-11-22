/// @file scenario.hpp
/// @brief JSON scenario files: arm, obstacles, start, goal and planner parameters.
///
/// File format (angles in degrees for readability; converted to radians on load):
///
/// @code{.json}
/// {
///   "name": "Narrow passage",
///   "description": "Free text shown in the GUI.",
///   "arm": {
///     "base": [0, 0],
///     "link_radius": 0.025,
///     "joints": [
///       { "length": 0.5, "lower_deg": -170, "upper_deg": 170 },
///       { "length": 0.4, "continuous": true }
///     ]
///   },
///   "bounds": { "min": [-1.2, -0.3], "max": [1.2, 1.2] },
///   "obstacles": [
///     { "type": "circle", "center": [0.5, 0.4], "radius": 0.1 },
///     { "type": "rectangle", "min": [0.2, 0.6], "max": [0.4, 0.9] },
///     { "type": "polygon", "vertices": [[0, 0], [0.2, 0], [0.1, 0.2]] }
///   ],
///   "collision": { "safety_margin": 0.005, "self_collision": true },
///   "start": { "joints_deg": [0, 0] },
///   "goal":  { "end_effector": [0.3, 0.6], "elbow": "up", "ik_seed_deg": [30, 30] },
///   "planner": { "step_size": 0.15, "max_iterations": 5000, "goal_bias": 0.05,
///                "goal_tolerance": 0.15, "seed": 1, "grid_resolution": 48 }
/// }
/// @endcode
///
/// "goal" may instead be { "joints_deg": [...] }. Missing optional fields take the defaults
/// of CollisionOptions and PlannerParams.
#pragma once

#include "amp/arm_model.hpp"
#include "amp/collision.hpp"
#include "amp/kinematics.hpp"
#include "amp/planner.hpp"

#include <filesystem>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace amp {

/// Thrown for unreadable or malformed scenario files; the message names the offending field.
class ScenarioError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

/// A complete planning problem.
struct Scenario {
    std::string name;
    std::string description;
    ArmModel arm;
    Workspace workspace;
    CollisionOptions collision;
    Config start;
    Config goal;                     ///< Always a joint configuration (IK already resolved).
    std::optional<Vec2> goalTarget;  ///< Set if the file specified the goal as an end-effector position.
    PlannerParams planner;
};

/// Parse a scenario from JSON text. @throws ScenarioError
[[nodiscard]] Scenario parseScenario(const std::string& jsonText);

/// Load and parse a scenario file. @throws ScenarioError (including the file name)
[[nodiscard]] Scenario loadScenario(const std::filesystem::path& file);

/// All *.json files in `directory`, sorted by file name (empty if the directory is missing).
[[nodiscard]] std::vector<std::filesystem::path> listScenarioFiles(const std::filesystem::path& directory);

/// Inverse kinematics that prefers a collision-free solution.
///
/// 2-DOF arms with `elbow` set use the analytic solver (that branch only). Otherwise DLS is
/// run from `seed` with restarts. If the solution collides, up to `extraAttempts` further
/// deterministic seeds are tried. Returns the first collision-free solution, or the plain IK
/// result if none was found (check it with the checker).
[[nodiscard]] IkResult solveIkCollisionFree(const CollisionChecker& checker, const Vec2& target, const Config& seed,
                                            std::optional<ElbowConfig> elbow = std::nullopt, int extraAttempts = 30);

}  // namespace amp
