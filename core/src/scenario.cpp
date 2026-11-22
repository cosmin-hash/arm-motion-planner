/// @file scenario.cpp
/// @brief JSON scenario parsing (nlohmann/json) and collision-aware goal IK.
#include "amp/scenario.hpp"

#include "amp/cspace.hpp"
#include "amp/rng.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <fstream>
#include <sstream>

namespace amp {

namespace {

using nlohmann::json;

constexpr double kDegToRad = kPi / 180.0;

/// Fetch a required member, with a clear error naming the field path.
const json& require(const json& obj, const char* key, const std::string& where) {
    if (!obj.is_object() || !obj.contains(key)) {
        throw ScenarioError("missing field '" + where + "." + key + "'");
    }
    return obj.at(key);
}

double number(const json& value, const std::string& where) {
    if (!value.is_number()) {
        throw ScenarioError("field '" + where + "' must be a number");
    }
    return value.get<double>();
}

Vec2 vec2(const json& value, const std::string& where) {
    if (!value.is_array() || value.size() != 2) {
        throw ScenarioError("field '" + where + "' must be an array of two numbers");
    }
    return {number(value[0], where + "[0]"), number(value[1], where + "[1]")};
}

Config degrees(const json& value, int expectedSize, const std::string& where) {
    if (!value.is_array() || static_cast<int>(value.size()) != expectedSize) {
        throw ScenarioError("field '" + where + "' must be an array of " + std::to_string(expectedSize) +
                            " angles in degrees");
    }
    Config q(expectedSize);
    for (int i = 0; i < expectedSize; ++i) {
        q[i] = number(value[static_cast<std::size_t>(i)], where) * kDegToRad;
    }
    return q;
}

ArmModel parseArm(const json& j) {
    // Bound by value: GCC 13's -Wdangling-reference cannot tell that require() returns a
    // reference into `j` rather than into its temporary std::string argument. Copies are cheap here.
    const json joints = require(j, "joints", "arm");
    if (!joints.is_array()) {
        throw ScenarioError("field 'arm.joints' must be an array");
    }
    std::vector<double> lengths;
    std::vector<JointLimits> limits;
    for (std::size_t i = 0; i < joints.size(); ++i) {
        const std::string where = "arm.joints[" + std::to_string(i) + "]";
        const json& jj = joints[i];
        lengths.push_back(number(require(jj, "length", where), where + ".length"));
        JointLimits lim;
        lim.continuous = jj.value("continuous", false);
        if (!lim.continuous) {
            lim.lower = number(require(jj, "lower_deg", where), where + ".lower_deg") * kDegToRad;
            lim.upper = number(require(jj, "upper_deg", where), where + ".upper_deg") * kDegToRad;
        }
        limits.push_back(lim);
    }
    const Vec2 base = j.contains("base") ? vec2(j.at("base"), "arm.base") : Vec2::Zero();
    const double radius = j.contains("link_radius") ? number(j.at("link_radius"), "arm.link_radius") : 0.02;
    try {
        return ArmModel(std::move(lengths), std::move(limits), base, radius);
    } catch (const std::invalid_argument& e) {
        throw ScenarioError(std::string("invalid arm: ") + e.what());
    }
}

Obstacle parseObstacle(const json& j, const std::string& where) {
    const std::string type = require(j, "type", where).get<std::string>();
    try {
        if (type == "circle") {
            return Circle{vec2(require(j, "center", where), where + ".center"),
                          number(require(j, "radius", where), where + ".radius")};
        }
        if (type == "rectangle") {
            return ConvexPolygon::rectangle(vec2(require(j, "min", where), where + ".min"),
                                            vec2(require(j, "max", where), where + ".max"));
        }
        if (type == "polygon") {
            const json& vs = require(j, "vertices", where);
            std::vector<Vec2> vertices;
            for (std::size_t i = 0; i < vs.size(); ++i) {
                vertices.push_back(vec2(vs[i], where + ".vertices[" + std::to_string(i) + "]"));
            }
            return ConvexPolygon(std::move(vertices));
        }
    } catch (const std::invalid_argument& e) {
        throw ScenarioError("invalid obstacle '" + where + "': " + e.what());
    }
    throw ScenarioError("unknown obstacle type '" + type + "' at '" + where + "'");
}

PlannerParams parsePlanner(const json& j) {
    PlannerParams p;
    if (j.is_null()) {
        return p;
    }
    p.stepSize = j.value("step_size", p.stepSize);
    p.maxIterations = j.value("max_iterations", p.maxIterations);
    p.goalBias = j.value("goal_bias", p.goalBias);
    p.goalTolerance = j.value("goal_tolerance", p.goalTolerance);
    p.seed = j.value("seed", p.seed);
    p.gridResolution = j.value("grid_resolution", p.gridResolution);
    p.maxRewireRadius = j.value("max_rewire_radius", p.maxRewireRadius);
    return p;
}

}  // namespace

IkResult solveIkCollisionFree(const CollisionChecker& checker, const Vec2& target, const Config& seed,
                              std::optional<ElbowConfig> elbow, int extraAttempts) {
    const ArmModel& arm = checker.arm();
    if (arm.dof() == 2 && elbow) {
        return solveIkAnalytic2Link(arm, target, *elbow);
    }
    DlsParams params;
    params.restarts = 10;
    IkResult r = solveIkDls(arm, target, seed, params);
    if (r.ok() && checker.isValid(r.q)) {
        return r;
    }
    // A redundant arm reaches most targets in many ways; look for one that does not collide.
    Rng rng(7);
    for (int attempt = 0; attempt < extraAttempts; ++attempt) {
        const IkResult candidate = solveIkDls(arm, target, sampleUniform(arm, rng));
        if (candidate.ok() && checker.isValid(candidate.q)) {
            return candidate;
        }
    }
    return r;
}

Scenario parseScenario(const std::string& jsonText) {
    json j;
    try {
        j = json::parse(jsonText);
    } catch (const json::parse_error& e) {
        throw ScenarioError(std::string("JSON syntax error: ") + e.what());
    }

    try {
        ArmModel arm = parseArm(require(j, "arm", "scenario"));
        Workspace workspace;
        if (j.contains("bounds")) {
            const json& b = j.at("bounds");
            workspace.bounds = Bounds{vec2(require(b, "min", "bounds"), "bounds.min"),
                                      vec2(require(b, "max", "bounds"), "bounds.max")};
        }
        if (j.contains("obstacles")) {
            const json& obs = j.at("obstacles");
            for (std::size_t i = 0; i < obs.size(); ++i) {
                workspace.obstacles.push_back(parseObstacle(obs[i], "obstacles[" + std::to_string(i) + "]"));
            }
        }
        CollisionOptions collision;
        if (j.contains("collision")) {
            const json& c = j.at("collision");
            collision.safetyMargin = c.value("safety_margin", collision.safetyMargin);
            collision.selfCollision = c.value("self_collision", collision.selfCollision);
        }

        const int dof = arm.dof();
        Config start =
            degrees(require(require(j, "start", "scenario"), "joints_deg", "start"), dof, "start.joints_deg");

        Scenario s{j.value("name", std::string("Unnamed scenario")),
                   j.value("description", std::string()),
                   arm,
                   workspace,
                   collision,
                   start,
                   Config(),
                   std::nullopt,
                   parsePlanner(j.contains("planner") ? j.at("planner") : json())};

        const json goal = require(j, "goal", "scenario");  // by value: see parseArm()
        if (goal.contains("joints_deg")) {
            s.goal = degrees(goal.at("joints_deg"), dof, "goal.joints_deg");
        } else if (goal.contains("end_effector")) {
            // Resolve the workspace goal to joint angles here, so planners always get a configuration.
            const Vec2 target = vec2(goal.at("end_effector"), "goal.end_effector");
            const Config seed =
                goal.contains("ik_seed_deg") ? degrees(goal.at("ik_seed_deg"), dof, "goal.ik_seed_deg") : start;
            std::optional<ElbowConfig> elbow;
            if (goal.contains("elbow")) {
                const std::string e = goal.at("elbow").get<std::string>();
                if (e != "up" && e != "down") {
                    throw ScenarioError("field 'goal.elbow' must be \"up\" or \"down\"");
                }
                elbow = e == "up" ? ElbowConfig::Up : ElbowConfig::Down;
            }
            const CollisionChecker checker(arm, workspace, collision);
            const IkResult ik = solveIkCollisionFree(checker, target, seed, elbow);
            if (!ik.ok()) {
                throw ScenarioError(std::string("goal.end_effector: inverse kinematics failed (") +
                                    toString(ik.status) + ")");
            }
            s.goal = ik.q;
            s.goalTarget = target;
        } else {
            throw ScenarioError("field 'goal' needs either 'joints_deg' or 'end_effector'");
        }
        return s;
    } catch (const json::exception& e) {
        throw ScenarioError(std::string("wrong value type: ") + e.what());
    }
}

Scenario loadScenario(const std::filesystem::path& file) {
    std::ifstream in(file);
    if (!in) {
        throw ScenarioError("cannot open scenario file '" + file.string() + "'");
    }
    std::stringstream buffer;
    buffer << in.rdbuf();
    try {
        return parseScenario(buffer.str());
    } catch (const ScenarioError& e) {
        throw ScenarioError(file.filename().string() + ": " + e.what());
    }
}

std::vector<std::filesystem::path> listScenarioFiles(const std::filesystem::path& directory) {
    std::vector<std::filesystem::path> files;
    std::error_code ec;
    if (!std::filesystem::is_directory(directory, ec)) {
        return files;
    }
    for (const auto& entry : std::filesystem::directory_iterator(directory, ec)) {
        if (entry.is_regular_file() && entry.path().extension() == ".json") {
            files.push_back(entry.path());
        }
    }
    std::sort(files.begin(), files.end());
    return files;
}

}  // namespace amp
