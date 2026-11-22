/// @file scene_model.cpp
/// @brief Implementation of the GUI state model.
#include "scene_model.hpp"

#include "amp/cspace.hpp"
#include "amp/kinematics.hpp"
#include "amp/rng.hpp"

#include <algorithm>
#include <vector>

namespace amp::gui {

namespace {

constexpr double kDefaultReach = 1.0;                // metres
constexpr double kJointLimit = 170.0 * kPi / 180.0;  // +/-170 deg: realistic, and leaves a visible
                                                     // forbidden band in the C-space view
constexpr double kLinkRadius = 0.025;

ArmModel makeDefaultArm(int dof) {
    const auto n = static_cast<std::size_t>(dof);
    return ArmModel(std::vector<double>(n, kDefaultReach / dof),
                    std::vector<JointLimits>(n, JointLimits{-kJointLimit, kJointLimit, false}), Vec2::Zero(),
                    kLinkRadius);
}

Config makeDefaultConfig(int dof) {
    // A gently curled arm reads better on screen than a straight one.
    Config q = Config::Constant(dof, 0.5);
    q[0] = 0.3;
    return q;
}

/// A small demo workspace: a "table" boundary just below the base, and three obstacles.
Workspace makeDefaultWorkspace() {
    Workspace ws;
    ws.bounds = Bounds{Vec2(-1.15, -0.3), Vec2(1.15, 1.15)};
    ws.obstacles.push_back(Circle{Vec2(-0.45, 0.45), 0.12});
    ws.obstacles.push_back(ConvexPolygon::rectangle(Vec2(0.55, 0.62), Vec2(0.78, 0.86)));
    ws.obstacles.push_back(ConvexPolygon({Vec2(0.55, -0.22), Vec2(0.85, -0.22), Vec2(0.7, 0.02)}));
    return ws;
}

const Vec2 kDefaultGoal{0.35, 0.55};

}  // namespace

SceneModel::SceneModel(QObject* parent)
    : QObject(parent),
      arm_(makeDefaultArm(3)),
      workspace_(makeDefaultWorkspace()),
      current_(makeDefaultConfig(3)),
      goalTarget_(kDefaultGoal) {
    rebuildChecker();
    solveGoal();
    // Any change to the scene makes a stored plan stale (its start, goal or obstacles differ).
    for (auto signal : {&SceneModel::armChanged, &SceneModel::obstaclesChanged, &SceneModel::currentChanged,
                        &SceneModel::goalChanged}) {
        connect(this, signal, this, &SceneModel::invalidatePlan);
    }
}

SceneModel::~SceneModel() = default;

void SceneModel::rebuildChecker() {
    checker_ = std::make_unique<CollisionChecker>(arm_, workspace_, collisionOptions_);
}

void SceneModel::setDof(int dof) {
    if (dof == arm_.dof()) {
        return;
    }
    arm_ = makeDefaultArm(dof);
    current_ = makeDefaultConfig(dof);
    rebuildChecker();
    solveGoal();
    emit armChanged();
    emit obstaclesChanged();
    emit traceCleared();
    emit currentChanged();
    emit goalChanged();
}

void SceneModel::setJoint(int index, double angle) {
    if (index < 0 || index >= arm_.dof()) {
        return;
    }
    Config q = current_;
    q[index] = angle;
    setCurrent(q);
}

void SceneModel::setCurrent(const Config& q) {
    if (q.size() != arm_.dof()) {
        return;
    }
    current_ = arm_.normalize(q);
    emit currentChanged();
}

void SceneModel::moveEndEffectorTo(const Vec2& target) {
    const IkResult r = solveIk(target, current_, /*allowRestarts=*/false);
    // NotConverged from DLS still moves the arm to the closest reachable pose, which is the
    // natural behaviour when the mouse is dragged outside the workspace.
    if (r.ok() || r.status == IkStatus::NotConverged) {
        current_ = r.q;
        emit currentChanged();
    }
    if (!r.ok()) {
        emit statusMessage(QStringLiteral("End-effector IK: %1 (residual %2 mm)")
                               .arg(QString::fromLatin1(toString(r.status)))
                               .arg(r.residual * 1000.0, 0, 'f', 1));
    }
}

void SceneModel::setGoalTarget(const Vec2& target) {
    goalTarget_ = target;
    solveGoal();
    emit goalChanged();
}

void SceneModel::setIkMode(IkMode mode) {
    ikMode_ = mode;
    solveGoal();
    emit goalChanged();
}

void SceneModel::moveObstacle(int index, const Vec2& delta) {
    if (index < 0 || index >= static_cast<int>(workspace_.obstacles.size())) {
        return;
    }
    translateObstacle(workspace_.obstacles[static_cast<std::size_t>(index)], delta);
    rebuildChecker();
    emit obstaclesChanged();
    emit currentChanged();  // validity of the current pose may have changed
    emit goalChanged();
}

void SceneModel::setSafetyMargin(double margin) {
    collisionOptions_.safetyMargin = margin;
    collisionOptions_.maxStepDisplacement = 0.0;  // re-derive 2 * margin
    if (margin <= 0.0) {
        // Zero margin gives up the continuous guarantee; still need a finite step.
        collisionOptions_.maxStepDisplacement = 0.002;
    }
    rebuildChecker();
    emit obstaclesChanged();
    emit currentChanged();
    emit goalChanged();
}

void SceneModel::setSelfCollision(bool enabled) {
    collisionOptions_.selfCollision = enabled;
    rebuildChecker();
    emit obstaclesChanged();
    emit currentChanged();
    emit goalChanged();
}

void SceneModel::loadDemoScene() {
    arm_ = makeDefaultArm(3);
    collisionOptions_ = CollisionOptions{};
    scenarioName_.clear();
    emit armChanged();
    reset();
    emit scenarioLoaded(PlannerParams{});
}

void SceneModel::loadScenario(const Scenario& scenario) {
    arm_ = scenario.arm;
    workspace_ = scenario.workspace;
    collisionOptions_ = scenario.collision;
    rebuildChecker();
    current_ = scenario.start;
    goalConfig_ = scenario.goal;
    goalIkOk_ = true;
    goalTarget_ = scenario.goalTarget.value_or(endEffectorPosition(arm_, scenario.goal));
    scenarioName_ = QString::fromStdString(scenario.name);
    emit armChanged();
    emit obstaclesChanged();
    emit traceCleared();
    emit currentChanged();
    emit goalChanged();
    emit scenarioLoaded(scenario.planner);
    emit statusMessage(QStringLiteral("Loaded scenario: %1").arg(scenarioName_));
}

void SceneModel::setPlan(const PlanResult& result, PlannerType planner) {
    plan_ = result;
    planPlanner_ = planner;
    hasPlan_ = true;
    liveTree_.clear();
    motionPath_.clear();
    hasMotion_ = false;
    hasLog_ = false;
    executionTime_ = -1.0;
    emit planChanged();
    emit motionChanged();
    emit executionChanged();
}

void SceneModel::setLiveTree(const std::vector<TreeNode>& tree) {
    liveTree_ = tree;
    emit liveTreeChanged();
}

void SceneModel::setMotion(const std::vector<Config>& path, const Trajectory& trajectory) {
    motionPath_ = path;
    trajectory_ = trajectory;
    hasMotion_ = !path.empty();
    hasLog_ = false;
    executionTime_ = -1.0;
    emit motionChanged();
    emit executionChanged();
}

void SceneModel::setTrackingLog(const TrackingLog& log) {
    trackingLog_ = log;
    hasLog_ = true;
    emit executionChanged();
}

void SceneModel::setExecutionTime(double t) {
    executionTime_ = t;
    emit executionChanged();
}

std::optional<TrackingSample> SceneModel::executionSample() const {
    if (!hasLog_ || executionTime_ < 0.0 || trackingLog_.samples.empty()) {
        return std::nullopt;
    }
    const auto& samples = trackingLog_.samples;
    // First sample at or after the playback time; interpolate linearly with the one before.
    const auto it = std::lower_bound(samples.begin(), samples.end(), executionTime_,
                                     [](const TrackingSample& s, double t) { return s.time < t; });
    if (it == samples.begin()) {
        return samples.front();
    }
    if (it == samples.end()) {
        return samples.back();
    }
    const TrackingSample& b = *it;
    const TrackingSample& a = *(it - 1);
    const double w = b.time > a.time ? (executionTime_ - a.time) / (b.time - a.time) : 0.0;
    TrackingSample s = a;
    s.time = executionTime_;
    s.reference = a.reference + w * (b.reference - a.reference);
    s.actual = a.actual + w * (b.actual - a.actual);
    s.error = s.reference - s.actual;
    return s;
}

void SceneModel::invalidatePlan() {
    if (!liveTree_.empty()) {
        liveTree_.clear();
        emit liveTreeChanged();
    }
    if (hasPlan_ || hasMotion_ || hasLog_) {
        hasPlan_ = false;
        plan_ = PlanResult{};
        motionPath_.clear();
        hasMotion_ = false;
        hasLog_ = false;
        executionTime_ = -1.0;
        emit planChanged();
        emit motionChanged();
        emit executionChanged();
    }
}

void SceneModel::reset() {
    current_ = makeDefaultConfig(arm_.dof());
    goalTarget_ = kDefaultGoal;
    workspace_ = makeDefaultWorkspace();
    rebuildChecker();
    solveGoal();
    emit obstaclesChanged();
    emit traceCleared();
    emit currentChanged();
    emit goalChanged();
}

IkResult SceneModel::solveIk(const Vec2& target, const Config& seed, bool allowRestarts) const {
    if (arm_.dof() == 2 && ikMode_ != IkMode::DampedLeastSquares) {
        const ElbowConfig elbow = ikMode_ == IkMode::AnalyticElbowUp ? ElbowConfig::Up : ElbowConfig::Down;
        return solveIkAnalytic2Link(arm_, target, elbow);
    }
    DlsParams params;
    params.restarts = allowRestarts ? 10 : 0;
    return solveIkDls(arm_, target, seed, params);
}

void SceneModel::solveGoal() {
    // Seed from the current configuration so the goal pose resembles the current pose.
    IkResult r = solveIk(goalTarget_, current_, /*allowRestarts=*/true);

    // A redundant arm has many IK solutions; a planner needs one that is collision-free.
    // Try a few more deterministic seeds before giving up on a colliding solution.
    if (r.ok() && !checker_->isValid(r.q) && ikMode_ == IkMode::DampedLeastSquares) {
        Rng rng(7);
        for (int attempt = 0; attempt < 30; ++attempt) {
            const IkResult candidate = solveIkDls(arm_, goalTarget_, sampleUniform(arm_, rng));
            if (candidate.ok() && checker_->isValid(candidate.q)) {
                r = candidate;
                break;
            }
        }
    }

    goalIkOk_ = r.ok();
    goalConfig_ = r.ok() ? r.q : current_;
    if (!r.ok()) {
        emit statusMessage(QStringLiteral("Goal IK failed: %1").arg(QString::fromLatin1(toString(r.status))));
    } else if (!checker_->isValid(r.q)) {
        emit statusMessage(QStringLiteral("Goal configuration is in collision (%1)")
                               .arg(QString::fromLatin1(toString(checker_->check(r.q).kind))));
    }
}

}  // namespace amp::gui
