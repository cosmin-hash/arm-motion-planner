/// @file scene_model.hpp
/// @brief The GUI's single source of truth: the arm, obstacles, current configuration and goal.
///
/// Views never own state. They read from SceneModel and call its slots; SceneModel emits
/// signals and every view repaints. The model also holds the plan result, the timed
/// trajectory and the tracking log of the last execution.
#pragma once

#include "amp/arm_model.hpp"
#include "amp/collision.hpp"
#include "amp/kinematics.hpp"
#include "amp/planner.hpp"
#include "amp/scenario.hpp"
#include "amp/simulation.hpp"
#include "amp/trajectory.hpp"
#include "amp/types.hpp"

#include <QObject>
#include <QString>

#include <memory>
#include <optional>
#include <vector>

namespace amp::gui {

/// Observable application state shared by every view.
class SceneModel : public QObject {
    Q_OBJECT

public:
    /// How end-effector targets are converted to joint angles.
    enum class IkMode {
        AnalyticElbowUp,    ///< Closed-form 2-link solution, elbow up (2-DOF only).
        AnalyticElbowDown,  ///< Closed-form 2-link solution, elbow down (2-DOF only).
        DampedLeastSquares  ///< Iterative solver; works for any DOF.
    };

    /// Start with the built-in 3-DOF demo scene.
    explicit SceneModel(QObject* parent = nullptr);
    /// Default destructor, defined in the .cpp next to the other members.
    ~SceneModel() override;

    /// The arm geometry.
    [[nodiscard]] const ArmModel& arm() const { return arm_; }
    /// Obstacles and workspace bounds.
    [[nodiscard]] const Workspace& workspace() const { return workspace_; }
    /// Collision checker for the current arm, workspace and options.
    [[nodiscard]] const CollisionChecker& checker() const { return *checker_; }
    /// Current joint configuration (what the arm in the workspace shows).
    [[nodiscard]] const Config& current() const { return current_; }
    /// Validity of the current configuration.
    [[nodiscard]] CollisionReport currentReport() const { return checker_->check(current_); }
    /// Goal end-effector position in the workspace.
    [[nodiscard]] const Vec2& goalTarget() const { return goalTarget_; }
    /// Goal joint configuration obtained from the goal target by IK.
    [[nodiscard]] const Config& goalConfig() const { return goalConfig_; }
    /// True if IK found a configuration for the goal target.
    [[nodiscard]] bool goalIkOk() const { return goalIkOk_; }
    /// Validity of the goal configuration (meaningful only if goalIkOk()).
    [[nodiscard]] CollisionReport goalReport() const { return checker_->check(goalConfig_); }
    /// Selected IK method.
    [[nodiscard]] IkMode ikMode() const { return ikMode_; }
    /// The most recent planning result for the current start, goal and obstacles, if any.
    [[nodiscard]] const PlanResult* plan() const { return hasPlan_ ? &plan_ : nullptr; }
    /// Which planner produced plan().
    [[nodiscard]] PlannerType planPlanner() const { return planPlanner_; }
    /// Snapshot of the search tree while a planner is running (empty otherwise).
    [[nodiscard]] const std::vector<TreeNode>& liveTree() const { return liveTree_; }
    /// The path that will be executed: the plan after optional smoothing (empty if none).
    [[nodiscard]] const std::vector<Config>& motionPath() const { return motionPath_; }
    /// Time-parameterized motionPath(), or nullptr if there is none.
    [[nodiscard]] const Trajectory* trajectory() const { return hasMotion_ ? &trajectory_ : nullptr; }
    /// Result of the last tracking simulation, or nullptr.
    [[nodiscard]] const TrackingLog* trackingLog() const { return hasLog_ ? &trackingLog_ : nullptr; }
    /// Reference and actual configuration at the current playback time, if playing back.
    [[nodiscard]] std::optional<TrackingSample> executionSample() const;
    /// Current playback time (s), or a negative value when not playing back.
    [[nodiscard]] double executionTime() const { return executionTime_; }
    /// Name of the loaded scenario (empty for the built-in demo scene).
    [[nodiscard]] const QString& scenarioName() const { return scenarioName_; }

public slots:
    /// Replace the arm with a default arm of `dof` equal links (total reach 1 m).
    void setDof(int dof);
    /// Set one joint angle (radians); clamped to the joint limits.
    void setJoint(int index, double angle);
    /// Set the full configuration; clamped to the joint limits.
    void setCurrent(const amp::Config& q);
    /// Move the end effector toward `target` with IK (used for mouse dragging).
    void moveEndEffectorTo(const amp::Vec2& target);
    /// Move the goal marker and re-solve the goal configuration.
    void setGoalTarget(const amp::Vec2& target);
    /// Change the IK method and re-solve the goal configuration.
    void setIkMode(amp::gui::SceneModel::IkMode mode);
    /// Translate obstacle `index` by `delta` (interactive editing).
    void moveObstacle(int index, const amp::Vec2& delta);
    /// Change the safety margin (metres) used for all collision checks.
    void setSafetyMargin(double margin);
    /// Enable or disable self-collision checking.
    void setSelfCollision(bool enabled);
    /// Restore the built-in 3-DOF demo scene.
    void loadDemoScene();
    /// Replace the whole scene with a scenario (arm, obstacles, options, start and goal).
    void loadScenario(const amp::Scenario& scenario);
    /// Store a planning result (it is cleared automatically when the scene changes).
    void setPlan(const amp::PlanResult& result, amp::PlannerType planner);
    /// Show a partial search tree while planning (cleared by setPlan and by scene changes).
    void setLiveTree(const std::vector<amp::TreeNode>& tree);
    /// Store the post-processed path and its trajectory (clears any tracking log).
    void setMotion(const std::vector<amp::Config>& path, const amp::Trajectory& trajectory);
    /// Store a tracking simulation result.
    void setTrackingLog(const amp::TrackingLog& log);
    /// Move the playback cursor (negative: stop showing the executed arm).
    void setExecutionTime(double t);
    /// Restore the default configuration, goal and obstacles, and clear traces.
    void reset();

signals:
    /// The arm geometry (DOF, link lengths, limits) changed.
    void armChanged();
    /// Obstacles, bounds or collision options changed (C-space must be recomputed).
    void obstaclesChanged();
    /// The current configuration changed.
    void currentChanged();
    /// The goal target or goal configuration changed.
    void goalChanged();
    /// Views should discard any accumulated traces.
    void traceCleared();
    /// A human-readable status message (IK results and the like) for the status bar.
    void statusMessage(const QString& text);
    /// plan() changed (new result, or cleared because the scene changed).
    void planChanged();
    /// liveTree() changed.
    void liveTreeChanged();
    /// motionPath() / trajectory() changed.
    void motionChanged();
    /// The tracking log or the playback time changed.
    void executionChanged();
    /// A scenario was loaded; controls should pick up its parameters.
    void scenarioLoaded(const amp::PlannerParams& params);

private:
    /// Solve IK for `target` with the selected method. Restarts are only used for the goal:
    /// while dragging, jumping to a different IK branch would look like a glitch.
    [[nodiscard]] IkResult solveIk(const Vec2& target, const Config& seed, bool allowRestarts) const;
    /// Recompute the goal configuration, preferring a collision-free IK solution.
    void solveGoal();
    /// Rebuild the collision checker after the arm, workspace or options change.
    void rebuildChecker();
    /// Drop the stored plan: it no longer matches the scene.
    void invalidatePlan();

    ArmModel arm_;
    Workspace workspace_;
    CollisionOptions collisionOptions_;
    std::unique_ptr<CollisionChecker> checker_;  // no default constructor, rebuilt on change
    Config current_;
    Vec2 goalTarget_;
    Config goalConfig_;
    bool goalIkOk_ = false;
    IkMode ikMode_ = IkMode::DampedLeastSquares;
    PlanResult plan_;
    bool hasPlan_ = false;
    PlannerType planPlanner_ = PlannerType::Rrt;
    QString scenarioName_;
    std::vector<TreeNode> liveTree_;
    std::vector<Config> motionPath_;
    Trajectory trajectory_;
    bool hasMotion_ = false;
    TrackingLog trackingLog_;
    bool hasLog_ = false;
    double executionTime_ = -1.0;
};

}  // namespace amp::gui
