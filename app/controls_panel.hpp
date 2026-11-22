/// @file controls_panel.hpp
/// @brief Side panel: arm settings, joint sliders, and planner / post-processing / control
/// parameters.
///
/// The panel holds no state of its own beyond widget values: arm edits go straight to
/// SceneModel, and the planning, post-processing and control settings are read on demand.
#pragma once

#include "amp/planner.hpp"
#include "amp/simulation.hpp"
#include "amp/smoothing.hpp"
#include "amp/trajectory.hpp"

#include <QWidget>

#include <vector>

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QGroupBox;
class QLabel;
class QPushButton;
class QSlider;
class QSpinBox;
class QVBoxLayout;

namespace amp::gui {

class SceneModel;

/// The controls dock contents.
class ControlsPanel : public QWidget {
    Q_OBJECT

public:
    /// Build all control groups and bind the arm controls to `model`.
    explicit ControlsPanel(SceneModel* model, QWidget* parent = nullptr);

    /// Planner selected in the combo box.
    [[nodiscard]] PlannerType plannerType() const;
    /// Planner parameters as currently entered.
    [[nodiscard]] PlannerParams plannerParams() const;
    /// Switch buttons between "idle" and "planning" states.
    void setPlanning(bool running);
    /// Select a planner in the combo box.
    void setPlannerType(PlannerType type);
    /// Select the scenario whose file path is `path` in the combo box (no load is triggered).
    void selectScenario(const QString& path);
    /// Show a one-line summary of the last planning result.
    void setPlanSummary(const QString& text);
    /// Show a summary of the last tracking simulation.
    void setTrackingSummary(const QString& text);

    /// Whether shortcut smoothing is enabled, and its parameters.
    [[nodiscard]] bool smoothingEnabled() const;
    /// Smoothing parameters (the seed follows the planner seed).
    [[nodiscard]] SmoothingParams smoothingParams() const;
    /// Velocity and acceleration limits for time parameterization (same on every joint).
    [[nodiscard]] MotionLimits motionLimits(int dof) const;
    /// PID gains and feedforward setting for the tracking simulation.
    [[nodiscard]] TrackingOptions trackingOptions() const;
    /// Whether gravity acts on the simulated arm.
    [[nodiscard]] bool gravityEnabled() const;
    /// Tick or untick acceleration feedforward (used by the --no-feedforward option).
    void setFeedforward(bool on);
    /// Playback speed multiplier for executed trajectories.
    [[nodiscard]] double playbackSpeed() const;

signals:
    /// The user picked a scenario file (empty path: the built-in demo scene).
    void scenarioRequested(const QString& path);
    /// The user asked to plan with the current parameters (Plan button or F5).
    void planRequested();
    /// The user asked to re-plan with 100 more iterations (Step button or F6).
    void stepRequested();
    /// The user asked to cancel the running planner.
    void cancelRequested();
    /// Smoothing or time-parameterization settings changed.
    void postProcessingChanged();
    /// Any controller or plant setting changed (gains are applied live).
    void controlSettingsChanged();
    /// The user asked to execute the current trajectory (Execute button or F7).
    void executeRequested();

private:
    QGroupBox* buildArmGroup();
    QGroupBox* buildCollisionGroup();
    QGroupBox* buildScenarioGroup();
    QGroupBox* buildPlannerGroup();
    QGroupBox* buildPostProcessingGroup();
    QGroupBox* buildControlGroup();

    /// Recreate one slider row per joint (called when the DOF changes).
    void rebuildJointSliders();
    /// Push the model's current configuration into the sliders without feedback loops.
    void syncSlidersFromModel();

    SceneModel* model_;

    // Arm
    QSpinBox* dofSpin_ = nullptr;
    QComboBox* ikModeCombo_ = nullptr;
    QVBoxLayout* jointLayout_ = nullptr;
    std::vector<QSlider*> jointSliders_;
    std::vector<QLabel*> jointLabels_;

    // Collision
    QDoubleSpinBox* marginSpin_ = nullptr;

    // Scenario
    QComboBox* scenarioCombo_ = nullptr;
    QLabel* scenarioDescription_ = nullptr;

    // Planner
    QComboBox* plannerCombo_ = nullptr;
    QSpinBox* seedSpin_ = nullptr;
    QDoubleSpinBox* stepSizeSpin_ = nullptr;
    QSpinBox* iterationsSpin_ = nullptr;
    QDoubleSpinBox* goalBiasSpin_ = nullptr;
    QSpinBox* gridSpin_ = nullptr;
    QComboBox* nnCombo_ = nullptr;
    QLabel* summaryLabel_ = nullptr;
    QPushButton* planButton_ = nullptr;
    QPushButton* stepButton_ = nullptr;
    QPushButton* cancelButton_ = nullptr;

    // Post-processing
    QCheckBox* smoothingCheck_ = nullptr;
    QSpinBox* smoothingIterationsSpin_ = nullptr;
    QDoubleSpinBox* maxVelocitySpin_ = nullptr;
    QDoubleSpinBox* maxAccelerationSpin_ = nullptr;

    // Control
    QCheckBox* feedforwardCheck_ = nullptr;
    QCheckBox* antiWindupCheck_ = nullptr;
    QCheckBox* gravityCheck_ = nullptr;
    QDoubleSpinBox* kpSpin_ = nullptr;
    QDoubleSpinBox* kiSpin_ = nullptr;
    QDoubleSpinBox* kdSpin_ = nullptr;
    QDoubleSpinBox* filterSpin_ = nullptr;
    QDoubleSpinBox* torqueLimitSpin_ = nullptr;
    QPushButton* executeButton_ = nullptr;
    QComboBox* speedCombo_ = nullptr;
    QLabel* trackingLabel_ = nullptr;
};

}  // namespace amp::gui
