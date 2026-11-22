/// @file controls_panel.cpp
/// @brief Widgets and wiring for the controls panel.
#include "controls_panel.hpp"

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFileInfo>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QSignalBlocker>
#include <QSlider>
#include <QSpinBox>
#include <QVBoxLayout>

#include <cmath>
#include <cstdint>

#include "scenario_paths.hpp"
#include "scene_model.hpp"

namespace amp::gui {

namespace {

// Sliders are integer widgets: use tenths of a degree for smooth motion.
constexpr double kSliderUnitsPerRadian = 1800.0 / kPi;

int toSlider(double radians) {
    return static_cast<int>(std::lround(radians * kSliderUnitsPerRadian));
}
double fromSlider(int units) {
    return units / kSliderUnitsPerRadian;
}

QString jointLabelText(int index, double radians) {
    return QStringLiteral("θ%1  %2°").arg(index + 1).arg(radians * 180.0 / kPi, 7, 'f', 1);
}

}  // namespace

ControlsPanel::ControlsPanel(SceneModel* model, QWidget* parent) : QWidget(parent), model_(model) {
    auto* layout = new QVBoxLayout(this);
    layout->addWidget(buildArmGroup());
    layout->addWidget(buildCollisionGroup());
    layout->addWidget(buildScenarioGroup());
    layout->addWidget(buildPlannerGroup());
    layout->addWidget(buildPostProcessingGroup());
    layout->addWidget(buildControlGroup());
    layout->addStretch(1);

    connect(model_, &SceneModel::armChanged, this, &ControlsPanel::rebuildJointSliders);
    connect(model_, &SceneModel::currentChanged, this, &ControlsPanel::syncSlidersFromModel);
    rebuildJointSliders();
}

QGroupBox* ControlsPanel::buildArmGroup() {
    auto* box = new QGroupBox(QStringLiteral("Arm"));
    auto* layout = new QVBoxLayout(box);
    auto* form = new QFormLayout;

    dofSpin_ = new QSpinBox;
    dofSpin_->setRange(2, 6);
    dofSpin_->setValue(model_->arm().dof());
    dofSpin_->setToolTip(QStringLiteral("Number of revolute joints. Total reach stays 1 m."));
    form->addRow(QStringLiteral("Joints (DOF)"), dofSpin_);

    ikModeCombo_ = new QComboBox;
    ikModeCombo_->addItem(QStringLiteral("Damped least squares"),
                          static_cast<int>(SceneModel::IkMode::DampedLeastSquares));
    ikModeCombo_->addItem(QStringLiteral("Analytic, elbow up (2-DOF)"),
                          static_cast<int>(SceneModel::IkMode::AnalyticElbowUp));
    ikModeCombo_->addItem(QStringLiteral("Analytic, elbow down (2-DOF)"),
                          static_cast<int>(SceneModel::IkMode::AnalyticElbowDown));
    ikModeCombo_->setToolTip(
        QStringLiteral("How goal and drag targets are converted to joint angles. "
                       "Analytic modes apply to 2-DOF arms only."));
    form->addRow(QStringLiteral("IK solver"), ikModeCombo_);
    layout->addLayout(form);

    jointLayout_ = new QVBoxLayout;
    jointLayout_->setSpacing(2);
    layout->addLayout(jointLayout_);

    auto* resetButton = new QPushButton(QStringLiteral("Reset pose and goal"));
    layout->addWidget(resetButton);

    connect(dofSpin_, &QSpinBox::valueChanged, model_, &SceneModel::setDof);
    connect(ikModeCombo_, &QComboBox::currentIndexChanged, this, [this](int index) {
        model_->setIkMode(static_cast<SceneModel::IkMode>(ikModeCombo_->itemData(index).toInt()));
    });
    connect(resetButton, &QPushButton::clicked, model_, &SceneModel::reset);
    return box;
}

QGroupBox* ControlsPanel::buildCollisionGroup() {
    auto* box = new QGroupBox(QStringLiteral("Collision checking"));
    auto* layout = new QVBoxLayout(box);
    auto* form = new QFormLayout;

    auto* margin = marginSpin_ = new QDoubleSpinBox;
    margin->setRange(0.0, 50.0);
    margin->setDecimals(1);
    margin->setSingleStep(0.5);
    margin->setSuffix(QStringLiteral(" mm"));
    margin->setValue(model_->checker().options().safetyMargin * 1000.0);
    margin->setToolTip(
        QStringLiteral("Every link is inflated by this margin. Edges are sampled so no arm point moves more "
                       "than 2 x margin between checks, which guarantees the real arm never touches an obstacle "
                       "in between."));
    form->addRow(QStringLiteral("Safety margin"), margin);
    layout->addLayout(form);

    auto* self = new QCheckBox(QStringLiteral("Self-collision (non-adjacent links)"));
    self->setChecked(model_->checker().options().selfCollision);
    layout->addWidget(self);

    auto* hint = new QLabel(QStringLiteral("Drag obstacles in the workspace to reshape the C-space."));
    hint->setWordWrap(true);
    hint->setEnabled(false);  // dim, informational
    layout->addWidget(hint);

    connect(margin, &QDoubleSpinBox::valueChanged, this, [this](double mm) { model_->setSafetyMargin(mm / 1000.0); });
    connect(self, &QCheckBox::toggled, model_, &SceneModel::setSelfCollision);
    return box;
}

QGroupBox* ControlsPanel::buildScenarioGroup() {
    auto* box = new QGroupBox(QStringLiteral("Scenario"));
    auto* layout = new QVBoxLayout(box);
    scenarioCombo_ = new QComboBox;
    scenarioCombo_->addItem(QStringLiteral("Built-in demo scene"), QString());
    for (const auto& file : listScenarioFiles(scenarioDirectory())) {
        QString label = QString::fromStdString(file.stem().string());
        try {
            label = QString::fromStdString(loadScenario(file).name);
        } catch (const ScenarioError&) {
            label += QStringLiteral(" (invalid)");
        }
        scenarioCombo_->addItem(label, QString::fromStdString(file.string()));
    }
    layout->addWidget(scenarioCombo_);

    scenarioDescription_ = new QLabel;
    scenarioDescription_->setWordWrap(true);
    scenarioDescription_->setEnabled(false);  // dim, informational
    layout->addWidget(scenarioDescription_);

    connect(scenarioCombo_, &QComboBox::activated, this, [this](int index) {
        const QString path = scenarioCombo_->itemData(index).toString();
        scenarioDescription_->clear();
        if (!path.isEmpty()) {
            try {
                scenarioDescription_->setText(QString::fromStdString(loadScenario(path.toStdString()).description));
            } catch (const ScenarioError&) {
            }
        }
        emit scenarioRequested(path);
    });
    return box;
}

QGroupBox* ControlsPanel::buildPlannerGroup() {
    auto* box = new QGroupBox(QStringLiteral("Planner"));
    auto* layout = new QVBoxLayout(box);
    auto* form = new QFormLayout;

    plannerCombo_ = new QComboBox;
    plannerCombo_->addItem(QStringLiteral("RRT"), static_cast<int>(PlannerType::Rrt));
    plannerCombo_->addItem(QStringLiteral("RRT*"), static_cast<int>(PlannerType::RrtStar));
    plannerCombo_->addItem(QStringLiteral("A* (grid)"), static_cast<int>(PlannerType::AStar));
    form->addRow(QStringLiteral("Algorithm"), plannerCombo_);

    seedSpin_ = new QSpinBox;
    seedSpin_->setRange(0, 1'000'000);
    seedSpin_->setValue(1);
    seedSpin_->setToolTip(QStringLiteral("Same seed + same scenario = identical plan."));
    form->addRow(QStringLiteral("Seed"), seedSpin_);

    stepSizeSpin_ = new QDoubleSpinBox;
    stepSizeSpin_->setRange(0.01, 1.0);
    stepSizeSpin_->setSingleStep(0.01);
    stepSizeSpin_->setValue(0.15);
    stepSizeSpin_->setSuffix(QStringLiteral(" rad"));
    form->addRow(QStringLiteral("Step size"), stepSizeSpin_);

    iterationsSpin_ = new QSpinBox;
    iterationsSpin_->setRange(100, 500'000);
    iterationsSpin_->setSingleStep(1000);
    iterationsSpin_->setValue(5000);
    form->addRow(QStringLiteral("Iterations"), iterationsSpin_);

    goalBiasSpin_ = new QDoubleSpinBox;
    goalBiasSpin_->setRange(0.0, 1.0);
    goalBiasSpin_->setSingleStep(0.01);
    goalBiasSpin_->setValue(0.05);
    form->addRow(QStringLiteral("Goal bias"), goalBiasSpin_);

    gridSpin_ = new QSpinBox;
    gridSpin_->setRange(4, 256);
    gridSpin_->setValue(48);
    gridSpin_->setToolTip(QStringLiteral("A* only: cells per joint axis."));
    form->addRow(QStringLiteral("A* grid cells"), gridSpin_);

    nnCombo_ = new QComboBox;
    nnCombo_->addItem(QStringLiteral("k-d tree"), static_cast<int>(NnStrategy::KdTree));
    nnCombo_->addItem(QStringLiteral("Brute force"), static_cast<int>(NnStrategy::BruteForce));
    nnCombo_->setToolTip(QStringLiteral("Changes speed only: both return identical neighbours."));
    form->addRow(QStringLiteral("Nearest neighbour"), nnCombo_);
    layout->addLayout(form);

    auto* buttons = new QHBoxLayout;
    planButton_ = new QPushButton(QStringLiteral("Plan"));
    stepButton_ = new QPushButton(QStringLiteral("Step"));
    stepButton_->setToolTip(
        QStringLiteral("Re-plan from scratch with 100 more iterations than last time. Because planning is "
                       "deterministic, this shows the same tree growing step by step."));
    cancelButton_ = new QPushButton(QStringLiteral("Cancel"));
    cancelButton_->setEnabled(false);
    buttons->addWidget(planButton_);
    buttons->addWidget(stepButton_);
    buttons->addWidget(cancelButton_);
    layout->addLayout(buttons);

    summaryLabel_ = new QLabel(QStringLiteral("No plan yet."));
    summaryLabel_->setWordWrap(true);
    layout->addWidget(summaryLabel_);

    connect(planButton_, &QPushButton::clicked, this, &ControlsPanel::planRequested);
    connect(stepButton_, &QPushButton::clicked, this, &ControlsPanel::stepRequested);
    connect(cancelButton_, &QPushButton::clicked, this, &ControlsPanel::cancelRequested);
    connect(model_, &SceneModel::scenarioLoaded, this, [this](const PlannerParams& p) {
        seedSpin_->setValue(static_cast<int>(p.seed));
        stepSizeSpin_->setValue(p.stepSize);
        iterationsSpin_->setValue(p.maxIterations);
        goalBiasSpin_->setValue(p.goalBias);
        gridSpin_->setValue(p.gridResolution);
        const QSignalBlocker blocker(marginSpin_);
        marginSpin_->setValue(model_->checker().options().safetyMargin * 1000.0);
    });
    return box;
}

PlannerType ControlsPanel::plannerType() const {
    return static_cast<PlannerType>(plannerCombo_->currentData().toInt());
}

PlannerParams ControlsPanel::plannerParams() const {
    PlannerParams p;
    p.seed = static_cast<std::uint64_t>(seedSpin_->value());
    p.stepSize = stepSizeSpin_->value();
    p.goalTolerance = p.stepSize;
    p.maxIterations = iterationsSpin_->value();
    p.goalBias = goalBiasSpin_->value();
    p.gridResolution = gridSpin_->value();
    p.nearestNeighbor = static_cast<NnStrategy>(nnCombo_->currentData().toInt());
    return p;
}

void ControlsPanel::setPlanning(bool running) {
    planButton_->setEnabled(!running);
    stepButton_->setEnabled(!running);
    cancelButton_->setEnabled(running);
    scenarioCombo_->setEnabled(!running);
    executeButton_->setEnabled(!running);
}

void ControlsPanel::setPlannerType(PlannerType type) {
    plannerCombo_->setCurrentIndex(plannerCombo_->findData(static_cast<int>(type)));
}

void ControlsPanel::selectScenario(const QString& path) {
    for (int i = 0; i < scenarioCombo_->count(); ++i) {
        // Match by file name: the combo lists the copies next to the executable, while the
        // command line may point at the source tree.
        if (!scenarioCombo_->itemData(i).toString().isEmpty() &&
            QFileInfo(scenarioCombo_->itemData(i).toString()).fileName() == QFileInfo(path).fileName()) {
            scenarioCombo_->setCurrentIndex(i);
            try {
                scenarioDescription_->setText(QString::fromStdString(loadScenario(path.toStdString()).description));
            } catch (const ScenarioError&) {
            }
            return;
        }
    }
}

void ControlsPanel::setPlanSummary(const QString& text) {
    summaryLabel_->setText(text);
}

QGroupBox* ControlsPanel::buildPostProcessingGroup() {
    auto* box = new QGroupBox(QStringLiteral("Post-processing"));
    auto* layout = new QVBoxLayout(box);
    auto* form = new QFormLayout;

    smoothingCheck_ = new QCheckBox(QStringLiteral("Shortcut smoothing"));
    smoothingCheck_->setChecked(true);
    layout->addWidget(smoothingCheck_);

    smoothingIterationsSpin_ = new QSpinBox;
    smoothingIterationsSpin_->setRange(0, 5000);
    smoothingIterationsSpin_->setSingleStep(50);
    smoothingIterationsSpin_->setValue(200);
    form->addRow(QStringLiteral("Shortcut attempts"), smoothingIterationsSpin_);

    maxVelocitySpin_ = new QDoubleSpinBox;
    maxVelocitySpin_->setRange(0.05, 10.0);
    maxVelocitySpin_->setSingleStep(0.1);
    maxVelocitySpin_->setValue(1.5);
    maxVelocitySpin_->setSuffix(QStringLiteral(" rad/s"));
    form->addRow(QStringLiteral("Max joint velocity"), maxVelocitySpin_);

    maxAccelerationSpin_ = new QDoubleSpinBox;
    maxAccelerationSpin_->setRange(0.05, 50.0);
    maxAccelerationSpin_->setSingleStep(0.5);
    maxAccelerationSpin_->setValue(3.0);
    maxAccelerationSpin_->setSuffix(QStringLiteral(" rad/s²"));
    form->addRow(QStringLiteral("Max joint accel."), maxAccelerationSpin_);
    layout->addLayout(form);

    auto* note = new QLabel(QStringLiteral("Trapezoidal profile per segment, all joints synchronized."));
    note->setWordWrap(true);
    note->setEnabled(false);
    layout->addWidget(note);

    connect(smoothingCheck_, &QCheckBox::toggled, this, &ControlsPanel::postProcessingChanged);
    connect(smoothingIterationsSpin_, &QSpinBox::valueChanged, this, &ControlsPanel::postProcessingChanged);
    connect(maxVelocitySpin_, &QDoubleSpinBox::valueChanged, this, &ControlsPanel::postProcessingChanged);
    connect(maxAccelerationSpin_, &QDoubleSpinBox::valueChanged, this, &ControlsPanel::postProcessingChanged);
    return box;
}

QGroupBox* ControlsPanel::buildControlGroup() {
    auto* box = new QGroupBox(QStringLiteral("Control (PID per joint)"));
    auto* layout = new QVBoxLayout(box);
    auto* form = new QFormLayout;

    const PidGains defaults;
    const auto makeSpin = [](double value, double max, int decimals, const QString& suffix) {
        auto* spin = new QDoubleSpinBox;
        spin->setRange(0.0, max);
        spin->setDecimals(decimals);
        spin->setSingleStep(max / 100.0);
        spin->setValue(value);
        spin->setSuffix(suffix);
        return spin;
    };
    kpSpin_ = makeSpin(defaults.kp, 500.0, 1, QString());
    kiSpin_ = makeSpin(defaults.ki, 200.0, 1, QString());
    kdSpin_ = makeSpin(defaults.kd, 100.0, 2, QString());
    filterSpin_ = makeSpin(defaults.derivativeCutoffHz, 500.0, 0, QStringLiteral(" Hz"));
    filterSpin_->setToolTip(QStringLiteral("Low-pass cutoff on the measured joint rate (0 = no filter)."));
    torqueLimitSpin_ = makeSpin(defaults.torqueLimit, 200.0, 1, QStringLiteral(" N·m"));
    torqueLimitSpin_->setMinimum(0.1);
    form->addRow(QStringLiteral("Kp"), kpSpin_);
    form->addRow(QStringLiteral("Ki"), kiSpin_);
    form->addRow(QStringLiteral("Kd"), kdSpin_);
    form->addRow(QStringLiteral("D filter"), filterSpin_);
    form->addRow(QStringLiteral("Torque limit"), torqueLimitSpin_);
    layout->addLayout(form);

    feedforwardCheck_ = new QCheckBox(QStringLiteral("Acceleration feedforward"));
    // On by default: with feedback alone the default gains leave tracking errors of 50 to 80 mrad,
    // enough for the executed arm to clip obstacles in some scenarios (try turning it off).
    feedforwardCheck_->setChecked(true);
    antiWindupCheck_ = new QCheckBox(QStringLiteral("Integral anti-windup"));
    antiWindupCheck_->setChecked(true);
    gravityCheck_ = new QCheckBox(QStringLiteral("Gravity (vertical plane)"));
    gravityCheck_->setToolTip(QStringLiteral("Adds a load torque. Watch the integral term remove the droop."));
    layout->addWidget(feedforwardCheck_);
    layout->addWidget(antiWindupCheck_);
    layout->addWidget(gravityCheck_);

    auto* executeRow = new QHBoxLayout;
    executeButton_ = new QPushButton(QStringLiteral("Execute trajectory"));
    executeButton_->setToolTip(
        QStringLiteral("Simulate the PID-controlled arm following the planned trajectory (F7)."));
    speedCombo_ = new QComboBox;
    for (double speed : {0.25, 0.5, 1.0, 2.0, 4.0}) {
        speedCombo_->addItem(QStringLiteral("%1x").arg(speed), speed);
    }
    speedCombo_->setCurrentIndex(2);
    speedCombo_->setToolTip(QStringLiteral("Playback speed"));
    executeRow->addWidget(executeButton_, 1);
    executeRow->addWidget(speedCombo_);
    layout->addLayout(executeRow);
    trackingLabel_ = new QLabel;
    trackingLabel_->setWordWrap(true);
    layout->addWidget(trackingLabel_);

    for (QDoubleSpinBox* spin : {kpSpin_, kiSpin_, kdSpin_, filterSpin_, torqueLimitSpin_}) {
        connect(spin, &QDoubleSpinBox::valueChanged, this, &ControlsPanel::controlSettingsChanged);
    }
    for (QCheckBox* check : {feedforwardCheck_, antiWindupCheck_, gravityCheck_}) {
        connect(check, &QCheckBox::toggled, this, &ControlsPanel::controlSettingsChanged);
    }
    connect(executeButton_, &QPushButton::clicked, this, &ControlsPanel::executeRequested);
    return box;
}

bool ControlsPanel::smoothingEnabled() const {
    return smoothingCheck_->isChecked();
}

SmoothingParams ControlsPanel::smoothingParams() const {
    SmoothingParams p;
    p.iterations = smoothingIterationsSpin_->value();
    p.seed = static_cast<std::uint64_t>(seedSpin_->value());
    return p;
}

MotionLimits ControlsPanel::motionLimits(int dof) const {
    return MotionLimits::uniform(dof, maxVelocitySpin_->value(), maxAccelerationSpin_->value());
}

TrackingOptions ControlsPanel::trackingOptions() const {
    TrackingOptions o;
    o.gains.kp = kpSpin_->value();
    o.gains.ki = kiSpin_->value();
    o.gains.kd = kdSpin_->value();
    o.gains.derivativeCutoffHz = filterSpin_->value();
    o.gains.torqueLimit = torqueLimitSpin_->value();
    o.gains.antiWindup = antiWindupCheck_->isChecked();
    o.accelerationFeedforward = feedforwardCheck_->isChecked();
    o.settleTime = 1.0;
    return o;
}

bool ControlsPanel::gravityEnabled() const {
    return gravityCheck_->isChecked();
}

void ControlsPanel::setFeedforward(bool on) {
    feedforwardCheck_->setChecked(on);
}

double ControlsPanel::playbackSpeed() const {
    return speedCombo_->currentData().toDouble();
}

void ControlsPanel::setTrackingSummary(const QString& text) {
    trackingLabel_->setText(text);
}

void ControlsPanel::rebuildJointSliders() {
    // Remove the old rows (DOF may have changed).
    while (QLayoutItem* item = jointLayout_->takeAt(0)) {
        delete item->widget();
        delete item;
    }
    jointSliders_.clear();
    jointLabels_.clear();

    {
        const QSignalBlocker blocker(dofSpin_);
        dofSpin_->setValue(model_->arm().dof());
    }

    const ArmModel& arm = model_->arm();
    for (int i = 0; i < arm.dof(); ++i) {
        auto* row = new QWidget;
        auto* rowLayout = new QHBoxLayout(row);
        rowLayout->setContentsMargins(0, 0, 0, 0);

        auto* label = new QLabel;
        label->setMinimumWidth(92);
        auto* slider = new QSlider(Qt::Horizontal);
        const JointLimits& lim = arm.limits(i);
        slider->setRange(toSlider(lim.continuous ? -kPi : lim.lower), toSlider(lim.continuous ? kPi : lim.upper));

        rowLayout->addWidget(label);
        rowLayout->addWidget(slider, 1);
        jointLayout_->addWidget(row);
        jointSliders_.push_back(slider);
        jointLabels_.push_back(label);

        connect(slider, &QSlider::valueChanged, this, [this, i](int value) { model_->setJoint(i, fromSlider(value)); });
    }
    syncSlidersFromModel();
}

void ControlsPanel::syncSlidersFromModel() {
    const Config& q = model_->current();
    for (std::size_t i = 0; i < jointSliders_.size() && static_cast<Eigen::Index>(i) < q.size(); ++i) {
        const double angle = q[static_cast<Eigen::Index>(i)];
        const QSignalBlocker blocker(jointSliders_[i]);  // avoid slider -> model -> slider loops
        jointSliders_[i]->setValue(toSlider(angle));
        jointLabels_[i]->setText(jointLabelText(static_cast<int>(i), angle));
    }
}

}  // namespace amp::gui
