/// @file main_window.cpp
/// @brief Window assembly, menus and status bar.
#include "main_window.hpp"

#include "amp/dynamics.hpp"
#include "amp/kinematics.hpp"
#include "amp/scenario.hpp"
#include "amp/simulation.hpp"
#include "amp/smoothing.hpp"

#include <QApplication>
#include <QDockWidget>
#include <QFile>
#include <QFileDialog>
#include <QLabel>
#include <QMenuBar>
#include <QMessageBox>
#include <QScreen>
#include <QScrollArea>
#include <QSplitter>
#include <QStatusBar>
#include <QTabWidget>
#include <QTextStream>
#include <QTimer>

#include "controls_panel.hpp"
#include "cspace_view.hpp"
#include "planning_controller.hpp"
#include "plot_widget.hpp"
#include "scenario_paths.hpp"
#include "scene_model.hpp"
#include "workspace_view.hpp"

namespace amp::gui {

MainWindow::MainWindow(int initialDof, QWidget* parent)
    : QMainWindow(parent), model_(new SceneModel(this)), planning_(new PlanningController(this)) {
    setWindowTitle(QStringLiteral("arm-motion-planner"));

    // Right column: C-space view above the plots.
    costPlot_ = new PlotWidget(QStringLiteral("RRT* solution cost"), QStringLiteral("iteration"),
                               QStringLiteral("path cost [rad]"));
    costPlot_->setPlaceholder(QStringLiteral("Run RRT* to see the best path cost fall as the tree grows."));
    trackingPlot_ = new PlotWidget(QStringLiteral("Joint tracking error  (reference - actual)"),
                                   QStringLiteral("time [s]"), QStringLiteral("error [mrad]"));
    trackingPlot_->setPlaceholder(
        QStringLiteral("Plan a path, then press Execute trajectory to simulate the PID-controlled arm."));
    auto* plots = plotTabs_ = new QTabWidget;
    plots->addTab(costPlot_, QStringLiteral("RRT* cost"));
    plots->addTab(trackingPlot_, QStringLiteral("Tracking error"));

    auto* right = new QSplitter(Qt::Vertical);
    right->addWidget(new CSpaceView(model_));
    right->addWidget(plots);
    right->setStretchFactor(0, 3);
    right->setStretchFactor(1, 2);

    auto* central = new QSplitter(Qt::Horizontal);
    central->addWidget(new WorkspaceView(model_));
    central->addWidget(right);
    central->setStretchFactor(0, 3);
    central->setStretchFactor(1, 2);
    setCentralWidget(central);

    // Controls dock (scrollable so it works on small screens).
    auto* scroll = new QScrollArea;
    panel_ = new ControlsPanel(model_);
    scroll->setWidget(panel_);
    scroll->setWidgetResizable(true);
    scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    scroll->setMinimumWidth(300);
    auto* dock = new QDockWidget(QStringLiteral("Controls"));
    dock->setWidget(scroll);
    dock->setFeatures(QDockWidget::DockWidgetMovable | QDockWidget::DockWidgetFloatable);
    addDockWidget(Qt::LeftDockWidgetArea, dock);

    // Status bar: transient messages on the left, a permanent readout on the right.
    readout_ = new QLabel;
    statusBar()->addPermanentWidget(readout_);
    connect(model_, &SceneModel::statusMessage, this,
            [this](const QString& text) { statusBar()->showMessage(text, 4000); });
    connect(model_, &SceneModel::currentChanged, this, &MainWindow::updateReadout);
    connect(model_, &SceneModel::armChanged, this, &MainWindow::updateReadout);
    connect(model_, &SceneModel::obstaclesChanged, this, &MainWindow::updateReadout);

    // Planning: buttons -> background job -> SceneModel::setPlan.
    connect(panel_, &ControlsPanel::planRequested, this, [this] {
        stepIterations_ = 0;
        startPlanning(0);
    });
    connect(panel_, &ControlsPanel::stepRequested, this, [this] {
        stepIterations_ += 100;
        startPlanning(stepIterations_);
    });
    connect(panel_, &ControlsPanel::cancelRequested, planning_, &PlanningController::cancel);
    connect(panel_, &ControlsPanel::scenarioRequested, this, &MainWindow::loadScenarioFile);
    connect(panel_, &ControlsPanel::postProcessingChanged, this, &MainWindow::processPlan);
    connect(panel_, &ControlsPanel::executeRequested, this, &MainWindow::execute);
    connect(panel_, &ControlsPanel::controlSettingsChanged, this, [this] {
        // Live tuning: re-run the (fast) simulation with the new gains and keep playing.
        if (model_->trackingLog() != nullptr) {
            const double t = model_->executionTime();
            simulate();
            model_->setExecutionTime(t);
        }
    });
    playbackTimer_ = new QTimer(this);
    playbackTimer_->setInterval(16);  // ~60 frames per second
    connect(playbackTimer_, &QTimer::timeout, this, &MainWindow::onPlaybackTick);
    connect(model_, &SceneModel::executionChanged, this, [this] {
        if (model_->trackingLog() == nullptr) {
            playbackTimer_->stop();
            trackingPlot_->clear();
            panel_->setTrackingSummary(QString());
        }
    });
    connect(planning_, &PlanningController::finished, this, &MainWindow::onPlanningFinished);
    connect(planning_, &PlanningController::treeSnapshot, this, [this](const std::vector<TreeNode>& tree) {
        if (planning_->isRunning()) {  // ignore a snapshot that arrives after the job ended
            model_->setLiveTree(tree);
        }
    });
    connect(planning_, &PlanningController::progress, this, [this](int iteration, int nodes, double best) {
        panel_->setPlanSummary(
            QStringLiteral("Planning... iteration %1, %2 nodes%3")
                .arg(iteration)
                .arg(nodes)
                .arg(best >= 0 ? QStringLiteral(", best cost %1 rad").arg(best, 0, 'f', 3) : QString()));
        if (best >= 0 && (liveCost_.isEmpty() || best < liveCost_.back().y())) {
            liveCost_.append(QPointF(iteration, best));
            costPlot_->setSeries({PlotSeries{QStringLiteral("best cost"), QColor(0xf5, 0xa6, 0x23), liveCost_}});
        }
    });
    // Track scene edits so a result computed for an outdated scene is never displayed.
    for (auto signal : {&SceneModel::armChanged, &SceneModel::obstaclesChanged, &SceneModel::currentChanged,
                        &SceneModel::goalChanged}) {
        connect(model_, signal, this, [this] { ++sceneRevision_; });
    }
    connect(model_, &SceneModel::planChanged, this, [this] {
        if (model_->plan() == nullptr) {
            stepIterations_ = 0;  // the next Step starts a fresh sequence
        }
    });

    buildMenus();
    model_->setDof(initialDof);
    updateReadout();
    // Open at 85% of the available screen area so the layout fits on scaled (high-DPI) displays.
    if (const QScreen* screen = QGuiApplication::primaryScreen()) {
        resize(screen->availableGeometry().size() * 0.85);
    }
}

void MainWindow::buildMenus() {
    QMenu* file = menuBar()->addMenu(QStringLiteral("&File"));
    file->addAction(QStringLiteral("&Open scenario..."), QKeySequence::Open, this, [this] {
        const QString path = QFileDialog::getOpenFileName(this, QStringLiteral("Open scenario"),
                                                          QString::fromStdString(scenarioDirectory().string()),
                                                          QStringLiteral("Scenarios (*.json)"));
        if (!path.isEmpty()) {
            loadScenarioFile(path);
        }
    });
    file->addAction(QStringLiteral("Save &screenshot..."), QKeySequence(QStringLiteral("Ctrl+Shift+S")), this, [this] {
        const QString path =
            QFileDialog::getSaveFileName(this, QStringLiteral("Save screenshot"), QStringLiteral("screenshot.png"),
                                         QStringLiteral("PNG images (*.png)"));
        if (!path.isEmpty() && !saveScreenshot(path)) {
            QMessageBox::warning(this, QStringLiteral("Screenshot"), QStringLiteral("Could not save %1").arg(path));
        }
    });
    file->addAction(
        QStringLiteral("&Export tracking log (CSV)..."), QKeySequence(QStringLiteral("Ctrl+E")), this, [this] {
            if (model_->trackingLog() == nullptr) {
                QMessageBox::information(this, QStringLiteral("Export"), QStringLiteral("Execute a trajectory first."));
                return;
            }
            const QString path =
                QFileDialog::getSaveFileName(this, QStringLiteral("Export tracking log"),
                                             QStringLiteral("tracking.csv"), QStringLiteral("CSV files (*.csv)"));
            if (!path.isEmpty() && !exportTrackingCsv(path)) {
                QMessageBox::warning(this, QStringLiteral("Export"), QStringLiteral("Could not write %1").arg(path));
            }
        });
    file->addSeparator();
    file->addAction(QStringLiteral("&Quit"), QKeySequence::Quit, qApp, &QApplication::quit);

    QMenu* run = menuBar()->addMenu(QStringLiteral("&Run"));
    run->addAction(QStringLiteral("&Plan"), QKeySequence(Qt::Key_F5), panel_, &ControlsPanel::planRequested);
    run->addAction(QStringLiteral("&Step (+100 iterations)"), QKeySequence(Qt::Key_F6), panel_,
                   &ControlsPanel::stepRequested);
    run->addAction(QStringLiteral("&Cancel planning"), QKeySequence(Qt::Key_Escape), planning_,
                   &PlanningController::cancel);
    run->addAction(QStringLiteral("&Execute trajectory"), QKeySequence(Qt::Key_F7), this, &MainWindow::execute);

    QMenu* help = menuBar()->addMenu(QStringLiteral("&Help"));
    help->addAction(QStringLiteral("&About"), this, [this] {
        QMessageBox::about(
            this, QStringLiteral("About arm-motion-planner"),
            QStringLiteral("<b>arm-motion-planner</b><br>"
                           "Planar robot arm motion planning and control demo (C++20, Qt 6, Eigen).<br><br>"
                           "Demonstration project only. Not clinical software."));
    });
}

void MainWindow::startPlanning(int iterations) {
    if (planning_->isRunning()) {
        return;
    }
    if (!model_->goalIkOk() || !model_->goalReport().ok()) {
        panel_->setPlanSummary(QStringLiteral("Cannot plan: the goal configuration is invalid."));
        return;
    }
    PlannerParams params = panel_->plannerParams();
    if (iterations > 0) {
        params.maxIterations = iterations;
    }
    const PlannerType type = panel_->plannerType();
    planRevision_ = sceneRevision_;
    liveCost_.clear();
    costPlot_->clear();
    panel_->setPlanning(true);
    panel_->setPlanSummary(QStringLiteral("Planning with %1...").arg(QString::fromLatin1(toString(type))));
    planning_->start(type, model_->checker(), model_->current(), model_->goalConfig(), params);
}

void MainWindow::onPlanningFinished() {
    panel_->setPlanning(false);
    const PlanResult& r = planning_->result();
    if (planRevision_ != sceneRevision_) {
        panel_->setPlanSummary(QStringLiteral("Result discarded: the scene changed while planning."));
        return;
    }
    const PlannerType type = panel_->plannerType();
    QString summary = QStringLiteral("%1: %2\n%3 iterations, %4 nodes, %5 ms")
                          .arg(QString::fromLatin1(toString(type)))
                          .arg(QString::fromStdString(r.message))
                          .arg(r.iterations)
                          .arg(static_cast<qulonglong>(r.nodes))
                          .arg(r.planningTimeMs, 0, 'f', 1);
    if (r.success) {
        summary += QStringLiteral("\npath: %1 waypoints, cost %2 rad").arg(r.path.size()).arg(r.pathCost, 0, 'f', 3);
    }
    planSummary_ = summary;
    panel_->setPlanSummary(summary);

    if (type == PlannerType::RrtStar && !r.costHistory.empty()) {
        QVector<QPointF> points;
        for (const auto& [iteration, cost] : r.costHistory) {
            if (!points.isEmpty()) {
                points.append(QPointF(iteration, points.back().y()));  // step plot: cost holds until improved
            }
            points.append(QPointF(iteration, cost));
        }
        points.append(QPointF(r.iterations, r.costHistory.back().second));
        costPlot_->setSeries({PlotSeries{QStringLiteral("best cost"), QColor(0xf5, 0xa6, 0x23), points}});
        plotTabs_->setCurrentWidget(costPlot_);
    } else {
        costPlot_->clear();
        costPlot_->setPlaceholder(
            r.success ? QStringLiteral("%1 returns a single solution (cost %2 rad). Run RRT* to see the cost improve "
                                       "over iterations.")
                            .arg(QString::fromLatin1(toString(type)))
                            .arg(r.pathCost, 0, 'f', 3)
                      : QStringLiteral("No solution."));
    }
    model_->setPlan(r, type);
    processPlan();
    if (autoExecute_ && r.success) {
        autoExecute_ = false;
        execute();
    }
}

void MainWindow::processPlan() {
    const PlanResult* plan = model_->plan();
    if (plan == nullptr || !plan->success) {
        return;
    }
    std::vector<Config> path = plan->path;
    QString detail;
    if (panel_->smoothingEnabled()) {
        const SmoothingResult s = shortcutSmooth(model_->checker(), path, panel_->smoothingParams());
        path = s.path;
        detail = QStringLiteral("\nsmoothed: %1 -> %2 rad, %3 waypoints")
                     .arg(s.costBefore, 0, 'f', 3)
                     .arg(s.costAfter, 0, 'f', 3)
                     .arg(s.path.size());
    }
    const Trajectory trajectory(model_->arm(), path, panel_->motionLimits(model_->arm().dof()));
    detail += QStringLiteral("\ntrajectory: %1 s").arg(trajectory.duration(), 0, 'f', 2);
    panel_->setPlanSummary(planSummary_ + detail);
    model_->setMotion(path, trajectory);
}

bool MainWindow::simulate() {
    const Trajectory* trajectory = model_->trajectory();
    if (trajectory == nullptr) {
        panel_->setTrackingSummary(QStringLiteral("Plan a path first."));
        return false;
    }
    JointDynamics dynamics = JointDynamics::defaultsFor(model_->arm());
    dynamics.gravity = panel_->gravityEnabled();
    const TrackingOptions options = panel_->trackingOptions();
    const TrackingLog log = simulateTracking(model_->arm(), dynamics, *trajectory, options);
    model_->setTrackingLog(log);

    QStringList maxErr;
    QStringList finalErr;
    for (Eigen::Index j = 0; j < log.maxAbsError.size(); ++j) {
        maxErr << QString::number(log.maxAbsError[j] * 1000.0, 'f', 1);
        finalErr << QString::number(std::abs(log.finalError[j]) * 1000.0, 'f', 2);
    }
    // The planner proved the *reference* collision-free; the controlled arm deviates from it by
    // the tracking error. Check every logged actual pose of the uninflated arm, so a poorly tuned
    // controller (error larger than the safety margin) is visible.
    CollisionOptions exact = model_->checker().options();
    exact.safetyMargin = 0.0;
    exact.maxStepDisplacement = 0.001;
    const CollisionChecker exactChecker(model_->arm(), model_->workspace(), exact);
    int collisions = 0;
    for (const TrackingSample& sample : log.samples) {
        collisions += exactChecker.isValid(sample.actual) ? 0 : 1;
    }
    const QString safety = collisions == 0
                               ? QStringLiteral("executed motion: collision-free")
                               : QStringLiteral("WARNING: executed arm collides in %1 logged samples").arg(collisions);
    statusBar()->showMessage(QStringLiteral("Execution: max tracking error %1 mrad; %2")
                                 .arg(log.maxAbsError.maxCoeff() * 1000.0, 0, 'f', 1)
                                 .arg(safety));
    panel_->setTrackingSummary(
        QStringLiteral("max |error| per joint: %1 mrad\nfinal |error|: %2 mrad\nsaturated %3% of steps\n%4")
            .arg(maxErr.join(QStringLiteral(", ")))
            .arg(finalErr.join(QStringLiteral(", ")))
            .arg(100.0 * log.saturatedSteps / std::max(1, log.totalSteps), 0, 'f', 1)
            .arg(safety));
    updateTrackingPlot();
    return true;
}

void MainWindow::execute() {
    if (!simulate()) {
        return;
    }
    plotTabs_->setCurrentWidget(trackingPlot_);
    playbackTime_ = 0.0;
    playbackClock_.start();
    model_->setExecutionTime(0.0);
    playbackTimer_->start();
}

void MainWindow::onPlaybackTick() {
    const TrackingLog* log = model_->trackingLog();
    if (log == nullptr || log->samples.empty()) {
        playbackTimer_->stop();
        return;
    }
    // Advance by the wall-clock time since the last tick times the chosen speed, so changing
    // the speed mid-playback takes effect smoothly.
    playbackTime_ += panel_->playbackSpeed() * static_cast<double>(playbackClock_.restart()) / 1000.0;
    const double end = pauseAt_ >= 0.0 ? std::min(pauseAt_, log->samples.back().time) : log->samples.back().time;
    if (playbackTime_ >= end) {
        playbackTimer_->stop();
        model_->setExecutionTime(end);  // hold the final (or paused) pose
        return;
    }
    model_->setExecutionTime(playbackTime_);
}

bool MainWindow::saveScreenshot(const QString& path) {
    QPixmap shot = grab();
    if (shot.width() > 1800) {  // keep README images a reasonable size on high-DPI screens
        shot = shot.scaledToWidth(1800, Qt::SmoothTransformation);
    }
    return shot.save(path, "PNG");
}

void MainWindow::screenshotAndQuit(const QString& path, int delayMs) {
    QTimer::singleShot(delayMs, this, [this, path] {
        if (!saveScreenshot(path)) {
            qWarning("Could not save screenshot to %s", qPrintable(path));
        }
        QApplication::quit();
    });
}

bool MainWindow::exportTrackingCsv(const QString& path) {
    const TrackingLog* log = model_->trackingLog();
    QFile file(path);
    if (log == nullptr || log->samples.empty() || !file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        return false;
    }
    QTextStream out(&file);
    const auto dof = log->samples.front().reference.size();
    out << "time_s";
    for (const char* name : {"ref", "actual", "error", "torque"}) {
        for (Eigen::Index j = 0; j < dof; ++j) {
            out << ',' << name << '_' << (j + 1);
        }
    }
    out << '\n';
    for (const TrackingSample& s : log->samples) {
        out << s.time;
        for (const Config* v : {&s.reference, &s.actual, &s.error, &s.torque}) {
            for (Eigen::Index j = 0; j < dof; ++j) {
                out << ',' << (*v)[j];
            }
        }
        out << '\n';
    }
    return true;
}

void MainWindow::updateTrackingPlot() {
    const TrackingLog* log = model_->trackingLog();
    if (log == nullptr || log->samples.empty()) {
        trackingPlot_->clear();
        return;
    }
    // One colour per joint (colour-blind-friendly qualitative palette).
    static const QColor kJointColors[] = {QColor(0x4f, 0x9d, 0xde), QColor(0xf5, 0xa6, 0x23), QColor(0x3e, 0xcf, 0x8e),
                                          QColor(0xe5, 0x53, 0x4b), QColor(0xa0, 0x7c, 0xd8), QColor(0xd9, 0xd9, 0xd9)};
    QVector<PlotSeries> series;
    const auto dof = log->samples.front().error.size();
    for (Eigen::Index j = 0; j < dof; ++j) {
        PlotSeries s{QStringLiteral("joint %1").arg(j + 1), kJointColors[j % 6], {}};
        for (const TrackingSample& sample : log->samples) {
            s.points.append(QPointF(sample.time, sample.error[j] * 1000.0));
        }
        series.append(s);
    }
    trackingPlot_->setSeries(series);
}

void MainWindow::planWith(PlannerType type) {
    panel_->setPlannerType(type);
    stepIterations_ = 0;
    startPlanning(0);
}

void MainWindow::loadScenarioFile(const QString& path) {
    if (path.isEmpty()) {
        model_->loadDemoScene();
        return;
    }
    try {
        model_->loadScenario(amp::loadScenario(path.toStdString()));
        panel_->selectScenario(path);
    } catch (const ScenarioError& e) {
        QMessageBox::warning(this, QStringLiteral("Cannot load scenario"), QString::fromStdString(e.what()));
    }
}

void MainWindow::updateReadout() {
    const ArmModel& arm = model_->arm();
    const Config& q = model_->current();
    QStringList angles;
    for (int i = 0; i < q.size(); ++i) {
        angles << QString::number(q[i] * 180.0 / kPi, 'f', 1);
    }
    const Vec2 ee = endEffectorPosition(arm, q);
    const double w = manipulability(arm, q);  // 0 at singular poses (arm stretched or folded)
    const CollisionReport report = model_->currentReport();
    QString validity = QStringLiteral("valid");
    if (!report.ok()) {
        validity = QStringLiteral("INVALID: %1").arg(QString::fromLatin1(toString(report.kind)));
        if (report.kind == CollisionKind::ObstacleContact) {
            validity += QStringLiteral(" (link %1, obstacle %2)").arg(report.link + 1).arg(report.other + 1);
        } else if (report.kind == CollisionKind::Self) {
            validity += QStringLiteral(" (links %1 and %2)").arg(report.link + 1).arg(report.other + 1);
        } else if (report.link >= 0) {
            validity += QStringLiteral(" (link %1)").arg(report.link + 1);
        }
    }
    readout_->setText(QStringLiteral("q = [%1]°   end effector = (%2, %3) m   manipulability = %4   %5")
                          .arg(angles.join(QStringLiteral(", ")))
                          .arg(ee.x(), 0, 'f', 3)
                          .arg(ee.y(), 0, 'f', 3)
                          .arg(w, 0, 'f', 3)
                          .arg(validity));
}

}  // namespace amp::gui
