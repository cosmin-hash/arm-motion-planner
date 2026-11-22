/// @file main_window.hpp
/// @brief Top-level window: workspace view, C-space view, plots, controls dock, status bar.
#pragma once

#include "amp/planner.hpp"

#include <QElapsedTimer>
#include <QMainWindow>
#include <QPointF>
#include <QVector>

class QLabel;
class QTimer;
class QTabWidget;

namespace amp::gui {

class SceneModel;
class PlotWidget;
class ControlsPanel;
class PlanningController;

/// Assembles the views around one SceneModel.
///
/// Layout: controls dock on the left; workspace view in the centre; C-space view above
/// the plots on the right.
class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    /// @param initialDof Number of joints of the arm shown at start-up (2..6).
    explicit MainWindow(int initialDof = 3, QWidget* parent = nullptr);

    /// Load a scenario file, or the built-in demo scene for an empty path.
    void loadScenarioFile(const QString& path);
    /// Select `type` in the controls and start planning (used by the --plan option).
    void planWith(amp::PlannerType type);
    /// Execute the trajectory automatically when the next plan finishes (--execute option).
    void setAutoExecute(bool on) { autoExecute_ = on; }
    /// The controls panel (for command-line options that preset controls).
    [[nodiscard]] ControlsPanel* controls() const { return panel_; }
    /// Freeze playback at time t (s) instead of playing to the end (--pause-at option).
    void setPauseAt(double t) { pauseAt_ = t; }
    /// Save a screenshot of the window to `path` after `delayMs`, then quit (--screenshot option).
    void screenshotAndQuit(const QString& path, int delayMs);
    /// Save a PNG screenshot of the main window (scaled to at most 1800 px wide).
    bool saveScreenshot(const QString& path);

private:
    void buildMenus();
    /// Refresh the permanent status-bar readout (joint angles, end effector, manipulability).
    void updateReadout();
    /// Start a background planning job with the panel's parameters, overriding the iteration
    /// budget if `iterations` > 0 (used by Step).
    void startPlanning(int iterations);
    /// Planning job finished: store the result (unless the scene changed meanwhile) and plot it.
    void onPlanningFinished();
    /// Smooth and time-parameterize the stored plan with the panel's settings.
    void processPlan();
    /// Run the closed-loop tracking simulation for the current trajectory and plot the error.
    /// Returns false if there is nothing to simulate.
    bool simulate();
    /// Simulate and start real-time playback of the controlled arm.
    void execute();
    /// Advance playback (timer tick).
    void onPlaybackTick();
    /// Fill the tracking-error plot from the model's tracking log.
    void updateTrackingPlot();
    /// Write the tracking log as CSV (time, then reference/actual/error/torque per joint).
    bool exportTrackingCsv(const QString& path);

    SceneModel* model_;
    PlotWidget* costPlot_ = nullptr;
    PlotWidget* trackingPlot_ = nullptr;
    QLabel* readout_ = nullptr;
    QTabWidget* plotTabs_ = nullptr;
    ControlsPanel* panel_ = nullptr;
    PlanningController* planning_ = nullptr;
    int stepIterations_ = 0;      ///< Budget of the last Step (0 = Step not used yet).
    unsigned sceneRevision_ = 0;  ///< Bumped on every scene change, to detect stale results.
    unsigned planRevision_ = 0;   ///< sceneRevision_ when the running job started.
    QVector<QPointF> liveCost_;   ///< RRT* best cost reported during planning.
    QString planSummary_;         ///< Planner part of the panel summary.
    QTimer* playbackTimer_ = nullptr;
    QElapsedTimer playbackClock_;  ///< Wall-clock time since the previous playback tick.
    double playbackTime_ = 0.0;    ///< Simulated time shown (advances at the chosen speed).
    double pauseAt_ = -1.0;        ///< Freeze playback here (--pause-at); negative: play to the end.
    bool autoExecute_ = false;     ///< Execute as soon as the next plan succeeds (--execute).
};

}  // namespace amp::gui
