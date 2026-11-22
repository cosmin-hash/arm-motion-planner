/// @file planning_controller.hpp
/// @brief Runs a planner on a worker thread so the UI never freezes.
///
/// Threading model
/// ---------------
/// start() copies everything the planner needs (collision checker, start, goal, params) and
/// runs the planner on a QThread. The planner never touches GUI objects or SceneModel, so no
/// locking is needed. Progress and the final result are marshalled back to the GUI thread with
/// queued QMetaObject::invokeMethod calls. cancel() sets an atomic flag that the planner polls
/// once per iteration.
#pragma once

#include "amp/collision.hpp"
#include "amp/planner.hpp"

#include <QObject>

#include <atomic>
#include <memory>
#include <vector>

class QThread;

namespace amp::gui {

/// Owns at most one background planning job at a time.
class PlanningController : public QObject {
    Q_OBJECT

public:
    /// Create an idle controller.
    explicit PlanningController(QObject* parent = nullptr);
    /// Cancels and joins a running job.
    ~PlanningController() override;

    /// True while a job is running.
    [[nodiscard]] bool isRunning() const { return thread_ != nullptr; }
    /// The result of the last completed job.
    [[nodiscard]] const PlanResult& result() const { return result_; }

public slots:
    /// Start planning. Ignored if a job is already running.
    void start(amp::PlannerType type, const amp::CollisionChecker& checker, const amp::Config& start,
               const amp::Config& goal, const amp::PlannerParams& params);
    /// Ask the running job to stop at its next iteration.
    void cancel();

signals:
    /// Periodic progress from the running job (delivered on the GUI thread).
    void progress(int iteration, int nodes, double bestCost);
    /// A copy of the growing search tree, at most about 10 times per second (GUI thread).
    void treeSnapshot(const std::vector<amp::TreeNode>& tree);
    /// The job finished; read result(). Delivered on the GUI thread.
    void finished();

private:
    QThread* thread_ = nullptr;
    std::shared_ptr<std::atomic<bool>> cancel_;
    PlanResult result_;
};

}  // namespace amp::gui
