/// @file planning_controller.cpp
/// @brief Background planning with progress reporting and cancellation.
#include "planning_controller.hpp"

#include <QMetaObject>
#include <QPointer>
#include <QThread>

#include <chrono>
#include <utility>

namespace amp::gui {

PlanningController::PlanningController(QObject* parent) : QObject(parent) {}

PlanningController::~PlanningController() {
    // Never leave a thread running past the controller's lifetime.
    if (thread_ != nullptr) {
        cancel();
        thread_->wait();
    }
}

void PlanningController::start(PlannerType type, const CollisionChecker& checker, const Config& start,
                               const Config& goal, const PlannerParams& params) {
    if (thread_ != nullptr) {
        return;
    }
    cancel_ = std::make_shared<std::atomic<bool>>(false);

    // Everything the worker uses is captured by value: it shares no mutable state with the GUI.
    auto cancelFlag = cancel_;
    QPointer<PlanningController> self(this);
    thread_ = QThread::create([self, cancelFlag, type, checker, start, goal, params]() {
        const auto planner = makePlanner(type, checker);
        PlannerCallbacks callbacks;
        callbacks.cancel = cancelFlag.get();
        callbacks.progressInterval = 200;
        callbacks.onProgress = [self](const PlanProgress& p) {
            if (self) {
                QMetaObject::invokeMethod(
                    self.data(),
                    [self, p] {
                        if (self) {
                            emit self->progress(p.iteration, static_cast<int>(p.nodes), p.bestCost);
                        }
                    },
                    Qt::QueuedConnection);
            }
        };
        // Tree snapshots: checked every 100 iterations, but forwarded at most every 100 ms, so
        // copying a large tree never slows planning noticeably.
        callbacks.treeInterval = 100;
        auto lastSnapshot = std::chrono::steady_clock::now();
        callbacks.onTree = [self, &lastSnapshot](const std::vector<TreeNode>& tree) {
            const auto now = std::chrono::steady_clock::now();
            if (now - lastSnapshot < std::chrono::milliseconds(100)) {
                return;
            }
            lastSnapshot = now;
            QMetaObject::invokeMethod(
                self.data(), [self, copy = tree] { emit self->treeSnapshot(copy); }, Qt::QueuedConnection);
        };
        PlanResult result = planner->plan(start, goal, params, callbacks);
        if (self) {
            QMetaObject::invokeMethod(
                self.data(),
                [self, r = std::move(result)]() mutable {
                    if (!self) {
                        return;
                    }
                    self->result_ = std::move(r);
                    self->thread_->wait();  // the lambda has returned; the thread is finishing
                    self->thread_->deleteLater();
                    self->thread_ = nullptr;
                    emit self->finished();
                },
                Qt::QueuedConnection);
        }
    });
    thread_->start();
}

void PlanningController::cancel() {
    if (cancel_) {
        cancel_->store(true);
    }
}

}  // namespace amp::gui
