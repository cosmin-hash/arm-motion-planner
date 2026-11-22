/// @file workspace_view.hpp
/// @brief Top-down view of the arm in the 2D workspace (metres, y up).
#pragma once

#include "amp/simulation.hpp"
#include "amp/types.hpp"

#include <QWidget>

#include <vector>

namespace amp::gui {

class SceneModel;

/// Paints the workspace bounds, obstacles, arm, goal, reach circle and end-effector trace.
/// The user can drag the end effector (driving IK), the goal marker, and obstacles.
/// Links in collision are drawn red.
class WorkspaceView : public QWidget {
    Q_OBJECT

public:
    /// Create the view; it observes `model` and repaints on its signals.
    explicit WorkspaceView(SceneModel* model, QWidget* parent = nullptr);

    [[nodiscard]] QSize sizeHint() const override { return {720, 720}; }
    [[nodiscard]] QSize minimumSizeHint() const override { return {320, 320}; }

protected:
    void paintEvent(QPaintEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;

private:
    enum class DragTarget { None, EndEffector, Goal, Obstacle };

    /// Result of picking at a screen position.
    struct Hit {
        DragTarget target = DragTarget::None;
        int obstacle = -1;  ///< Obstacle index when target == Obstacle.
    };

    /// Pixels per metre for the current widget size (fits the arm's reach with a margin).
    [[nodiscard]] double pixelsPerMetre() const;
    /// World (metres, y up, origin at the arm base) to widget pixels.
    [[nodiscard]] QPointF toScreen(const Vec2& p) const;
    /// Widget pixels to world coordinates.
    [[nodiscard]] Vec2 toWorld(const QPointF& p) const;
    /// What, if anything, is under the mouse at `pos`.
    [[nodiscard]] Hit hitTest(const QPointF& pos) const;

    void drawGrid(QPainter& painter) const;
    void drawBounds(QPainter& painter) const;
    void drawObstacles(QPainter& painter) const;
    void drawReach(QPainter& painter) const;
    void drawTrace(QPainter& painter) const;
    void drawArm(QPainter& painter, const Config& q, bool ghost) const;
    void drawGoal(QPainter& painter) const;
    void drawOverlay(QPainter& painter) const;
    /// Planned path: end-effector curve plus faint arm poses along it.
    void drawPlan(QPainter& painter) const;
    /// Playback: reference skeleton, controlled arm and executed end-effector trail.
    void drawExecution(QPainter& painter, const TrackingSample& sample) const;

    SceneModel* model_;
    Hit drag_;
    Vec2 lastDragWorld_ = Vec2::Zero();  ///< Previous mouse position while dragging an obstacle.
    std::vector<Vec2> trace_;            ///< Recent end-effector positions (world coordinates).
};

}  // namespace amp::gui
