/// @file cspace_view.hpp
/// @brief Joint-space (configuration space) view for 2-DOF arms.
///
/// Every point in this square is one complete pose of the arm. Planning here turns
/// "move a jointed arm among obstacles" into "move a point among (strangely shaped) regions",
/// which is the central idea behind sampling-based motion planning.
#pragma once

#include "amp/types.hpp"

#include <QImage>
#include <QWidget>

#include <vector>

namespace amp::gui {

class SceneModel;

/// Paints the (theta1, theta2) square: the C-obstacle region (sampled from the collision
/// checker), joint-limit bands, current and goal configurations, both analytic IK solutions of
/// the goal, and the configuration trace. Clicking or dragging sets the configuration directly.
class CSpaceView : public QWidget {
    Q_OBJECT

public:
    /// Create the view; it observes `model` and repaints on its signals.
    explicit CSpaceView(SceneModel* model, QWidget* parent = nullptr);

    [[nodiscard]] QSize sizeHint() const override { return {420, 420}; }
    [[nodiscard]] QSize minimumSizeHint() const override { return {240, 240}; }

protected:
    void paintEvent(QPaintEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;

private:
    /// Square plotting area inside the widget (leaves room for axis labels).
    [[nodiscard]] QRectF plotRect() const;
    /// Joint angles (theta1, theta2) to widget pixels.
    [[nodiscard]] QPointF toScreen(double q1, double q2) const;
    /// Widget pixels to joint angles (theta1, theta2).
    [[nodiscard]] Config toConfig(const QPointF& p) const;

    void drawAxes(QPainter& painter, const QRectF& area) const;
    void drawJointLimits(QPainter& painter, const QRectF& area) const;
    void drawTrace(QPainter& painter) const;
    void drawGoalSolutions(QPainter& painter) const;
    void drawCurrent(QPainter& painter) const;
    void drawLegend(QPainter& painter, const QRectF& area) const;
    /// Search tree edges and the planned path.
    void drawPlan(QPainter& painter, const QRectF& area) const;
    /// Re-sample the C-obstacle region into occupancy_ (2-DOF only).
    void recomputeOccupancy();
    void drawUnsupported(QPainter& painter) const;

    SceneModel* model_;
    bool dragging_ = false;
    std::vector<Config> trace_;  ///< Recent configurations, for the C-space trace.
    QImage occupancy_;           ///< One pixel per C-space grid cell; transparent where free.
};

}  // namespace amp::gui
