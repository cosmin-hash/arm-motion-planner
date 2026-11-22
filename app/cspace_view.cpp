/// @file cspace_view.cpp
/// @brief QPainter rendering and mouse interaction for the 2-DOF C-space view.
#include "cspace_view.hpp"

#include "amp/cspace_grid.hpp"
#include "amp/kinematics.hpp"

#include <QMouseEvent>
#include <QPainter>

#include <algorithm>
#include <cmath>

#include "scene_model.hpp"
#include "view_style.hpp"

namespace amp::gui {

namespace {

constexpr double kLeftMargin = 52.0;
constexpr double kBottomMargin = 56.0;
constexpr double kTopMargin = 34.0;
constexpr double kRightMargin = 14.0;
constexpr std::size_t kMaxTrace = 4000;
// 200 x 200 = 40k collision checks: a few milliseconds, fast enough to redo while dragging
// an obstacle. (Planning, which is much heavier, runs on a worker thread.)
constexpr int kGridResolution = 200;

QColor colorFor(CollisionKind kind) {
    switch (kind) {
        case CollisionKind::ObstacleContact:
            return QColor(style::kObstacleFill.red(), style::kObstacleFill.green(), style::kObstacleFill.blue(), 200);
        case CollisionKind::Boundary:
            return QColor(style::kBoundary.red(), style::kBoundary.green(), style::kBoundary.blue(), 150);
        case CollisionKind::Self:
            return QColor(style::kSelfCollision.red(), style::kSelfCollision.green(), style::kSelfCollision.blue(),
                          170);
        case CollisionKind::JointLimit:  // drawn separately as dark bands
        case CollisionKind::None:
            break;
    }
    return Qt::transparent;
}

}  // namespace

CSpaceView::CSpaceView(SceneModel* model, QWidget* parent) : QWidget(parent), model_(model) {
    connect(model_, &SceneModel::currentChanged, this, [this] {
        const Config& q = model_->current();
        if (trace_.empty() || (trace_.back() - q).norm() > 1e-4) {
            trace_.push_back(q);
            if (trace_.size() > kMaxTrace) {
                trace_.erase(trace_.begin(), trace_.begin() + static_cast<std::ptrdiff_t>(kMaxTrace / 4));
            }
        }
        update();
    });
    connect(model_, &SceneModel::goalChanged, this, qOverload<>(&QWidget::update));
    connect(model_, &SceneModel::armChanged, this, qOverload<>(&QWidget::update));
    connect(model_, &SceneModel::planChanged, this, qOverload<>(&QWidget::update));
    connect(model_, &SceneModel::motionChanged, this, qOverload<>(&QWidget::update));
    connect(model_, &SceneModel::liveTreeChanged, this, qOverload<>(&QWidget::update));
    connect(model_, &SceneModel::executionChanged, this, qOverload<>(&QWidget::update));
    connect(model_, &SceneModel::obstaclesChanged, this, [this] {
        recomputeOccupancy();
        update();
    });
    recomputeOccupancy();
    connect(model_, &SceneModel::traceCleared, this, [this] {
        trace_.clear();
        update();
    });
}

QRectF CSpaceView::plotRect() const {
    const double side =
        std::max(10.0, std::min(width() - kLeftMargin - kRightMargin, height() - kTopMargin - kBottomMargin));
    const double x = kLeftMargin + (width() - kLeftMargin - kRightMargin - side) / 2.0;
    return {x, kTopMargin, side, side};
}

QPointF CSpaceView::toScreen(double q1, double q2) const {
    const QRectF r = plotRect();
    return {r.left() + (q1 + kPi) / kTwoPi * r.width(), r.bottom() - (q2 + kPi) / kTwoPi * r.height()};
}

Config CSpaceView::toConfig(const QPointF& p) const {
    const QRectF r = plotRect();
    Config q(2);
    q << (p.x() - r.left()) / r.width() * kTwoPi - kPi, (r.bottom() - p.y()) / r.height() * kTwoPi - kPi;
    return q;
}

void CSpaceView::recomputeOccupancy() {
    if (model_->arm().dof() != 2) {
        occupancy_ = QImage();
        return;
    }
    const CSpaceGrid2D grid = computeCSpaceGrid2D(model_->checker(), kGridResolution);
    occupancy_ = QImage(kGridResolution, kGridResolution, QImage::Format_ARGB32);
    for (int j = 0; j < kGridResolution; ++j) {
        // Image rows grow downward; theta2 grows upward.
        auto* row = reinterpret_cast<QRgb*>(occupancy_.scanLine(kGridResolution - 1 - j));
        for (int i = 0; i < kGridResolution; ++i) {
            row[i] = colorFor(grid.at(i, j)).rgba();
        }
    }
}

// ---------------------------------------------------------------------------
// Painting
// ---------------------------------------------------------------------------

void CSpaceView::paintEvent(QPaintEvent* /*event*/) {
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    painter.fillRect(rect(), style::kBackground);

    painter.setFont(style::annotationFont(10));
    painter.setPen(style::kText);
    painter.drawText(QPointF(12, 22), QStringLiteral("Configuration space"));

    if (model_->arm().dof() != 2) {
        drawUnsupported(painter);
        return;
    }

    const QRectF area = plotRect();
    painter.fillRect(area, style::kPanel);
    if (!occupancy_.isNull()) {
        painter.drawImage(area, occupancy_);  // nearest-neighbour scaling keeps cell edges honest
    }
    drawJointLimits(painter, area);
    drawAxes(painter, area);
    drawTrace(painter);
    drawPlan(painter, area);
    drawGoalSolutions(painter);
    drawCurrent(painter);
    drawLegend(painter, area);
}

void CSpaceView::drawAxes(QPainter& painter, const QRectF& area) const {
    // Grid and tick labels at multiples of pi/2.
    static const QString kLabels[] = {QStringLiteral("−π"), QStringLiteral("−π/2"), QStringLiteral("0"),
                                      QStringLiteral("π/2"), QStringLiteral("π")};
    painter.setFont(style::annotationFont(8));
    for (int k = 0; k <= 4; ++k) {
        const double angle = -kPi + k * kPi / 2.0;
        const QPointF px = toScreen(angle, -kPi);
        const QPointF py = toScreen(-kPi, angle);
        painter.setPen(QPen(k == 2 ? style::kAxis : style::kGridMajor, 1));
        painter.drawLine(QPointF(px.x(), area.top()), QPointF(px.x(), area.bottom()));
        painter.drawLine(QPointF(area.left(), py.y()), QPointF(area.right(), py.y()));

        painter.setPen(style::kTextDim);
        painter.drawText(QRectF(px.x() - 24, area.bottom() + 3, 48, 14), Qt::AlignCenter, kLabels[k]);
        painter.drawText(QRectF(area.left() - 44, py.y() - 7, 40, 14), Qt::AlignRight | Qt::AlignVCenter, kLabels[k]);
    }

    painter.setPen(QPen(style::kAxis, 1));
    painter.setBrush(Qt::NoBrush);
    painter.drawRect(area);

    painter.setFont(style::annotationFont(9));
    painter.setPen(style::kText);
    painter.drawText(QRectF(area.left(), area.bottom() + 18, area.width(), 16), Qt::AlignCenter,
                     QStringLiteral("θ₁ (shoulder) [rad]"));
    painter.save();
    painter.translate(14, area.center().y());
    painter.rotate(-90);
    painter.drawText(QRectF(-area.height() / 2, -8, area.height(), 16), Qt::AlignCenter,
                     QStringLiteral("θ₂ (elbow) [rad]"));
    painter.restore();
}

void CSpaceView::drawJointLimits(QPainter& painter, const QRectF& area) const {
    // Configurations outside the joint limits are unreachable just like obstacles are,
    // so they are shaded the same way the C-obstacles will be.
    const ArmModel& arm = model_->arm();
    painter.setPen(Qt::NoPen);
    painter.setBrush(style::kForbidden);
    const JointLimits& l1 = arm.limits(0);
    const JointLimits& l2 = arm.limits(1);
    if (!l1.continuous) {
        painter.drawRect(QRectF(area.topLeft(), QPointF(toScreen(l1.lower, 0).x(), area.bottom())));
        painter.drawRect(QRectF(QPointF(toScreen(l1.upper, 0).x(), area.top()), area.bottomRight()));
    }
    if (!l2.continuous) {
        painter.drawRect(QRectF(QPointF(area.left(), toScreen(0, l2.lower).y()), area.bottomRight()));
        painter.drawRect(QRectF(area.topLeft(), QPointF(area.right(), toScreen(0, l2.upper).y())));
    }
}

void CSpaceView::drawTrace(QPainter& painter) const {
    if (trace_.size() < 2) {
        return;
    }
    painter.setPen(QPen(style::kTrace, 1.5));
    for (std::size_t i = 1; i < trace_.size(); ++i) {
        const Config& a = trace_[i - 1];
        const Config& b = trace_[i];
        if (a.size() != 2 || b.size() != 2) {
            continue;
        }
        painter.drawLine(toScreen(a[0], a[1]), toScreen(b[0], b[1]));
    }
}

void CSpaceView::drawGoalSolutions(QPainter& painter) const {
    // One workspace point has (generically) two IK solutions for a 2-link arm: one goal in the
    // workspace is TWO separate goals in C-space. Showing both makes that concrete.
    const ArmModel& arm = model_->arm();
    painter.setFont(style::annotationFont(8));
    for (ElbowConfig elbow : {ElbowConfig::Up, ElbowConfig::Down}) {
        const IkResult r = solveIkAnalytic2Link(arm, model_->goalTarget(), elbow);
        if (r.status == IkStatus::Unreachable || r.status == IkStatus::InvalidInput) {
            continue;
        }
        const QPointF p = toScreen(r.q[0], r.q[1]);
        // Green only if the solution is both within limits and collision-free.
        const QColor c = r.ok() && model_->checker().isValid(r.q) ? style::kGoal : style::kInvalid;
        painter.setPen(QPen(c, 1.5));
        painter.setBrush(Qt::NoBrush);
        painter.drawEllipse(p, 7, 7);
        const QString label =
            elbow == ElbowConfig::Up ? QStringLiteral("goal (elbow up)") : QStringLiteral("goal (elbow down)");
        // Put the label on whichever side of the marker has room, so it is never clipped.
        const double textWidth = painter.fontMetrics().horizontalAdvance(label);
        const bool fitsRight = p.x() + 10 + textWidth < plotRect().right();
        painter.drawText(p + QPointF(fitsRight ? 10.0 : -10.0 - textWidth, 4), label);
    }
    if (model_->goalIkOk()) {
        const Config& g = model_->goalConfig();
        painter.setPen(Qt::NoPen);
        painter.setBrush(style::kGoal);
        painter.drawEllipse(toScreen(g[0], g[1]), 4, 4);
    }
}

void CSpaceView::drawPlan(QPainter& painter, const QRectF& area) const {
    const PlanResult* plan = model_->plan();
    if (model_->arm().dof() != 2) {
        return;
    }
    // While planning there is no result yet, only the growing tree.
    const std::vector<TreeNode>& tree = plan != nullptr ? plan->tree : model_->liveTree();
    // A segment that jumps by more than pi on an axis wraps around a continuous joint; drawing it
    // as a straight line across the whole plot would be misleading, so such segments are skipped.
    const auto wraps = [](const Config& a, const Config& b) {
        return std::abs(a[0] - b[0]) > kPi || std::abs(a[1] - b[1]) > kPi;
    };
    painter.save();
    painter.setClipRect(area);
    painter.setPen(QPen(style::kTree, 1.0));
    for (const TreeNode& node : tree) {
        if (node.parent >= 0) {
            const Config& p = tree[static_cast<std::size_t>(node.parent)].q;
            if (!wraps(p, node.q)) {
                painter.drawLine(toScreen(p[0], p[1]), toScreen(node.q[0], node.q[1]));
            }
        }
    }
    const auto drawPath = [&](const std::vector<Config>& path) {
        for (std::size_t i = 1; i < path.size(); ++i) {
            if (!wraps(path[i - 1], path[i])) {
                painter.drawLine(toScreen(path[i - 1][0], path[i - 1][1]), toScreen(path[i][0], path[i][1]));
            }
        }
    };
    if (plan != nullptr && plan->success) {
        const bool smoothed = !model_->motionPath().empty() && model_->motionPath().size() != plan->path.size();
        painter.setPen(QPen(smoothed ? style::kRawPath : style::kPath, smoothed ? 1.5 : 2.5, Qt::SolidLine,
                            Qt::RoundCap, Qt::RoundJoin));
        drawPath(plan->path);
        if (smoothed) {
            painter.setPen(QPen(style::kPath, 2.5, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
            drawPath(model_->motionPath());
        }
    }
    // Playback: executed joint trajectory so far, and the current actual configuration.
    if (const auto sample = model_->executionSample()) {
        QPolygonF trail;
        for (const TrackingSample& s : model_->trackingLog()->samples) {
            if (s.time > sample->time) {
                break;
            }
            trail << toScreen(s.actual[0], s.actual[1]);
        }
        painter.setPen(QPen(style::kEndEffector, 1.5));
        painter.drawPolyline(trail);
        painter.setBrush(style::kEndEffector);
        painter.drawEllipse(toScreen(sample->actual[0], sample->actual[1]), 5, 5);
    }
    painter.restore();
}

void CSpaceView::drawLegend(QPainter& painter, const QRectF& area) const {
    struct Entry {
        QColor color;
        QString label;
    };
    const Entry entries[] = {
        {colorFor(CollisionKind::ObstacleContact), QStringLiteral("obstacle")},
        {colorFor(CollisionKind::Boundary), QStringLiteral("boundary")},
        {colorFor(CollisionKind::Self), QStringLiteral("self-collision")},
        {style::kForbidden, QStringLiteral("joint limit")},
    };
    painter.setFont(style::annotationFont(8));
    double x = area.left();
    const double y = area.bottom() + 36;
    for (const Entry& e : entries) {
        painter.setPen(QPen(style::kAxis, 1));
        painter.setBrush(e.color);
        painter.drawRect(QRectF(x, y, 10, 10));
        painter.setPen(style::kTextDim);
        painter.drawText(QPointF(x + 14, y + 9), e.label);
        x += 22 + painter.fontMetrics().horizontalAdvance(e.label);
    }
}

void CSpaceView::drawCurrent(QPainter& painter) const {
    const Config& q = model_->current();
    const QPointF p = toScreen(q[0], q[1]);
    painter.setPen(QPen(style::kEndEffector.darker(180), 2));
    painter.setBrush(model_->currentReport().ok() ? style::kEndEffector : style::kInvalid);
    painter.drawEllipse(p, 6, 6);
}

void CSpaceView::drawUnsupported(QPainter& painter) const {
    painter.setFont(style::annotationFont(10));
    painter.setPen(style::kTextDim);
    painter.drawText(rect().adjusted(24, 40, -24, -24), Qt::AlignCenter | Qt::TextWordWrap,
                     QStringLiteral("The C-space view is drawn for 2-DOF arms only.\n\n"
                                    "This arm has %1 joints, so its configuration space is %1-dimensional "
                                    "and cannot be shown as a flat image.\nSet DOF to 2 to explore joint space.")
                         .arg(model_->arm().dof()));
}

// ---------------------------------------------------------------------------
// Mouse interaction: click or drag to set the configuration directly.
// ---------------------------------------------------------------------------

void CSpaceView::mousePressEvent(QMouseEvent* event) {
    if (event->button() == Qt::LeftButton && model_->arm().dof() == 2 && plotRect().contains(event->position())) {
        dragging_ = true;
        model_->setCurrent(toConfig(event->position()));
    }
}

void CSpaceView::mouseMoveEvent(QMouseEvent* event) {
    if (dragging_) {
        model_->setCurrent(toConfig(event->position()));
    }
}

void CSpaceView::mouseReleaseEvent(QMouseEvent* event) {
    if (event->button() == Qt::LeftButton) {
        dragging_ = false;
    }
}

}  // namespace amp::gui
