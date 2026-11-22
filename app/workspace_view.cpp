/// @file workspace_view.cpp
/// @brief QPainter rendering and mouse interaction for the workspace view.
#include "workspace_view.hpp"

#include "amp/collision.hpp"
#include "amp/cspace.hpp"
#include "amp/kinematics.hpp"

#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>

#include <algorithm>
#include <cmath>
#include <type_traits>
#include <variant>

#include "scene_model.hpp"
#include "view_style.hpp"

namespace amp::gui {

namespace {

constexpr double kViewMargin = 1.2;      // show 20% beyond the arm's reach
constexpr double kGridStep = 0.1;        // metres between minor grid lines
constexpr int kMajorEvery = 5;           // every 5th line is a major line
constexpr double kHitRadiusPx = 14.0;    // grab distance for draggable handles
constexpr std::size_t kMaxTrace = 4000;  // bound memory for long sessions

}  // namespace

WorkspaceView::WorkspaceView(SceneModel* model, QWidget* parent) : QWidget(parent), model_(model) {
    setMouseTracking(true);  // needed for hover cursor feedback without a button pressed
    setAutoFillBackground(false);

    connect(model_, &SceneModel::currentChanged, this, [this] {
        const Vec2 ee = endEffectorPosition(model_->arm(), model_->current());
        if (trace_.empty() || (trace_.back() - ee).norm() > 1e-4) {
            trace_.push_back(ee);
            if (trace_.size() > kMaxTrace) {
                trace_.erase(trace_.begin(), trace_.begin() + static_cast<std::ptrdiff_t>(kMaxTrace / 4));
            }
        }
        update();
    });
    connect(model_, &SceneModel::goalChanged, this, qOverload<>(&QWidget::update));
    connect(model_, &SceneModel::armChanged, this, qOverload<>(&QWidget::update));
    connect(model_, &SceneModel::obstaclesChanged, this, qOverload<>(&QWidget::update));
    connect(model_, &SceneModel::planChanged, this, qOverload<>(&QWidget::update));
    connect(model_, &SceneModel::motionChanged, this, qOverload<>(&QWidget::update));
    connect(model_, &SceneModel::executionChanged, this, qOverload<>(&QWidget::update));
    connect(model_, &SceneModel::traceCleared, this, [this] {
        trace_.clear();
        update();
    });
}

double WorkspaceView::pixelsPerMetre() const {
    const double extent = model_->arm().reach() * kViewMargin;
    return std::min(width(), height()) / (2.0 * extent);
}

QPointF WorkspaceView::toScreen(const Vec2& p) const {
    const double s = pixelsPerMetre();
    const Vec2 d = p - model_->arm().base();
    // Screen y grows downward, world y grows upward: flip.
    return {width() / 2.0 + d.x() * s, height() / 2.0 - d.y() * s};
}

Vec2 WorkspaceView::toWorld(const QPointF& p) const {
    const double s = pixelsPerMetre();
    return model_->arm().base() + Vec2((p.x() - width() / 2.0) / s, (height() / 2.0 - p.y()) / s);
}

WorkspaceView::Hit WorkspaceView::hitTest(const QPointF& pos) const {
    const QPointF ee = toScreen(endEffectorPosition(model_->arm(), model_->current()));
    const QPointF goal = toScreen(model_->goalTarget());
    const auto dist = [&](const QPointF& a) { return std::hypot(a.x() - pos.x(), a.y() - pos.y()); };
    // Priority: end effector, then goal, then obstacles (the small handles win over big shapes).
    if (dist(ee) <= kHitRadiusPx) {
        return {DragTarget::EndEffector, -1};
    }
    if (dist(goal) <= kHitRadiusPx) {
        return {DragTarget::Goal, -1};
    }
    const Vec2 world = toWorld(pos);
    const auto& obstacles = model_->workspace().obstacles;
    for (int i = static_cast<int>(obstacles.size()) - 1; i >= 0; --i) {  // topmost (last drawn) first
        if (obstacleContains(obstacles[static_cast<std::size_t>(i)], world)) {
            return {DragTarget::Obstacle, i};
        }
    }
    return {};
}

// ---------------------------------------------------------------------------
// Painting
// ---------------------------------------------------------------------------

void WorkspaceView::paintEvent(QPaintEvent* /*event*/) {
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    painter.fillRect(rect(), style::kBackground);

    drawGrid(painter);
    drawBounds(painter);
    drawReach(painter);
    drawObstacles(painter);
    drawTrace(painter);
    drawPlan(painter);
    if (model_->goalIkOk()) {
        drawArm(painter, model_->goalConfig(), /*ghost=*/true);
    }
    drawGoal(painter);
    if (const auto sample = model_->executionSample()) {
        drawExecution(painter, *sample);  // playback: controlled arm instead of the current pose
    } else {
        drawArm(painter, model_->current(), /*ghost=*/false);
    }
    drawOverlay(painter);
}

void WorkspaceView::drawGrid(QPainter& painter) const {
    const Vec2 topLeft = toWorld(QPointF(0, 0));
    const Vec2 bottomRight = toWorld(QPointF(width(), height()));
    const auto first = [](double v) { return static_cast<int>(std::floor(v / kGridStep)); };
    const auto last = [](double v) { return static_cast<int>(std::ceil(v / kGridStep)); };

    for (int i = first(topLeft.x()); i <= last(bottomRight.x()); ++i) {
        painter.setPen(QPen(i == 0 ? style::kAxis : (i % kMajorEvery == 0 ? style::kGridMajor : style::kGridMinor), 1));
        const double x = toScreen(Vec2(i * kGridStep, 0)).x();
        painter.drawLine(QPointF(x, 0), QPointF(x, height()));
    }
    for (int j = first(bottomRight.y()); j <= last(topLeft.y()); ++j) {
        painter.setPen(QPen(j == 0 ? style::kAxis : (j % kMajorEvery == 0 ? style::kGridMajor : style::kGridMinor), 1));
        const double y = toScreen(Vec2(0, j * kGridStep)).y();
        painter.drawLine(QPointF(0, y), QPointF(width(), y));
    }
}

void WorkspaceView::drawBounds(QPainter& painter) const {
    const auto& bounds = model_->workspace().bounds;
    if (!bounds) {
        return;
    }
    // Shade everything outside the allowed rectangle, then outline it.
    const QRectF inside(toScreen(Vec2(bounds->min.x(), bounds->max.y())),
                        toScreen(Vec2(bounds->max.x(), bounds->min.y())));
    QPainterPath outside;
    outside.addRect(QRectF(rect()));
    QPainterPath hole;
    hole.addRect(inside);
    painter.setPen(Qt::NoPen);
    painter.setBrush(QBrush(style::kGridMajor, Qt::BDiagPattern));
    painter.drawPath(outside.subtracted(hole));
    painter.setPen(QPen(style::kBoundary, 1.5));
    painter.setBrush(Qt::NoBrush);
    painter.drawRect(inside);
}

void WorkspaceView::drawObstacles(QPainter& painter) const {
    const CollisionReport current = model_->currentReport();
    const auto& obstacles = model_->workspace().obstacles;
    for (std::size_t i = 0; i < obstacles.size(); ++i) {
        const bool hit = current.kind == CollisionKind::ObstacleContact && current.other == static_cast<int>(i);
        const bool dragged = drag_.target == DragTarget::Obstacle && drag_.obstacle == static_cast<int>(i);
        painter.setPen(QPen(hit ? style::kInvalid : style::kObstacleEdge, dragged ? 2.5 : 1.5));
        painter.setBrush(style::kObstacleFill);
        std::visit(
            [&](const auto& shape) {
                using T = std::decay_t<decltype(shape)>;
                if constexpr (std::is_same_v<T, Circle>) {
                    const double r = shape.radius * pixelsPerMetre();
                    painter.drawEllipse(toScreen(shape.center), r, r);
                } else {
                    QPolygonF poly;
                    for (const Vec2& v : shape.vertices()) {
                        poly << toScreen(v);
                    }
                    painter.drawPolygon(poly);
                }
            },
            obstacles[i]);
    }
}

void WorkspaceView::drawPlan(QPainter& painter) const {
    const PlanResult* plan = model_->plan();
    if (plan == nullptr || !plan->success || plan->path.size() < 2) {
        return;
    }
    const ArmModel& arm = model_->arm();
    // The path that will be executed (smoothed if smoothing is on), else the raw plan.
    const std::vector<Config>& path = model_->motionPath().empty() ? plan->path : model_->motionPath();

    // End-effector curve of a C-space path: densely interpolate each segment (straight in
    // C-space is curved in the workspace).
    const auto eeCurve = [&](const std::vector<Config>& p) {
        QPolygonF curve;
        for (std::size_t i = 1; i < p.size(); ++i) {
            const int samples = std::max(2, static_cast<int>(configDistance(arm, p[i - 1], p[i]) / 0.02));
            for (int k = 0; k <= samples; ++k) {
                curve << toScreen(endEffectorPosition(arm, interpolate(arm, p[i - 1], p[i], double(k) / samples)));
            }
        }
        return curve;
    };

    // Ghost poses evenly spaced along the path (by C-space arc length), hidden during playback.
    if (!model_->executionSample()) {
        constexpr int kGhosts = 9;
        const double total = pathLength(arm, path);
        QColor ghost = style::kPath;
        ghost.setAlpha(45);
        painter.setPen(
            QPen(ghost, std::max(2.0 * arm.linkRadius() * pixelsPerMetre(), 3.0), Qt::SolidLine, Qt::RoundCap));
        for (int g = 1; g < kGhosts; ++g) {
            double target = total * g / kGhosts;
            for (std::size_t i = 1; i < path.size(); ++i) {
                const double seg = configDistance(arm, path[i - 1], path[i]);
                if (target <= seg || i + 1 == path.size()) {
                    const Config q = interpolate(arm, path[i - 1], path[i], seg > 0.0 ? target / seg : 0.0);
                    const std::vector<Vec2> pts = forwardKinematics(arm, q);
                    for (std::size_t k = 0; k + 1 < pts.size(); ++k) {
                        painter.drawLine(toScreen(pts[k]), toScreen(pts[k + 1]));
                    }
                    break;
                }
                target -= seg;
            }
        }
    }

    painter.setBrush(Qt::NoBrush);
    if (&path != &plan->path) {
        painter.setPen(QPen(style::kRawPath, 1.2, Qt::DashLine));  // the raw plan, for comparison
        painter.drawPolyline(eeCurve(plan->path));
    }
    painter.setPen(QPen(style::kPath, 2.0));
    painter.drawPolyline(eeCurve(path));
}

void WorkspaceView::drawExecution(QPainter& painter, const TrackingSample& sample) const {
    const ArmModel& arm = model_->arm();
    // Executed end-effector trail up to the playback time.
    if (const TrackingLog* log = model_->trackingLog()) {
        QPolygonF trail;
        for (const TrackingSample& s : log->samples) {
            if (s.time > sample.time) {
                break;
            }
            trail << toScreen(endEffectorPosition(arm, s.actual));
        }
        painter.setPen(QPen(style::kEndEffector, 1.5));
        painter.setBrush(Qt::NoBrush);
        painter.drawPolyline(trail);
    }
    // Reference pose as a thin skeleton, then the controlled arm on top.
    const std::vector<Vec2> ref = forwardKinematics(arm, sample.reference);
    painter.setPen(QPen(style::kPath, 2.0, Qt::DashLine, Qt::RoundCap));
    for (std::size_t k = 0; k + 1 < ref.size(); ++k) {
        painter.drawLine(toScreen(ref[k]), toScreen(ref[k + 1]));
    }
    drawArm(painter, sample.actual, /*ghost=*/false);
}

void WorkspaceView::drawReach(QPainter& painter) const {
    // Dashed circle: the boundary of what the end effector can reach with joint limits ignored.
    const double r = model_->arm().reach() * pixelsPerMetre();
    QPen pen(style::kTextDim, 1, Qt::DashLine);
    painter.setPen(pen);
    painter.setBrush(Qt::NoBrush);
    painter.drawEllipse(toScreen(model_->arm().base()), r, r);
}

void WorkspaceView::drawTrace(QPainter& painter) const {
    if (trace_.size() < 2) {
        return;
    }
    QPolygonF poly;
    poly.reserve(static_cast<qsizetype>(trace_.size()));
    for (const Vec2& p : trace_) {
        poly << toScreen(p);
    }
    painter.setPen(QPen(style::kTrace, 1.5));
    painter.setBrush(Qt::NoBrush);
    painter.drawPolyline(poly);
}

void WorkspaceView::drawArm(QPainter& painter, const Config& q, bool ghost) const {
    const ArmModel& arm = model_->arm();
    const std::vector<Vec2> pts = forwardKinematics(arm, q);
    const double s = pixelsPerMetre();
    // Links are capsules of radius linkRadius; draw them at true scale but never too thin to see.
    const double linkWidth = std::max(2.0 * arm.linkRadius() * s, 6.0);

    // Which links are in collision? Ghost (goal) arms turn entirely red if invalid.
    const CollisionReport report = model_->checker().check(q);
    const auto linkHit = [&](int k) {
        if (report.kind == CollisionKind::ObstacleContact || report.kind == CollisionKind::Boundary) {
            return report.link == k;
        }
        return report.kind == CollisionKind::Self && (report.link == k || report.other == k);
    };
    const QColor ghostColor = report.ok() ? style::kGoal : style::kInvalid;
    QColor link = ghost ? ghostColor : style::kLink;
    QColor outline = ghost ? ghostColor.darker(250) : style::kLinkOutline;
    if (ghost) {
        link.setAlpha(70);
        outline.setAlpha(70);
    }

    // Base mount: a small hatched block under joint 0.
    if (!ghost) {
        const QPointF b = toScreen(pts.front());
        const QRectF mount(b.x() - linkWidth * 1.4, b.y() - linkWidth * 0.2, linkWidth * 2.8, linkWidth * 1.4);
        painter.setPen(QPen(style::kAxis, 1));
        painter.setBrush(QBrush(style::kGridMajor, Qt::BDiagPattern));
        painter.drawRect(mount);
    }

    for (std::size_t i = 0; i + 1 < pts.size(); ++i) {
        const QPointF a = toScreen(pts[i]);
        const QPointF b = toScreen(pts[i + 1]);
        const bool hit = !ghost && linkHit(static_cast<int>(i));
        painter.setPen(QPen(hit ? style::kInvalid.darker(200) : outline, linkWidth + 3.0, Qt::SolidLine, Qt::RoundCap));
        painter.drawLine(a, b);
        painter.setPen(QPen(hit ? style::kInvalid : link, linkWidth, Qt::SolidLine, Qt::RoundCap));
        painter.drawLine(a, b);
    }

    // Joints (including the base) as small discs.
    const double jointR = std::max(linkWidth * 0.32, 3.0);
    QColor joint = style::kJoint;
    if (ghost) {
        joint.setAlpha(80);
    }
    painter.setPen(QPen(outline, 1.5));
    painter.setBrush(joint);
    for (std::size_t i = 0; i + 1 < pts.size(); ++i) {
        painter.drawEllipse(toScreen(pts[i]), jointR, jointR);
    }

    if (!ghost) {
        const QPointF ee = toScreen(pts.back());
        painter.setPen(QPen(style::kEndEffector.darker(180), 2));
        painter.setBrush(style::kEndEffector);
        painter.drawEllipse(ee, jointR * 1.5, jointR * 1.5);
    }
}

void WorkspaceView::drawGoal(QPainter& painter) const {
    const QPointF g = toScreen(model_->goalTarget());
    const bool valid = model_->goalIkOk() && model_->goalReport().ok();
    const QColor color = valid ? style::kGoal : style::kInvalid;
    constexpr double r = 9.0;
    painter.setPen(QPen(color, 2));
    painter.setBrush(Qt::NoBrush);
    painter.drawEllipse(g, r, r);
    painter.drawLine(QPointF(g.x() - r - 5, g.y()), QPointF(g.x() + r + 5, g.y()));
    painter.drawLine(QPointF(g.x(), g.y() - r - 5), QPointF(g.x(), g.y() + r + 5));

    painter.setFont(style::annotationFont());
    painter.drawText(QPointF(g.x() + r + 6, g.y() - r - 2), !model_->goalIkOk()
                                                                ? QStringLiteral("goal (no IK solution)")
                                                            : valid ? QStringLiteral("goal")
                                                                    : QStringLiteral("goal (in collision)"));
}

void WorkspaceView::drawOverlay(QPainter& painter) const {
    painter.setFont(style::annotationFont(10));
    painter.setPen(style::kText);
    painter.drawText(QPointF(12, 22), QStringLiteral("Workspace"));

    painter.setFont(style::annotationFont());
    painter.setPen(style::kTextDim);
    painter.drawText(QPointF(12, 40),
                     QStringLiteral("Drag the orange end effector (IK), the green goal marker, or any obstacle"));
    painter.drawText(
        QPointF(12, height() - 12.0),
        QStringLiteral("Blue: planned path (dashed: before smoothing). Orange: executed end-effector trail."));

    // Scale bar: 10 cm, bottom right.
    const double bar = kGridStep * pixelsPerMetre();
    const QPointF end(width() - 16.0, height() - 18.0);
    const QPointF start(end.x() - bar, end.y());
    painter.setPen(QPen(style::kText, 2));
    painter.drawLine(start, end);
    painter.drawLine(start + QPointF(0, -4), start + QPointF(0, 4));
    painter.drawLine(end + QPointF(0, -4), end + QPointF(0, 4));
    painter.drawText(QRectF(start.x() - 20, end.y() - 22, bar + 40, 16), Qt::AlignCenter, QStringLiteral("10 cm"));
}

// ---------------------------------------------------------------------------
// Mouse interaction
// ---------------------------------------------------------------------------

void WorkspaceView::mousePressEvent(QMouseEvent* event) {
    if (event->button() == Qt::LeftButton) {
        drag_ = hitTest(event->position());
        lastDragWorld_ = toWorld(event->position());
        if (drag_.target != DragTarget::None) {
            setCursor(Qt::ClosedHandCursor);
            update();
        }
    }
}

void WorkspaceView::mouseMoveEvent(QMouseEvent* event) {
    const QPointF pos = event->position();
    switch (drag_.target) {
        case DragTarget::EndEffector:
            model_->moveEndEffectorTo(toWorld(pos));
            break;
        case DragTarget::Goal:
            model_->setGoalTarget(toWorld(pos));
            break;
        case DragTarget::Obstacle: {
            const Vec2 world = toWorld(pos);
            model_->moveObstacle(drag_.obstacle, world - lastDragWorld_);
            lastDragWorld_ = world;
            break;
        }
        case DragTarget::None:
            setCursor(hitTest(pos).target == DragTarget::None ? Qt::ArrowCursor : Qt::OpenHandCursor);
            break;
    }
}

void WorkspaceView::mouseReleaseEvent(QMouseEvent* event) {
    if (event->button() == Qt::LeftButton) {
        drag_ = {};
        setCursor(hitTest(event->position()).target == DragTarget::None ? Qt::ArrowCursor : Qt::OpenHandCursor);
        update();
    }
}

}  // namespace amp::gui
