/// @file plot_widget.cpp
/// @brief QPainter implementation of the line plot.
#include "plot_widget.hpp"

#include <QPainter>

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "view_style.hpp"

namespace amp::gui {

namespace {

constexpr double kLeft = 58.0;
constexpr double kRight = 16.0;
constexpr double kTop = 30.0;
constexpr double kBottom = 40.0;

/// A "nice" tick spacing (1, 2 or 5 times a power of ten) giving about `targetTicks` ticks.
double niceStep(double range, int targetTicks = 5) {
    if (!(range > 0.0)) {
        return 1.0;
    }
    const double raw = range / targetTicks;
    const double magnitude = std::pow(10.0, std::floor(std::log10(raw)));
    const double normalized = raw / magnitude;
    const double nice = normalized < 1.5 ? 1.0 : normalized < 3.0 ? 2.0 : normalized < 7.0 ? 5.0 : 10.0;
    return nice * magnitude;
}

}  // namespace

PlotWidget::PlotWidget(QString title, QString xLabel, QString yLabel, QWidget* parent)
    : QWidget(parent), title_(std::move(title)), xLabel_(std::move(xLabel)), yLabel_(std::move(yLabel)) {}

void PlotWidget::setSeries(QVector<PlotSeries> series) {
    series_ = std::move(series);
    update();
}

void PlotWidget::clear() {
    series_.clear();
    update();
}

void PlotWidget::setPlaceholder(const QString& text) {
    placeholder_ = text;
    update();
}

void PlotWidget::paintEvent(QPaintEvent* /*event*/) {
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    painter.fillRect(rect(), style::kBackground);

    const QRectF area(kLeft, kTop, std::max(10.0, width() - kLeft - kRight), std::max(10.0, height() - kTop - kBottom));
    painter.fillRect(area, style::kPanel);

    painter.setFont(style::annotationFont(10));
    painter.setPen(style::kText);
    painter.drawText(QPointF(12, 20), title_);

    // Data bounds over all series.
    double xMin = std::numeric_limits<double>::max(), xMax = std::numeric_limits<double>::lowest();
    double yMin = xMin, yMax = xMax;
    for (const PlotSeries& s : series_) {
        for (const QPointF& p : s.points) {
            xMin = std::min(xMin, p.x());
            xMax = std::max(xMax, p.x());
            yMin = std::min(yMin, p.y());
            yMax = std::max(yMax, p.y());
        }
    }
    const bool hasData = xMin <= xMax;

    painter.setFont(style::annotationFont(9));
    painter.setPen(style::kTextDim);
    painter.drawText(QRectF(area.left(), area.bottom() + 20, area.width(), 16), Qt::AlignCenter, xLabel_);
    painter.save();
    painter.translate(12, area.center().y());
    painter.rotate(-90);
    painter.drawText(QRectF(-area.height() / 2, -8, area.height(), 16), Qt::AlignCenter, yLabel_);
    painter.restore();

    if (!hasData) {
        painter.setPen(QPen(style::kAxis, 1));
        painter.drawRect(area);
        painter.setPen(style::kTextDim);
        painter.drawText(area.adjusted(12, 8, -12, -8), Qt::AlignCenter | Qt::TextWordWrap, placeholder_);
        return;
    }

    // Avoid a zero-height or zero-width range, then pad y by 5% so lines do not touch the frame.
    if (xMax - xMin < 1e-12) {
        xMax = xMin + 1.0;
    }
    if (yMax - yMin < 1e-12) {
        yMin -= 0.5;
        yMax += 0.5;
    }
    const double pad = 0.05 * (yMax - yMin);
    yMin -= pad;
    yMax += pad;

    const auto map = [&](const QPointF& p) {
        return QPointF(area.left() + (p.x() - xMin) / (xMax - xMin) * area.width(),
                       area.bottom() - (p.y() - yMin) / (yMax - yMin) * area.height());
    };

    // Grid and tick labels.
    painter.setFont(style::annotationFont(8));
    const double xStep = niceStep(xMax - xMin);
    for (double x = std::ceil(xMin / xStep) * xStep; x <= xMax + 1e-9 * xStep; x += xStep) {
        const double px = map(QPointF(x, yMin)).x();
        painter.setPen(QPen(style::kGridMinor, 1));
        painter.drawLine(QPointF(px, area.top()), QPointF(px, area.bottom()));
        painter.setPen(style::kTextDim);
        painter.drawText(QRectF(px - 30, area.bottom() + 3, 60, 14), Qt::AlignCenter, QString::number(x, 'g', 6));
    }
    const double yStep = niceStep(yMax - yMin);
    for (double y = std::ceil(yMin / yStep) * yStep; y <= yMax + 1e-9 * yStep; y += yStep) {
        const double py = map(QPointF(xMin, y)).y();
        painter.setPen(QPen(std::abs(y) < 1e-9 * yStep ? style::kAxis : style::kGridMinor, 1));
        painter.drawLine(QPointF(area.left(), py), QPointF(area.right(), py));
        painter.setPen(style::kTextDim);
        painter.drawText(QRectF(area.left() - 54, py - 7, 50, 14), Qt::AlignRight | Qt::AlignVCenter,
                         QString::number(y, 'g', 6));
    }
    painter.setPen(QPen(style::kAxis, 1));
    painter.setBrush(Qt::NoBrush);
    painter.drawRect(area);

    // Series, clipped to the plotting area.
    painter.save();
    painter.setClipRect(area);
    for (const PlotSeries& s : series_) {
        QPolygonF poly;
        poly.reserve(s.points.size());
        for (const QPointF& p : s.points) {
            poly << map(p);
        }
        painter.setPen(QPen(s.color, 1.8));
        painter.drawPolyline(poly);
    }
    painter.restore();

    // Legend (top right inside the plot).
    painter.setFont(style::annotationFont(8));
    double ly = area.top() + 8;
    for (const PlotSeries& s : series_) {
        if (s.name.isEmpty()) {
            continue;
        }
        const double lx = area.right() - 110;
        painter.setPen(QPen(s.color, 2.5));
        painter.drawLine(QPointF(lx, ly + 6), QPointF(lx + 16, ly + 6));
        painter.setPen(style::kText);
        painter.drawText(QPointF(lx + 22, ly + 10), s.name);
        ly += 16;
    }
}

}  // namespace amp::gui
