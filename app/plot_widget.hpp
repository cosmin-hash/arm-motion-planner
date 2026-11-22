/// @file plot_widget.hpp
/// @brief Minimal line-plot widget drawn with QPainter (no QtCharts dependency).
#pragma once

#include <QColor>
#include <QPointF>
#include <QString>
#include <QVector>
#include <QWidget>

namespace amp::gui {

/// One named polyline in a PlotWidget.
struct PlotSeries {
    QString name;
    QColor color;
    QVector<QPointF> points;  ///< (x, y) samples, x non-decreasing for a sensible plot.
};

/// A simple auto-scaling line plot with axis labels, grid, and legend.
///
/// Used for the RRT* cost-versus-iterations and tracking-error-versus-time plots. When no
/// data is present it shows a placeholder message instead of empty axes.
class PlotWidget : public QWidget {
    Q_OBJECT

public:
    /// Create an empty plot with a title and axis labels.
    PlotWidget(QString title, QString xLabel, QString yLabel, QWidget* parent = nullptr);

    /// Replace all series and repaint.
    void setSeries(QVector<PlotSeries> series);
    /// Remove all data (the placeholder text is shown again).
    void clear();
    /// Text shown in the middle of the plot while it has no data.
    void setPlaceholder(const QString& text);

    [[nodiscard]] QSize sizeHint() const override { return {420, 240}; }
    [[nodiscard]] QSize minimumSizeHint() const override { return {200, 140}; }

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    QString title_;
    QString xLabel_;
    QString yLabel_;
    QString placeholder_;
    QVector<PlotSeries> series_;
};

}  // namespace amp::gui
