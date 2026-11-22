/// @file main.cpp
/// @brief GUI entry point: applies a dark Fusion theme and shows the main window.
#include <QApplication>
#include <QCommandLineParser>
#include <QPalette>
#include <QStyleFactory>

#include <algorithm>

#include "controls_panel.hpp"
#include "main_window.hpp"

namespace {

/// A dark palette so the native widgets match the dark custom-painted views.
QPalette darkPalette() {
    QPalette p;
    const QColor window(0x2b, 0x2f, 0x37);
    const QColor base(0x1f, 0x23, 0x2a);
    const QColor text(0xd8, 0xdc, 0xe3);
    const QColor disabledText(0x6c, 0x73, 0x80);
    const QColor highlight(0x4f, 0x9d, 0xde);

    p.setColor(QPalette::Window, window);
    p.setColor(QPalette::WindowText, text);
    p.setColor(QPalette::Base, base);
    p.setColor(QPalette::AlternateBase, window);
    p.setColor(QPalette::ToolTipBase, base);
    p.setColor(QPalette::ToolTipText, text);
    p.setColor(QPalette::Text, text);
    p.setColor(QPalette::Button, window);
    p.setColor(QPalette::ButtonText, text);
    p.setColor(QPalette::Highlight, highlight);
    p.setColor(QPalette::HighlightedText, Qt::white);
    p.setColor(QPalette::Link, highlight);
    p.setColor(QPalette::PlaceholderText, disabledText);
    for (QPalette::ColorRole role : {QPalette::WindowText, QPalette::Text, QPalette::ButtonText}) {
        p.setColor(QPalette::Disabled, role, disabledText);
    }
    return p;
}

}  // namespace

int main(int argc, char* argv[]) {
    QApplication app(argc, argv);
    QApplication::setApplicationName(QStringLiteral("arm-motion-planner"));
    QApplication::setStyle(QStyleFactory::create(QStringLiteral("Fusion")));
    QApplication::setPalette(darkPalette());

    QCommandLineParser parser;
    parser.setApplicationDescription(QStringLiteral("Planar robot arm motion planning and control demo"));
    parser.addHelpOption();
    const QCommandLineOption dofOption(QStringLiteral("dof"), QStringLiteral("Joints of the initial arm (2-6)."),
                                       QStringLiteral("n"), QStringLiteral("3"));
    parser.addOption(dofOption);
    const QCommandLineOption scenarioOption(QStringLiteral("scenario"), QStringLiteral("Scenario JSON file to load."),
                                            QStringLiteral("file"));
    parser.addOption(scenarioOption);
    const QCommandLineOption planOption(QStringLiteral("plan"),
                                        QStringLiteral("Plan immediately with rrt, rrtstar or astar."),
                                        QStringLiteral("planner"));
    parser.addOption(planOption);
    const QCommandLineOption executeOption(QStringLiteral("execute"),
                                           QStringLiteral("After --plan succeeds, execute the trajectory."));
    parser.addOption(executeOption);
    const QCommandLineOption pauseOption(QStringLiteral("pause-at"),
                                         QStringLiteral("Freeze execution playback at this time (s)."),
                                         QStringLiteral("seconds"));
    parser.addOption(pauseOption);
    const QCommandLineOption screenshotOption(
        QStringLiteral("screenshot"), QStringLiteral("Save a PNG of the window, then quit."), QStringLiteral("file"));
    parser.addOption(screenshotOption);
    const QCommandLineOption delayOption(QStringLiteral("delay"),
                                         QStringLiteral("Milliseconds to wait before --screenshot (default 3000)."),
                                         QStringLiteral("ms"), QStringLiteral("3000"));
    parser.addOption(delayOption);
    const QCommandLineOption noFeedforwardOption(QStringLiteral("no-feedforward"),
                                                 QStringLiteral("Start with acceleration feedforward off."));
    parser.addOption(noFeedforwardOption);
    parser.process(app);
    const int dof = std::clamp(parser.value(dofOption).toInt(), 2, 6);

    amp::gui::MainWindow window(dof);
    window.show();
    if (parser.isSet(scenarioOption)) {
        window.loadScenarioFile(parser.value(scenarioOption));
    }
    if (parser.isSet(noFeedforwardOption)) {
        window.controls()->setFeedforward(false);
    }
    window.setAutoExecute(parser.isSet(executeOption));
    if (parser.isSet(pauseOption)) {
        window.setPauseAt(parser.value(pauseOption).toDouble());
    }
    if (parser.isSet(screenshotOption)) {
        window.screenshotAndQuit(parser.value(screenshotOption), parser.value(delayOption).toInt());
    }
    if (parser.isSet(planOption)) {
        const QString p = parser.value(planOption).toLower();
        window.planWith(p == QStringLiteral("rrtstar") ? amp::PlannerType::RrtStar
                        : p == QStringLiteral("astar") ? amp::PlannerType::AStar
                                                       : amp::PlannerType::Rrt);
    }
    return QApplication::exec();
}
