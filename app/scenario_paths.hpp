/// @file scenario_paths.hpp
/// @brief Where the GUI looks for scenario files.
#pragma once

#include <QCoreApplication>
#include <QDir>

#include <filesystem>

#ifndef AMP_SCENARIO_DIR
#define AMP_SCENARIO_DIR "scenarios"
#endif

namespace amp::gui {

/// The `scenarios` folder next to the executable (copied there at build time), falling back
/// to the source tree's folder so the app also works straight from a build directory.
inline std::filesystem::path scenarioDirectory() {
    const QDir besideExe(QCoreApplication::applicationDirPath() + QStringLiteral("/scenarios"));
    if (besideExe.exists()) {
        return besideExe.absolutePath().toStdString();
    }
    return AMP_SCENARIO_DIR;
}

}  // namespace amp::gui
