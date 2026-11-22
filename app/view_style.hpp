/// @file view_style.hpp
/// @brief Shared colour palette and fonts for the custom-painted views, so the workspace,
/// C-space and plot views read as one visual system.
#pragma once

#include <QColor>
#include <QFont>

namespace amp::gui::style {

inline const QColor kBackground{0x1b, 0x1e, 0x24};
inline const QColor kPanel{0x22, 0x26, 0x2e};
inline const QColor kGridMinor{0x29, 0x2e, 0x37};
inline const QColor kGridMajor{0x36, 0x3c, 0x48};
inline const QColor kAxis{0x55, 0x5d, 0x6c};
inline const QColor kText{0xc8, 0xcc, 0xd4};
inline const QColor kTextDim{0x7d, 0x85, 0x93};

inline const QColor kLink{0x4f, 0x9d, 0xde};
inline const QColor kLinkOutline{0x1f, 0x4f, 0x78};
inline const QColor kJoint{0xe6, 0xe9, 0xef};
inline const QColor kEndEffector{0xf5, 0xa6, 0x23};
inline const QColor kGoal{0x3e, 0xcf, 0x8e};
inline const QColor kInvalid{0xe5, 0x53, 0x4b};
inline const QColor kTrace{0xf5, 0xa6, 0x23, 150};
inline const QColor kForbidden{0x0b, 0x0d, 0x10, 170};

inline const QColor kObstacleFill{0xc2, 0x5b, 0x56, 110};
inline const QColor kObstacleEdge{0xd9, 0x7a, 0x74};
inline const QColor kBoundary{0x9a, 0x8a, 0x6a};
inline const QColor kSelfCollision{0xa0, 0x7c, 0xd8};

inline const QColor kPath{0x7f, 0xd3, 0xff};
inline const QColor kTree{0x6f, 0x8f, 0xb0, 110};
inline const QColor kRawPath{0x7f, 0xd3, 0xff, 90};  ///< Unsmoothed planner output (faint).

/// Small UI font used for annotations inside the painted views.
inline QFont annotationFont(int pointSize = 9) {
    QFont f;
    f.setPointSize(pointSize);
    return f;
}

}  // namespace amp::gui::style
