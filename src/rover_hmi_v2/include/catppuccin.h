#pragma once

#include <QApplication>
#include <QString>

// Pure black terminal aesthetic with white borders
namespace theme {

constexpr const char* Bg         = "#000000";
constexpr const char* BgPanel    = "#0a0a0a";
constexpr const char* Border     = "#ffffff";
constexpr const char* BorderDim  = "#333333";
constexpr const char* Text       = "#ffffff";
constexpr const char* TextDim    = "#777777";
constexpr const char* HeaderBg   = "#111111";
constexpr const char* Green      = "#00ff88";
constexpr const char* Red        = "#ff4466";
constexpr const char* Yellow     = "#ffcc00";
constexpr const char* Cyan       = "#44ddcc";

constexpr int BorderRadius = 12;
constexpr int BorderWidth  = 2;
constexpr int FontSize     = 15;
constexpr int FontSizeLg   = 18;
constexpr int FontSizeSm   = 13;

// Per-motor colors (indexed 0-5)
constexpr const char* MotorColors[] = {
    "#5599ff",  // blue
    "#cc77ff",  // purple
    "#00ff88",  // green
    "#ff9944",  // orange
    "#ff4466",  // red
    "#44ddcc",  // teal
};

inline void applyGlobalStylesheet(QApplication& app) {
    app.setStyleSheet(QString(R"(
        * { font-family: monospace; font-size: %1px; }
        QMainWindow { background: %2; }
        QLabel { color: %3; font-size: %1px; }
        QPushButton {
            background: %4; color: %3; border: 1px solid %5;
            border-radius: 6px; padding: 10px 16px; font-weight: bold;
            font-size: %1px;
        }
        QPushButton:hover { background: #1a1a1a; border-color: %3; }
        QPushButton:pressed { background: #222222; }
        QComboBox {
            background: %4; color: %3; border: 1px solid %5;
            border-radius: 6px; padding: 8px 12px; font-size: %1px;
        }
        QComboBox::drop-down { border: none; }
        QComboBox QAbstractItemView {
            background: %4; color: %3; selection-background-color: #222222;
            font-size: %1px;
        }
        QDoubleSpinBox, QSpinBox {
            background: %4; color: %3; border: 1px solid %5;
            border-radius: 6px; padding: 8px 12px; font-size: %1px;
        }
        QCheckBox { color: %3; spacing: 8px; font-size: %1px; }
        QCheckBox::indicator {
            width: 18px; height: 18px; border: 1px solid %5;
            border-radius: 3px; background: %4;
        }
        QCheckBox::indicator:checked { background: %3; border-color: %3; }
        QTabWidget::pane { border: 1px solid %5; background: %2; border-radius: 6px; }
        QTabBar::tab {
            background: %4; color: %3; padding: 10px 20px;
            border: 1px solid %5; border-bottom: none;
            border-top-left-radius: 6px; border-top-right-radius: 6px;
            font-size: %1px;
        }
        QTabBar::tab:selected { background: %2; border-color: %3; }
        QTabBar::tab:hover { background: #1a1a1a; }
        QTextEdit {
            background: %4; color: %3; border: none;
            font-family: monospace; font-size: %6px;
            selection-background-color: #333333;
        }
    )")
    .arg(FontSize).arg(Bg).arg(Text).arg(BgPanel).arg(BorderDim).arg(FontSizeSm));
}

} // namespace theme
