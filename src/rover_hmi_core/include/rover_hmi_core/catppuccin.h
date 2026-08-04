#pragma once

#include <QApplication>
#include <QRegularExpression>
#include <QString>
#include <QVariant>
#include <QWidget>

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
constexpr int FontSize     = 20;   // app-wide baseline (px in stylesheets)
constexpr int FontSizeLg   = 26;
constexpr int FontSizeSm   = 16;
constexpr int FontSizeXl    = 40;  // big touch-target control buttons
constexpr int FontSizeTitle = 44;  // panel titles — largest text in any panel

// Runtime font zoom (Ctrl+= / Ctrl+- in the host). All font sizes above are
// base values; use px() wherever a size is consumed at paint/style time.
inline double FontScale = 1.0;
inline int px(int base) { return qMax(6, qRound(base * FontScale)); }

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
    .arg(px(FontSize)).arg(Bg).arg(Text).arg(BgPanel).arg(BorderDim).arg(px(FontSizeSm)));
}

// Rewrite every "font-size:NNpx" in a stylesheet to the current scale.
inline QString scaleStyleSheet(const QString& base)
{
    static const QRegularExpression re("font-size\\s*:\\s*(\\d+)px");
    QString out;
    out.reserve(base.size());
    int last = 0;
    auto it = re.globalMatch(base);
    while (it.hasNext()) {
        auto m = it.next();
        out += base.mid(last, m.capturedStart() - last);
        out += QString("font-size:%1px").arg(px(m.captured(1).toInt()));
        last = m.capturedEnd();
    }
    out += base.mid(last);
    return out;
}

// Rescale one widget's stylesheet fonts (and explicitly-set QFonts) in place.
// Modules always author styles at base scale; the unscaled original is kept in
// a "ssBase" property so repeated zooms never compound. "ssApplied" tracks the
// last sheet we wrote, so a module overwriting its style is detected as a new
// base rather than re-scaled.
inline void rescaleWidget(QWidget* w)
{
    const QString cur = w->styleSheet();
    if (cur.contains("font-size")) {
        if (cur != w->property("ssApplied").toString())
            w->setProperty("ssBase", cur);
        const QString scaled = scaleStyleSheet(w->property("ssBase").toString());
        w->setProperty("ssApplied", scaled);
        if (scaled != cur) w->setStyleSheet(scaled);
    }
    if (w->testAttribute(Qt::WA_SetFont)) {
        QVariant base_pt = w->property("fontBasePt");
        if (!base_pt.isValid()) {
            base_pt = w->font().pointSize();
            w->setProperty("fontBasePt", base_pt);
        }
        if (base_pt.toInt() > 0) {
            QFont f = w->font();
            f.setPointSize(qMax(6, qRound(base_pt.toInt() * FontScale)));
            w->setFont(f);
        }
    }
}

inline void applyFontScale(QApplication& app, double scale)
{
    FontScale = qBound(0.5, scale, 2.0);
    applyGlobalStylesheet(app);
    for (QWidget* w : app.allWidgets()) rescaleWidget(w);
    for (QWidget* w : app.topLevelWidgets()) w->update();
}

} // namespace theme
