#include <rover_hmi_core/science/spectrum_view.h>
#include <rover_hmi_core/catppuccin.h>

#include <QPainter>
#include <QPainterPath>
#include <QMouseEvent>
#include <QFile>
#include <QTextStream>
#include <QFontMetrics>
#include <algorithm>
#include <cmath>

namespace {
constexpr int kMarginL = 84;   // room for intensity labels
constexpr int kMarginR = 12;
constexpr int kMarginT = 10;
constexpr int kMarginB = 62;   // room for wavelength ticks and axis title
constexpr double kDefaultLo = 380.0;
constexpr double kDefaultHi = 780.0;
}

SpectrumView::SpectrumView(QWidget* parent) : QWidget(parent)
{
    setMinimumHeight(200);
    setMouseTracking(true);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
}

// ── data ────────────────────────────────────────────────────────────────────

void SpectrumView::setSpectrum(const QString& label,
                               std::vector<double> wavelengths,
                               std::vector<double> intensities)
{
    traces_.clear();
    addTrace(label, std::move(wavelengths), std::move(intensities));
}

void SpectrumView::addTrace(const QString& label,
                            std::vector<double> wavelengths,
                            std::vector<double> intensities)
{
    if (wavelengths.empty() || wavelengths.size() != intensities.size()) return;

    // Raw captures come out in pixel order, and the calibration polynomial is
    // not necessarily monotonic, so wavelengths can double back. Sort here as
    // curve_gen.py does — drawing and the cursor lookup both need ascending x.
    //
    // A fold is worth reporting: it means several pixels claim the same
    // wavelength, so any single-wavelength reading picks between them
    // arbitrarily. That is a calibration fault, not sensor noise.
    Trace t;
    t.label       = label;
    t.wavelengths = std::move(wavelengths);
    t.intensities = std::move(intensities);

    // Split at turning points; each run is drawn on its own.
    int seg_start = 0;
    for (size_t i = 1; i + 1 < t.wavelengths.size(); i++) {
        const double d0 = t.wavelengths[i]     - t.wavelengths[i - 1];
        const double d1 = t.wavelengths[i + 1] - t.wavelengths[i];
        if (d0 * d1 < 0.0) {
            t.segments.emplace_back(seg_start, static_cast<int>(i));
            seg_start = static_cast<int>(i);
        }
    }
    t.segments.emplace_back(seg_start,
                            static_cast<int>(t.wavelengths.size()) - 1);

    if (t.segments.size() > 1) {
        const size_t folds = t.segments.size() - 1;
        quality_warning_ = QStringLiteral(
            "%1: calibration not monotonic (%2 turning point%3) — "
            "several pixels share a wavelength")
            .arg(label).arg(folds).arg(folds == 1 ? "" : "s");
    }
    // Trace 0 is the primary (white); overlays cycle the motor palette.
    t.color = traces_.empty()
        ? QString(theme::Text)
        : QString(theme::MotorColors[(traces_.size() - 1) % 6]);
    traces_.push_back(std::move(t));

    max_intensity_ = 0.0;
    for (const auto& tr : traces_)
        for (double v : tr.intensities)
            max_intensity_ = std::max(max_intensity_, v);
    // Keep the familiar 0-255 scale unless the data genuinely exceeds it.
    if (max_intensity_ < 255.0) max_intensity_ = 255.0;

    update();
}

void SpectrumView::clearTraces()
{
    traces_.clear();
    quality_warning_.clear();
    max_intensity_ = 255.0;
    update();
}

void SpectrumView::setMarkerWavelength(double nm) { marker_nm_ = nm; update(); }
void SpectrumView::setPeaksVisible(bool on)       { peaks_on_  = on; update(); }
void SpectrumView::setFillVisible(bool on)        { fill_on_   = on; update(); }

// ── CSV ─────────────────────────────────────────────────────────────────────

bool SpectrumView::loadCsv(const QString& path,
                           std::vector<double>& wavelengths,
                           std::vector<double>& intensities,
                           QString& error)
{
    QFile f(path);
    if (!f.open(QIODevice::ReadOnly | QIODevice::Text)) {
        error = QStringLiteral("cannot open %1").arg(path);
        return false;
    }

    wavelengths.clear();
    intensities.clear();

    QTextStream in(&f);
    int line_no = 0;
    while (!in.atEnd()) {
        const QString line = in.readLine().trimmed();
        line_no++;
        if (line.isEmpty()) continue;

        const QStringList cols = line.split(',');
        if (cols.size() < 2) continue;

        bool ok_w = false, ok_i = false;
        const double w = cols[0].trimmed().toDouble(&ok_w);
        const double i = cols[1].trimmed().toDouble(&ok_i);
        if (!ok_w || !ok_i) {
            // The header row is expected; anything else is a real problem.
            if (line_no == 1) continue;
            error = QStringLiteral("%1: unparseable line %2").arg(path).arg(line_no);
            return false;
        }
        wavelengths.push_back(w);
        intensities.push_back(i);
    }

    if (wavelengths.empty()) {
        error = QStringLiteral("%1: no data rows").arg(path);
        return false;
    }
    return true;
}

bool SpectrumView::saveCsv(const QString& path,
                           const std::vector<double>& wavelengths,
                           const std::vector<double>& intensities,
                           QString& error)
{
    if (wavelengths.size() != intensities.size() || wavelengths.empty()) {
        error = QStringLiteral("nothing to save");
        return false;
    }

    QFile f(path);
    if (!f.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate)) {
        error = QStringLiteral("cannot write %1").arg(path);
        return false;
    }

    // PySpectrometer2 format: 1 dp wavelength, integer intensity clipped 0-255.
    QTextStream out(&f);
    out << "Wavelength,Intensity\n";
    for (size_t i = 0; i < wavelengths.size(); i++) {
        const int v = static_cast<int>(std::lround(
            std::clamp(intensities[i], 0.0, 255.0)));
        out << QString::number(wavelengths[i], 'f', 1) << ',' << v << '\n';
    }
    return true;
}

// ── colour + peaks ──────────────────────────────────────────────────────────

QColor SpectrumView::wavelengthToRgb(double nm)
{
    double r = 0, g = 0, b = 0;
    if      (nm >= 380 && nm < 440) { r = -(nm - 440) / 60.0; b = 1.0; }
    else if (nm >= 440 && nm < 490) { g =  (nm - 440) / 50.0; b = 1.0; }
    else if (nm >= 490 && nm < 510) { g = 1.0; b = -(nm - 510) / 20.0; }
    else if (nm >= 510 && nm < 580) { r =  (nm - 510) / 70.0; g = 1.0; }
    else if (nm >= 580 && nm < 645) { r = 1.0; g = -(nm - 645) / 65.0; }
    else if (nm >= 645 && nm <= 780) { r = 1.0; }
    else return QColor(20, 20, 20);

    // Eye sensitivity falls off at both ends.
    double s = 1.0;
    if      (nm >= 380 && nm < 420) s = 0.3 + 0.7 * (nm - 380) / 40.0;
    else if (nm > 700 && nm <= 780) s = 0.3 + 0.7 * (780 - nm) / 80.0;

    const auto ch = [s](double v) {
        return static_cast<int>(std::lround(std::pow(v * s, 0.8) * 255.0));
    };
    return QColor(ch(r), ch(g), ch(b));
}

std::vector<int> SpectrumView::peakIndexes(const std::vector<double>& y,
                                           double thres, int min_dist)
{
    std::vector<int> peaks;
    if (y.size() < 3) return peaks;

    const auto [mn_it, mx_it] = std::minmax_element(y.begin(), y.end());
    const double mn = *mn_it, mx = *mx_it;
    if (mx - mn <= 0.0) return peaks;

    const double cutoff = mn + thres * (mx - mn);

    std::vector<int> candidates;
    for (size_t i = 1; i + 1 < y.size(); i++)
        if (y[i] > y[i - 1] && y[i] >= y[i + 1] && y[i] >= cutoff)
            candidates.push_back(static_cast<int>(i));

    // Highest first, then drop anything too close to an accepted peak.
    std::sort(candidates.begin(), candidates.end(),
              [&y](int a, int b) { return y[a] > y[b]; });
    for (int c : candidates) {
        bool too_close = false;
        for (int p : peaks)
            if (std::abs(p - c) < min_dist) { too_close = true; break; }
        if (!too_close) peaks.push_back(c);
    }
    std::sort(peaks.begin(), peaks.end());
    return peaks;
}

// ── coordinate mapping ──────────────────────────────────────────────────────

void SpectrumView::wavelengthRange(double& lo, double& hi) const
{
    lo =  1e9;
    hi = -1e9;
    // Full span, not endpoints — a folded trace does not start at its minimum.
    for (const auto& t : traces_) {
        if (t.wavelengths.empty()) continue;
        const auto [mn, mx] = std::minmax_element(t.wavelengths.begin(),
                                                  t.wavelengths.end());
        lo = std::min(lo, *mn);
        hi = std::max(hi, *mx);
    }
    if (lo >= hi) { lo = kDefaultLo; hi = kDefaultHi; }
}

int SpectrumView::wavelengthToX(double nm, const QRect& plot) const
{
    double lo, hi;
    wavelengthRange(lo, hi);
    return plot.left() + static_cast<int>((nm - lo) / (hi - lo) * plot.width());
}

double SpectrumView::xToWavelength(int x, const QRect& plot) const
{
    double lo, hi;
    wavelengthRange(lo, hi);
    if (plot.width() <= 0) return lo;
    return lo + (double(x - plot.left()) / plot.width()) * (hi - lo);
}

int SpectrumView::intensityToY(double v, const QRect& plot) const
{
    const double frac = std::clamp(v / max_intensity_, 0.0, 1.0);
    return plot.bottom() - static_cast<int>(frac * plot.height());
}

// ── painting ────────────────────────────────────────────────────────────────

void SpectrumView::paintEvent(QPaintEvent*)
{
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, true);
    p.fillRect(rect(), QColor(theme::Bg));

    const QRect plot(kMarginL, kMarginT,
                     std::max(1, width()  - kMarginL - kMarginR),
                     std::max(1, height() - kMarginT - kMarginB));

    drawGraticule(p, plot);

    if (traces_.empty()) {
        p.setPen(QColor(theme::TextDim));
        p.drawText(plot, Qt::AlignCenter,
                   QStringLiteral("no spectrum — capture or load a CSV"));
        return;
    }

    // Overlays first so the primary trace stays readable on top of them.
    for (size_t i = 1; i < traces_.size(); i++)
        drawTrace(p, plot, traces_[i], false);
    drawTrace(p, plot, traces_[0], true);

    if (peaks_on_) drawPeaks(p, plot, traces_[0]);
    drawCursor(p, plot);

    if (!quality_warning_.isEmpty()) {
        QFont wf = p.font();
        wf.setPointSize(theme::FontSize);
        p.setFont(wf);
        p.setPen(QColor(theme::Yellow));
        // Stop short of the legend and elide rather than overlapping it.
        const int avail = plot.width() - 12 - (traces_.size() > 1 ? 250 : 0);
        const QString text = QFontMetrics(wf).elidedText(
            QStringLiteral("⚠ ") + quality_warning_, Qt::ElideRight,
            std::max(60, avail));
        p.drawText(plot.left() + 6, plot.top() + 4, std::max(60, avail), 22,
                   Qt::AlignLeft | Qt::AlignTop, text);
    }

    // Legend, only when there is something to disambiguate.
    if (traces_.size() > 1) {
        int y = plot.top() + 6;
        QFont f = p.font();
        f.setPointSize(theme::FontSize);
        p.setFont(f);
        for (const auto& t : traces_) {
            p.setPen(QColor(t.color));
            p.drawText(plot.right() - 240, y, 236, 20,
                       Qt::AlignRight | Qt::AlignVCenter, t.label);
            y += 20;
        }
    }
}

void SpectrumView::drawGraticule(QPainter& p, const QRect& plot)
{
    double lo, hi;
    wavelengthRange(lo, hi);

    p.setPen(QPen(QColor(theme::BorderDim), 1));
    p.drawRect(plot);

    QFont f = p.font();
    f.setPointSize(theme::FontSize);
    p.setFont(f);

    // 10 nm ticks, labelled every 50 nm — PySpectrometer2's convention.
    const double start = std::ceil(lo / 10.0) * 10.0;
    for (double nm = start; nm <= hi; nm += 10.0) {
        const int x = wavelengthToX(nm, plot);
        const bool labelled = std::fmod(nm, 50.0) < 0.001;

        p.setPen(QPen(QColor(labelled ? theme::BorderDim : "#1a1a1a"), 1));
        p.drawLine(x, plot.top(), x, plot.bottom());

        p.setPen(QColor(theme::BorderDim));
        p.drawLine(x, plot.bottom(), x, plot.bottom() + (labelled ? 6 : 3));

        if (labelled) {
            p.setPen(QColor(theme::TextDim));
            p.drawText(x - 34, plot.bottom() + 9, 68, 20,
                       Qt::AlignHCenter | Qt::AlignTop,
                       QString::number(static_cast<int>(nm)));
        }
    }

    // Intensity axis in quarters of the current scale.
    for (int i = 0; i <= 4; i++) {
        const double v = max_intensity_ * i / 4.0;
        const int    y = intensityToY(v, plot);
        p.setPen(QPen(QColor("#1a1a1a"), 1));
        p.drawLine(plot.left(), y, plot.right(), y);
        p.setPen(QColor(theme::TextDim));
        p.drawText(2, y - 11, kMarginL - 10, 22,
                   Qt::AlignRight | Qt::AlignVCenter,
                   QString::number(v, 'f', 0));
    }

    p.setPen(QColor(theme::TextDim));
    p.drawText(plot.left(), plot.bottom() + 36, plot.width(), 22,
               Qt::AlignHCenter | Qt::AlignTop, QStringLiteral("wavelength (nm)"));

    if (marker_nm_ > 0 && marker_nm_ >= lo && marker_nm_ <= hi) {
        const int x = wavelengthToX(marker_nm_, plot);
        p.setPen(QPen(QColor(theme::Red), 1, Qt::DashLine));
        p.drawLine(x, plot.top(), x, plot.bottom());
        p.setPen(QColor(theme::Red));
        const int label_y = plot.top() + (quality_warning_.isEmpty() ? 2 : 28);
        p.drawText(x + 5, label_y, 120, 20, Qt::AlignLeft | Qt::AlignTop,
                   QStringLiteral("λ %1").arg(marker_nm_, 0, 'f', 1));
    }
}

void SpectrumView::drawTrace(QPainter& p, const QRect& plot,
                             const Trace& t, bool primary)
{
    if (t.wavelengths.size() < 2) return;

    // Fill under the primary trace with each wavelength's own colour — the
    // effect that makes upstream's graph readable at a glance.
    if (primary && fill_on_) {
        for (size_t i = 0; i + 1 < t.wavelengths.size(); i++) {
            const int x0 = wavelengthToX(t.wavelengths[i], plot);
            const int x1 = wavelengthToX(t.wavelengths[i + 1], plot);
            const int xl = std::min(x0, x1);
            if (std::max(x0, x1) < plot.left() || xl > plot.right()) continue;

            const int y0 = intensityToY(t.intensities[i], plot);
            QColor c = wavelengthToRgb(t.wavelengths[i]);
            c.setAlpha(90);
            p.fillRect(QRect(xl, y0, std::max(1, std::abs(x1 - x0)),
                             plot.bottom() - y0), c);
        }
    }

    p.setPen(QPen(QColor(t.color), primary ? 2 : 1));
    for (const auto& [start, end] : t.segments) {
        if (end <= start) continue;
        QPainterPath path;
        path.moveTo(wavelengthToX(t.wavelengths[start], plot),
                    intensityToY(t.intensities[start], plot));
        for (int i = start + 1; i <= end; i++)
            path.lineTo(wavelengthToX(t.wavelengths[i], plot),
                        intensityToY(t.intensities[i], plot));
        p.drawPath(path);
    }
}

void SpectrumView::drawPeaks(QPainter& p, const QRect& plot, const Trace& t)
{
    // min_dist in samples scaled so labels do not collide on wide spectra.
    const int min_dist = std::max(8, static_cast<int>(t.intensities.size() / 40));
    const auto peaks = peakIndexes(t.intensities, 0.55, min_dist);

    QFont f = p.font();
    f.setPointSize(theme::FontSize);
    p.setFont(f);

    // Tallest first, and keep labels apart in screen space so they stay
    // legible however densely the samples fall.
    std::vector<int> ordered = peaks;
    std::sort(ordered.begin(), ordered.end(),
              [&t](int a, int b) { return t.intensities[a] > t.intensities[b]; });

    std::vector<int> placed_x;
    for (int idx : ordered) {
        if (placed_x.size() >= 6) break;
        const int x = wavelengthToX(t.wavelengths[idx], plot);

        bool collides = false;
        for (int px : placed_x)
            if (std::abs(px - x) < 92) { collides = true; break; }
        if (collides) continue;

        const int y = intensityToY(t.intensities[idx], plot);
        p.setPen(QPen(QColor(theme::Yellow), 1));
        p.drawLine(x, y - 4, x, y - 10);
        p.drawText(x - 44, y - 30, 88, 20, Qt::AlignHCenter | Qt::AlignBottom,
                   QString::number(t.wavelengths[idx], 'f', 1));
        placed_x.push_back(x);
    }
}

void SpectrumView::drawCursor(QPainter& p, const QRect& plot)
{
    if (cursor_x_ < plot.left() || cursor_x_ > plot.right()) return;
    if (traces_.empty()) return;

    const double nm = xToWavelength(cursor_x_, plot);

    p.setPen(QPen(QColor(theme::Cyan), 1, Qt::DotLine));
    p.drawLine(cursor_x_, plot.top(), cursor_x_, plot.bottom());

    // Nearest sample by wavelength — a linear scan, since a folded trace is
    // not sorted and 800 points costs nothing.
    const Trace& t = traces_[0];
    QString readout = QStringLiteral("%1 nm").arg(nm, 0, 'f', 1);
    int    best_i = -1;
    double best_d = 1e9;
    for (size_t i = 0; i < t.wavelengths.size(); i++) {
        const double d = std::abs(t.wavelengths[i] - nm);
        if (d < best_d) { best_d = d; best_i = static_cast<int>(i); }
    }
    if (best_i >= 0)
        readout += QStringLiteral("   I %1").arg(t.intensities[best_i], 0, 'f', 0);

    QFont f = p.font();
    f.setPointSize(theme::FontSize);
    p.setFont(f);
    const QFontMetrics fm(f);
    const int w = fm.horizontalAdvance(readout) + 12;
    int bx = cursor_x_ + 8;
    if (bx + w > plot.right()) bx = cursor_x_ - w - 8;

    const QRect box(bx, plot.top() + 4, w, 26);
    p.fillRect(box, QColor(theme::HeaderBg));
    p.setPen(QColor(theme::Cyan));
    p.drawRect(box);
    p.drawText(box, Qt::AlignCenter, readout);
}

void SpectrumView::mouseMoveEvent(QMouseEvent* event)
{
    cursor_x_ = event->pos().x();
    update();
}

void SpectrumView::leaveEvent(QEvent*)
{
    cursor_x_ = -1;
    update();
}
