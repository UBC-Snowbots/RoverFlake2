// spectrum_view.h — wavelength-domain spectrum display.
//
// Qt port of PySpectrometer2's graph: 10 nm graticule with 50 nm labels, the
// trace filled with the visible-spectrum colour of each wavelength, and peak
// labels. Styled to the HMI theme rather than upstream's OpenCV look.
//
// Holds several traces so a sample can be compared against its blank.
#pragma once

#include <QWidget>
#include <QString>
#include <vector>

class SpectrumView : public QWidget {
    Q_OBJECT
public:
    explicit SpectrumView(QWidget* parent = nullptr);

    struct Trace {
        QString             label;
        // Kept in capture (pixel) order. A non-monotonic calibration makes the
        // wavelength axis fold, so sorting would interleave unrelated parts of
        // the sensor into one zigzag; segments preserves each monotonic run as
        // its own polyline instead.
        std::vector<double> wavelengths;  // nm
        std::vector<double> intensities;  // 0-255
        std::vector<std::pair<int, int>> segments;  // [start, end] inclusive
        QString             color;
    };

    // Replaces trace 0 and clears the rest — the live/primary spectrum.
    void setSpectrum(const QString& label,
                     std::vector<double> wavelengths,
                     std::vector<double> intensities);

    // Overlay for comparison. Colors cycle through the theme's motor palette.
    void addTrace(const QString& label,
                  std::vector<double> wavelengths,
                  std::vector<double> intensities);

    void clearTraces();

    // Dashed marker at the assay wavelength. Negative hides it.
    void setMarkerWavelength(double nm);

    void setPeaksVisible(bool on);
    void setFillVisible(bool on);

    // Non-empty when a loaded trace's wavelengths fold back on themselves,
    // meaning the pixel-to-wavelength calibration is not monotonic and several
    // pixels claim the same wavelength. Empty when the data is well formed.
    QString qualityWarning() const { return quality_warning_; }

    // Parses a PySpectrometer2 CSV (Wavelength,Intensity). Returns false and
    // sets error on a malformed file.
    static bool loadCsv(const QString& path,
                        std::vector<double>& wavelengths,
                        std::vector<double>& intensities,
                        QString& error);

    static bool saveCsv(const QString& path,
                        const std::vector<double>& wavelengths,
                        const std::vector<double>& intensities,
                        QString& error);

protected:
    void paintEvent(QPaintEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void leaveEvent(QEvent* event) override;

private:
    // Visible-spectrum colour for a wavelength, per PySpectrometer2's
    // wavelength_to_rgb(). Outside 380-780 nm returns near-black.
    static QColor wavelengthToRgb(double nm);

    // Local maxima above thres (fraction of range) and at least min_dist
    // samples apart — the shape of PySpectrometer2's peakIndexes().
    static std::vector<int> peakIndexes(const std::vector<double>& y,
                                        double thres, int min_dist);

    void drawGraticule(QPainter& p, const QRect& plot);
    void drawTrace(QPainter& p, const QRect& plot, const Trace& t, bool primary);
    void drawPeaks(QPainter& p, const QRect& plot, const Trace& t);
    void drawCursor(QPainter& p, const QRect& plot);

    double xToWavelength(int x, const QRect& plot) const;
    int    wavelengthToX(double nm, const QRect& plot) const;
    int    intensityToY(double v, const QRect& plot) const;

    // Wavelength span across all traces; falls back to 380-780.
    void   wavelengthRange(double& lo, double& hi) const;

    std::vector<Trace> traces_;
    QString quality_warning_;
    double marker_nm_    = -1.0;
    bool   peaks_on_     = true;
    bool   fill_on_      = true;
    int    cursor_x_     = -1;
    double max_intensity_ = 255.0;
};
