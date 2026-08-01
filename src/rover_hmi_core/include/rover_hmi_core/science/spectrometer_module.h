// spectrometer_module.h — "Spectrometer"
//
// Camera + diffraction-grating instrument: capture a spectrum, view it, save
// or load PySpectrometer2-format CSVs, and predict concentration against a
// standard curve. Drives the vendored Python in scripts/spectrometer/.
//
// Distinct from the six-vial absorbance readout in Science Analysis, which
// comes from the science embedded node over /science/sensor_data.
#pragma once

#include <rover_hmi_core/gui_module.h>

#include <QLabel>
#include <QPushButton>
#include <QPlainTextEdit>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QCheckBox>
#include <QComboBox>
#include <vector>

class SpectrumView;
class PythonRunner;
class CurveBuilder;

class SpectrometerModule : public rover_hmi_core::GuiModule {
public:
    std::string name()           const override { return "Spectrometer"; }
    std::string sectionName()    const override { return "Science"; }
    std::string layoutHint()     const override { return "main"; }
    bool        defaultVisible() const override { return false; }

    QWidget* createWidget(QWidget* parent) override;
    void     stop() override;

private:
    QWidget* buildControls(QWidget* parent);

    void doCapture();
    void doLoadCsv();
    void doSaveCsv();
    void doOverlayCsv();
    void doClear();

    void log(const QString& text);
    void setBusy(bool busy);
    // Pulls "Absorbance = x" / "Predicted = y unit" out of the script output.
    void showPrediction(const QString& output);

    SpectrumView*   view_        = nullptr;
    CurveBuilder*   curve_       = nullptr;
    PythonRunner*   runner_      = nullptr;
    QPlainTextEdit* log_         = nullptr;
    QLabel*         status_      = nullptr;
    QLabel*         result_      = nullptr;

    QPushButton*    capture_btn_ = nullptr;
    QPushButton*    load_btn_    = nullptr;
    QPushButton*    save_btn_    = nullptr;
    QPushButton*    overlay_btn_ = nullptr;
    QPushButton*    clear_btn_   = nullptr;

    QSpinBox*       frames_      = nullptr;
    QComboBox*      camera_      = nullptr;
    QSpinBox*       device_      = nullptr;
    QDoubleSpinBox* marker_      = nullptr;
    QCheckBox*      peaks_       = nullptr;
    QCheckBox*      fill_        = nullptr;

    // Spectrum currently shown, kept so it can be saved.
    std::vector<double> wavelengths_;
    std::vector<double> intensities_;
    QString             last_capture_path_;
};
