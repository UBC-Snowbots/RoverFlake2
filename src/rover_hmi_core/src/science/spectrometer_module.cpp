#include <rover_hmi_core/science/spectrometer_module.h>
#include <rover_hmi_core/science/spectrum_view.h>
#include <rover_hmi_core/science/python_runner.h>
#include <rover_hmi_core/science/curve_builder.h>
#include <rover_hmi_core/spectro_paths.h>
#include <rover_hmi_core/catppuccin.h>

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QFileDialog>
#include <QFileInfo>
#include <QDateTime>
#include <QRegularExpression>
#include <QGroupBox>
#include <QTabWidget>

#include <pluginlib/class_list_macros.hpp>

namespace sp = rover_hmi_core::spectro_paths;

namespace {

QLabel* makeHeader(const QString& text, const char* color)
{
    auto* l = new QLabel(text);
    l->setStyleSheet(QString("color:%1; font-weight:bold; font-size:%2px;")
                         .arg(color).arg(theme::FontSize));
    return l;
}

}  // namespace

QWidget* SpectrometerModule::createWidget(QWidget* parent)
{
    auto* root_w = new QWidget(parent);
    auto* root   = new QVBoxLayout(root_w);
    root->setContentsMargins(8, 8, 8, 8);
    root->setSpacing(8);

    runner_ = new PythonRunner(root_w);

    auto* tabs = new QTabWidget(root_w);

    // Spectrum tab: the live/loaded trace plus its controls.
    auto* spectrum_tab = new QWidget(tabs);
    auto* spectrum_lay = new QVBoxLayout(spectrum_tab);
    spectrum_lay->setContentsMargins(6, 6, 6, 6);
    spectrum_lay->setSpacing(6);
    view_ = new SpectrumView(spectrum_tab);
    spectrum_lay->addWidget(view_, 1);
    spectrum_lay->addWidget(buildControls(spectrum_tab));
    tabs->addTab(spectrum_tab, "Spectrum");

    // Standard curve tab: build a model, predict unknowns, view the plots.
    curve_ = new CurveBuilder(tabs);
    curve_->setLogHandler([this](const QString& t) { log(t); });
    tabs->addTab(curve_, "Standard Curve");

    root->addWidget(tabs, 1);

    log_ = new QPlainTextEdit(root_w);
    log_->setReadOnly(true);
    log_->setMaximumHeight(120);
    log_->setStyleSheet(QString("background:%1; color:%2; border:1px solid %3;"
                                "border-radius:6px; font-size:%4px;")
                            .arg(theme::BgPanel, theme::TextDim,
                                 theme::BorderDim)
                            .arg(theme::FontSize));
    root->addWidget(log_);

    // Report an unusable environment up front rather than on first click.
    QString env_err;
    if (!PythonRunner::checkEnvironment(env_err)) {
        status_->setText(env_err);
        status_->setStyleSheet(QString("color:%1;").arg(theme::Red));
        capture_btn_->setEnabled(false);
        log(QStringLiteral("environment check failed: %1").arg(env_err));
    } else if (sp::scriptsDir().isEmpty()) {
        const QString why = sp::lastError();
        status_->setText(why);
        status_->setStyleSheet(QString("color:%1;").arg(theme::Red));
        capture_btn_->setEnabled(false);
        log(QStringLiteral("scripts unavailable: %1").arg(why));
    } else {
        log(QStringLiteral("scripts: %1").arg(sp::scriptsDir()));
        log(QStringLiteral("data:    %1").arg(sp::dataDir()));
    }

    return root_w;
}

QWidget* SpectrometerModule::buildControls(QWidget* parent)
{
    auto* box  = new QWidget(parent);
    auto* grid = new QGridLayout(box);
    grid->setContentsMargins(0, 0, 0, 0);
    grid->setSpacing(6);

    int row = 0;

    grid->addWidget(makeHeader("CAPTURE", theme::Cyan), row, 0, 1, 6);
    row++;

    capture_btn_ = new QPushButton("Capture Spectrum");
    load_btn_    = new QPushButton("Load CSV");
    overlay_btn_ = new QPushButton("Overlay CSV");
    save_btn_    = new QPushButton("Save CSV");
    clear_btn_   = new QPushButton("Clear");

    camera_ = new QComboBox();
    camera_->addItems({"Pi camera", "USB camera"});

    frames_ = new QSpinBox();
    frames_->setRange(1, 100);
    frames_->setValue(10);
    frames_->setPrefix("frames ");

    device_ = new QSpinBox();
    device_->setRange(0, 9);
    device_->setPrefix("dev ");
    device_->setEnabled(false);

    QObject::connect(camera_, &QComboBox::currentTextChanged, box,
                     [this](const QString& t) {
                         device_->setEnabled(t.startsWith("USB"));
                     });

    grid->addWidget(capture_btn_, row, 0);
    grid->addWidget(camera_,      row, 1);
    grid->addWidget(frames_,      row, 2);
    grid->addWidget(device_,      row, 3);
    grid->addWidget(load_btn_,    row, 4);
    grid->addWidget(save_btn_,    row, 5);
    row++;

    grid->addWidget(makeHeader("VIEW", theme::Cyan), row, 0, 1, 6);
    row++;

    marker_ = new QDoubleSpinBox();
    marker_->setRange(0.0, 1200.0);
    marker_->setDecimals(1);
    marker_->setValue(0.0);
    marker_->setPrefix("λ ");
    marker_->setSuffix(" nm");
    marker_->setToolTip("Assay wavelength marker — 0 hides it");

    peaks_ = new QCheckBox("peaks");
    peaks_->setChecked(true);
    fill_  = new QCheckBox("spectrum fill");
    fill_->setChecked(true);

    grid->addWidget(marker_,      row, 0);
    grid->addWidget(peaks_,       row, 1);
    grid->addWidget(fill_,        row, 2);
    grid->addWidget(overlay_btn_, row, 3);
    grid->addWidget(clear_btn_,   row, 4);
    row++;

    result_ = new QLabel("—");
    result_->setStyleSheet(QString("color:%1; font-weight:bold; font-size:%2px;")
                               .arg(theme::Green).arg(theme::FontSizeLg));
    grid->addWidget(result_, row, 0, 1, 6);
    row++;

    status_ = new QLabel("ready");
    status_->setStyleSheet(QString("color:%1;").arg(theme::TextDim));
    grid->addWidget(status_, row, 0, 1, 6);

    QObject::connect(capture_btn_, &QPushButton::clicked, box, [this] { doCapture(); });
    QObject::connect(load_btn_,    &QPushButton::clicked, box, [this] { doLoadCsv(); });
    QObject::connect(overlay_btn_, &QPushButton::clicked, box, [this] { doOverlayCsv(); });
    QObject::connect(save_btn_,    &QPushButton::clicked, box, [this] { doSaveCsv(); });
    QObject::connect(clear_btn_,   &QPushButton::clicked, box, [this] { doClear(); });

    QObject::connect(marker_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                     box, [this](double nm) {
                         view_->setMarkerWavelength(nm > 0.0 ? nm : -1.0);
                     });
    QObject::connect(peaks_, &QCheckBox::toggled, box,
                     [this](bool on) { view_->setPeaksVisible(on); });
    QObject::connect(fill_, &QCheckBox::toggled, box,
                     [this](bool on) { view_->setFillVisible(on); });

    return box;
}

// ── actions ─────────────────────────────────────────────────────────────────

void SpectrometerModule::doCapture()
{
    const QString script = sp::script("capture_and_predict.py");
    if (script.isEmpty()) {
        log(QStringLiteral("cannot capture: %1").arg(sp::lastError()));
        return;
    }

    const QString data_dir = sp::dataDir();
    if (data_dir.isEmpty()) {
        log(QStringLiteral("cannot capture: %1").arg(sp::lastError()));
        return;
    }

    // capture_and_predict.py needs a trained model; without one, tell the
    // operator what is missing instead of letting argparse fail.
    const QString suggested = (curve_ && !curve_->lastModelPath().isEmpty())
        ? curve_->lastModelPath() : data_dir;
    const QString model = QFileDialog::getOpenFileName(
        nullptr, "Select curve model JSON", suggested, "Model (*.json)");
    if (model.isEmpty()) {
        log(QStringLiteral("capture cancelled — no model selected"));
        return;
    }

    const QString stamp = QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss");
    last_capture_path_  = QStringLiteral("%1/capture_%2.csv").arg(data_dir, stamp);

    QStringList args;
    args << "--model"  << model
         << "--output" << last_capture_path_
         << "--frames" << QString::number(frames_->value());
    if (camera_->currentText().startsWith("USB"))
        args << "--usb" << "--device" << QString::number(device_->value());

    runner_->setOutputHandler([this](const QString& chunk) { log(chunk.trimmed()); });
    runner_->setFinishedHandler([this](bool ok, const QString& output) {
        setBusy(false);
        if (!ok) {
            status_->setText(QStringLiteral("capture failed — see log"));
            status_->setStyleSheet(QString("color:%1;").arg(theme::Red));
            return;
        }
        status_->setText(QStringLiteral("capture complete"));
        status_->setStyleSheet(QString("color:%1;").arg(theme::TextDim));
        showPrediction(output);

        std::vector<double> wl, iv;
        QString err;
        if (SpectrumView::loadCsv(last_capture_path_, wl, iv, err)) {
            wavelengths_ = wl;
            intensities_ = iv;
            view_->setSpectrum(QFileInfo(last_capture_path_).fileName(), wl, iv);
        } else {
            log(QStringLiteral("captured, but CSV unreadable: %1").arg(err));
        }
    });

    if (!runner_->run(script, args, data_dir)) {
        log(QStringLiteral("cannot start: %1").arg(runner_->error()));
        return;
    }
    setBusy(true);
    status_->setText(QStringLiteral("capturing %1 frame(s)…").arg(frames_->value()));
    log(QStringLiteral("→ capture_and_predict.py %1").arg(args.join(' ')));
}

void SpectrometerModule::doLoadCsv()
{
    const QString start = sp::dataDir();
    const QString path  = QFileDialog::getOpenFileName(
        nullptr, "Load spectrum CSV", start, "Spectrum (*.csv)");
    if (path.isEmpty()) return;

    std::vector<double> wl, iv;
    QString err;
    if (!SpectrumView::loadCsv(path, wl, iv, err)) {
        log(QStringLiteral("load failed: %1").arg(err));
        status_->setText(err);
        status_->setStyleSheet(QString("color:%1;").arg(theme::Red));
        return;
    }

    wavelengths_ = wl;
    intensities_ = iv;
    view_->setSpectrum(QFileInfo(path).fileName(), wl, iv);

    const QString warn = view_->qualityWarning();
    if (warn.isEmpty()) {
        status_->setText(QStringLiteral("loaded %1 points").arg(wl.size()));
        status_->setStyleSheet(QString("color:%1;").arg(theme::TextDim));
    } else {
        status_->setText(warn);
        status_->setStyleSheet(QString("color:%1;").arg(theme::Yellow));
        log(QStringLiteral("warning: %1").arg(warn));
    }
    log(QStringLiteral("loaded %1 (%2 points, %3–%4 nm)")
            .arg(QFileInfo(path).fileName())
            .arg(wl.size())
            .arg(wl.front(), 0, 'f', 1)
            .arg(wl.back(), 0, 'f', 1));
}

void SpectrometerModule::doOverlayCsv()
{
    const QString path = QFileDialog::getOpenFileName(
        nullptr, "Overlay spectrum CSV", sp::dataDir(), "Spectrum (*.csv)");
    if (path.isEmpty()) return;

    std::vector<double> wl, iv;
    QString err;
    if (!SpectrumView::loadCsv(path, wl, iv, err)) {
        log(QStringLiteral("overlay failed: %1").arg(err));
        return;
    }
    view_->addTrace(QFileInfo(path).fileName(), wl, iv);
    log(QStringLiteral("overlaid %1").arg(QFileInfo(path).fileName()));
}

void SpectrometerModule::doSaveCsv()
{
    if (wavelengths_.empty()) {
        log(QStringLiteral("nothing to save — capture or load first"));
        return;
    }

    const QString suggested = QStringLiteral("%1/spectrum_%2.csv")
        .arg(sp::dataDir(),
             QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss"));
    const QString path = QFileDialog::getSaveFileName(
        nullptr, "Save spectrum CSV", suggested, "Spectrum (*.csv)");
    if (path.isEmpty()) return;

    QString err;
    if (!SpectrumView::saveCsv(path, wavelengths_, intensities_, err)) {
        log(QStringLiteral("save failed: %1").arg(err));
        return;
    }
    log(QStringLiteral("saved %1").arg(path));
    status_->setText(QStringLiteral("saved"));
}

void SpectrometerModule::doClear()
{
    wavelengths_.clear();
    intensities_.clear();
    view_->clearTraces();
    result_->setText("—");
    status_->setText("ready");
    status_->setStyleSheet(QString("color:%1;").arg(theme::TextDim));
}

// ── helpers ─────────────────────────────────────────────────────────────────

void SpectrometerModule::showPrediction(const QString& output)
{
    static const QRegularExpression re_abs(R"(Absorbance\s*=\s*([-\d.]+))");
    static const QRegularExpression re_pred(R"(Predicted\s*=\s*([-\d.]+)\s*(\S*))");

    const auto m_abs  = re_abs.match(output);
    const auto m_pred = re_pred.match(output);
    if (!m_pred.hasMatch()) return;

    QString text = QStringLiteral("%1 %2")
        .arg(m_pred.captured(1), m_pred.captured(2));
    if (m_abs.hasMatch())
        text += QStringLiteral("    (A = %1)").arg(m_abs.captured(1));
    result_->setText(text);
}

void SpectrometerModule::log(const QString& text)
{
    if (!log_ || text.isEmpty()) return;
    log_->appendPlainText(text);
}

void SpectrometerModule::setBusy(bool busy)
{
    capture_btn_->setEnabled(!busy);
    capture_btn_->setText(busy ? "Capturing…" : "Capture Spectrum");
}

void SpectrometerModule::stop()
{
    if (runner_) runner_->cancel();
}

PLUGINLIB_EXPORT_CLASS(SpectrometerModule, rover_hmi_core::GuiModule)
