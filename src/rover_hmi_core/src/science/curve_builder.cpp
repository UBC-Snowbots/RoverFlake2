#include <rover_hmi_core/science/curve_builder.h>
#include <rover_hmi_core/science/image_pane.h>
#include <rover_hmi_core/science/python_runner.h>
#include <rover_hmi_core/spectro_paths.h>
#include <rover_hmi_core/catppuccin.h>

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QSplitter>
#include <QHeaderView>
#include <QFileDialog>
#include <QFileInfo>
#include <QInputDialog>
#include <QRegularExpression>

namespace sp = rover_hmi_core::spectro_paths;

namespace {

QLabel* header(const QString& text)
{
    auto* l = new QLabel(text);
    l->setStyleSheet(QString("color:%1; font-weight:bold; font-size:%2px;")
                         .arg(theme::Cyan).arg(theme::FontSize));
    return l;
}

}  // namespace

CurveBuilder::CurveBuilder(QWidget* parent) : QWidget(parent)
{
    runner_ = new PythonRunner(this);

    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(0, 0, 0, 0);
    root->setSpacing(6);

    root->addWidget(header("STANDARDS"));

    standards_ = new QTableWidget(0, 2, this);
    standards_->setHorizontalHeaderLabels({"Concentration", "Spectrum CSV"});
    standards_->horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
    standards_->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Stretch);
    standards_->verticalHeader()->setVisible(false);
    standards_->setSelectionBehavior(QAbstractItemView::SelectRows);
    standards_->setMaximumHeight(150);
    standards_->setStyleSheet(
        QString("QTableWidget{background:%1; color:%2; border:1px solid %3;"
                "gridline-color:%3;} QHeaderView::section{background:%4;"
                "color:%5; border:none; padding:4px;}")
            .arg(theme::BgPanel, theme::Text, theme::BorderDim,
                 theme::HeaderBg, theme::TextDim));
    root->addWidget(standards_);

    auto* row1 = new QHBoxLayout();
    add_btn_    = new QPushButton("Add standard");
    remove_btn_ = new QPushButton("Remove");
    blank_btn_  = new QPushButton("Blank…");
    blank_      = new QLineEdit();
    blank_->setPlaceholderText("blank CSV (required)");
    blank_->setReadOnly(true);
    row1->addWidget(add_btn_);
    row1->addWidget(remove_btn_);
    row1->addWidget(blank_btn_);
    row1->addWidget(blank_, 1);
    root->addLayout(row1);

    auto* row2 = new QHBoxLayout();
    wavelength_ = new QDoubleSpinBox();
    wavelength_->setRange(200.0, 1200.0);
    wavelength_->setDecimals(1);
    wavelength_->setValue(525.0);
    wavelength_->setPrefix("λ ");
    wavelength_->setSuffix(" nm");

    unit_ = new QLineEdit("ppm");
    unit_->setMaximumWidth(90);
    unit_->setToolTip("Concentration unit label");

    name_ = new QLineEdit("Life Detection Assay");
    name_->setToolTip("Assay name used in plot titles");

    build_btn_   = new QPushButton("Build Curve");
    predict_btn_ = new QPushButton("Predict Unknown…");

    row2->addWidget(wavelength_);
    row2->addWidget(unit_);
    row2->addWidget(name_, 1);
    row2->addWidget(build_btn_);
    row2->addWidget(predict_btn_);
    root->addLayout(row2);

    fit_ = new QLabel("—");
    fit_->setStyleSheet(QString("color:%1; font-weight:bold; font-size:%2px;")
                            .arg(theme::Green).arg(theme::FontSizeLg));
    root->addWidget(fit_);

    status_ = new QLabel("add standards, pick a blank, then build");
    status_->setStyleSheet(QString("color:%1;").arg(theme::TextDim));
    root->addWidget(status_);

    auto* plots = new QSplitter(Qt::Horizontal, this);
    curve_plot_   = new ImagePane("standard curve", plots);
    spectra_plot_ = new ImagePane("spectra overlay", plots);
    plots->addWidget(curve_plot_);
    plots->addWidget(spectra_plot_);
    root->addWidget(plots, 1);

    connect(add_btn_,    &QPushButton::clicked, this, [this] { addStandard(); });
    connect(remove_btn_, &QPushButton::clicked, this, [this] { removeStandard(); });
    connect(blank_btn_,  &QPushButton::clicked, this, [this] { chooseBlank(); });
    connect(build_btn_,  &QPushButton::clicked, this, [this] { build(); });
    connect(predict_btn_,&QPushButton::clicked, this, [this] { predict(); });
}

// ── standards table ─────────────────────────────────────────────────────────

void CurveBuilder::addStandard()
{
    const QStringList paths = QFileDialog::getOpenFileNames(
        this, "Select standard spectrum CSV(s)", sp::dataDir(), "Spectrum (*.csv)");
    if (paths.isEmpty()) return;

    for (const QString& path : paths) {
        bool ok = false;
        const double conc = QInputDialog::getDouble(
            this, "Concentration",
            QStringLiteral("Concentration for %1 (%2):")
                .arg(QFileInfo(path).fileName(), unit_->text()),
            0.0, -1e9, 1e9, 4, &ok);
        if (!ok) continue;

        const int row = standards_->rowCount();
        standards_->insertRow(row);
        standards_->setItem(row, 0, new QTableWidgetItem(QString::number(conc)));
        standards_->setItem(row, 1, new QTableWidgetItem(path));
    }
}

void CurveBuilder::removeStandard()
{
    const auto rows = standards_->selectionModel()->selectedRows();
    // Descending so earlier removals do not shift later indices.
    QList<int> indices;
    for (const auto& r : rows) indices << r.row();
    std::sort(indices.begin(), indices.end(), std::greater<int>());
    for (int r : indices) standards_->removeRow(r);
}

void CurveBuilder::chooseBlank()
{
    const QString path = QFileDialog::getOpenFileName(
        this, "Select blank CSV", sp::dataDir(), "Spectrum (*.csv)");
    if (!path.isEmpty()) blank_->setText(path);
}

// ── build / predict ─────────────────────────────────────────────────────────

void CurveBuilder::build()
{
    if (standards_->rowCount() < 2) {
        status_->setText("need at least 2 standards to fit a curve");
        status_->setStyleSheet(QString("color:%1;").arg(theme::Red));
        return;
    }
    if (blank_->text().isEmpty()) {
        status_->setText("pick a blank CSV first");
        status_->setStyleSheet(QString("color:%1;").arg(theme::Red));
        return;
    }

    const QString script = sp::script("curve_gen.py");
    const QString data_dir = sp::dataDir();
    if (script.isEmpty() || data_dir.isEmpty()) {
        log(QStringLiteral("cannot build: %1").arg(sp::lastError()));
        return;
    }

    // Name the model after the assay so several assays can coexist.
    QString slug = name_->text().toLower();
    slug.replace(QRegularExpression("[^a-z0-9]+"), "_");
    slug = slug.mid(0, 40);
    if (slug.isEmpty()) slug = "curve";

    last_model_      = QStringLiteral("%1/%2_model.json").arg(data_dir, slug);
    out_curve_png_   = QStringLiteral("%1/%2_standard_curve.png").arg(data_dir, slug);
    out_spectra_png_ = QStringLiteral("%1/%2_spectra_overlay.png").arg(data_dir, slug);

    QStringList args;
    args << "build"
         << "--blank"      << blank_->text()
         << "--wavelength" << QString::number(wavelength_->value())
         << "--unit"       << unit_->text()
         << "--name"       << name_->text()
         << "--model-out"  << last_model_
         << "--results-csv" << QStringLiteral("%1/%2_results.csv").arg(data_dir, slug)
         << "--plot-curve"   << out_curve_png_
         << "--plot-spectra" << out_spectra_png_
         << "--standards";
    for (int r = 0; r < standards_->rowCount(); r++) {
        args << QStringLiteral("%1,%2")
                    .arg(standards_->item(r, 0)->text(),
                         standards_->item(r, 1)->text());
    }

    runner_->setOutputHandler([this](const QString& c) { log(c.trimmed()); });
    runner_->setFinishedHandler([this](bool ok, const QString& output) {
        setBusy(false);
        if (!ok) {
            status_->setText("build failed — see log");
            status_->setStyleSheet(QString("color:%1;").arg(theme::Red));
            return;
        }
        showFit(output);
        curve_plot_->load(out_curve_png_);
        spectra_plot_->load(out_spectra_png_);
        status_->setText(QStringLiteral("model saved: %1")
                             .arg(QFileInfo(last_model_).fileName()));
        status_->setStyleSheet(QString("color:%1;").arg(theme::TextDim));
    });

    if (!runner_->run(script, args, data_dir)) {
        log(QStringLiteral("cannot start: %1").arg(runner_->error()));
        return;
    }
    setBusy(true);
    status_->setText("building curve…");
    log(QStringLiteral("→ curve_gen.py build (%1 standards)")
            .arg(standards_->rowCount()));
}

void CurveBuilder::predict()
{
    const QString model = last_model_.isEmpty()
        ? QFileDialog::getOpenFileName(this, "Select model JSON",
                                       sp::dataDir(), "Model (*.json)")
        : last_model_;
    if (model.isEmpty()) return;

    const QStringList samples = QFileDialog::getOpenFileNames(
        this, "Select unknown sample CSV(s)", sp::dataDir(), "Spectrum (*.csv)");
    if (samples.isEmpty()) return;

    const QString script   = sp::script("curve_gen.py");
    const QString data_dir = sp::dataDir();
    if (script.isEmpty()) {
        log(QStringLiteral("cannot predict: %1").arg(sp::lastError()));
        return;
    }

    QStringList args;
    args << "predict" << "--model" << model << "--sample" << samples;
    if (!blank_->text().isEmpty()) args << "--blank" << blank_->text();

    runner_->setOutputHandler([this](const QString& c) { log(c.trimmed()); });
    runner_->setFinishedHandler([this](bool ok, const QString& output) {
        setBusy(false);
        if (!ok) {
            status_->setText("predict failed — see log");
            status_->setStyleSheet(QString("color:%1;").arg(theme::Red));
            return;
        }
        static const QRegularExpression re(
            R"(Predicted\s*=\s*([-\d.]+)\s*(\S*))");
        auto it = re.globalMatch(output);
        QStringList results;
        while (it.hasNext()) {
            const auto m = it.next();
            results << QStringLiteral("%1 %2").arg(m.captured(1), m.captured(2));
        }
        if (!results.isEmpty()) fit_->setText(results.join("   |   "));
        status_->setText("prediction complete");
        status_->setStyleSheet(QString("color:%1;").arg(theme::TextDim));
    });

    if (!runner_->run(script, args, data_dir)) {
        log(QStringLiteral("cannot start: %1").arg(runner_->error()));
        return;
    }
    setBusy(true);
    status_->setText(QStringLiteral("predicting %1 sample(s)…").arg(samples.size()));
}

// ── helpers ─────────────────────────────────────────────────────────────────

void CurveBuilder::showFit(const QString& output)
{
    static const QRegularExpression re_fit(
        R"(Best fit\s*:\s*(\w+)\s+R²\s*=\s*([\d.]+))");
    static const QRegularExpression re_eq(R"(Equation\s*:\s*(A =[^\n]+))");

    const auto m_fit = re_fit.match(output);
    const auto m_eq  = re_eq.match(output);

    QString text;
    if (m_fit.hasMatch())
        text = QStringLiteral("%1   R² = %2")
                   .arg(m_fit.captured(1), m_fit.captured(2));
    if (m_eq.hasMatch())
        text += (text.isEmpty() ? "" : "\n") + m_eq.captured(1).trimmed();
    if (!text.isEmpty()) fit_->setText(text);

    // R² below 0.99 usually means a pipetting error or a drifting lamp.
    if (m_fit.hasMatch() && m_fit.captured(2).toDouble() < 0.99)
        log(QStringLiteral("note: R² %1 is below 0.99 — check standards and lamp")
                .arg(m_fit.captured(2)));
}

void CurveBuilder::log(const QString& text)
{
    if (log_ && !text.isEmpty()) log_(text);
}

void CurveBuilder::setBusy(bool busy)
{
    build_btn_->setEnabled(!busy);
    predict_btn_->setEnabled(!busy);
    build_btn_->setText(busy ? "Working…" : "Build Curve");
}
