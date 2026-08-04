// curve_builder.h — standard-curve panel.
//
// Drives `curve_gen.py build` from a list of known standards and
// `curve_gen.py predict` for unknowns, showing the fit and the plots the
// script writes.
#pragma once

#include <QWidget>
#include <QLabel>
#include <QPushButton>
#include <QTableWidget>
#include <QDoubleSpinBox>
#include <QLineEdit>
#include <QPlainTextEdit>
#include <functional>

class ImagePane;
class PythonRunner;

class CurveBuilder : public QWidget {
public:
    explicit CurveBuilder(QWidget* parent = nullptr);

    // Log lines are forwarded to the module's shared log pane.
    void setLogHandler(std::function<void(const QString&)> h) { log_ = std::move(h); }

    // Path of the model most recently built, so capture can preselect it.
    QString lastModelPath() const { return last_model_; }

private:
    void addStandard();
    void removeStandard();
    void chooseBlank();
    void build();
    void predict();

    void log(const QString& text);
    void setBusy(bool busy);
    // Pulls "Best fit : X   R² = Y" and the equation out of the script output.
    void showFit(const QString& output);

    PythonRunner*   runner_    = nullptr;
    QTableWidget*   standards_ = nullptr;
    QLineEdit*      blank_     = nullptr;
    QLineEdit*      name_      = nullptr;
    QLineEdit*      unit_      = nullptr;
    QDoubleSpinBox* wavelength_ = nullptr;

    QPushButton* add_btn_     = nullptr;
    QPushButton* remove_btn_  = nullptr;
    QPushButton* blank_btn_   = nullptr;
    QPushButton* build_btn_   = nullptr;
    QPushButton* predict_btn_ = nullptr;

    QLabel*    fit_    = nullptr;
    QLabel*    status_ = nullptr;
    ImagePane* curve_plot_   = nullptr;
    ImagePane* spectra_plot_ = nullptr;

    QString last_model_;
    QString out_curve_png_;
    QString out_spectra_png_;

    std::function<void(const QString&)> log_;
};
