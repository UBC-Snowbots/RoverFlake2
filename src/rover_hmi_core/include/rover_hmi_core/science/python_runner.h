// python_runner.h — runs a vendored spectrometer script without blocking the UI.
#pragma once

#include <QObject>
#include <QProcess>
#include <QString>
#include <QStringList>
#include <functional>

class PythonRunner : public QObject {
    Q_OBJECT
public:
    explicit PythonRunner(QObject* parent = nullptr);

    // Called with each chunk of output as it arrives (stdout and stderr merged).
    void setOutputHandler(std::function<void(const QString&)> h) { on_output_ = std::move(h); }

    // Called once when the run ends. ok is false on non-zero exit or launch
    // failure; output is everything collected.
    void setFinishedHandler(std::function<void(bool ok, const QString& output)> h)
    { on_finished_ = std::move(h); }

    // Starts script_path with args in working_dir. Returns false if a run is
    // already in flight, with the reason in error().
    bool run(const QString& script_path,
             const QStringList& args,
             const QString& working_dir);

    bool    busy() const;
    void    cancel();
    QString error() const { return error_; }

    // True when python3 and the modules the scripts need are importable.
    // Returns the failure reason in error on false.
    static bool checkEnvironment(QString& error);

private:
    QProcess proc_;
    QString  collected_;
    QString  error_;
    std::function<void(const QString&)>            on_output_;
    std::function<void(bool, const QString&)>      on_finished_;
};
