#include <rover_hmi_core/science/python_runner.h>

PythonRunner::PythonRunner(QObject* parent) : QObject(parent)
{
    proc_.setProcessChannelMode(QProcess::MergedChannels);

    connect(&proc_, &QProcess::readyRead, this, [this]() {
        const QString chunk = QString::fromLocal8Bit(proc_.readAll());
        collected_ += chunk;
        if (on_output_) on_output_(chunk);
    });

    connect(&proc_, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
            this, [this](int code, QProcess::ExitStatus status) {
        const bool ok = (status == QProcess::NormalExit && code == 0);
        if (!ok) error_ = QStringLiteral("exit code %1").arg(code);
        if (on_finished_) on_finished_(ok, collected_);
    });

    connect(&proc_, &QProcess::errorOccurred, this,
            [this](QProcess::ProcessError e) {
        if (e == QProcess::FailedToStart) {
            error_ = QStringLiteral("python3 failed to start — is it installed?");
            if (on_finished_) on_finished_(false, collected_);
        }
    });
}

bool PythonRunner::busy() const
{
    return proc_.state() != QProcess::NotRunning;
}

bool PythonRunner::run(const QString& script_path,
                       const QStringList& args,
                       const QString& working_dir)
{
    if (busy()) {
        error_ = QStringLiteral("a run is already in progress");
        return false;
    }
    if (script_path.isEmpty()) {
        error_ = QStringLiteral("script not found");
        return false;
    }

    collected_.clear();
    error_.clear();
    if (!working_dir.isEmpty()) proc_.setWorkingDirectory(working_dir);

    QStringList full_args;
    full_args << script_path << args;
    // -u so output reaches the log pane as it happens, not at exit.
    proc_.start(QStringLiteral("python3"), QStringList{"-u"} + full_args);
    return true;
}

void PythonRunner::cancel()
{
    if (!busy()) return;
    proc_.terminate();
    if (!proc_.waitForFinished(2000)) proc_.kill();
}

bool PythonRunner::checkEnvironment(QString& error)
{
    QProcess p;
    p.start(QStringLiteral("python3"),
            {"-c", "import numpy, pandas, scipy, matplotlib"});
    if (!p.waitForStarted(3000)) {
        error = QStringLiteral("python3 not found");
        return false;
    }
    if (!p.waitForFinished(20000)) {
        p.kill();
        error = QStringLiteral("python3 import check timed out");
        return false;
    }
    if (p.exitCode() != 0) {
        const QString detail = QString::fromLocal8Bit(p.readAllStandardError())
                                   .trimmed().section('\n', -1);
        error = QStringLiteral("missing Python deps: %1 "
                               "(pip3 install -r scripts/spectrometer/requirements.txt)")
                    .arg(detail);
        return false;
    }
    return true;
}
