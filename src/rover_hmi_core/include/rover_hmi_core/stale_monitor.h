// Arrival-time staleness watchdog: call stamp() in your subscription callback,
// attach() once at widget creation. on_change(true) fires when data stops
// arriving (only after data has flowed at least once), on_change(false) when
// it resumes. Steady clock; sender stamps are never consulted.
#pragma once
#include <QElapsedTimer>
#include <QTimer>
#include <QWidget>
#include <functional>

namespace rover_hmi_core {

class StaleMonitor {
public:
    void stamp() {
        if (!clock_.isValid()) clock_.start();
        last_ms_ = clock_.elapsed();
        seen_ = true;
    }
    void attach(QWidget* parent, int threshold_ms, std::function<void(bool)> on_change) {
        if (!clock_.isValid()) clock_.start();
        auto* t = new QTimer(parent);
        QObject::connect(t, &QTimer::timeout, [this, threshold_ms, cb = std::move(on_change)]() {
            const bool stale = seen_ && (clock_.elapsed() - last_ms_ > threshold_ms);
            if (stale != stale_) { stale_ = stale; cb(stale); }
        });
        t->start(500);
    }
private:
    QElapsedTimer clock_;
    qint64 last_ms_ = 0;
    bool seen_ = false, stale_ = false;
};

}  // namespace rover_hmi_core
