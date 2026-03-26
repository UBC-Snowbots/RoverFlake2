#include "motor_status_module.h"
#include "catppuccin.h"

#include <QGridLayout>
#include <QFont>

static const char* JOINT_NAMES[] = {
    "Base", "Shoulder", "Elbow", "Wrist Pitch", "Wrist Roll", "End Effector"
};

static const char* FIELD_HEADERS[] = {
    "Mode", "Fault", "Position (rev)", "Velocity (rev/s)",
    "Torque (Nm)", "Voltage (V)", "Temp (C)"
};

QWidget* MotorStatusModule::createWidget(QWidget* parent) {
    auto* widget = new QWidget(parent);
    auto* grid = new QGridLayout(widget);
    grid->setSpacing(2);

    QFont mono("monospace", theme::FontSize);
    QFont monoBold("monospace", theme::FontSize, QFont::Bold);

    auto headerStyle = QString("background: %1; color: %2; padding: 8px;")
        .arg(theme::HeaderBg).arg(theme::Text);
    auto cellStyle = QString("background: %1; color: %2; padding: 8px;")
        .arg(theme::Bg).arg(theme::Text);
    auto jointStyle = QString("background: %1; color: %2; padding: 8px;")
        .arg(theme::Bg).arg(theme::MotorColors[0]);

    // Header
    auto* corner = new QLabel("Joint");
    corner->setFont(monoBold);
    corner->setStyleSheet(headerStyle);
    corner->setAlignment(Qt::AlignCenter);
    grid->addWidget(corner, 0, 0);

    for (int c = 0; c < NUM_FIELDS; c++) {
        auto* lbl = new QLabel(FIELD_HEADERS[c]);
        lbl->setFont(monoBold);
        lbl->setStyleSheet(headerStyle);
        lbl->setAlignment(Qt::AlignCenter);
        grid->addWidget(lbl, 0, c + 1);
    }

    // Data rows
    for (int r = 0; r < NUM_MOTORS; r++) {
        auto* jname = new QLabel(JOINT_NAMES[r]);
        jname->setFont(monoBold);
        jname->setStyleSheet(
            QString("background: %1; color: %2; padding: 8px;")
            .arg(theme::Bg).arg(theme::MotorColors[r]));
        grid->addWidget(jname, r + 1, 0);

        for (int c = 0; c < NUM_FIELDS; c++) {
            auto* lbl = new QLabel("--");
            lbl->setFont(mono);
            lbl->setStyleSheet(cellStyle);
            lbl->setAlignment(Qt::AlignCenter);
            grid->addWidget(lbl, r + 1, c + 1);
            cells_[r][c] = lbl;
        }
    }

    // Status bar
    status_ = new QLabel("Waiting for data...");
    status_->setFont(mono);
    status_->setStyleSheet(
        QString("background: %1; color: %2; padding: 8px;")
        .arg(theme::Bg).arg(theme::TextDim));
    grid->addWidget(status_, NUM_MOTORS + 1, 0, 1, NUM_FIELDS + 1);

    for (int c = 0; c <= NUM_FIELDS; c++)
        grid->setColumnStretch(c, 1);

    return widget;
}

void MotorStatusModule::setDataBus(MoteusDataBus* bus) {
    QObject::connect(bus, &MoteusDataBus::telemetryUpdated,
                     [this](const std::array<MotorState, NUM_MOTORS>& s) {
                         onTelemetry(s);
                     });
}

void MotorStatusModule::onTelemetry(const std::array<MotorState, NUM_MOTORS>& states) {
    auto faultStyle = QString("background: %1; color: %2; padding: 8px; font-weight: bold;")
        .arg(theme::Bg).arg(theme::Red);
    auto okStyle = QString("background: %1; color: %2; padding: 8px;")
        .arg(theme::Bg).arg(theme::Green);
    auto cellStyle = QString("background: %1; color: %2; padding: 8px;")
        .arg(theme::Bg).arg(theme::Text);

    int active = 0;
    for (const auto& s : states) {
        if (s.id < 1 || s.id > NUM_MOTORS) continue;
        active++;
        int r = s.id - 1;

        cells_[r][0]->setText(QString::number(s.mode));
        cells_[r][1]->setText(QString::number(s.fault));
        cells_[r][2]->setText(QString::number(s.position, 'f', 3));
        cells_[r][3]->setText(QString::number(s.velocity, 'f', 3));
        cells_[r][4]->setText(QString::number(s.torque, 'f', 3));
        cells_[r][5]->setText(QString::number(s.voltage, 'f', 1));
        cells_[r][6]->setText(QString::number(s.temperature, 'f', 1));

        cells_[r][1]->setStyleSheet(s.fault != 0 ? faultStyle : okStyle);

        for (int c = 0; c < NUM_FIELDS; c++) {
            if (c != 1) cells_[r][c]->setStyleSheet(cellStyle);
        }
    }

    status_->setText(QString("Receiving from %1 motors").arg(active));
}
