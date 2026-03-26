#include "motor_config_module.h"
#include "catppuccin.h"

#include <QGridLayout>
#include <QFont>

static const char* JOINT_NAMES[] = {
    "Base", "Shoulder", "Elbow", "Wrist Pitch", "Wrist Roll", "End Effector"
};

static const char* FIELD_HEADERS[] = {
    "Kp", "Kd", "Max I (A)", "Max Vel (rev/s)",
    "Pos Min (rev)", "Pos Max (rev)", "Max V (V)", "Gear Ratio"
};

QWidget* MotorConfigModule::createWidget(QWidget* parent) {
    auto* widget = new QWidget(parent);
    auto* grid = new QGridLayout(widget);
    grid->setSpacing(2);

    QFont mono("monospace", theme::FontSize);
    QFont monoBold("monospace", theme::FontSize, QFont::Bold);

    auto headerStyle = QString("background: %1; color: %2; padding: 8px;")
        .arg(theme::HeaderBg).arg(theme::Text);
    auto cellStyle = QString("background: %1; color: %2; padding: 8px;")
        .arg(theme::Bg).arg(theme::Text);

    // Header row
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
    status_ = new QLabel("Waiting for config from driver...");
    status_->setFont(mono);
    status_->setStyleSheet(
        QString("background: %1; color: %2; padding: 8px;")
        .arg(theme::Bg).arg(theme::TextDim));
    grid->addWidget(status_, NUM_MOTORS + 1, 0, 1, NUM_FIELDS + 1);

    for (int c = 0; c <= NUM_FIELDS; c++)
        grid->setColumnStretch(c, 1);

    return widget;
}

void MotorConfigModule::setDataBus(MoteusDataBus* bus) {
    QObject::connect(bus, &MoteusDataBus::configUpdated,
                     [this](const std::array<MotorConfigInfo, NUM_MOTORS>& c) {
                         onConfig(c);
                     });
}

void MotorConfigModule::onConfig(const std::array<MotorConfigInfo, NUM_MOTORS>& configs) {
    auto cellStyle = QString("background: %1; color: %2; padding: 8px;")
        .arg(theme::Bg).arg(theme::Text);

    for (int r = 0; r < NUM_MOTORS; r++) {
        const auto& c = configs[r];
        cells_[r][0]->setText(QString::number(c.kp, 'f', 0));
        cells_[r][1]->setText(QString::number(c.kd, 'f', 0));
        cells_[r][2]->setText(QString::number(c.max_current, 'f', 1));
        cells_[r][3]->setText(QString::number(c.max_velocity, 'f', 3));
        cells_[r][4]->setText(QString::number(c.position_min, 'f', 2));
        cells_[r][5]->setText(QString::number(c.position_max, 'f', 2));
        cells_[r][6]->setText(QString::number(c.max_voltage, 'f', 1));
        cells_[r][7]->setText(c.gear_reduction > 0
            ? QString("1/%1").arg(qRound(1.0f / c.gear_reduction))
            : "--");

        for (int col = 0; col < NUM_FIELDS; col++)
            cells_[r][col]->setStyleSheet(cellStyle);
    }

    status_->setText("Config received from driver");
    status_->setStyleSheet(
        QString("background: %1; color: %2; padding: 8px;")
        .arg(theme::Bg).arg(theme::Green));
}
