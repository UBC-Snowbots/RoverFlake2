#include "science_sim_module.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QSizePolicy>
#include <algorithm>
#include <cmath>

#include <pluginlib/class_list_macros.hpp>
#include <rover_hmi_core/catppuccin.h>

static QString btnStyle(const char* bg, const char* fg, const char* border)
{
    return QString(
        "QPushButton{background:%1;color:%2;border:3px solid %3;"
        "border-radius:10px;padding:12px;font-size:%4px;font-weight:bold;}")
        .arg(bg, fg, border).arg(theme::FontSizeXl);
}

static const QString kOn    = btnStyle("#003322", theme::Green, theme::Green);
static const QString kOff   = btnStyle(theme::BgPanel, "#999999", theme::BorderDim);
static const QString kLoud  = btnStyle("#332200", theme::Yellow, theme::Yellow);

static QLabel* makeSectionLabel(const QString& title)
{
    auto* lbl = new QLabel(title.toUpper());
    lbl->setStyleSheet(QString(
        "color:#777777;font-size:%1px;font-weight:bold;"
        "letter-spacing:2px;padding-top:8px;").arg(theme::FontSizeSm));
    return lbl;
}

QWidget* ScienceSimModule::createWidget(QWidget* parent)
{
    auto* container = new QWidget(parent);
    container->setStyleSheet("background:#000000;");
    auto* root = new QVBoxLayout(container);
    root->setContentsMargins(10, 8, 10, 10);
    root->setSpacing(8);

    master_btn_ = new QPushButton();
    master_btn_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    master_btn_->setMinimumHeight(90);
    QObject::connect(master_btn_, &QPushButton::clicked, [this]() {
        sim_on_ = !sim_on_;
        if (sim_on_) timer_->start(100);
        else         timer_->stop();
        updateBtns();
    });
    root->addWidget(master_btn_);

    status_lbl_ = new QLabel();
    status_lbl_->setStyleSheet(QString("color:#777777;font-size:%1px;padding:2px 0;")
        .arg(theme::FontSizeSm));
    root->addWidget(status_lbl_);

    root->addWidget(makeSectionLabel("Drill (ultrasonic depth)"));
    auto* drillRow = new QHBoxLayout();
    drillRow->setSpacing(12);
    lower_btn_ = new QPushButton("Lower ▼");
    hold_btn_  = new QPushButton("Hold");
    raise_btn_ = new QPushButton("Raise ▲");
    depth_lbl_ = new QLabel("0.00 in");
    depth_lbl_->setStyleSheet(QString(
        "color:#ffffff;font-size:%1px;font-weight:bold;background:#0a0a0a;"
        "border:3px solid #333333;border-radius:10px;padding:12px;")
        .arg(theme::FontSizeXl));
    depth_lbl_->setAlignment(Qt::AlignCenter);
    QObject::connect(lower_btn_, &QPushButton::clicked, [this]() { drill_dir_ = 1;  updateBtns(); });
    QObject::connect(hold_btn_,  &QPushButton::clicked, [this]() { drill_dir_ = 0;  updateBtns(); });
    QObject::connect(raise_btn_, &QPushButton::clicked, [this]() { drill_dir_ = -1; updateBtns(); });
    for (auto* b : { lower_btn_, hold_btn_, raise_btn_ }) {
        b->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        b->setMinimumHeight(90);
    }
    drillRow->addWidget(lower_btn_);
    drillRow->addWidget(hold_btn_);
    drillRow->addWidget(raise_btn_);
    drillRow->addWidget(depth_lbl_);
    root->addLayout(drillRow);

    root->addWidget(makeSectionLabel("Sensors"));
    auto* grid = new QGridLayout();
    grid->setSpacing(12);
    struct ToggleDef { QPushButton** btn; bool* flag; const char* label; };
    const ToggleDef toggles[] = {
        { &sample_btn_,  &sample_,  "Sample Loaded" },
        { &spectro_btn_, &spectro_, "Spectro Ready" },
    };
    for (int i = 0; i < 2; i++) {
        auto* b = *toggles[i].btn = new QPushButton(toggles[i].label);
        auto* flag = toggles[i].flag;
        b->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        b->setMinimumHeight(150);
        QObject::connect(b, &QPushButton::clicked, [this, b, flag]() {
            *flag = !*flag;
            b->setStyleSheet(*flag ? kOn : kOff);
        });
        grid->addWidget(b, i / 2, i % 2);
    }
    grid->setRowStretch(0, 1);
    grid->setRowStretch(1, 1);
    root->addLayout(grid, 1);

    timer_ = new QTimer(container);
    QObject::connect(timer_, &QTimer::timeout, [this]() { onTick(); });

    updateBtns();
    return container;
}

void ScienceSimModule::setNode(rclcpp::Node::SharedPtr node)
{
    node_ = node;
    pub_ = node_->create_publisher<rover_msgs::msg::ScienceSensorData>(
        "/science/sensor_data",
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile());
}

void ScienceSimModule::onTick()
{
    tick_++;
    depth_in_ = std::clamp(depth_in_ + 0.1f * drill_dir_, 0.0f, 12.0f);
    depth_lbl_->setText(QString("%1 in").arg(static_cast<double>(depth_in_), 0, 'f', 2));

    auto jitter = [this](float amp, int phase) {
        return amp * std::sin(tick_ * 0.3f + phase);
    };

    rover_msgs::msg::ScienceSensorData msg;
    msg.ultrasonic_distance_in = depth_in_;
    msg.npk_nitrogen    = sample_ ? 42.0f  + jitter(1.5f, 2) : 0.0f;
    msg.npk_phosphorus  = sample_ ? 13.0f  + jitter(0.8f, 3) : 0.0f;
    msg.npk_potassium   = sample_ ? 31.0f  + jitter(1.2f, 4) : 0.0f;
    msg.fluorometer_value = sample_ ? 1.8f + jitter(0.1f, 5) : 0.0f;
    msg.gas_sensor_value  = 410.0f + jitter(8.0f, 6);
    msg.spectro_ready = spectro_;
    for (int i = 0; i < 6; i++)
        msg.spectro_absorbance[i] = spectro_ ? 0.15f + 0.1f * i + jitter(0.01f, i) : 0.0f;

    if (pub_) pub_->publish(msg);
}

void ScienceSimModule::updateBtns()
{
    master_btn_->setText(sim_on_ ? "■ SIM ON — PUBLISHING FAKE DATA" : "▶ SIM OFF");
    master_btn_->setStyleSheet(sim_on_ ? kLoud : kOff);
    status_lbl_->setText(sim_on_ ? "10 Hz → /science/sensor_data (simulated)"
                                 : "Idle — no data published");
    lower_btn_->setStyleSheet(drill_dir_ == 1  ? kOn  : kOff);
    hold_btn_ ->setStyleSheet(drill_dir_ == 0  ? kOn  : kOff);
    raise_btn_->setStyleSheet(drill_dir_ == -1 ? kOn  : kOff);
    sample_btn_ ->setStyleSheet(sample_  ? kOn : kOff);
    spectro_btn_->setStyleSheet(spectro_ ? kOn : kOff);
}

PLUGINLIB_EXPORT_CLASS(ScienceSimModule, rover_hmi_core::GuiModule)
