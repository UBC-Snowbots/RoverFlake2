// morse_module.cpp — "Morse"

#include "morse_module.h"
#include "morse_code.h"
#include <rover_hmi_core/catppuccin.h>

#include <QHBoxLayout>
#include <QVBoxLayout>

#include <pluginlib/class_list_macros.hpp>

namespace {

QLabel* sectionLabel(const QString& text) {
    auto* l = new QLabel(text);
    l->setStyleSheet(QString("color: %1; font-size: %2px; font-weight: bold;")
                         .arg(theme::TextDim).arg(theme::FontSizeSm));
    return l;
}

QLineEdit* themedInput(const QString& placeholder) {
    auto* e = new QLineEdit();
    e->setPlaceholderText(placeholder);
    e->setStyleSheet(QString("background: %1; color: %2; border: 1px solid %3;"
                             " border-radius: 6px; padding: 8px;")
                         .arg(theme::BgPanel, theme::Text, theme::BorderDim));
    return e;
}

}  // namespace

QWidget* MorseModule::createWidget(QWidget* parent) {
    auto* widget = new QWidget(parent);
    widget->setStyleSheet(QString("background: %1;").arg(theme::Bg));
    auto* layout = new QVBoxLayout(widget);
    layout->setSpacing(10);
    layout->setContentsMargins(16, 16, 16, 16);

    // --- live pipeline ---
    layout->addWidget(sectionLabel("LIVE LED (rover_vision pipeline)"));
    auto* live_row = new QHBoxLayout();
    led_dot_ = new QLabel();
    led_dot_->setFixedSize(22, 22);
    led_dot_->setStyleSheet("background: #1c1c1c; border-radius: 11px;");
    live_row->addWidget(led_dot_);
    bar_ = new QProgressBar();
    bar_->setRange(0, 255);
    bar_->setFormat(QString("%v / %1").arg(kBrightnessThreshold));
    bar_->setStyleSheet(QString(
        "QProgressBar { background: %1; border: 1px solid %2; border-radius: 4px;"
        " color: %3; text-align: center; font-size: %4px; }"
        "QProgressBar::chunk { background: %5; }")
        .arg(theme::BgPanel, theme::BorderDim, theme::Text)
        .arg(theme::FontSizeSm).arg(theme::Yellow));
    live_row->addWidget(bar_, 1);
    signal_lbl_ = new QLabel("WAITING");
    signal_lbl_->setStyleSheet(QString("color: %1; font-size: %2px;")
                                   .arg(theme::TextDim).arg(theme::FontSizeSm));
    live_row->addWidget(signal_lbl_);
    layout->addLayout(live_row);

    current_lbl_ = new QLabel("—");
    current_lbl_->setAlignment(Qt::AlignCenter);
    current_lbl_->setStyleSheet(QString("color: %1; font-size: %2px; font-weight: bold;")
                                    .arg(theme::Green).arg(theme::FontSizeXl));
    layout->addWidget(current_lbl_);

    history_ = new QPlainTextEdit();
    history_->setReadOnly(true);
    history_->setMaximumBlockCount(200);
    history_->setPlaceholderText("completed words…");
    history_->setStyleSheet(QString("background: %1; color: %2; border: 1px solid %3;"
                                    " border-radius: 6px; font-size: %4px;")
                                .arg(theme::BgPanel, theme::Text, theme::BorderDim)
                                .arg(theme::FontSizeSm));
    history_->setMaximumHeight(120);
    layout->addWidget(history_);

    // --- manual decode ---
    layout->addWidget(sectionLabel("MANUAL DECODE (.- groups, / = word gap)"));
    manual_in_ = themedInput(".... ..  /  ..-");
    QObject::connect(manual_in_, &QLineEdit::textChanged, [this](const QString& t) {
        manual_out_->setText(rover_hmi_core::morse::decodeGroups(t));
    });
    layout->addWidget(manual_in_);
    manual_out_ = new QLabel();
    manual_out_->setTextInteractionFlags(Qt::TextSelectableByMouse);
    manual_out_->setStyleSheet(QString("color: %1; font-size: %2px;")
                                   .arg(theme::Cyan).arg(theme::FontSizeLg));
    layout->addWidget(manual_out_);

    // --- encode (vault code) ---
    layout->addWidget(sectionLabel("ENCODE (for keying the vault)"));
    enc_in_ = themedInput("vault code text");
    QObject::connect(enc_in_, &QLineEdit::textChanged, [this](const QString& t) {
        enc_out_->setText(rover_hmi_core::morse::encodeText(t));
    });
    layout->addWidget(enc_in_);
    enc_out_ = new QLabel();
    enc_out_->setWordWrap(true);
    enc_out_->setTextInteractionFlags(Qt::TextSelectableByMouse);
    enc_out_->setStyleSheet(QString("color: %1; font-size: %2px; font-weight: bold;"
                                    " letter-spacing: 2px;")
                                .arg(theme::Yellow).arg(theme::FontSizeLg));
    layout->addWidget(enc_out_);

    layout->addStretch(1);

    stale_.attach(widget, 1500, [this](bool stale) {
        signal_lbl_->setText(stale ? "NO SIGNAL" : "LIVE");
        signal_lbl_->setStyleSheet(QString("color: %1; font-size: %2px;")
                                       .arg(stale ? theme::Red : theme::Green)
                                       .arg(theme::FontSizeSm));
    });
    return widget;
}

void MorseModule::setNode(rclcpp::Node::SharedPtr node) {
    node_ = node;
    subscribe();
}

void MorseModule::subscribe() {
    brightness_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
        "cam_0/morse_led_brightness", rclcpp::QoS(10).best_effort(),
        [this](const std_msgs::msg::Int32::SharedPtr msg) { onBrightness(msg->data); });
    decoded_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/morse_decoded", rclcpp::QoS(10).reliable(),
        [this](const std_msgs::msg::String::SharedPtr msg) {
            onDecoded(QString::fromStdString(msg->data));
        });
}

void MorseModule::onVisibility(bool on) {
    if (on) {
        if (node_ && !brightness_sub_) subscribe();
    } else {
        brightness_sub_.reset();
        decoded_sub_.reset();
    }
}

void MorseModule::onBrightness(int v) {
    stale_.stamp();
    if (bar_) bar_->setValue(qBound(0, v, 255));
    if (led_dot_)
        led_dot_->setStyleSheet(QString("background: %1; border-radius: 11px;")
                                    .arg(v > kBrightnessThreshold ? theme::Yellow : "#1c1c1c"));
}

void MorseModule::onDecoded(const QString& s) {
    // The decoder republishes the growing word and starts over at word gaps.
    if (!last_word_.isEmpty() && !s.startsWith(last_word_))
        history_->appendPlainText(last_word_.toUpper());
    last_word_ = s;
    current_lbl_->setText(s.isEmpty() ? "—" : s.toUpper());
}

PLUGINLIB_EXPORT_CLASS(MorseModule, rover_hmi_core::GuiModule)
