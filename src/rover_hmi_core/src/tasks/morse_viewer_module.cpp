#include "rover_hmi_core/tasks/morse_viewer_module.h"
#include "rover_hmi_core/catppuccin.h"
#include <QVBoxLayout>
#include <QPushButton>
#include <pluginlib/class_list_macros.hpp>

void MorseViewerModule::setNode(rclcpp::Node::SharedPtr node) {
    sub_ = node->create_subscription<std_msgs::msg::String>(
        "/morse_decoded", 10,
        std::bind(&MorseViewerModule::onDecoded, this, std::placeholders::_1));
}

QWidget* MorseViewerModule::createWidget(QWidget* parent) {
    auto* root = new QWidget(parent);
    auto* col = new QVBoxLayout(root);

    auto* hint = new QLabel("listening on /morse_decoded (rover_vision morse_decoder)");
    hint->setStyleSheet(QString("color:%1;").arg(theme::TextDim));
    col->addWidget(hint);

    word_ = new QLabel("—");
    word_->setFont(QFont("monospace", theme::FontSizeLg * 2, QFont::Bold));
    word_->setStyleSheet(QString("color:%1; background:%2; padding:10px;"
                                 " border:1px solid %3; border-radius:6px;")
                             .arg(theme::Green, theme::BgPanel, theme::BorderDim));
    word_->setAlignment(Qt::AlignCenter);
    col->addWidget(word_);

    message_ = new QLabel();
    message_->setFont(QFont("monospace", theme::FontSizeLg));
    message_->setStyleSheet(QString("color:%1;").arg(theme::Text));
    message_->setWordWrap(true);
    col->addWidget(message_);

    auto* clear = new QPushButton("Clear");
    QObject::connect(clear, &QPushButton::clicked, [this]() {
        words_.clear();
        last_.clear();
        word_->setText("—");
        message_->clear();
    });
    col->addWidget(clear, 0, Qt::AlignLeft);
    col->addStretch();
    return root;
}

// The decoder republishes the whole in-progress word after each letter and
// starts a fresh (shorter) string on a word gap — so a payload that doesn't
// extend the previous one means the previous word is complete.
void MorseViewerModule::onDecoded(std_msgs::msg::String::SharedPtr msg) {
    if (!word_) return;
    const QString now = QString::fromStdString(msg->data);
    if (!last_.isEmpty() && !now.startsWith(last_)) {
        words_ << last_;
        message_->setText(words_.join(' '));
    }
    last_ = now;
    word_->setText(now.isEmpty() ? "—" : now);
}

PLUGINLIB_EXPORT_CLASS(MorseViewerModule, rover_hmi_core::GuiModule)
