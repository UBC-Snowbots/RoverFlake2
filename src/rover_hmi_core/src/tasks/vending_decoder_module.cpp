// vending_decoder_module.cpp — "Vending Decoder"

#include "vending_decoder_module.h"
#include "seven_seg_widget.h"
#include <rover_hmi_core/catppuccin.h>

#include <QHBoxLayout>
#include <QPushButton>
#include <QRegularExpression>
#include <QVBoxLayout>

#include <pluginlib/class_list_macros.hpp>

namespace {

// Accepts "0b01000001", bare binary "01000001", or "0x41". Returns -1 if invalid.
int parseByte(QString s) {
    s = s.trimmed();
    if (s.isEmpty()) return -1;
    bool ok = false;
    int  v  = -1;
    static const QRegularExpression bare_bin("^[01]{1,8}$");
    if (s.startsWith("0b"))            v = s.mid(2).toInt(&ok, 2);
    else if (s.startsWith("0x"))       v = s.mid(2).toInt(&ok, 16);
    else if (bare_bin.match(s).hasMatch()) v = s.toInt(&ok, 2);
    return (ok && v >= 0 && v < 256) ? v : -1;
}

QString charFor(int v) {
    return (v >= 32 && v < 127) ? QString(QChar(v)) : QStringLiteral("?");
}

QString bin8(int v) {
    return QStringLiteral("0b%1").arg(v, 8, 2, QChar('0'));
}

QLabel* sectionLabel(const QString& text) {
    auto* l = new QLabel(text);
    l->setStyleSheet(QString("color: %1; font-size: %2px; font-weight: bold;")
                         .arg(theme::TextDim).arg(theme::FontSizeSm));
    return l;
}

}  // namespace

QWidget* VendingDecoderModule::createWidget(QWidget* parent) {
    auto* widget = new QWidget(parent);
    widget->setStyleSheet(QString("background: %1;").arg(theme::Bg));
    auto* layout = new QVBoxLayout(widget);
    layout->setSpacing(12);
    layout->setContentsMargins(16, 16, 16, 16);

    // --- 7-seg decode ---
    layout->addWidget(sectionLabel("7-SEG DECODE (click segments)"));
    auto* seg_row = new QHBoxLayout();
    seg_ = new SevenSegWidget(widget);
    seg_->onChanged = [this]() { updateSegReadout(); };
    seg_row->addWidget(seg_);

    auto* seg_side = new QVBoxLayout();
    seg_readout_ = new QLabel();
    seg_readout_->setStyleSheet(QString("color: %1; font-size: %2px;")
                                    .arg(theme::Cyan).arg(theme::FontSizeLg));
    seg_side->addWidget(seg_readout_);
    auto* seg_btns = new QHBoxLayout();
    auto* clear_btn = new QPushButton("Clear");
    auto* push_btn  = new QPushButton("→ Grid");
    QObject::connect(clear_btn, &QPushButton::clicked, [this]() {
        seg_->setBits(0);
        updateSegReadout();
    });
    QObject::connect(push_btn, &QPushButton::clicked, [this]() {
        pushByteToGrid(seg_->bits());
    });
    seg_btns->addWidget(clear_btn);
    seg_btns->addWidget(push_btn);
    seg_btns->addStretch(1);
    seg_side->addLayout(seg_btns);
    seg_side->addStretch(1);
    seg_row->addLayout(seg_side, 1);
    layout->addLayout(seg_row);

    // --- column XOR ---
    layout->addWidget(sectionLabel("COLUMN XOR → SWITCH SEQUENCE (Err2)"));
    auto* grid_host = new QWidget(widget);
    grid_ = new QGridLayout(grid_host);
    grid_->setSpacing(6);
    layout->addWidget(grid_host);

    auto* xor_row = new QHBoxLayout();
    for (int c = 0; c < kCols; ++c) {
        auto* l = new QLabel("—");
        l->setAlignment(Qt::AlignCenter);
        l->setStyleSheet(QString("color: %1; font-size: %2px;")
                             .arg(theme::TextDim).arg(theme::FontSizeSm));
        xor_lbls_.push_back(l);
        xor_row->addWidget(l, 1);
    }
    layout->addLayout(xor_row);

    auto* grid_btns = new QHBoxLayout();
    auto* add_row_btn = new QPushButton("+ Row");
    auto* del_row_btn = new QPushButton("− Row");
    auto* clr_grid_btn = new QPushButton("Clear");
    QObject::connect(add_row_btn, &QPushButton::clicked, [this]() { addGridRow(); });
    QObject::connect(del_row_btn, &QPushButton::clicked, [this]() { removeGridRow(); });
    QObject::connect(clr_grid_btn, &QPushButton::clicked, [this]() {
        for (auto& row : cells_)
            for (auto* cell : row) cell->clear();
    });
    grid_btns->addWidget(add_row_btn);
    grid_btns->addWidget(del_row_btn);
    grid_btns->addWidget(clr_grid_btn);
    grid_btns->addStretch(1);
    xor_result_ = new QLabel("—");
    xor_result_->setStyleSheet(QString("color: %1; font-size: %2px; font-weight: bold;")
                                   .arg(theme::Yellow).arg(theme::FontSizeXl));
    grid_btns->addWidget(xor_result_);
    layout->addLayout(grid_btns);

    for (int r = 0; r < 3; ++r) addGridRow();  // manual example shows 3 cycles

    // --- free converter ---
    layout->addWidget(sectionLabel("CONVERTER (binary/hex ↔ text)"));
    conv_in_ = new QLineEdit();
    conv_in_->setPlaceholderText("0b01000001 0b00111001 …  or plain text");
    conv_in_->setStyleSheet(QString("background: %1; color: %2; border: 1px solid %3;"
                                    " border-radius: 6px; padding: 8px;")
                                .arg(theme::BgPanel, theme::Text, theme::BorderDim));
    QObject::connect(conv_in_, &QLineEdit::textChanged, [this]() { convertFree(); });
    layout->addWidget(conv_in_);
    conv_out_ = new QLabel();
    conv_out_->setWordWrap(true);
    conv_out_->setTextInteractionFlags(Qt::TextSelectableByMouse);
    conv_out_->setStyleSheet(QString("color: %1; font-size: %2px;")
                                 .arg(theme::Green).arg(theme::FontSizeLg));
    layout->addWidget(conv_out_);

    layout->addStretch(1);
    updateSegReadout();
    return widget;
}

void VendingDecoderModule::addGridRow() {
    const int r = cells_.size();
    QVector<QLineEdit*> row;
    for (int c = 0; c < kCols; ++c) {
        auto* cell = new QLineEdit();
        cell->setPlaceholderText("0b…");
        cell->setMaxLength(10);
        cell->setAlignment(Qt::AlignCenter);
        cell->setStyleSheet(QString("background: %1; color: %2; border: 1px solid %3;"
                                    " border-radius: 4px; padding: 6px; font-size: %4px;")
                                .arg(theme::BgPanel, theme::Text, theme::BorderDim)
                                .arg(theme::FontSizeSm));
        QObject::connect(cell, &QLineEdit::textChanged, [this]() { recomputeXor(); });
        grid_->addWidget(cell, r, c);
        row.push_back(cell);
    }
    cells_.push_back(row);
}

void VendingDecoderModule::removeGridRow() {
    if (cells_.size() <= 1) return;
    for (auto* cell : cells_.takeLast()) {
        grid_->removeWidget(cell);
        cell->deleteLater();
    }
    recomputeXor();
}

void VendingDecoderModule::pushByteToGrid(int value) {
    for (auto& row : cells_)
        for (auto* cell : row)
            if (cell->text().trimmed().isEmpty()) {
                cell->setText(bin8(value));
                return;
            }
    addGridRow();
    cells_.last()[0]->setText(bin8(value));
}

void VendingDecoderModule::recomputeXor() {
    QString combined;
    for (int c = 0; c < kCols; ++c) {
        int  acc = 0;
        bool any = false, bad = false;
        for (const auto& row : cells_) {
            const QString t = row[c]->text().trimmed();
            if (t.isEmpty()) continue;
            const int v = parseByte(t);
            if (v < 0) { bad = true; break; }
            acc ^= v;
            any = true;
        }
        if (bad) {
            xor_lbls_[c]->setText("ERR");
            xor_lbls_[c]->setStyleSheet(QString("color: %1; font-size: %2px;")
                                            .arg(theme::Red).arg(theme::FontSizeSm));
            combined += '?';
        } else if (!any) {
            xor_lbls_[c]->setText("—");
            xor_lbls_[c]->setStyleSheet(QString("color: %1; font-size: %2px;")
                                            .arg(theme::TextDim).arg(theme::FontSizeSm));
        } else {
            xor_lbls_[c]->setText(QString("%1\n'%2'").arg(bin8(acc), charFor(acc)));
            xor_lbls_[c]->setStyleSheet(QString("color: %1; font-size: %2px;")
                                            .arg(theme::Cyan).arg(theme::FontSizeSm));
            combined += charFor(acc);
        }
    }
    xor_result_->setText(combined.isEmpty() ? "—" : combined);
}

void VendingDecoderModule::updateSegReadout() {
    const int v = seg_->bits();
    seg_readout_->setText(QString("%1\n0x%2  %3  '%4'")
                              .arg(bin8(v))
                              .arg(v, 2, 16, QChar('0'))
                              .arg(v)
                              .arg(charFor(v)));
}

void VendingDecoderModule::convertFree() {
    const QString in = conv_in_->text().trimmed();
    if (in.isEmpty()) { conv_out_->clear(); return; }

    static const QRegularExpression ws("\\s+");
    const QStringList tokens = in.split(ws, Qt::SkipEmptyParts);
    bool all_bytes = true;
    QString text;
    for (const QString& tok : tokens) {
        const int v = parseByte(tok);
        if (v < 0) { all_bytes = false; break; }
        text += charFor(v);
    }
    if (all_bytes) {
        conv_out_->setText(text);
        return;
    }
    QString bits;
    for (QChar qc : in) bits += bin8(qc.toLatin1()) + ' ';
    conv_out_->setText(bits.trimmed());
}

PLUGINLIB_EXPORT_CLASS(VendingDecoderModule, rover_hmi_core::GuiModule)
