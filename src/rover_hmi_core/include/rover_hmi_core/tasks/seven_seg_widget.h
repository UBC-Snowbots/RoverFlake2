// seven_seg_widget.h — clickable 7-segment digit
//
// Click segments to mirror what the vending machine display shows.
// Bit order matches the 6408-M manual: 0b0gfedcba (bit0=a .. bit6=g).

#pragma once

#include <QWidget>
#include <functional>

class SevenSegWidget : public QWidget {
public:
    explicit SevenSegWidget(QWidget* parent = nullptr);

    quint8 bits() const { return bits_; }
    void   setBits(quint8 b);

    // Fired after any click toggles a segment.
    std::function<void()> onChanged;

protected:
    void paintEvent(QPaintEvent*) override;
    void mousePressEvent(QMouseEvent*) override;

private:
    QRectF segRect(int i) const;  // i: 0=a 1=b 2=c 3=d 4=e 5=f 6=g
    quint8 bits_ = 0;
};
