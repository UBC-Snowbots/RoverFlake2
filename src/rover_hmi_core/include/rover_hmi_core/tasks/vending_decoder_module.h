// vending_decoder_module.h — "Vending Decoder"
//
// Snack Run (6408-M) Err2 helper: 7-seg pattern → byte → ASCII, column-XOR
// switch-sequence calculator, and a free binary↔text converter.
// Section: Tasks. Pure UI, no ROS. defaultVisible: false.

#pragma once

#include <rover_hmi_core/gui_module.h>

#include <QGridLayout>
#include <QLabel>
#include <QLineEdit>
#include <QVector>

class SevenSegWidget;

class VendingDecoderModule : public rover_hmi_core::GuiModule {
public:
    std::string name()        const override { return "Vending Decoder"; }
    std::string sectionName() const override { return "Tasks"; }
    std::string layoutHint()  const override { return "main"; }
    bool        defaultVisible() const override { return false; }

    QWidget* createWidget(QWidget* parent) override;

private:
    static constexpr int kCols = 4;  // Err2 encodes 4 bytes per cycle

    void addGridRow();
    void removeGridRow();
    void pushByteToGrid(int value);
    void recomputeXor();
    void updateSegReadout();
    void convertFree();

    SevenSegWidget* seg_         = nullptr;
    QLabel*         seg_readout_ = nullptr;
    QGridLayout*    grid_        = nullptr;
    QVector<QVector<QLineEdit*>> cells_;
    QVector<QLabel*> xor_lbls_;
    QLabel*         xor_result_  = nullptr;
    QLineEdit*      conv_in_     = nullptr;
    QLabel*         conv_out_    = nullptr;
};
