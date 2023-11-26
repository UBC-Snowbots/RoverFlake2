#include "mainwindow.h"

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent) {
    ui.setupUi(this);  // This sets up the GUI as designed in Qt Designer

    // Connect signals to slots here, for example:
    // connect(ui.startButton, &QPushButton::clicked, this, &MainWindow::onStartButtonClicked);
}