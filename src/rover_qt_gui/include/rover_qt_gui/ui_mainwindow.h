/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.15.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QLCDNumber>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QProgressBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_ArmWindow
{
public:
    QWidget *centralwidget;
    QLCDNumber *test_lcd;
    QLabel *label;
    QProgressBar *axis1_pos;
    QPushButton *homeButton;
    QPushButton *pushButton_2;
    QPushButton *commButton;
    QProgressBar *axis2_pos;
    QProgressBar *axis3_pos;
    QProgressBar *axis4_pos;
    QProgressBar *axis5_pos;
    QProgressBar *axis6_pos;
    QPushButton *commButton_2;
    QPushButton *commButton_3;
    QMenuBar *menubar;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *ArmWindow)
    {
        if (ArmWindow->objectName().isEmpty())
            ArmWindow->setObjectName(QString::fromUtf8("ArmWindow"));
        ArmWindow->resize(800, 600);
        QPalette palette;
        QBrush brush(QColor(255, 255, 255, 255));
        brush.setStyle(Qt::SolidPattern);
        palette.setBrush(QPalette::Active, QPalette::WindowText, brush);
        QBrush brush1(QColor(36, 31, 49, 255));
        brush1.setStyle(Qt::SolidPattern);
        palette.setBrush(QPalette::Active, QPalette::Button, brush1);
        QBrush brush2(QColor(54, 46, 73, 255));
        brush2.setStyle(Qt::SolidPattern);
        palette.setBrush(QPalette::Active, QPalette::Light, brush2);
        QBrush brush3(QColor(45, 38, 61, 255));
        brush3.setStyle(Qt::SolidPattern);
        palette.setBrush(QPalette::Active, QPalette::Midlight, brush3);
        QBrush brush4(QColor(18, 15, 24, 255));
        brush4.setStyle(Qt::SolidPattern);
        palette.setBrush(QPalette::Active, QPalette::Dark, brush4);
        QBrush brush5(QColor(24, 21, 33, 255));
        brush5.setStyle(Qt::SolidPattern);
        palette.setBrush(QPalette::Active, QPalette::Mid, brush5);
        palette.setBrush(QPalette::Active, QPalette::Text, brush);
        palette.setBrush(QPalette::Active, QPalette::BrightText, brush);
        palette.setBrush(QPalette::Active, QPalette::ButtonText, brush);
        QBrush brush6(QColor(0, 0, 0, 255));
        brush6.setStyle(Qt::SolidPattern);
        palette.setBrush(QPalette::Active, QPalette::Base, brush6);
        palette.setBrush(QPalette::Active, QPalette::Window, brush1);
        palette.setBrush(QPalette::Active, QPalette::Shadow, brush6);
        palette.setBrush(QPalette::Active, QPalette::AlternateBase, brush4);
        QBrush brush7(QColor(255, 255, 220, 255));
        brush7.setStyle(Qt::SolidPattern);
        palette.setBrush(QPalette::Active, QPalette::ToolTipBase, brush7);
        palette.setBrush(QPalette::Active, QPalette::ToolTipText, brush6);
        QBrush brush8(QColor(255, 255, 255, 128));
        brush8.setStyle(Qt::SolidPattern);
#if QT_VERSION >= QT_VERSION_CHECK(5, 12, 0)
        palette.setBrush(QPalette::Active, QPalette::PlaceholderText, brush8);
#endif
        palette.setBrush(QPalette::Inactive, QPalette::WindowText, brush);
        palette.setBrush(QPalette::Inactive, QPalette::Button, brush1);
        palette.setBrush(QPalette::Inactive, QPalette::Light, brush2);
        palette.setBrush(QPalette::Inactive, QPalette::Midlight, brush3);
        palette.setBrush(QPalette::Inactive, QPalette::Dark, brush4);
        palette.setBrush(QPalette::Inactive, QPalette::Mid, brush5);
        palette.setBrush(QPalette::Inactive, QPalette::Text, brush);
        palette.setBrush(QPalette::Inactive, QPalette::BrightText, brush);
        palette.setBrush(QPalette::Inactive, QPalette::ButtonText, brush);
        palette.setBrush(QPalette::Inactive, QPalette::Base, brush6);
        palette.setBrush(QPalette::Inactive, QPalette::Window, brush1);
        palette.setBrush(QPalette::Inactive, QPalette::Shadow, brush6);
        palette.setBrush(QPalette::Inactive, QPalette::AlternateBase, brush4);
        palette.setBrush(QPalette::Inactive, QPalette::ToolTipBase, brush7);
        palette.setBrush(QPalette::Inactive, QPalette::ToolTipText, brush6);
#if QT_VERSION >= QT_VERSION_CHECK(5, 12, 0)
        palette.setBrush(QPalette::Inactive, QPalette::PlaceholderText, brush8);
#endif
        palette.setBrush(QPalette::Disabled, QPalette::WindowText, brush4);
        palette.setBrush(QPalette::Disabled, QPalette::Button, brush1);
        palette.setBrush(QPalette::Disabled, QPalette::Light, brush2);
        palette.setBrush(QPalette::Disabled, QPalette::Midlight, brush3);
        palette.setBrush(QPalette::Disabled, QPalette::Dark, brush4);
        palette.setBrush(QPalette::Disabled, QPalette::Mid, brush5);
        palette.setBrush(QPalette::Disabled, QPalette::Text, brush4);
        palette.setBrush(QPalette::Disabled, QPalette::BrightText, brush);
        palette.setBrush(QPalette::Disabled, QPalette::ButtonText, brush4);
        palette.setBrush(QPalette::Disabled, QPalette::Base, brush1);
        palette.setBrush(QPalette::Disabled, QPalette::Window, brush1);
        palette.setBrush(QPalette::Disabled, QPalette::Shadow, brush6);
        palette.setBrush(QPalette::Disabled, QPalette::AlternateBase, brush1);
        palette.setBrush(QPalette::Disabled, QPalette::ToolTipBase, brush7);
        palette.setBrush(QPalette::Disabled, QPalette::ToolTipText, brush6);
#if QT_VERSION >= QT_VERSION_CHECK(5, 12, 0)
        palette.setBrush(QPalette::Disabled, QPalette::PlaceholderText, brush8);
#endif
        ArmWindow->setPalette(palette);
        centralwidget = new QWidget(ArmWindow);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        test_lcd = new QLCDNumber(centralwidget);
        test_lcd->setObjectName(QString::fromUtf8("test_lcd"));
        test_lcd->setGeometry(QRect(540, 390, 141, 81));
        label = new QLabel(centralwidget);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(20, 20, 221, 61));
        QPalette palette1;
        QBrush brush9(QColor(119, 118, 123, 255));
        brush9.setStyle(Qt::SolidPattern);
        palette1.setBrush(QPalette::Active, QPalette::WindowText, brush9);
        QBrush brush10(QColor(153, 193, 241, 255));
        brush10.setStyle(Qt::SolidPattern);
        palette1.setBrush(QPalette::Active, QPalette::Button, brush10);
        palette1.setBrush(QPalette::Active, QPalette::Light, brush);
        QBrush brush11(QColor(204, 224, 248, 255));
        brush11.setStyle(Qt::SolidPattern);
        palette1.setBrush(QPalette::Active, QPalette::Midlight, brush11);
        QBrush brush12(QColor(76, 96, 120, 255));
        brush12.setStyle(Qt::SolidPattern);
        palette1.setBrush(QPalette::Active, QPalette::Dark, brush12);
        QBrush brush13(QColor(102, 129, 161, 255));
        brush13.setStyle(Qt::SolidPattern);
        palette1.setBrush(QPalette::Active, QPalette::Mid, brush13);
        palette1.setBrush(QPalette::Active, QPalette::Text, brush6);
        palette1.setBrush(QPalette::Active, QPalette::BrightText, brush);
        palette1.setBrush(QPalette::Active, QPalette::ButtonText, brush6);
        palette1.setBrush(QPalette::Active, QPalette::Base, brush);
        palette1.setBrush(QPalette::Active, QPalette::Window, brush10);
        palette1.setBrush(QPalette::Active, QPalette::Shadow, brush6);
        palette1.setBrush(QPalette::Active, QPalette::AlternateBase, brush11);
        palette1.setBrush(QPalette::Active, QPalette::ToolTipBase, brush7);
        palette1.setBrush(QPalette::Active, QPalette::ToolTipText, brush6);
        QBrush brush14(QColor(0, 0, 0, 128));
        brush14.setStyle(Qt::SolidPattern);
#if QT_VERSION >= QT_VERSION_CHECK(5, 12, 0)
        palette1.setBrush(QPalette::Active, QPalette::PlaceholderText, brush14);
#endif
        palette1.setBrush(QPalette::Inactive, QPalette::WindowText, brush9);
        palette1.setBrush(QPalette::Inactive, QPalette::Button, brush10);
        palette1.setBrush(QPalette::Inactive, QPalette::Light, brush);
        palette1.setBrush(QPalette::Inactive, QPalette::Midlight, brush11);
        palette1.setBrush(QPalette::Inactive, QPalette::Dark, brush12);
        palette1.setBrush(QPalette::Inactive, QPalette::Mid, brush13);
        palette1.setBrush(QPalette::Inactive, QPalette::Text, brush6);
        palette1.setBrush(QPalette::Inactive, QPalette::BrightText, brush);
        palette1.setBrush(QPalette::Inactive, QPalette::ButtonText, brush6);
        palette1.setBrush(QPalette::Inactive, QPalette::Base, brush);
        palette1.setBrush(QPalette::Inactive, QPalette::Window, brush10);
        palette1.setBrush(QPalette::Inactive, QPalette::Shadow, brush6);
        palette1.setBrush(QPalette::Inactive, QPalette::AlternateBase, brush11);
        palette1.setBrush(QPalette::Inactive, QPalette::ToolTipBase, brush7);
        palette1.setBrush(QPalette::Inactive, QPalette::ToolTipText, brush6);
#if QT_VERSION >= QT_VERSION_CHECK(5, 12, 0)
        palette1.setBrush(QPalette::Inactive, QPalette::PlaceholderText, brush14);
#endif
        palette1.setBrush(QPalette::Disabled, QPalette::WindowText, brush12);
        palette1.setBrush(QPalette::Disabled, QPalette::Button, brush10);
        palette1.setBrush(QPalette::Disabled, QPalette::Light, brush);
        palette1.setBrush(QPalette::Disabled, QPalette::Midlight, brush11);
        palette1.setBrush(QPalette::Disabled, QPalette::Dark, brush12);
        palette1.setBrush(QPalette::Disabled, QPalette::Mid, brush13);
        palette1.setBrush(QPalette::Disabled, QPalette::Text, brush12);
        palette1.setBrush(QPalette::Disabled, QPalette::BrightText, brush);
        palette1.setBrush(QPalette::Disabled, QPalette::ButtonText, brush12);
        palette1.setBrush(QPalette::Disabled, QPalette::Base, brush10);
        palette1.setBrush(QPalette::Disabled, QPalette::Window, brush10);
        palette1.setBrush(QPalette::Disabled, QPalette::Shadow, brush6);
        palette1.setBrush(QPalette::Disabled, QPalette::AlternateBase, brush10);
        palette1.setBrush(QPalette::Disabled, QPalette::ToolTipBase, brush7);
        palette1.setBrush(QPalette::Disabled, QPalette::ToolTipText, brush6);
#if QT_VERSION >= QT_VERSION_CHECK(5, 12, 0)
        palette1.setBrush(QPalette::Disabled, QPalette::PlaceholderText, brush14);
#endif
        label->setPalette(palette1);
        QFont font;
        font.setFamily(QString::fromUtf8("Uroob"));
        font.setPointSize(34);
        label->setFont(font);
        label->setTextFormat(Qt::PlainText);
        axis1_pos = new QProgressBar(centralwidget);
        axis1_pos->setObjectName(QString::fromUtf8("axis1_pos"));
        axis1_pos->setGeometry(QRect(50, 210, 211, 41));
        axis1_pos->setValue(24);
        homeButton = new QPushButton(centralwidget);
        homeButton->setObjectName(QString::fromUtf8("homeButton"));
        homeButton->setGeometry(QRect(530, 210, 171, 81));
        pushButton_2 = new QPushButton(centralwidget);
        pushButton_2->setObjectName(QString::fromUtf8("pushButton_2"));
        pushButton_2->setGeometry(QRect(530, 300, 171, 81));
        commButton = new QPushButton(centralwidget);
        commButton->setObjectName(QString::fromUtf8("commButton"));
        commButton->setGeometry(QRect(530, 120, 171, 81));
        axis2_pos = new QProgressBar(centralwidget);
        axis2_pos->setObjectName(QString::fromUtf8("axis2_pos"));
        axis2_pos->setGeometry(QRect(50, 260, 211, 41));
        axis2_pos->setValue(24);
        axis3_pos = new QProgressBar(centralwidget);
        axis3_pos->setObjectName(QString::fromUtf8("axis3_pos"));
        axis3_pos->setGeometry(QRect(50, 310, 211, 41));
        axis3_pos->setValue(24);
        axis4_pos = new QProgressBar(centralwidget);
        axis4_pos->setObjectName(QString::fromUtf8("axis4_pos"));
        axis4_pos->setGeometry(QRect(50, 360, 211, 41));
        axis4_pos->setValue(24);
        axis5_pos = new QProgressBar(centralwidget);
        axis5_pos->setObjectName(QString::fromUtf8("axis5_pos"));
        axis5_pos->setGeometry(QRect(50, 410, 211, 41));
        axis5_pos->setValue(24);
        axis6_pos = new QProgressBar(centralwidget);
        axis6_pos->setObjectName(QString::fromUtf8("axis6_pos"));
        axis6_pos->setGeometry(QRect(50, 460, 211, 41));
        axis6_pos->setValue(24);
        commButton_2 = new QPushButton(centralwidget);
        commButton_2->setObjectName(QString::fromUtf8("commButton_2"));
        commButton_2->setGeometry(QRect(260, 510, 141, 31));
        commButton_3 = new QPushButton(centralwidget);
        commButton_3->setObjectName(QString::fromUtf8("commButton_3"));
        commButton_3->setGeometry(QRect(340, 120, 171, 81));
        ArmWindow->setCentralWidget(centralwidget);
        menubar = new QMenuBar(ArmWindow);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 800, 23));
        ArmWindow->setMenuBar(menubar);
        statusbar = new QStatusBar(ArmWindow);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        ArmWindow->setStatusBar(statusbar);

        retranslateUi(ArmWindow);

        QMetaObject::connectSlotsByName(ArmWindow);
    } // setupUi

    void retranslateUi(QMainWindow *ArmWindow)
    {
        ArmWindow->setWindowTitle(QCoreApplication::translate("ArmWindow", "MainWindow", nullptr));
        label->setText(QCoreApplication::translate("ArmWindow", "Arm Control", nullptr));
        homeButton->setText(QCoreApplication::translate("ArmWindow", "Home Request", nullptr));
        pushButton_2->setText(QCoreApplication::translate("ArmWindow", "Send Target Pos", nullptr));
        commButton->setText(QCoreApplication::translate("ArmWindow", "POS COMM ON", nullptr));
        commButton_2->setText(QCoreApplication::translate("ArmWindow", "save pos", nullptr));
        commButton_3->setText(QCoreApplication::translate("ArmWindow", "JOYSTICK ACTIVATE", nullptr));
    } // retranslateUi

};

namespace Ui {
    class ArmWindow: public Ui_ArmWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
