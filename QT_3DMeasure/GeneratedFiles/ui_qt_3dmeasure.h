/********************************************************************************
** Form generated from reading UI file 'qt_3dmeasure.ui'
**
** Created by: Qt User Interface Compiler version 5.6.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_QT_3DMEASURE_H
#define UI_QT_3DMEASURE_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QGraphicsView>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QListWidget>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTextBrowser>
#include <QtWidgets/QWidget>
#include "QVTKWidget.h"

QT_BEGIN_NAMESPACE

class Ui_QT_3DMeasureClass
{
public:
    QAction *actionOpenFile;
    QAction *actionScanUSB;
    QAction *actionCalibrate;
    QAction *action3DMeasure;
    QAction *actionOpenCamera;
    QAction *actionToFileOpen;
    QAction *actionToFileClose;
    QAction *actionCloseCamera;
    QAction *actionCalibrateFromFile;
    QAction *actionImportCamera;
    QAction *actionSetCameraZero;
    QAction *actionDeleteCache;
    QWidget *centralWidget;
    QGroupBox *groupBox;
    QTextBrowser *textBrowser;
    QListWidget *CameralistView;
    QVTKWidget *qvtkWidget3D;
    QGraphicsView *graphicsView_1;
    QMenuBar *menuBar;
    QMenu *menu;
    QMenu *menu_2;
    QMenu *menu_4;
    QMenu *menu_3;
    QMenu *menu_5;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *QT_3DMeasureClass)
    {
        if (QT_3DMeasureClass->objectName().isEmpty())
            QT_3DMeasureClass->setObjectName(QStringLiteral("QT_3DMeasureClass"));
        QT_3DMeasureClass->resize(1800, 1074);
        actionOpenFile = new QAction(QT_3DMeasureClass);
        actionOpenFile->setObjectName(QStringLiteral("actionOpenFile"));
        actionScanUSB = new QAction(QT_3DMeasureClass);
        actionScanUSB->setObjectName(QStringLiteral("actionScanUSB"));
        actionCalibrate = new QAction(QT_3DMeasureClass);
        actionCalibrate->setObjectName(QStringLiteral("actionCalibrate"));
        action3DMeasure = new QAction(QT_3DMeasureClass);
        action3DMeasure->setObjectName(QStringLiteral("action3DMeasure"));
        actionOpenCamera = new QAction(QT_3DMeasureClass);
        actionOpenCamera->setObjectName(QStringLiteral("actionOpenCamera"));
        actionToFileOpen = new QAction(QT_3DMeasureClass);
        actionToFileOpen->setObjectName(QStringLiteral("actionToFileOpen"));
        actionToFileClose = new QAction(QT_3DMeasureClass);
        actionToFileClose->setObjectName(QStringLiteral("actionToFileClose"));
        actionCloseCamera = new QAction(QT_3DMeasureClass);
        actionCloseCamera->setObjectName(QStringLiteral("actionCloseCamera"));
        actionCalibrateFromFile = new QAction(QT_3DMeasureClass);
        actionCalibrateFromFile->setObjectName(QStringLiteral("actionCalibrateFromFile"));
        actionImportCamera = new QAction(QT_3DMeasureClass);
        actionImportCamera->setObjectName(QStringLiteral("actionImportCamera"));
        actionSetCameraZero = new QAction(QT_3DMeasureClass);
        actionSetCameraZero->setObjectName(QStringLiteral("actionSetCameraZero"));
        actionDeleteCache = new QAction(QT_3DMeasureClass);
        actionDeleteCache->setObjectName(QStringLiteral("actionDeleteCache"));
        centralWidget = new QWidget(QT_3DMeasureClass);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        groupBox = new QGroupBox(centralWidget);
        groupBox->setObjectName(QStringLiteral("groupBox"));
        groupBox->setGeometry(QRect(0, 0, 141, 1021));
        textBrowser = new QTextBrowser(groupBox);
        textBrowser->setObjectName(QStringLiteral("textBrowser"));
        textBrowser->setGeometry(QRect(0, 390, 141, 631));
        CameralistView = new QListWidget(groupBox);
        CameralistView->setObjectName(QStringLiteral("CameralistView"));
        CameralistView->setGeometry(QRect(0, 0, 141, 111));
        qvtkWidget3D = new QVTKWidget(centralWidget);
        qvtkWidget3D->setObjectName(QStringLiteral("qvtkWidget3D"));
        qvtkWidget3D->setGeometry(QRect(140, 0, 1241, 1021));
        graphicsView_1 = new QGraphicsView(centralWidget);
        graphicsView_1->setObjectName(QStringLiteral("graphicsView_1"));
        graphicsView_1->setGeometry(QRect(1385, 1, 411, 1021));
        QT_3DMeasureClass->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(QT_3DMeasureClass);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 1800, 23));
        menu = new QMenu(menuBar);
        menu->setObjectName(QStringLiteral("menu"));
        menu_2 = new QMenu(menuBar);
        menu_2->setObjectName(QStringLiteral("menu_2"));
        menu_4 = new QMenu(menu_2);
        menu_4->setObjectName(QStringLiteral("menu_4"));
        menu_3 = new QMenu(menuBar);
        menu_3->setObjectName(QStringLiteral("menu_3"));
        menu_5 = new QMenu(menuBar);
        menu_5->setObjectName(QStringLiteral("menu_5"));
        QT_3DMeasureClass->setMenuBar(menuBar);
        statusBar = new QStatusBar(QT_3DMeasureClass);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        QFont font;
        font.setPointSize(13);
        statusBar->setFont(font);
        QT_3DMeasureClass->setStatusBar(statusBar);

        menuBar->addAction(menu->menuAction());
        menuBar->addAction(menu_2->menuAction());
        menuBar->addAction(menu_3->menuAction());
        menuBar->addAction(menu_5->menuAction());
        menu->addAction(actionOpenFile);
        menu->addAction(actionDeleteCache);
        menu_2->addAction(actionScanUSB);
        menu_2->addAction(actionOpenCamera);
        menu_2->addAction(actionCloseCamera);
        menu_2->addAction(actionCalibrate);
        menu_2->addAction(menu_4->menuAction());
        menu_2->addAction(actionCalibrateFromFile);
        menu_2->addAction(actionImportCamera);
        menu_4->addAction(actionToFileOpen);
        menu_4->addAction(actionToFileClose);
        menu_3->addAction(action3DMeasure);
        menu_5->addAction(actionSetCameraZero);

        retranslateUi(QT_3DMeasureClass);

        QMetaObject::connectSlotsByName(QT_3DMeasureClass);
    } // setupUi

    void retranslateUi(QMainWindow *QT_3DMeasureClass)
    {
        QT_3DMeasureClass->setWindowTitle(QApplication::translate("QT_3DMeasureClass", "\344\270\211\347\273\264\346\265\213\351\207\217", Q_NULLPTR));
        actionOpenFile->setText(QApplication::translate("QT_3DMeasureClass", "\346\211\223\345\274\200", Q_NULLPTR));
        actionScanUSB->setText(QApplication::translate("QT_3DMeasureClass", "\346\211\253\346\217\217\350\256\276\345\244\207", Q_NULLPTR));
        actionCalibrate->setText(QApplication::translate("QT_3DMeasureClass", "L\345\236\213\346\240\207\345\256\232", Q_NULLPTR));
        action3DMeasure->setText(QApplication::translate("QT_3DMeasureClass", "\344\270\211\347\273\264\346\265\213\351\207\217", Q_NULLPTR));
        actionOpenCamera->setText(QApplication::translate("QT_3DMeasureClass", "\346\211\223\345\274\200\347\233\270\346\234\272", Q_NULLPTR));
        actionToFileOpen->setText(QApplication::translate("QT_3DMeasureClass", "\346\211\223\345\274\200", Q_NULLPTR));
        actionToFileClose->setText(QApplication::translate("QT_3DMeasureClass", "\345\205\263\351\227\255", Q_NULLPTR));
        actionCloseCamera->setText(QApplication::translate("QT_3DMeasureClass", "\345\205\263\351\227\255\347\233\270\346\234\272", Q_NULLPTR));
        actionCalibrateFromFile->setText(QApplication::translate("QT_3DMeasureClass", "\344\273\216\346\226\207\344\273\266\346\240\207\345\256\232", Q_NULLPTR));
        actionImportCamera->setText(QApplication::translate("QT_3DMeasureClass", "\345\257\274\345\205\245\347\233\270\346\234\272\345\217\202\346\225\260", Q_NULLPTR));
        actionSetCameraZero->setText(QApplication::translate("QT_3DMeasureClass", "\350\256\276\347\275\256\347\233\270\346\234\2720", Q_NULLPTR));
        actionDeleteCache->setText(QApplication::translate("QT_3DMeasureClass", "\345\210\240\351\231\244\347\274\223\345\255\230\346\226\207\344\273\266", Q_NULLPTR));
        groupBox->setTitle(QString());
        menu->setTitle(QApplication::translate("QT_3DMeasureClass", "\346\226\207\344\273\266", Q_NULLPTR));
        menu_2->setTitle(QApplication::translate("QT_3DMeasureClass", "\347\233\270\346\234\272", Q_NULLPTR));
        menu_4->setTitle(QApplication::translate("QT_3DMeasureClass", "\350\276\223\345\207\272\345\210\260\346\226\207\344\273\266", Q_NULLPTR));
        menu_3->setTitle(QApplication::translate("QT_3DMeasureClass", "\346\265\213\351\207\217", Q_NULLPTR));
        menu_5->setTitle(QApplication::translate("QT_3DMeasureClass", "\350\247\206\345\233\276", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class QT_3DMeasureClass: public Ui_QT_3DMeasureClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_QT_3DMEASURE_H
