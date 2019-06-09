#ifndef COMMON_H
#define COMMON_H


#include "stdlib.h"
#include "stdio.h"
#include "stdbool.h"
#include "strmif.h"
#include <initguid.h>
#include <vector>
#include <string>

#include <QDesktopWidget>
#include <QMainWindow>
#include "qgraphicsview.h"
#include "qgraphicsscene.h"
#include "qmessagebox.h"
#include <qtextcodec.h>
#include "qlabel.h"
#include "qlistwidget.h"
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QApplication>
#include <QDebug>
#include <QString>
#include <QFileDialog>
#include <QTime>
#include <windows.h>
#include "qthread.h"
#include "qobject.h"
#include <QFileDialog>
#include <QGraphicsPixmapItem>
#include <windows.h>
#include <QCoreApplication>
#include <QFileInfo>
#include <QDir>
#include <qtimer.h>
#include <qmutex.h>

using namespace std;
extern void DebugMessageHandler(QtMsgType type, const QMessageLogContext& Context, const QString &msg);

#endif