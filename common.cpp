#include "common.h"
#include "qt_3dmeasure.h"

QT_3DMeasure *MainWindowStatic = nullptr;
void DebugMessageHandler(QtMsgType type, const QMessageLogContext& Context, const QString &msg)
{
   static QMutex mutex;
   mutex.lock();
   QString txt;
   switch (type) {
   case QtDebugMsg:
       txt = QString("%1").arg(msg);
   break;
   case QtWarningMsg:
       txt = QString("Warning: %1").arg(msg);
   break;
   case QtCriticalMsg:
       txt = QString("Critical: %1").arg(msg);
   break;
   case QtFatalMsg:
       txt = QString("Fatal: %1").arg(msg);
       abort();
   }
   if(MainWindowStatic != nullptr)
   {
        MainWindowStatic->ui.textBrowser->append(txt);
   }
   mutex.unlock();
}
