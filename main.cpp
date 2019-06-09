#include "qt_3dmeasure.h"
#include "common.h"
#include <QtWidgets/QApplication>


int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	qInstallMessageHandler(DebugMessageHandler);
	QT_3DMeasure w;
	w.show();
	w.setUi();
	return a.exec();
}
