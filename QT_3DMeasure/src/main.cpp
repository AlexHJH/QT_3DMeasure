#include "qt_3dmeasure.h"
#include <QtWidgets/QApplication>


int main(int argc, char *argv[])
{

	QApplication a(argc, argv);
	QT_3DMeasure w;
	w.show();
	return a.exec();
}
