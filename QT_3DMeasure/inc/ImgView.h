#ifndef ImgView_h
#define ImgView_h


#include <QThread>
#include <QImage>
#include "opencv.hpp"
#include <QObject>
#include <QGraphicsPixmapItem>
#include "qgraphicsview.h"
#include "qgraphicsscene.h"

using namespace std;

class ImgViewItem : public QObject
{
	Q_OBJECT
public:
	explicit ImgViewItem(QGraphicsView * img_window, int i, std::string str = "Camera_000");
public:
	QGraphicsPixmapItem * img_item;
	std::string CameraDeviceStr;
public slots:
	void ImgViewProcess(cv::Mat *);

public:
	void release();

private:
	int WindowH;
	int WindowW;
	int index;
private:
	int * CalViewPosition(int windowW, int windowH, int imgW, int imgH, int index, int interval);
};



#endif
