#include "ImgView.h"
#include "MyConvert.h"

void ImgViewItem::release()
{
	free(img_item);
}

ImgViewItem::ImgViewItem(QGraphicsView * img_window, int i, std::string str)
{
	img_item = new QGraphicsPixmapItem;
	index = i;
	WindowW = img_window->width();
	WindowH = img_window->height();
	CameraDeviceStr = str;
}


void ImgViewItem::ImgViewProcess(cv::Mat * img)
{
	int width = 0, height = 0;

	width = WindowW;
	height = img->rows * WindowW / img->cols;

	if (width == 0 || height == 0)
		return;

	int * imgPos = CalViewPosition(WindowW, WindowH, width, height, index, 50);

	if (imgPos[0] < 0 || imgPos[1] < 0)
	{
		qDebug() << "ImgItem site Error" << endl;
		return;
	}
	img_item->setPos(imgPos[0], imgPos[1]);

	delete imgPos;

	cv::Mat ResizeMat(cv::Size(width, height), img->type());
	cv::resize(*img, ResizeMat, cv::Size(width, height));

	putText(ResizeMat, CameraDeviceStr, cv::Point(0, height * 0.05), cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(64, 124, 166), 1, 8, false);//ÔÚÍ¼Æ¬ÉÏÐ´ÎÄ×Ö

	QImage qimg = MyConverter::cvMat2QImage(ResizeMat);
	img_item->setPixmap(QPixmap::fromImage(qimg));
	ResizeMat.release();
}

int * ImgViewItem::CalViewPosition(int windowW, int windowH, int imgW, int imgH, int index, int interval)
{
	int * pos = new int[2];
	pos[0] = 0;
	pos[1] = 0;
	int i = 0;

	if (imgW > windowW || imgH > windowH)
	{
		pos[0] = -1;
		pos[1] = -1;
	}

	while(i < index)
	{
		pos[0] += imgW + interval;
		if (pos[0] > (windowW - imgW))
		{
			pos[0] = 0;
			pos[1] += imgH + interval;
		}
		i++;
	}
	if (pos[0] > (windowW - imgW) || pos[1] > (windowH - imgH))
	{
		pos[0] = -1;
		pos[1] = -1;
	}
	return pos;
}