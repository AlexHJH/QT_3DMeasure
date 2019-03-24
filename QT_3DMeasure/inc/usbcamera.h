#ifndef _USBCAMERA_H
#define _USBCAMERA_H

#include "common.h"
#include <QThread>
#include <QImage>
#include "opencv.hpp"
#include "CameraApi.h"



class CaptureThread : public QThread
{
	Q_OBJECT
public:

	explicit CaptureThread(QObject *parent, int device, std::string str = "Camera_000");
	
public:
	void release();
	void run();
	void stream();
	void pause();
	void stop();
	bool quitFlag;
	cv::Mat * OutputImg;
	unsigned char * InputArray;
	int width;
	int height;
	std::string CameraDeviceStr;
signals:
	void captured(cv::Mat * Img);
private:
	bool pause_status;
	long long FrameCount;
	/*****************************/

	int devicenum;
	unsigned char   * g_pRawBuffer;
	unsigned char   * g_pRgbBuffer;     //处理后数据缓存区
	
	tSdkFrameHead   g_tFrameHead;       //图像帧头信息
	tSdkCameraCapbility  g_tCapability;
	/*****************************/

	QVector<QRgb> grayColourTable;
	QVector<QRgb> ColourTable;
public slots:
};

int ListDevices(vector<string>& list);



#endif
