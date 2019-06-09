#ifndef ImgLib_h
#define ImgLib_h

#include "stdlib.h"
#include "opencv.hpp"
#include <QThread>
#include <QObject>
#include <QDebug>
#include <QString>
#include <QTreeWidget>
#include <QFrame>
#include "FileLib.h"
#include <Dense>


using namespace cv;
using namespace std;

typedef enum {
	CV_OTSU,
    CV_THRESHOLD,
}SEGMENTTYPEENUM;

typedef struct
{
	Point3d CirquePoint;   ////Բ��
	double Roundness;      ///Բ��
	int CirqueRadius;      ////�뾶
}CirquePointtypedef;


class ImgMainProcess : public QObject
{
	Q_OBJECT
public:
	explicit ImgMainProcess(string str = "Camera_000", int threshold_t = 200, double round = 0.1);
private:
	long long FrameCount;
	long long DealFrameCount;
	int Segthreshold;           ////�ָ���ֵ
	int CirqueCountLimitMin;
	int CirqueCountLimitMax;
	double RoundnessLimit;
	double ResizeMatK;
	double ExternSizeK;
	bool CalRunning;
	cv::Mat  CopyMainImg;        ////����ʹ�õĶ�������
	cv::Mat * OriMainImg;        ////ԭʼ����
	SEGMENTTYPEENUM SegmentType; ///////�ָ��
	std::vector<CirquePointtypedef> CirquePointTree;
	
private:
	bool FindCirqueFromContour(cv::Point3d &p, std::vector<std::vector<cv::Point>> contours,
		int index, double roundlimit, double& roundness, double& rave);
	void ImgProcessCacheInit(cv::Mat *);
	void ImgSegmentProcess();
	void ImgFindContoursProcess();
public:
	cv::Mat * RGBShowImg;         ///��ʾ��ͼ����
	std::string CameraDeviceStr;
public:
	void release();
	void CloseCal();
	void OpenCal();
public slots:
	
	void ImgProcessThread(cv::Mat *);

signals:
	void SendCirquePoint(std::vector<CirquePointtypedef> *);
	void SendShowRGBImg(cv::Mat *);
};

#endif
