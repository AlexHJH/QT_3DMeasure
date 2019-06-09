#ifndef Measure3DLib_h
#define Measure3DLib_h



#include "common.h"
#include <QThread>
#include "opencv.hpp"
#include "WandCalibrate.h"
#include "ImgLib.h"


typedef struct
{
	std::vector<CirquePointtypedef>	MarkPoint;
	std::string CameraDeviceStr;
}InputCameraPointtypedef;


typedef struct
{
	CameraParamenttypedef CameraParament;
	std::string CameraDeviceStr;
}InputCameraDevicetypedef;

class Measure3DPointThread : public QThread
{
	Q_OBJECT
public:
	explicit Measure3DPointThread(QObject *parent);
public:
	void release();
	void run();
	void stop();
	void AddCameraParament(CameraParamenttypedef &camera, std::string str);
	void AddWorldPoint(std::vector<Point3d> &q, std::vector<Point3d> &t);
	int ReCheckCount;
public slots:
	void FromCameraMark(std::vector<CirquePointtypedef> *);
	void ReceiveCameraChange(std::vector<std::vector<Eigen::Matrix<double, 1, 3>>>  *, std::vector<cv::Point3d> *);
signals:
	void SendWorldPoint(std::vector<cv::Point3d> *);
	void SendCameraChange(std::vector<std::vector<Eigen::Matrix<double, 1, 3>>> *, std::vector<cv::Point3d> *);
public:
	std::vector<InputCameraDevicetypedef> CameraParamentTree;
	std::vector<cv::Point3d> WorldPoint3D;
private:
	std::vector<InputCameraPointtypedef> DeviceMarkPointTree;
	std::vector<bool> DeviceCheckBox;
	std::vector<Point3d> RealworldPoint3D;
	std::vector<Point3d> selectPoint3D;
	std::vector<CameraParamenttypedef> ExternCameraParaMat;
	std::vector<std::vector<Eigen::Matrix<double, 1, 3>>> pointsMatchList;
	long long PointFrameCount;
	long long CalFrameCount;
	int CheckCount;

};




#endif
