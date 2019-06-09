#ifndef WandCalibrate_h
#define WandCalibrate_h

#include "opencv.hpp"
#include <QDebug>
#include <QString>
#include <QTreeWidget>
#include <QFrame>
#include "FileLib.h"
#include <Dense>
#include <QObject>
#include "ImgLib.h"


using namespace Eigen;
using namespace cv;
using namespace std;

struct TRigidTrans3D
{
	double matR[9];
	double X;
	double Y;
	double Z;
};

typedef struct
{
	cv::Mat CameraOuterMat;                   ///外参
	cv::Mat CameraInterMat;                   ///内参
	cv::Mat CameraDisMat;                     ///畸变
	double ReProjectErr;                      ///重投影误差 
}CameraParamenttypedef;

typedef struct 
{
	int Binarythreshold;
	int SizeLimit;
	double CirqueLimit;
	double StraightLimit;
	int MarkNum;
	int CheckCount;
	cv::Mat InterParamentMat;
	Matrix<double, 3, 3> InterParamentMatrix;     /////内参
	cv::Mat  distortParament;
	Matrix<double, 4, 3> WandDataMatrix;
	cv::Mat  WandDataMat;
}WandCheckDatatypedef;



class WandCalbrateProcess
{
public:
	explicit WandCalbrateProcess();
	explicit WandCalbrateProcess(string FolderName, string CameraStr);
	explicit WandCalbrateProcess(WandCheckDatatypedef &cameraPara, string FolderName, string CameraStr);
public:
	WandCheckDatatypedef WandCheckData;
	vector<Matrix<double, 4, 3>> MarkSiteGlobal;     ////Mark世界坐标
	vector<Matrix<double, 3, 4>> CalibrateGlobal;    ////
	vector<cv::Mat> CalibrateMatGlobal;
	vector<double> ReProjectErrGlobal;				 ////重投影误差
	CameraParamenttypedef CameraParament;
public:
	static void RebuildCamera(vector<CameraParamenttypedef> ExternCameraParaMat);
	bool WriteWandCalbrateData(string FileName);
	bool GetWandCalbrateInit(string FileName);
	void WandCalPoint(string FolderName);
	bool FindCirqueFromContour(Point3d &p, vector<vector<cv::Point>> contours, int index, int rth);
	void WandSolveMat();
	bool GetPicMarkDataFromYml(string FileName, vector<Matrix<double, 4, 3>> &MarkSiteBuff);
	bool WritePicMarkDataToYml(string FileName, vector<Matrix<double, 4, 3>> &MarkSiteBuff);
	Matrix<double, 1, 3> SpinMattoDegree(Matrix<double, 3, 3> ExR);
	bool WriteFinalCalMat(string FileName, vector<Matrix<double, 4, 3>> CalMat);
	static bool WriteCameraParamentToFile(string FileName, const vector<CameraParamenttypedef> &CameraMatrix);
	static bool GetCameraParamentFromFile(string FileName, vector<CameraParamenttypedef> &CameraMatrix);
	bool WriteCameraParamentToFile(string FileName, const vector<WandCalbrateProcess *> &WandCalbrateProcessList);
	void CameraCalbrateAll(vector<WandCalbrateProcess *> &WandCalbrateProcessList);
	static bool GetWandCalbrateInit(string FileName, WandCheckDatatypedef &cameraPara);
	static bool WriteWandCalbrateData(string FileName, WandCheckDatatypedef &cameraPara);
private:
	void WandL3MatCal(Matrix<double, 3, 3> InputMat, Matrix<double, 3, 3> MarkMat, Matrix<double, 3, 3>& Solve);
	Matrix<double, 3, 1>  SpintoShift(Matrix<double, 4, 3> CameraPoint, Matrix<double, 4, 3> LSizeMat, Matrix<double, 3, 3> SpinMat);
	void WandL3MatCal(vector<Matrix<double, 4, 3>> InputMat, Matrix<double, 4, 3> MarkMat, vector<Matrix<double, 4, 3>> &MarkSiteGlobal);
	Matrix<double, 3, 3> SpinMatCal(Matrix<double, 4, 3> InputMat, Matrix<double, 4, 3> LSizeMat);
	Matrix<double, 1, 3> CalPoint3Site(Matrix<double, 1, 3> MarkPoint0, Matrix<double, 1, 3> CameraSite3, Matrix<double, 1, 3> Vector123, double L3);
	double SolveWandCalibrateInit(Matrix<double, 4, 3>& MarkSiteCamera, Matrix<double, 3, 4>& CalibrateMat, unsigned char ResultType);
	double SolveWandCalibrateReProject(Matrix<double, 4, 3> &MarkSiteCamera, Matrix<double, 3, 4>& CalibrateMat);
private:
	string InterParaFile;  /////参数文件
	string MarkDataFile;   ////标记球文件
	string WorkFolder;     /////工作目录
	string CameraString;
	vector<Matrix<double, 4, 3>> MarkSiteCamera;
	vector<Matrix<double, 4, 3>> TransMatGlobal;
};


class CalibrateThread : public QObject , public WandCalbrateProcess
{
	Q_OBJECT
public:
	explicit CalibrateThread(string dir, int index);
	explicit CalibrateThread(WandCheckDatatypedef &cameraPara, string dir, int index);
signals:
	void ResultFull(bool);
public slots:
void CalibrateImgProcess(std::vector<CirquePointtypedef> * CirquePointTree);
public:
	bool FullImg;
public:
	string FileDir;
	vector<Matrix<double, 4, 3>> MarkSiteBuff;
	long FrameCount;
	int CameraIndex;
	long ValidCount;
public:
	void ResultToFile();
};



#endif // WandCalibrate_h
