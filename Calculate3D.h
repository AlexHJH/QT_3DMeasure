#ifndef Calculate3D_h
#define Calculate3D_h


#include "opencv.hpp"
#include <QDebug>
#include <QString>
#include <QTreeWidget>
#include <QFrame>
#include "FileLib.h"
#include <Dense>
#include "WandCalibrate.h"
//#include "Measure3DLib.h"
 
using namespace Eigen;
using namespace cv;
using namespace std;


typedef struct  
{
	WandCheckDatatypedef CameraCheck;
	Matrix<double, 4, 3> ExternMat;
}CameraParatypedef;


void CalPointGlobalSite(Matrix<double, 3, 4> Camera1ExternMat, Matrix<double, 3, 4> Camera2ExternMat,
	Matrix<double, 1, 3> Point1, Matrix<double, 1, 3> Point2, Eigen::Matrix<double, 1, 3> &output);
void CalPolarEquation(Matrix<double, 3, 4> Camera1ExternMat, Matrix<double, 3, 4> Camera2ExternMat,
	Matrix<double, 3, 3> Camera1InterMat, Matrix<double, 3, 3> Camera2InterMat, Matrix<double, 3, 1> Pointcal);



class Calculator3D
{

public:
	static void CalPointMatch(std::vector<std::vector<Eigen::Matrix<double, 1, 3>>> &Pointstree, std::vector<Eigen::Matrix<double, 3, 4>> CameraExternMatrix,
		std::vector<Matrix3d> CameraInterMatrix, int &size, std::vector<std::vector<bool>> &PointMatchTree);//, std::vector<InputCameraPointtypedef> &devicePoint);
	static void CalPointMatch(std::vector<std::vector<Eigen::Matrix<double, 1, 3>>> &Pointstree, std::vector<Eigen::Matrix<double, 3, 4>> CameraExternMatrix,
			std::vector<Matrix3d> CameraInterMatrix, int &size, std::vector<std::vector<Eigen::Matrix<double, 1, 3>>> &pointsMatchList);
	static void CalPointGlobalSite(const std::vector<Eigen::Matrix<double, 3, 4>> CameraExternMat,
		const std::vector<Eigen::Matrix<double, 1, 3>> Pointstree, Eigen::Matrix<double, 1, 3> &output);
private:

};

#endif
