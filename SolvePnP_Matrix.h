#ifndef SolvePnP_Matrix_H
#define SolvePnP_Matrix_H

#include <io.h>
#include <string>
#include <vector>
#include "opencv.hpp"
#include <QDebug>
#include <QString>
#include <QTreeWidget>
#include <QFrame>
#include "FileLib.h"
#include <Dense>
#include <QObject>

using namespace cv;
using namespace Eigen;

void pose_estimation_3d23d_SVD(const vector<Point3d>& pts1, //3DµãÈÝÆ÷
	const vector<Point3d>& pts2,
	Eigen::Matrix3d& R, Eigen::Vector3d& T);

void PCAalignmentTransMatCal(vector<Matrix<double, 3, 1>> SorceMat,
	vector<Matrix<double, 3, 1>> DstMat,
	Matrix<double, 3, 3>& ExR, Matrix<double, 3, 1>& ExT);

Matrix<double, 4, 1> CameraZeroCal(Matrix<double, 3, 4> RTMat, Matrix<double, 4, 1>& ZeroDot);
Matrix<double, 4, 1> CameraFocusCal(Matrix<double, 3, 4> RTMat, Matrix<double, 4, 1>& FocusDot, double Focus);

class MyDistortProcess
{
public:
	static void myUndistortPoints(const std::vector<cv::Point3d> &src, std::vector<cv::Point3d> &dst,
		const cv::Mat & cameraMatrix, const cv::Mat & distortionCoeff);

	static void myDistortPoints(const std::vector<cv::Point3d> &src, std::vector<cv::Point3d> &dst,
		const cv::Mat & cameraMatrix, const cv::Mat & distortionCoeff);

	static void DistortCorrectMatrix(vector<Matrix<double, 1, 3>> &OriP, const cv::Mat & cameraMatrix, const cv::Mat & distortionCoeff);
	static void DistortCorrectMatrix(Matrix<double, 1, 3> &OriP, const cv::Mat & cameraMatrix, const cv::Mat & distortionCoeff);
private:
};
#endif // 
