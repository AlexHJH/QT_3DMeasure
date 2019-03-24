#ifndef MyMathLib_h
#define MyMathLib_h

#include "opencv.hpp"
#include <Dense>
#include <io.h>
#include <string>
#include <vector>

using namespace cv;
using namespace std;
using namespace Eigen;

namespace MyMathLib
{
	double CalTowPointDistance(Matrix<double, 1, 3>  A, Matrix<double, 1, 3>  B);
	double DistanceOfPointToLine(Point3d a, Point3d b, Point3d s);
	double MyMax(double i, double j);
	long factorial(long number);
	int combinator(int n, int m);
	void drawCross(cv::Mat img, cv::Point point, cv::Scalar color, int size, int thickness);
	void CameraInterParaTransfer4x3(Matrix<double, 4, 3> &A, Matrix3d InterParaMat);
	void PointTransfer1x3(Matrix<double, 1, 3> &A, Matrix3d InterParaMat);
}


#endif 
