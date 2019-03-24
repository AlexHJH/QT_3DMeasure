#include "MyMathLib.h"


double MyMathLib::CalTowPointDistance(Matrix<double, 1, 3>  A, Matrix<double, 1, 3>  B)
{
	return sqrt(pow(A(0, 0) - B(0, 0), 2) + pow(A(0, 1) - B(0, 1), 2) + pow(A(0, 2) - B(0, 2), 2));
}


/////点到直线的距离
double MyMathLib::DistanceOfPointToLine(Point3d a, Point3d b, Point3d s)
{
	double ab = sqrt(pow(a.x - b.x, 2.0) + pow(a.y - b.y, 2.0) + pow(a.z - b.z, 2.0));
	double as = sqrt(pow((a.x - s.x), 2.0) + pow((a.y - s.y), 2.0) + pow((a.z - s.z), 2.0));
	double bs = sqrt(pow((s.x - b.x), 2.0) + pow((s.y - b.y), 2.0) + pow((s.z - b.z), 2.0));
	double cos_A = (pow(as, 2.0) + pow(ab, 2.0) - pow(bs, 2.0)) / (2 * ab * as);
	double sin_A = sqrt(1 - pow(cos_A, 2.0));
	return as * sin_A;
}

double MyMathLib::MyMax(double i, double j)
{
	if (i > j)
		return i;
	else
		return j;
}

long MyMathLib::factorial(long number)
{
	if (number <= 1)
		return 1;
	else
		return number * factorial(number - 1);
}

/////Cnm,
int MyMathLib::combinator(int n, int m)
{
	int temp;
	if (n < m)
	{
		temp = n;
		n = m;
		m = temp;
	}
	return factorial(n) / (factorial(m) * factorial(n - m));
}

void MyMathLib::drawCross(cv::Mat img, cv::Point point, cv::Scalar color, int size, int thickness)
{
	//绘制横线
	line(img, Point(point.x - size / 2, point.y), Point(point.x + size / 2, point.y), color, thickness, 8, 0);
	//绘制竖线
	line(img, Point(point.x, point.y - size / 2), Point(point.x, point.y + size / 2), color, thickness, 8, 0);
}

void MyMathLib::CameraInterParaTransfer4x3(Matrix<double, 4, 3> &A, Matrix3d InterParaMat)
{
	InterParaMat(0, 0) = 1 / InterParaMat(0, 0);
	InterParaMat(1, 1) = 1 / InterParaMat(1, 1);
	InterParaMat(0, 2) = -InterParaMat(0, 2) * InterParaMat(0, 0);
	InterParaMat(1, 2) = -InterParaMat(1, 2) * InterParaMat(1, 1);
	A = (InterParaMat * A.transpose()).transpose();
}

void MyMathLib::PointTransfer1x3(Matrix<double, 1, 3> &A, Matrix3d InterParaMat)
{
	InterParaMat(0, 0) = 1 / InterParaMat(0, 0);
	InterParaMat(1, 1) = 1 / InterParaMat(1, 1);
	InterParaMat(0, 2) = -InterParaMat(0, 2) * InterParaMat(0, 0);
	InterParaMat(1, 2) = -InterParaMat(1, 2) * InterParaMat(1, 1);
	A = (InterParaMat * A.transpose()).transpose();
}