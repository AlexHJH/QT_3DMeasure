#include "SolvePnP_Matrix.h"
#include "MyConvert.h"
#include "qmath.h"
#include <opencv2/core/eigen.hpp>

/////////畸变矫正
void MyDistortProcess::DistortCorrectMatrix(Matrix<double, 1, 3>& OriP, const cv::Mat & cameraMatrix, const cv::Mat & distortionCoeff)
{
	Point3d pt;
	vector<Point3d> pv;
	vector<Point3d> pd;

	pt.x = OriP(0, 0);
	pt.y = OriP(0, 1);
	pt.z = OriP(0, 2);
	pv.push_back(pt);

	myUndistortPoints(pv, pd, cameraMatrix, distortionCoeff);

	for (int i = 0; i < pd.size(); i++)
	{
		OriP(0, 0) = pd[i].x;
		OriP(0, 1) = pd[i].y;
		OriP(0, 2) = pd[i].z;
	}
}

/////////畸变矫正
void MyDistortProcess::DistortCorrectMatrix(vector<Matrix<double, 1, 3>>& OriP, const cv::Mat & cameraMatrix, const cv::Mat & distortionCoeff)
{
	Point3d pt;
	vector<Point3d> pv;
	vector<Point3d> pd;
	for (int i = 0; i < OriP.size(); i++)
	{
		pt.x = OriP[i](0, 0);
		pt.y = OriP[i](0, 1);
		pt.z = OriP[i](0, 2);
		pv.push_back(pt);
	}
	myUndistortPoints(pv, pd, cameraMatrix, distortionCoeff);

	for (int i = 0; i < pd.size(); i++)
	{
		OriP[i](0, 0) = pd[i].x;
		OriP[i](0, 1) = pd[i].y;
		OriP[i](0, 2) = pd[i].z;
	}
}

/////畸变
void MyDistortProcess::myDistortPoints(const std::vector<cv::Point3d> & src, std::vector<cv::Point3d> & dst,
	const cv::Mat & cameraMatrix, const cv::Mat & distortionCoeff)
{

	dst.clear();
	double fx = cameraMatrix.at<double>(0, 0);
	double fy = cameraMatrix.at<double>(1, 1);
	double ux = cameraMatrix.at<double>(0, 2);
	double uy = cameraMatrix.at<double>(1, 2);

	double k1 = distortionCoeff.at<double>(0, 0);
	double k2 = distortionCoeff.at<double>(1, 0);
	double p1 = distortionCoeff.at<double>(2, 0);
	double p2 = distortionCoeff.at<double>(3, 0);
	double k3 = distortionCoeff.at<double>(4, 0);
	double k4 = 0;
	double k5 = 0;
	double k6 = 0;

	for (unsigned int i = 0; i < src.size(); i++)
	{
		const cv::Point3d & p = src[i];

		//获取的点通常是图像的像素点，所以需要先通过小孔相机模型转换到归一化坐标系下；
		double xCorrected = (p.x - ux) / fx;
		double yCorrected = (p.y - uy) / fy;

		double xDistortion, yDistortion;

		//我们已知的是经过畸变矫正或理想点的坐标；
		double r2 = xCorrected*xCorrected + yCorrected*yCorrected;

		double deltaRa = 1. + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2;
		double deltaRb = 1 / (1. + k4 * r2 + k5 * r2 * r2 + k6 * r2 * r2 * r2);
		double deltaTx = 2. * p1 * xCorrected * yCorrected + p2 * (r2 + 2. * xCorrected * xCorrected);
		double deltaTy = p1 * (r2 + 2. * yCorrected * yCorrected) + 2. * p2 * xCorrected * yCorrected;

		//下面为畸变模型；
		xDistortion = xCorrected * deltaRa * deltaRb + deltaTx;
		yDistortion = yCorrected * deltaRa * deltaRb + deltaTy;

		//最后再次通过相机模型将归一化的坐标转换到像素坐标系下；
		xDistortion = xDistortion * fx + ux;
		yDistortion = yDistortion * fy + uy;

		dst.push_back(cv::Point3d(xDistortion, yDistortion, 1));
	}
}

///反畸变
void MyDistortProcess::myUndistortPoints(const std::vector<cv::Point3d> & src, std::vector<cv::Point3d> & dst,
	const cv::Mat & cameraMatrix, const cv::Mat & distortionCoeff)
{

	dst.clear();
	double fx = cameraMatrix.at<double>(0, 0);
	double fy = cameraMatrix.at<double>(1, 1);
	double ux = cameraMatrix.at<double>(0, 2);
	double uy = cameraMatrix.at<double>(1, 2);

	double k1 = distortionCoeff.at<double>(0, 0);
	double k2 = distortionCoeff.at<double>(1, 0);
	double p1 = distortionCoeff.at<double>(2, 0);
	double p2 = distortionCoeff.at<double>(3, 0);
	double k3 = distortionCoeff.at<double>(4, 0);
	double k4 = 0;
	double k5 = 0;
	double k6 = 0;

	for (unsigned int i = 0; i < src.size(); i++)
	{
		const cv::Point3d & p = src[i];

		//首先进行坐标转换；
		double xDistortion = (p.x - ux) / fx;
		double yDistortion = (p.y - uy) / fy;

		double xCorrected, yCorrected;

		double x0 = xDistortion;
		double y0 = yDistortion;

		//这里使用迭代的方式进行求解，因为根据2中的公式直接求解是困难的，所以通过设定初值进行迭代，这也是OpenCV的求解策略；
		for (int j = 0; j < 10; j++)
		{
			double r2 = xDistortion*xDistortion + yDistortion*yDistortion;

			double distRadialA = 1 / (1. + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2);
			double distRadialB = 1. + k4 * r2 + k5 * r2 * r2 + k6 * r2 * r2 * r2;

			double deltaX = 2. * p1 * xDistortion * yDistortion + p2 * (r2 + 2. * xDistortion * xDistortion);
			double deltaY = p1 * (r2 + 2. * yDistortion * yDistortion) + 2. * p2 * xDistortion * yDistortion;

			xCorrected = (x0 - deltaX)* distRadialA * distRadialB;
			yCorrected = (y0 - deltaY)* distRadialA * distRadialB;

			xDistortion = xCorrected;
			yDistortion = yCorrected;
		}

		//进行坐标变换；
		xCorrected = xCorrected * fx + ux;
		yCorrected = yCorrected * fy + uy;

		dst.push_back(cv::Point3d(xCorrected, yCorrected, 1));
	}

}

Matrix<double, 4, 1> CameraZeroCal(Matrix<double, 3, 4> RTMat, Matrix<double, 4, 1>& ZeroDot)
{
	Matrix<double, 4, 4> CameraOutMat;
	CameraOutMat.setIdentity();
	CameraOutMat.block(0, 0, 3, 4) = RTMat;
	ZeroDot << 0, 0, 0, 1;
	ZeroDot = CameraOutMat.inverse() * ZeroDot;
	ZeroDot = ZeroDot / ZeroDot(3, 0);
	return ZeroDot;
}

Matrix<double, 4, 1> CameraFocusCal(Matrix<double, 3, 4> RTMat, Matrix<double, 4, 1>& FocusDot, double Focus)
{
	Matrix<double, 4, 4> CameraOutMat;
	CameraOutMat.setIdentity();
	CameraOutMat.block(0, 0, 3, 4) = RTMat;
	FocusDot << 0, 0, Focus, 1;
	FocusDot = CameraOutMat.inverse() * FocusDot;
	FocusDot = FocusDot / FocusDot(3, 0);
	return FocusDot;
}

/////根据3D-3D点对求RT矩阵
void pose_estimation_3d23d_SVD(
	const vector<Point3d>& pts1,//3D点容器
	const vector<Point3d>& pts2,
	Eigen::Matrix3d& R, Eigen::Vector3d& T)
{
	//【1】 求中心点
	Point3d p1, p2;     //三维点集的中心点  center of mass
	int N = pts1.size(); //点对数量
	for (int i = 0; i < N; i++)
	{
		p1 += pts1[i];//各维度求和
		p2 += pts2[i];
	}
	p1 = Point3f(Vec3d(p1) / N);//求均值 得到中心点
	p2 = Point3f(Vec3d(p2) / N);
	// 【2】得到去中心坐标
	vector<Point3f>     q1(N), q2(N); // remove the center
	for (int i = 0; i < N; i++)
	{
		q1[i] = pts1[i] - p1;
		q2[i] = pts2[i] - p2;
	}
	// 【3】计算需要进行奇异值分解的 W = sum(qi * qi’转置) compute q1*q2^T
	Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
	for (int i = 0; i < N; i++)
	{
		W += Eigen::Vector3d(q1[i].x, q1[i].y, q1[i].z) * Eigen::Vector3d(q2[i].x, q2[i].y, q2[i].z).transpose();
	}
	// 【4】对  W 进行SVD 奇异值分解
	Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
	Eigen::Matrix3d U = svd.matrixU();
	Eigen::Matrix3d V = svd.matrixV();

	R= U * (V.transpose());
	T = Eigen::Vector3d(p1.x, p1.y, p1.z) - R * Eigen::Vector3d(p2.x, p2.y, p2.z);
}

void PCAalignmentTransMatCal(vector<Matrix<double, 3, 1>> SorceMat,
	vector<Matrix<double, 3, 1>> DstMat,
	Matrix<double, 3, 3>& ExR, Matrix<double, 3, 1>& ExT)
{
	ExR.setZero();
	vector<Point3d> SorceP, DstP;
	for (int i = 0; i < 4; i++)
	{
		Point3d pt, pm;
		pt.x = SorceMat[i](0, 0);
		pt.y = SorceMat[i](1, 1);
		pt.z = SorceMat[i](2, 2);

		pm.x = SorceMat[i](0, 0);
		pm.y = SorceMat[i](1, 1);
		pm.z = SorceMat[i](2, 2);

		SorceP.push_back(pt);
		DstP.push_back(pm);
	}
	pose_estimation_3d23d_SVD(SorceP, DstP, ExR, ExT);
}


