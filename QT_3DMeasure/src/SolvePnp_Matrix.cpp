#include "SolvePnP_Matrix.h"
#include "MyConvert.h"
#include "qmath.h"
#include <opencv2/core/eigen.hpp>

/////////�������
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

/////////�������
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

/////����
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

		//��ȡ�ĵ�ͨ����ͼ������ص㣬������Ҫ��ͨ��С�����ģ��ת������һ������ϵ�£�
		double xCorrected = (p.x - ux) / fx;
		double yCorrected = (p.y - uy) / fy;

		double xDistortion, yDistortion;

		//������֪���Ǿ���������������������ꣻ
		double r2 = xCorrected*xCorrected + yCorrected*yCorrected;

		double deltaRa = 1. + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2;
		double deltaRb = 1 / (1. + k4 * r2 + k5 * r2 * r2 + k6 * r2 * r2 * r2);
		double deltaTx = 2. * p1 * xCorrected * yCorrected + p2 * (r2 + 2. * xCorrected * xCorrected);
		double deltaTy = p1 * (r2 + 2. * yCorrected * yCorrected) + 2. * p2 * xCorrected * yCorrected;

		//����Ϊ����ģ�ͣ�
		xDistortion = xCorrected * deltaRa * deltaRb + deltaTx;
		yDistortion = yCorrected * deltaRa * deltaRb + deltaTy;

		//����ٴ�ͨ�����ģ�ͽ���һ��������ת������������ϵ�£�
		xDistortion = xDistortion * fx + ux;
		yDistortion = yDistortion * fy + uy;

		dst.push_back(cv::Point3d(xDistortion, yDistortion, 1));
	}
}

///������
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

		//���Ƚ�������ת����
		double xDistortion = (p.x - ux) / fx;
		double yDistortion = (p.y - uy) / fy;

		double xCorrected, yCorrected;

		double x0 = xDistortion;
		double y0 = yDistortion;

		//����ʹ�õ����ķ�ʽ������⣬��Ϊ����2�еĹ�ʽֱ����������ѵģ�����ͨ���趨��ֵ���е�������Ҳ��OpenCV�������ԣ�
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

		//��������任��
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

/////����3D-3D�����RT����
void pose_estimation_3d23d_SVD(
	const vector<Point3d>& pts1,//3D������
	const vector<Point3d>& pts2,
	Eigen::Matrix3d& R, Eigen::Vector3d& T)
{
	//��1�� �����ĵ�
	Point3d p1, p2;     //��ά�㼯�����ĵ�  center of mass
	int N = pts1.size(); //�������
	for (int i = 0; i < N; i++)
	{
		p1 += pts1[i];//��ά�����
		p2 += pts2[i];
	}
	p1 = Point3f(Vec3d(p1) / N);//���ֵ �õ����ĵ�
	p2 = Point3f(Vec3d(p2) / N);
	// ��2���õ�ȥ��������
	vector<Point3f>     q1(N), q2(N); // remove the center
	for (int i = 0; i < N; i++)
	{
		q1[i] = pts1[i] - p1;
		q2[i] = pts2[i] - p2;
	}
	// ��3��������Ҫ��������ֵ�ֽ�� W = sum(qi * qi��ת��) compute q1*q2^T
	Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
	for (int i = 0; i < N; i++)
	{
		W += Eigen::Vector3d(q1[i].x, q1[i].y, q1[i].z) * Eigen::Vector3d(q2[i].x, q2[i].y, q2[i].z).transpose();
	}
	// ��4����  W ����SVD ����ֵ�ֽ�
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


