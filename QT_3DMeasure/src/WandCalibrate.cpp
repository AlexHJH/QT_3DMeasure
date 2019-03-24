#include "WandCalibrate.h"
#include "MyConvert.h"
#include "MyMathLib.h"
#include "qmath.h"
#include <opencv2/core/eigen.hpp>
#include "SolvePnP_Matrix.h"
#include "ChineseDebug.h"
using namespace MyMathLib;

void PointOptimize(vector<Matrix<double, 4, 3>> InputMat)
{
	
}

Matrix<double, 3, 1>  WandCalbrateProcess::SpintoShift(Matrix<double, 4, 3> CameraPoint, Matrix<double, 4, 3> LSizeMat, Matrix<double, 3, 3> SpinMat)
{
	Matrix<double, 3, 1> T;
	Matrix<double, 8, 3> A;
	Matrix<double, 8, 1> B;
	A.setZero();
	B.setZero();
	T.setZero();
	for (int i = 0; i < 4; i++)
	{
		A(i * 2, 0) = 1;
		A(i * 2, 2) = -CameraPoint(i, 0);
		A(i * 2 + 1, 1) = 1;
		A(i * 2 + 1, 2) = -CameraPoint(i, 1);


		B(i * 2, 0) = (SpinMat.row(0) - CameraPoint(i, 0) * SpinMat.row(2)) * (LSizeMat.row(i).transpose());
		B(i * 2 + 1, 0) = (SpinMat.row(1) - CameraPoint(i, 1) * SpinMat.row(2)) * (LSizeMat.row(i).transpose());
	}
	T = (A.transpose() * A).inverse() * (A.transpose() * B);
	T = SpinMat.transpose() * T;
	return T;
}

Matrix<double, 1, 3> WandCalbrateProcess::SpinMattoDegree(Matrix<double, 3, 3> ExR)
{
	Matrix<double, 1, 3> DegreeMat;
	DegreeMat = ExR.inverse().eulerAngles(0, 1, 2).transpose();
	DegreeMat = DegreeMat * 180 / M_PI;
	DegreeMat(0, 2) -= 180;
	return DegreeMat;
}

Matrix<double, 3, 3> PCAalignmentSpinMatCal(Matrix<double, 4, 3> SorceMat, Matrix<double, 4, 3> DstMat)
{
	Matrix<double, 3, 3> ExR;
	Vector3d ExT;
	vector<Point3d> SorceP, DstP;
	for (int i = 0; i < SorceMat.rows(); i++)
	{
		Point3d pt, pm;
		pt.x = SorceMat(i, 0);
		pt.y = SorceMat(i, 1);
		pt.z = SorceMat(i, 2);

		pm.x = DstMat(i, 0);
		pm.y = DstMat(i, 1);
		pm.z = DstMat(i, 2);

		SorceP.push_back(pt);
		DstP.push_back(pm);
	}
	pose_estimation_3d23d_SVD(SorceP, DstP, ExR, ExT);
	return ExR;
}

Matrix<double, 3, 3> WandCalbrateProcess::SpinMatCal(Matrix<double, 4, 3> InputMat, Matrix<double, 4, 3> LSizeMat)
{
	Matrix<double, 4, 3> InputMatC;
	Matrix<double, 3, 3> ExR;
	ExR.setZero();
	InputMatC.setZero();
	for (int i = 0; i < InputMat.rows(); i++)
	{
		InputMatC(i, 0) = InputMat(i, 0) - InputMat(0, 0);
		InputMatC(i, 1) = InputMat(i, 1) - InputMat(0, 1);
		InputMatC(i, 2) = InputMat(i, 2) - InputMat(0, 2);
	}
#if 1
	Matrix<double, 12, 3> A;
	Matrix<double, 12, 1> B;
	A.setZero();
	B.setZero();
	for (int i = 0; i < InputMat.rows(); i++)
	{
		A(i * 3, 1) = InputMatC(i, 2) + LSizeMat(i, 2);
		A(i * 3, 2) = InputMatC(i, 1) + LSizeMat(i, 1);

		A(i * 3 + 1, 0) = InputMatC(i, 2) + LSizeMat(i, 2);
		A(i * 3 + 1, 2) = -InputMatC(i, 0) - LSizeMat(i, 0);

		A(i * 3 + 2, 0) = InputMatC(i, 1) + LSizeMat(i, 1);
		A(i * 3 + 2, 1) = InputMatC(i, 0) + LSizeMat(i, 0);

		B(i * 3, 0) = InputMatC(i, 0) - LSizeMat(i, 0);
		B(i * 3 + 1, 0) = InputMatC(i, 1) - LSizeMat(i, 1);
		B(i * 3 + 2, 0) = InputMatC(i, 2) - LSizeMat(i, 2);
	}
	Matrix<double, 3, 1> T = (A.transpose() * A).inverse() * (A.transpose() * B);
	Matrix<double, 3, 3> S;

	S.setZero();
	S(0, 1) = -T(2, 0);
	S(0, 2) = -T(1, 0);
	S(1, 0) = T(2, 0);
	S(1, 2) = -T(0, 0);

	S(2, 0) = T(1, 0);

	S(2, 1) = T(0, 0);

	Matrix<double, 3, 3> MatOne;
	MatOne.setIdentity();
	ExR = (MatOne + S) * ((MatOne - S).inverse());
#endif
	return ExR;
}
/////计算L型的第四个点
Matrix<double, 1, 3> WandCalbrateProcess::CalPoint3Site(Matrix<double, 1, 3> MarkPoint0, Matrix<double, 1, 3> CameraSite3, Matrix<double, 1, 3> Vector123, double L3)
{
	double Length03 = DistanceOfPointToLine(Point3d(0, 0, 0), Point3d(CameraSite3(0, 0), CameraSite3(0, 1), CameraSite3(0, 2)),
		Point3d(MarkPoint0(0, 0), MarkPoint0(0, 1), MarkPoint0(0, 2)));
	double CameraSiteLength = CalTowPointDistance(CameraSite3, Matrix<double, 1, 3>(0, 0, 0));
	double MarkPointLength = CalTowPointDistance(MarkPoint0, Matrix<double, 1, 3>(0, 0, 0));
	double theta0 = acos(CameraSite3(0, 0) / CameraSiteLength);
	double theta1 = acos(CameraSite3(0, 1) / CameraSiteLength);
	double theta2 = acos(CameraSite3(0, 2) / CameraSiteLength);
	double theta = acos((MarkPoint0(0, 0) * CameraSite3(0, 0) + MarkPoint0(0, 1) * CameraSite3(0, 1) + MarkPoint0(0, 2) * CameraSite3(0, 2)) /
		(CameraSiteLength * MarkPointLength));

	double L03_0 = MarkPointLength -
		sqrt(MyMax(0, L3 * L3 - Length03 * Length03));
	double L03_1 = MarkPointLength * cos(theta) +
		sqrt(MyMax(0, L3 * L3 - Length03 * Length03));

	/////得到两个候选点
	Matrix<double, 1, 3> CandidatePoint0, CandidatePoint1;
	CandidatePoint0(0, 0) = L03_0 * cos(theta0);
	CandidatePoint0(0, 1) = L03_0 * cos(theta1);
	CandidatePoint0(0, 2) = L03_0 * cos(theta2);

	CandidatePoint1(0, 0) = L03_1 * cos(theta0);
	CandidatePoint1(0, 1) = L03_1 * cos(theta1);
	CandidatePoint1(0, 2) = L03_1 * cos(theta2);

	double PointCheck[2];
	PointCheck[0] = abs(((CandidatePoint0(0, 0) - MarkPoint0(0, 0)) * Vector123(0, 0) + (CandidatePoint0(0, 1) - MarkPoint0(0, 1)) * Vector123(0, 1) +
		(CandidatePoint0(0, 2) - MarkPoint0(0, 2)) * Vector123(0, 2)) /
		CalTowPointDistance(CandidatePoint0, MarkPoint0));

	PointCheck[1] = abs((CandidatePoint1(0, 0) - MarkPoint0(0, 0)) * Vector123(0, 0) + (CandidatePoint1(0, 1) - MarkPoint0(0, 1)) * Vector123(0, 1) +
		(CandidatePoint1(0, 2) - MarkPoint0(0, 2)) * Vector123(0, 2) /
		CalTowPointDistance(CandidatePoint0, MarkPoint0));

	if (PointCheck[0] < PointCheck[1])
	{
		return CandidatePoint0;
	}
	else
		return CandidatePoint1;
}


WandCalbrateProcess::WandCalbrateProcess()
{
	////默认初始化数据
	WandCheckData.Binarythreshold = 200;
	WandCheckData.SizeLimit = 120;
	WandCheckData.MarkNum = 3;
	WandCheckData.StraightLimit = 5000;
	WandCheckData.CirqueLimit = 1;
	WandCheckData.WandMat.setZero();
	WandCheckData.WandMat(0, 0) = 0;
	WandCheckData.WandMat(1, 0) = 150;
	WandCheckData.WandMat(2, 0) = 600;
	WandCheckData.WandMat(3, 1) = 300;
	WandCheckData.CheckCount = 200;
}

WandCalbrateProcess::WandCalbrateProcess(string FolderName, string CameraStr)
{
/******************************************************************/
	////默认初始化数据
	WandCheckData.Binarythreshold = 200;
	WandCheckData.SizeLimit = 120;
	WandCheckData.MarkNum = 3;
	WandCheckData.StraightLimit = 5000;
	WandCheckData.CirqueLimit = 1;
	WandCheckData.WandMat.setZero();
	WandCheckData.WandMat(0, 0) = 0;
	WandCheckData.WandMat(1, 0) = 150;
	WandCheckData.WandMat(2, 0) = 600;
	WandCheckData.WandMat(3, 1) = 300;
	WandCheckData.CheckCount = 200;
/******************************************************************/
	WorkFolder = FolderName;                                /////这是单个相机的工作目录，并行执行
	CameraString = CameraStr;
	InterParaFile = WorkFolder + "/InterParament.yml";
	if (GetWandCalbrateInit(InterParaFile))
		qDebug() << QString::fromStdString(InterParaFile) << "内参读取成功" << endl;
	else
		qDebug() << QString::fromStdString(InterParaFile) << "读取失败" << endl;


	MarkDataFile = WorkFolder + "/MarkData.yml";

	if (GetPicMarkDataFromYml(MarkDataFile, MarkSiteCamera))
		qDebug() << QString::fromStdString(MarkDataFile) << "打开数据文件成功" << endl;
	else
		qDebug() << QString::fromStdString(MarkDataFile) << "打开数据文件失败" << endl;
}

bool WandCalbrateProcess::FindCirqueFromContour(Point3d &p, vector<vector<cv::Point>> contours, int index, int rth)
{
	int contourlength = contours[index].size();
	double avg_px = 0, avg_py = 0;
	int i = 0;
	double rtc = 0;
	double rave = 0;
	double * r = new double[contourlength];
	cv::Point2f pt;
	for (i = 0; i < contourlength; i++)
	{
		pt = contours[index][i];
		avg_px += pt.x;
		avg_py += pt.y;
	}
	p.x = avg_px / contourlength;
	p.y = avg_py / contourlength;

	for (i = 0; i < contourlength; i++)
	{
		r[i] = sqrt((p.x - contours[index][i].x) * (p.x - contours[index][i].x) + (p.y - contours[index][i].y) * (p.y - contours[index][i].y));
		rave += r[i];
	}
	rave = rave / contourlength;
	p.z = rave;

	for (i = 0; i < contourlength; i++)
	{
		rtc += abs(r[i] - rave);
	}
	rtc = rtc / contourlength;
	delete[] r;
	if (rtc > rth)
		return false;
	return true;
}

void MyProjectPoints(vector<Point3d> objeectPoints, cv::Mat ResMat, cv::Mat TMat, cv::Mat cameraMatrix, vector<Point2d> &CameraPoints)
{
	for (int i = 0; i < objeectPoints.size(); i++)
	{
		Eigen::Matrix<double, 4, 1> objP;
		Eigen::Matrix<double, 3, 1> camPoint;
		Point2d camPoints;
		Mat rotation_matrix = Mat(3, 3, CV_32FC1, Scalar::all(0)); /* 保存每幅图像的旋转矩阵 */
		Rodrigues(ResMat, rotation_matrix);

		//cout << cameraMatrix << endl;

		Eigen::Matrix<double, 3, 3> ResM;
		cv2eigen(rotation_matrix, ResM);
		Eigen::Matrix<double, 3, 1> TM = MyConverter::CvMat31toMatrix3d(TMat);
		Eigen::Matrix<double, 3, 3> CamM = MyConverter::CvMattoMatrix3d(cameraMatrix);
		objP << objeectPoints[i].x, objeectPoints[i].y, objeectPoints[i].z, 1;


		Eigen::Matrix<double, 3, 4> CamInter;
		Eigen::Matrix<double, 4, 4> CamOuter;
		Eigen::Matrix<double, 3, 4> CamP;
		CamInter.setZero();
		CamOuter.setZero();

		CamInter.block(0, 0, 3, 3) = CamM;
		CamOuter.block(0, 0, 3, 3) = ResM;
		CamOuter.block(0, 3, 3, 1) = TM;
		CamOuter(3, 3) = 1;

		CamP = CamInter * CamOuter;

		Eigen::Matrix<double, 4, 1> OriP;
		OriP << 0, 0, 0, 1;
		OriP = CamOuter.inverse() * OriP;

		camPoint = CamP * objP;
		camPoint = camPoint / camPoint(2, 0);

		camPoint = CamM * (ResM * objP.block(0, 0, 3, 1) + TM);
		camPoint = camPoint / camPoint(2, 0);

		camPoints.x = camPoint(0, 0);
		camPoints.y = camPoint(1, 0);
		CameraPoints.push_back(camPoints);
	}
}

double WandCalbrateProcess::SolveWandCalibrate_SelfCal(Matrix<double, 4, 3> &MarkSiteCamera, 
	Matrix<double, 3, 4>& CalibrateMat, unsigned char ResultType)
{
	double reProjectErr = 0;
	Matrix<double, 4, 3> OriMarkSiteCamera = MarkSiteCamera;
	Matrix<double, 4, 3> MarkSiteCameraCal = MarkSiteCamera;
	Matrix<double, 4, 3> MarkSite;
	Matrix<double, 3, 3> CameraInterMatrix = WandCheckData.InterParament;
	Matrix<double, 4, 3> WandMatrix = WandCheckData.WandMat;
	cv::Mat CameraInterMat = WandCheckData.InterParamentMat.clone();
	cv::Mat CameraDistortMat = WandCheckData.distortParament.clone();
// 	for (int i = 0; i < MarkSiteCamera.rows(); i++)
// 	{
// 		Matrix<double, 1, 3> markSiteCamera_temp = MarkSiteCamera.row(i);
// 		MyDistortProcess::DistortCorrectMatrix(markSiteCamera_temp, CameraInterMat, WandCheckData.distortParament);
// 		MarkSiteCamera.row(i) = markSiteCamera_temp;
// 	}
	CameraInterParaTransfer4x3(MarkSiteCameraCal, CameraInterMatrix);////归一化到F相机系

	Matrix<double, 4, 3> MarkSiteCameraFocus = MarkSiteCameraCal;   ////相机齐次坐标系
	

	Matrix<double, 3, 3> Mark3SiteCamera = MarkSiteCameraCal.block(0, 0, 3, 3);   //////直线3点对应相机坐标
	Matrix<double, 3, 3> Wand3Site = WandMatrix.block(0, 0, 3, 3);                //////直线3点对应世界坐标
	Matrix<double, 3, 3> Mark3SiteCameraWorld;
	WandL3MatCal(Mark3SiteCamera, Wand3Site, Mark3SiteCameraWorld);
	MarkSiteCameraCal.block(0, 0, 3, 3) = Mark3SiteCameraWorld;

	Matrix<double, 1, 3> Vector123;
	Matrix<double, 3, 4> TransMattemp;

	double LamdaL = CalTowPointDistance(WandMatrix.row(0), WandMatrix.row(1)) / 
		CalTowPointDistance(MarkSiteCameraCal.row(0), MarkSiteCameraCal.row(1));
		//////量纲系数
	MarkSiteCameraCal = LamdaL * MarkSiteCameraCal;                             /////已经转换到标准尺度坐标系

	Vector123(0, 0) = (MarkSiteCameraCal(1, 0) - 2 * MarkSiteCameraCal(0, 0) + MarkSiteCameraCal(2, 0)) / 2;
	Vector123(0, 1) = (MarkSiteCameraCal(1, 1) - 2 * MarkSiteCameraCal(0, 1) + MarkSiteCameraCal(2, 1)) / 2;
	Vector123(0, 2) = (MarkSiteCameraCal(1, 2) - 2 * MarkSiteCameraCal(0, 2) + MarkSiteCameraCal(2, 2)) / 2;


	MarkSiteCameraCal.row(3) = CalPoint3Site(MarkSiteCameraCal.row(0), MarkSiteCameraFocus.row(3),
			Vector123, WandMatrix(3, 1));                                                    /////计算第四个点

	
	//////至此MarkSiteCameraCal 已经全部转换到标准尺度坐标系
	if (ResultType & 1)
	{
		TransMattemp.block(0, 0, 3, 3) = SpinMatCal(MarkSiteCameraCal, WandMatrix);
	}
	else if (ResultType & 2)
	{
		TransMattemp.block(0, 0, 3, 3) = PCAalignmentSpinMatCal(MarkSiteCamera, WandMatrix);
	}
	else
	{

	}
	
	TransMattemp.col(3) = SpintoShift(MarkSiteCameraFocus, WandMatrix, TransMattemp.block(0, 0, 3, 3)).transpose();

	TransMattemp.col(3) = (-TransMattemp.block(0, 0, 3, 3).inverse() * TransMattemp.col(3));
	/************************************************************************/
	vector<Point3d> PointWorld;         /////世界坐标系
	vector<Point2d> PointCameraProject; ////重映射相机坐标系
	vector<Point2d> PointCameraOri;     /////原始相机坐标系

	cv::Mat rvecCorrect = cv::Mat::zeros(3, 1, CV_64FC1);
	cv::Mat tvecCorrect = (Mat_<double>(3, 1) << 
		TransMattemp(0, 3), TransMattemp(1, 3), TransMattemp(2, 3));


	PointWorld.clear();
	PointCameraOri.clear();
	for (int j = 0; j < WandMatrix.rows(); j++)
	{
		PointWorld.push_back(Point3d(WandMatrix(j, 0),
			WandMatrix(j, 1), WandMatrix(j, 2)));
	}
	for (int j = 0; j < OriMarkSiteCamera.rows(); j++)
	{
		PointCameraOri.push_back(Point2d(OriMarkSiteCamera(j, 0), OriMarkSiteCamera(j, 1)));
	}
	//旋转矩阵转轴角
	cv::Mat rvecMat;
	cv::eigen2cv(Matrix<double, 3, 3>(TransMattemp.block(0, 0, 3, 3)), rvecMat);
	cv::Rodrigues(rvecMat, rvecCorrect);
	//////代入PnP初始化上述参量进行LM优化
	cv::solvePnP(PointWorld, PointCameraOri, CameraInterMat, CameraDistortMat,
		rvecCorrect, tvecCorrect, true);


	//轴角转旋转矩阵
	cv::Rodrigues(rvecCorrect, rvecMat);
	Eigen::Matrix<double, 3, 3> RvecMatrix;
	Eigen::Matrix<double, 3, 1> TvecMatrix;

	cv::cv2eigen(rvecMat, RvecMatrix);
	cv::cv2eigen(tvecCorrect, TvecMatrix);
	TransMattemp.block(0, 0, 3, 3) = RvecMatrix;
	TransMattemp.col(3) = TvecMatrix;
	CalibrateMat = TransMattemp;

	/************************************************************************/
	PointWorld.clear();
	PointCameraProject.clear();
	for (int j = 0; j < WandCheckData.WandMat.rows(); j++)
	{
		PointWorld.push_back(Point3d(WandCheckData.WandMat(j, 0), WandCheckData.WandMat(j, 1),
			WandCheckData.WandMat(j, 2)));
	}

#if 0
	cv::projectPoints(PointWorld, rvecCorrect, tvecCorrect, CameraInterMat,
		WandCheckData.distortParament, PointCameraProject);
#else
	//////重新投影到相机坐标系
	MyProjectPoints(PointWorld, rvecCorrect, tvecCorrect, CameraInterMat, PointCameraProject);
#endif
	/////计算重投影误差
	reProjectErr = 0;
	for (int i = 0; i < PointCameraProject.size(); i++)
	{
		reProjectErr += sqrt(pow((PointCameraProject[i].x - PointCameraOri[i].x), 2) 
			+ pow((PointCameraProject[i].y - PointCameraOri[i].y), 2));  /////欧氏距离
	}
	reProjectErr /= PointCameraProject.size();
	/************************************************************************/

	return reProjectErr;
}


void WandCalbrateProcess::WandSolveMat()
{
	int i = 0;
	CalibrateGlobal.clear();
	ReProjectErrGlobal.clear();
	if (MarkSiteCamera.size() == 0)
	{
		qDebug() << "未读取数据文件" << endl;
		if (GetPicMarkDataFromYml(MarkDataFile, MarkSiteCamera))
		{
			qDebug() << QString::fromStdString(MarkDataFile) << "打开数据文件成功" << endl;
		}
		else
		{
			qDebug() << QString::fromStdString(MarkDataFile) << "打开数据文件失败" << endl;
			return;
		}
	}
#define WANDSOLVEFUN 0
#if (0 == WANDSOLVEFUN)
	//////完全静止不动
	Matrix<double, 4, 3> MarkSiteCameratemp;
	vector<Matrix<double, 4, 3>> MarkSiteCameraAve;
	Matrix<double, 3, 4> CameraRT;
	double reProErr = 0;
	MarkSiteCameratemp.setZero();
	for (i = 0; i < MarkSiteCamera.size(); i++)
	{
		MarkSiteCameratemp += MarkSiteCamera[i];
	}
	MarkSiteCameratemp /= MarkSiteCamera.size();

	MarkSiteCameraAve.push_back(MarkSiteCameratemp);   ////直接计算所有图片的均值位置

 	for (i = 0; i < MarkSiteCameraAve.size(); i++)
 	{	
		reProErr = SolveWandCalibrate_SelfCal(MarkSiteCameraAve[i], CameraRT, 1);
 	}
	ReProjectErrGlobal.push_back(reProErr);  /////重投影误差
	CalibrateGlobal.push_back(CameraRT);
	cv::Mat CameraRTMat;
	cv::eigen2cv(CameraRT, CameraRTMat);
	CalibrateMatGlobal.push_back(CameraRTMat);
#elif (1 == WANDSOLVEFUN)

#else

#endif
	//WritePicMarkDataToYml(WorkFolder + "/GlobalSite.yml", MarkSiteGlobal);
	return;
}

/////文件夹参数为当前相机的文件夹
void WandCalbrateProcess::WandCalPoint(string FolderName)
{
	int i = 0;
	int index = 0;

	string FileType = ".bmp";                               //需要查找的文件类型
	vector<string>FilesName;                              //存放文件名的容器
	char FileString[120];

	getFilesName(FolderName, FileType, FilesName);        //标定所用图像文件的路径
	vector<Matrix<double, 4, 3>> MarkSiteBuff;
	std::cout << "查找有效点 >>" << endl;
#if 1
	for (i = 0; i < FilesName.size(); i++)
	{
		
		Mat imgbuff = imread(FilesName[i], 0);
		
		cv::threshold(imgbuff, imgbuff, WandCheckData.Binarythreshold, 255, THRESH_BINARY);

		vector<vector<cv::Point>> contours;
		vector<cv::Vec4i>hierarchy;
		Point3d CenterPoint;
		findContours(imgbuff, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE);

		if (contours.size() > WandCheckData.SizeLimit)
			continue;

		vector<cv::Point3f> CirqueCenter;
		vector<vector<cv::Point>> CirqueContours;
		
		for (index = 0; index < contours.size(); index++)
		{
			if (contours[index].size() > WandCheckData.SizeLimit)
			{
				if (FindCirqueFromContour(CenterPoint, contours, index, WandCheckData.CirqueLimit) == true)
				{
					CirqueCenter.push_back(CenterPoint);
					CirqueContours.push_back(contours[index]);
				}
			}
		}

		if (CirqueCenter.size() != WandCheckData.MarkNum)
		{
			cout << "无效图" << endl;
			continue;
		}
		else
		{
			int MatchCount = combinator(WandCheckData.MarkNum, 3);                   ////计算组合数
			Matrix<double, 4, 3> temp, out;
			temp.setZero();
			out.setZero();
			double  MinCross = 100000000;
			for (int j = 0; j < WandCheckData.MarkNum; j++)
			{
				for (int k = j + 1; k < WandCheckData.MarkNum; k++)
				{
					for (int z = k + 1; z < WandCheckData.MarkNum; z++)
					{
						temp(0, 0) = CirqueCenter[j].x;
						temp(0, 1) = CirqueCenter[j].y;
						temp(0, 2) = 0;
						temp(1, 0) = CirqueCenter[k].x;
						temp(1, 1) = CirqueCenter[k].y;
						temp(1, 2) = 0;
						temp(2, 0) = CirqueCenter[z].x;
						temp(2, 1) = CirqueCenter[z].y;
						temp(2, 2) = 0;

						/////直接特殊情况处理
						if (WandCheckData.MarkNum == 4)
						{
							for (int a = 0; a < WandCheckData.MarkNum; a++)
							{
								if (a != z && a != j && a != k)
								{
									temp(3, 0) = CirqueCenter[a].x;
									temp(3, 1) = CirqueCenter[a].y;
									temp(3, 2) = 0;
								}
							}
						}
						double Crosstemp = abs((temp(0, 0) - temp(1, 0)) * (temp(0, 1) - temp(2, 1)) - (temp(0, 0) - temp(2, 0)) * (temp(0, 1) - temp(1, 1)));
						if (Crosstemp < MinCross)
						{
							out = temp;
							MinCross = Crosstemp;
						}
					}
				}
			}

			if (MinCross > WandCheckData.StraightLimit)
			{
				cout << "直线拟合度不足" << endl;
				continue;
			}
			else
			{
				double S[5];
				temp = out;
				S[0] = (out(1, 0) - out(0, 0)) * (out(2, 0) - out(0, 0)) + (out(1, 1) - out(0, 1)) * (out(2, 1) - out(0, 1));
				S[1] = (out(2, 0) - out(1, 0)) * (out(0, 0) - out(1, 0)) + (out(2, 1) - out(1, 1)) * (out(0, 1) - out(1, 1));
				S[2] = (out(1, 0) - out(2, 0)) * (out(0, 0) - out(2, 0)) + (out(1, 1) - out(2, 1)) * (out(0, 1) - out(2, 1));
				int minnum = 0, maxnum = 2;
				S[3] = S[0];
				S[4] = S[2];
				for (int j = 0; j < 3; j++)
				{
					if (S[j] < S[3])
					{
						minnum = j;
						S[3] = S[j];
					}
					if (S[j] > S[4])
					{
						maxnum = j;
						S[4] = S[j];
					}
				}
				temp(1, 0) = out(minnum, 0);
				temp(1, 1) = out(minnum, 1);
				temp(2, 0) = out(maxnum, 0);
				temp(2, 1) = out(maxnum, 1);
				for (int j = 0; j < 3; j++)
				{
					if (j != minnum && j != maxnum)
					{
						temp(0, 0) = out(j, 0);
						temp(0, 1) = out(j, 1);
						break;
					}
				}
				out = temp;
				out(0, 2) = 1;
				out(1, 2) = 1;
				out(2, 2) = 1;
				out(3, 2) = 1;
				MarkSiteBuff.push_back(out);

#if 0
				Mat imgBinary, imgRGB;
				cvtColor(imgbuff, imgRGB, COLOR_GRAY2BGR);
				
				drawCross(imgRGB, Point2d(out(0, 0), out(0, 1)), Scalar(0, 0, 255), 200, 20);
				drawCross(imgRGB, Point2d(out(1, 0), out(1, 1)), Scalar(0, 255, 0), 200, 20);
				drawCross(imgRGB, Point2d(out(2, 0), out(2, 1)), Scalar(0, 255, 0), 200, 20);
				drawCross(imgRGB, Point2d(out(3, 0), out(3, 1)), Scalar(255, 0, 0), 200, 20);


				resize(imgRGB, imgRGB, Size(imgRGB.cols / 4, imgRGB.rows / 4));
				resize(imgbuff, imgBinary, Size(imgbuff.cols / 4, imgbuff.rows / 4));
				imshow("imgRGB", imgRGB);
				waitKey(100);//暂停0.1S
#endif
#if 0
				sprintf_s(FileString, "%s/WandPictemp%06d.bmp", FolderName, i);
				imwrite(FileString, imgRGB);
#endif
			}
		}	
		cout << "第" << i << "张" << "   " << CirqueCenter.size() << endl;
		
	}
	sprintf_s(FileString, "%s/MarkData.yml", FolderName.c_str());
	WritePicMarkDataToYml(FileString, MarkSiteBuff);
#endif
}

void WandCalbrateProcess::WandL3MatCal(Matrix<double, 3, 3> InputMat, Matrix<double, 3, 3> MarkMat, Matrix<double, 3, 3>& Solve)
{
		Matrix<double, 9, 9> A;
		A.setZero();

		A(0, 0) = 1;
		A(0, 2) = -InputMat(0, 0);
		//
		A(1, 1) = 1;
		A(1, 2) = -InputMat(0, 1);
		//
		A(2, 3) = 1;
		A(2, 5) = -InputMat(1, 0);
		//
		A(3, 4) = 1;
		A(3, 5) = -InputMat(1, 1);
		//
		A(4, 6) = 1;
		A(4, 8) = -InputMat(2, 0);
		//
		A(5, 7) = 1;
		A(5, 8) = -InputMat(2, 1);
		//
		A(6, 0) = MarkMat(2, 0) - MarkMat(1, 0);
		A(6, 3) = -MarkMat(2, 0);
		A(6, 6) = MarkMat(1, 0);
		//
		A(7, 1) = A(6, 0);
		A(7, 4) = A(6, 3);
		A(7, 7) = A(6, 6);
		//
		A(8, 2) = A(6, 0);
		A(8, 5) = A(6, 3);
		A(8, 8) = A(6, 6);

		JacobiSVD<Eigen::MatrixXd> svd(A, ComputeThinU | ComputeThinV);
		Matrix<double, 9, 9> V = svd.matrixV();

		//Solve.setZero();

		for (int k = 0; k < 3; k++)
		{
			for (int j = 0; j < 3; j++)
			{
				if (V(2, 8) < 0)
				{
					Solve(k, j) = -V(k * 3 + j, 8);
				}
				else
					Solve(k, j) = V(k * 3 + j, 8);
			}
		}
}
 
void WandCalbrateProcess::WandL3MatCal(vector<Matrix<double, 4, 3>> InputMat, Matrix<double, 4, 3> MarkMat, vector<Matrix<double, 4, 3>> &MarkSiteGlobal)
{
	for (int i = 0; i < InputMat.size(); i++)
	{
		Matrix<double, 9, 9> A;
		A.setZero();

		Matrix<double, 4, 3> CalMat = InputMat[i];

		A(0, 0) = 1;
		A(0, 2) = -CalMat(0, 0);
		//
		A(1, 1) = 1;
		A(1, 2) = -CalMat(0, 1);
		//
		A(2, 3) = 1;
		A(2, 5) = -CalMat(1, 0);
		//
		A(3, 4) = 1;
		A(3, 5) = -CalMat(1, 1);
		//
		A(4, 6) = 1;
		A(4, 8) = -CalMat(2, 0);
		//
		A(5, 7) = 1;
		A(5, 8) = -CalMat(2, 1);
		//
		A(6, 0) = MarkMat(2, 0) - MarkMat(1, 0);
		A(6, 3) = -MarkMat(2, 0);
		A(6, 6) = MarkMat(1, 0);
		//
		A(7, 1) = A(6, 0);
		A(7, 4) = A(6, 3);
		A(7, 7) = A(6, 6);
		//
		A(8, 2) = A(6, 0);
		A(8, 5) = A(6, 3);
		A(8, 8) = A(6, 6);

		JacobiSVD<Eigen::MatrixXd> svd(A, ComputeThinU | ComputeThinV);
		Matrix<double, 9, 9> V = svd.matrixV();

		Matrix<double, 4, 3> Solve;
		Solve.setZero();

		for (int k = 0; k < 3; k++)
		{
			for (int j = 0; j < 3; j++)
			{
				if (V(2, 8) < 0)
				{
					Solve(k, j) = -V(k * 3 + j, 8);
				}
				else
					Solve(k, j) = V(k * 3 + j, 8);
			}
		}
		MarkSiteGlobal.push_back(Solve); 
	}
}


void WandCalbrateProcess::CameraCalbrateAll(vector<WandCalbrateProcess *> &WandCalbrateProcessList)
{
	int i = 0;
	int CameraNum = WandCalbrateProcessList.size();
	for (i = 0; i < CameraNum; i++)
	{
#if 1
		if (WandCalbrateProcessList[i]->CalibrateGlobal.size() > 0)
		{
			WandCalbrateProcessList[i]->CameraParament.CameraOuterMat = WandCalbrateProcessList[i]->CalibrateMatGlobal[0];
			WandCalbrateProcessList[i]->CameraParament.CameraInterMat = WandCalbrateProcessList[i]->WandCheckData.InterParamentMat.clone();
			WandCalbrateProcessList[i]->CameraParament.CameraDisMat = WandCalbrateProcessList[i]->WandCheckData.distortParament.clone();
			WandCalbrateProcessList[i]->CameraParament.ReProjectErr = WandCalbrateProcessList[i]->ReProjectErrGlobal[0];
		}
		else
			break;
#endif	
	}
}

bool WandCalbrateProcess::WriteCameraParamentToFile(string FileName, const vector<WandCalbrateProcess *> &WandCalbrateProcessList)
{
	FileStorage fs(FileName, FileStorage::WRITE);
	fs.state = FileStorage::UNDEFINED;
	if (fs.isOpened())
	{
		char MatrixnumString[100];
		cv::Mat Mattemp;
		int cameraNum = WandCalbrateProcessList.size();
		fs << "CameraNum" << cameraNum;

		for (int i = 0; i <cameraNum; i++)
		{
			fs.elname = WandCalbrateProcessList[i]->CameraString + "_OutMat";
			fs << WandCalbrateProcessList[i]->CameraParament.CameraOuterMat;

			fs.elname = WandCalbrateProcessList[i]->CameraString + "_InterMat";
			fs << WandCalbrateProcessList[i]->CameraParament.CameraInterMat;

			fs.elname = WandCalbrateProcessList[i]->CameraString + "_DisMat";
			fs << WandCalbrateProcessList[i]->CameraParament.CameraDisMat;

			fs.elname = WandCalbrateProcessList[i]->CameraString + "_ReProjectErr";
			fs << WandCalbrateProcessList[i]->CameraParament.ReProjectErr;
		}
		fs.release();
		return true;
	}
	return false;
}

bool WandCalbrateProcess::WriteCameraParamentToFile(string FileName, const vector<CameraParamenttypedef> &CameraMatrix)
{
	FileStorage fs(FileName, FileStorage::WRITE);
	fs.state = FileStorage::UNDEFINED;
	if (fs.isOpened())
	{
		char MatrixnumString[100];
		cv::Mat Mattemp;
		int cameraNum = CameraMatrix.size();
		fs << "CameraNum" << cameraNum;

		for (int i = 0; i < cameraNum; i++)
		{
			sprintf_s(MatrixnumString, "Camera_%03d_OutMat", i);
			fs.elname = MatrixnumString;
			fs << CameraMatrix[i].CameraOuterMat;

			sprintf_s(MatrixnumString, "Camera_%03d_InterMat", i);
			fs.elname = MatrixnumString;
			fs << CameraMatrix[i].CameraInterMat;

			sprintf_s(MatrixnumString, "Camera_%03d_DisMat", i);
			fs.elname = MatrixnumString;
			fs << CameraMatrix[i].CameraDisMat;

			sprintf_s(MatrixnumString, "Camera_%03d_ReProjectErr", i);
			fs.elname = MatrixnumString;
			fs << CameraMatrix[i].ReProjectErr;
		}
		fs.release();
		return true;
	}
	return false;
}

bool WandCalbrateProcess::GetCameraParamentFromFile(string FileName, vector<CameraParamenttypedef> &CameraMatrix)
{
	FileStorage fin(FileName, FileStorage::READ);
	if (fin.isOpened())
	{
		if (fin["CameraNum"].empty() == false)
		{
			int cameraNum = 0;
			char MatrixnumString[100];
			
			fin["CameraNum"] >> cameraNum;
			for (int i = 0; i < cameraNum; i++)
			{
				CameraParamenttypedef CameraMatrixtemp;
	
				sprintf_s(MatrixnumString, "Camera_%03d_OutMat", i);
				fin[MatrixnumString] >>  CameraMatrixtemp.CameraOuterMat;

				sprintf_s(MatrixnumString, "Camera_%03d_InterMat", i);
				fin[MatrixnumString] >> CameraMatrixtemp.CameraInterMat;
	
				sprintf_s(MatrixnumString, "Camera_%03d_DisMat", i);
				fin[MatrixnumString] >> CameraMatrixtemp.CameraDisMat;
	
				CameraMatrix.push_back(CameraMatrixtemp);
			}
		}
		fin.release();
		return true;
	}
	return false;
}

bool WandCalbrateProcess::GetPicMarkDataFromYml(string FileName, vector<Matrix<double, 4, 3>> &MarkSiteBuff)
{
	FileStorage fin(FileName, FileStorage::READ);
	if (fin.isOpened())
	{
		int Matnum;
		Mat Mattemp;
		char MatrixnumString[100];

		if (fin["MarkNum"].empty() == false)
		{
			fin["MarkNum"] >> Matnum;
			for (int i = 0; i < Matnum - 1; i++)
			{
				sprintf_s(MatrixnumString, "Mark%06d", i);
				fin[MatrixnumString] >> Mattemp;

				if (Mattemp.rows == 4 && Mattemp.cols == 3)
				{
					Matrix<double, 4, 3> WandMatrix;
					cv::cv2eigen(Mattemp, WandMatrix);
					MarkSiteBuff.push_back(WandMatrix);
				}
			}
		}
		fin.release();
		return true;
	}
	return false;
}

bool WandCalbrateProcess::WritePicMarkDataToYml(string FileName, vector<Matrix<double, 4, 3>> &MarkSiteBuff)
{
	FileStorage fs(FileName, FileStorage::WRITE);
	fs.state = FileStorage::UNDEFINED;
	if (fs.isOpened())
	{
		char MatrixnumString[100];
		int MarkSiteSize = MarkSiteBuff.size();
		
		fs << "MarkNum" << MarkSiteSize;
		for (int i = 0; i < MarkSiteBuff.size(); i++)
		{
			sprintf_s(MatrixnumString, "Mark%06d", i);
			Mat Mattemp;
			cv::eigen2cv(MarkSiteBuff[i], Mattemp);
			fs.elname = MatrixnumString;     ////不知道怎么解决只能这样强制的给节点
			fs << Mattemp;// MyConverter::Matrix43dtoCvMat(MarkSiteBuff[i]);
		}
		fs.release();
  		return true;
	}
	
	return false;
}

bool WandCalbrateProcess::GetWandCalbrateInit(string FileName)
{
	FileStorage fin(FileName, FileStorage::READ);
	if (fin.isOpened())
	{
		if (fin["BinaryTh"].empty())
		{
			return false;
		}
		fin["BinaryTh"] >> WandCheckData.Binarythreshold;
		fin["SizeLimit"] >> WandCheckData.SizeLimit;
		fin["CirqueLimt"] >> WandCheckData.CirqueLimit;
		fin["StraightLimit"] >> WandCheckData.StraightLimit;
		fin["MarkNum"] >> WandCheckData.MarkNum;

	
		fin["InterMat"] >> WandCheckData.InterParamentMat;
		cv::cv2eigen(WandCheckData.InterParamentMat, WandCheckData.InterParament);

		Mat Mattemp;
		fin["WandMat"] >> Mattemp;
		cv::cv2eigen(Mattemp, WandCheckData.WandMat);

		Mat MatRot = cv::Mat(5, 1, CV_64FC1);
		fin["distortMat"] >> MatRot;
	    MatRot.copyTo(WandCheckData.distortParament);

		fin["CheckCount"] >> WandCheckData.CheckCount;
		fin.release();
		return true;
	}
	else
		return false;

}

bool WandCalbrateProcess::WriteWandCalbrateData(string FileName)
{
	FileStorage fout(FileName, FileStorage::WRITE);
	if (fout.isOpened())
	{

		fout << "BinaryTh" << WandCheckData.Binarythreshold;
		fout << "SizeLimit" << WandCheckData.SizeLimit;
		fout << "CirqueLimt" << WandCheckData.CirqueLimit;
		fout << "StraightLimit" << WandCheckData.StraightLimit;
		fout << "MarkNum" << WandCheckData.MarkNum;
		fout << "InterMat" << MyConverter::Matrix3dtoCvMat(WandCheckData.InterParament);
		fout << "WandMat" << MyConverter::Matrix43dtoCvMat(WandCheckData.WandMat);
		fout << "distortMat" << WandCheckData.distortParament;
		fout << "CheckCount" << WandCheckData.CheckCount;

		fout.release();
		return true;
	}
	else
		return false;

}

bool WandCalbrateProcess::WriteFinalCalMat(string FileName, vector<Matrix<double, 4, 3>> CalMat)
{
	FileStorage fs(FileName, FileStorage::WRITE);
	char MatrixnumString[100];
	int MarkSiteSize = CalMat.size();
	fs << "CameraNum" << MarkSiteSize;
	for (int i = 0; i < CalMat.size(); i++)
	{
		//sprintf_s(MatrixnumString, "Camera%d", i);
		fs << "Mat" << MyConverter::Matrix43dtoCvMat(CalMat[i]);
	}
	fs.release();
	return true;
}


CalibrateThread::CalibrateThread(string dir, int index)
{

	GetWandCalbrateInit(dir + "/InterParament.yml");
	FileDir = dir;
	FrameCount = 0;
	ValidCount = 0;
	FullImg = false;
	CameraIndex = index;
}

void CalibrateThread::ResultToFile()
{
	WritePicMarkDataToYml(FileDir + "/MarkData.yml", MarkSiteBuff);
}


void CalibrateThread::CalibrateImgProcess(std::vector<CirquePointtypedef> * CirquePointTree)
{
	FrameCount++;
	if (ValidCount > WandCheckData.CheckCount && FullImg == false)
	{
		FullImg = true;
		emit ResultFull(FullImg);
	}

	Matrix<double, 4, 3> temp, out;
	temp.setZero();
	out.setZero();
	out(0, 2) = -1000;

	if ((*CirquePointTree).size() != WandCheckData.MarkNum)
	{
		qDebug() << "标定球数量错误" << endl;
	}
	else
	{
		int MatchCount = combinator(WandCheckData.MarkNum, 3);                   ////计算组合数

		double  MinCross = 1000;
		for (int j = 0; j < WandCheckData.MarkNum; j++)
		{
			for (int k = j + 1; k < WandCheckData.MarkNum; k++)
			{
				for (int z = k + 1; z < WandCheckData.MarkNum; z++)
				{
					temp(0, 0) = (*CirquePointTree)[j].CirquePoint.x;
					temp(0, 1) = (*CirquePointTree)[j].CirquePoint.y;
					temp(0, 2) = 0;
					temp(1, 0) = (*CirquePointTree)[k].CirquePoint.x;
					temp(1, 1) = (*CirquePointTree)[k].CirquePoint.y;
					temp(1, 2) = 0;
					temp(2, 0) = (*CirquePointTree)[z].CirquePoint.x;
					temp(2, 1) = (*CirquePointTree)[z].CirquePoint.y;
					temp(2, 2) = 0;

					/////直接特殊情况处理
					if (WandCheckData.MarkNum == 4)
					{
						for (int a = 0; a < WandCheckData.MarkNum; a++)
						{
							if (a != z && a != j && a != k)
							{
								temp(3, 0) = (*CirquePointTree)[a].CirquePoint.x;
								temp(3, 1) = (*CirquePointTree)[a].CirquePoint.y;
								temp(3, 2) = 0;
							}
						}
					}
					double Crosstemp = abs((temp(0, 0) - temp(1, 0)) * (temp(0, 1) - temp(2, 1)) - (temp(0, 0) - temp(2, 0)) * (temp(0, 1) - temp(1, 1)));
					double LengthCross = sqrt(pow((temp(0, 0) - temp(1, 0)), 2) + pow((temp(0, 1) - temp(1, 1)), 2)) * sqrt(pow((temp(0, 1) - temp(2, 1)), 2) + pow((temp(0, 0) - temp(2, 0)), 2));
					Crosstemp = asin(Crosstemp / LengthCross) * 180 / M_PI;
					if (Crosstemp < MinCross)
					{
						out = temp;
						MinCross = Crosstemp;
					}
				}
			}
		}
		if (MinCross > WandCheckData.StraightLimit)
		{
			out(0, 2) = -1000;
			cout << "最小偏差角" << MinCross << "直线拟合度不足" << endl;
		}
		else
		{
			double S[5];
			temp = out;
			S[0] = (out(1, 0) - out(0, 0)) * (out(2, 0) - out(0, 0)) + (out(1, 1) - out(0, 1)) * (out(2, 1) - out(0, 1));
			S[1] = (out(2, 0) - out(1, 0)) * (out(0, 0) - out(1, 0)) + (out(2, 1) - out(1, 1)) * (out(0, 1) - out(1, 1));
			S[2] = (out(1, 0) - out(2, 0)) * (out(0, 0) - out(2, 0)) + (out(1, 1) - out(2, 1)) * (out(0, 1) - out(2, 1));
			int minnum = 0, maxnum = 2;
			S[3] = S[0];
			S[4] = S[2];
			for (int j = 0; j < 3; j++)
			{
				if (S[j] < S[3])
				{
					minnum = j;
					S[3] = S[j];
				}
				if (S[j] > S[4])
				{
					maxnum = j;
					S[4] = S[j];
				}
			}
			temp(1, 0) = out(minnum, 0);
			temp(1, 1) = out(minnum, 1);
			temp(2, 0) = out(maxnum, 0);
			temp(2, 1) = out(maxnum, 1);
			for (int j = 0; j < 3; j++)
			{
				if (j != minnum && j != maxnum)
				{
					temp(0, 0) = out(j, 0);
					temp(0, 1) = out(j, 1);
					break;
				}
			}
			out = temp;
			out(0, 2) = FrameCount;
			out(1, 2) = 1;
			out(2, 2) = 1;
			out(3, 2) = 1;
			ValidCount++;
			cout << ValidCount << endl;
		}
	}
	MarkSiteBuff.push_back(out);
}

void CalibrateThread::CalibrateImgProcess(cv::Mat * imgori)
{
	int index = 0;
	Mat imgbuff = imgori->clone();
	
	FrameCount++;

	if (ValidCount > WandCheckData.CheckCount && FullImg == false)
	{
		FullImg = true;
		emit ResultFull(FullImg);
	}
	Matrix<double, 4, 3> temp, out;
	temp.setZero();
	out.setZero();
	out(0, 2) = -1000;
	cv::threshold(imgbuff, imgbuff, WandCheckData.Binarythreshold, 255, THRESH_BINARY);

	vector<vector<cv::Point>> contours;
	vector<cv::Vec4i>hierarchy;
	Point3d CenterPoint;
	cv::findContours(imgbuff, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE);

	if (contours.size() < WandCheckData.SizeLimit)
	{
		vector<cv::Point3f> CirqueCenter;
		vector<vector<cv::Point>> CirqueContours;

		for (index = 0; index < contours.size(); index++)
		{
			if (contours[index].size() > WandCheckData.SizeLimit)
			{
				if (FindCirqueFromContour(CenterPoint, contours, index, WandCheckData.CirqueLimit) == true)
				{
					CirqueCenter.push_back(CenterPoint);
					CirqueContours.push_back(contours[index]);
				}
			}
		}

		if (CirqueCenter.size() != WandCheckData.MarkNum)
		{
			out(0, 2) = -1000;
			cout << "无效图" << endl;
		}
		else
		{
			int MatchCount = combinator(WandCheckData.MarkNum, 3);                   ////计算组合数
			
			double  MinCross = 1000;
			for (int j = 0; j < WandCheckData.MarkNum; j++)
			{
				for (int k = j + 1; k < WandCheckData.MarkNum; k++)
				{
					for (int z = k + 1; z < WandCheckData.MarkNum; z++)
					{
						temp(0, 0) = CirqueCenter[j].x;
						temp(0, 1) = CirqueCenter[j].y;
						temp(0, 2) = 0;
						temp(1, 0) = CirqueCenter[k].x;
						temp(1, 1) = CirqueCenter[k].y;
						temp(1, 2) = 0;
						temp(2, 0) = CirqueCenter[z].x;
						temp(2, 1) = CirqueCenter[z].y;
						temp(2, 2) = 0;

						/////直接特殊情况处理
						if (WandCheckData.MarkNum == 4)
						{
							for (int a = 0; a < WandCheckData.MarkNum; a++)
							{
								if (a != z && a != j && a != k)
								{
									temp(3, 0) = CirqueCenter[a].x;
									temp(3, 1) = CirqueCenter[a].y;
									temp(3, 2) = 0;
								}
							}
						}
						double Crosstemp = abs((temp(0, 0) - temp(1, 0)) * (temp(0, 1) - temp(2, 1)) - (temp(0, 0) - temp(2, 0)) * (temp(0, 1) - temp(1, 1)));
						double LengthCross =  sqrt(pow((temp(0, 0) - temp(1, 0)), 2) + pow((temp(0, 1) - temp(1, 1)), 2)) * sqrt(pow((temp(0, 1) - temp(2, 1)), 2) + pow((temp(0, 0) - temp(2, 0)), 2));
						Crosstemp = asin(Crosstemp / LengthCross) * 180 /  M_PI;
						if (Crosstemp < MinCross)
						{
							out = temp;
							MinCross = Crosstemp;
						}
					}
				}
			}

			if (MinCross > WandCheckData.StraightLimit)
			{
				out(0, 2) = -1000;

				cout << "最小偏差角" << MinCross << "直线拟合度不足" << endl;
			}
			else
			{
				double S[5];
				temp = out;
				S[0] = (out(1, 0) - out(0, 0)) * (out(2, 0) - out(0, 0)) + (out(1, 1) - out(0, 1)) * (out(2, 1) - out(0, 1));
				S[1] = (out(2, 0) - out(1, 0)) * (out(0, 0) - out(1, 0)) + (out(2, 1) - out(1, 1)) * (out(0, 1) - out(1, 1));
				S[2] = (out(1, 0) - out(2, 0)) * (out(0, 0) - out(2, 0)) + (out(1, 1) - out(2, 1)) * (out(0, 1) - out(2, 1));
				int minnum = 0, maxnum = 2;
				S[3] = S[0];
				S[4] = S[2];
				for (int j = 0; j < 3; j++)
				{
					if (S[j] < S[3])
					{
						minnum = j;
						S[3] = S[j];
					}
					if (S[j] > S[4])
					{
						maxnum = j;
						S[4] = S[j];
					}
				}
				temp(1, 0) = out(minnum, 0);
				temp(1, 1) = out(minnum, 1);
				temp(2, 0) = out(maxnum, 0);
				temp(2, 1) = out(maxnum, 1);
				for (int j = 0; j < 3; j++)
				{
					if (j != minnum && j != maxnum)
					{
						temp(0, 0) = out(j, 0);
						temp(0, 1) = out(j, 1);
						break;
					}
				}
				out = temp;
				out(0, 2) = FrameCount;
				out(1, 2) = 1;
				out(2, 2) = 1;
				out(3, 2) = 1;
				ValidCount++;
				cout << ValidCount << endl;
			}
		}
	}
	MarkSiteBuff.push_back(out);
}
