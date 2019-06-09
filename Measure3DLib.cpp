#include "Measure3DLib.h"
#include <opencv2/core/eigen.hpp>
#include "SolvePnP_Matrix.h"
#include "Calculate3D.h"
#include "MyMathLib.h"

using namespace std;

Measure3DPointThread::Measure3DPointThread(QObject *parent) :
	QThread(parent)
{
	CalFrameCount = 0;
	PointFrameCount = 0;
	CheckCount = 0;
	ReCheckCount = 0;
	CameraParamentTree.clear();
	DeviceMarkPointTree.clear();
	DeviceCheckBox.clear();
}

void Measure3DPointThread::AddCameraParament(CameraParamenttypedef &camera, std::string str)
{
	InputCameraDevicetypedef temp;
	temp.CameraParament = camera;
	temp.CameraDeviceStr = str;
	CameraParamentTree.push_back(temp);
	InputCameraPointtypedef pt;
	pt.CameraDeviceStr = str;
	DeviceMarkPointTree.push_back(pt);
	DeviceCheckBox.push_back(false);
}

void Measure3DPointThread::release()
{
	std::vector<InputCameraPointtypedef>().swap(DeviceMarkPointTree);
	std::vector<bool>().swap(DeviceCheckBox);
	std::vector<InputCameraDevicetypedef>().swap(CameraParamentTree);
}

void Measure3DPointThread::FromCameraMark(std::vector<CirquePointtypedef> * CameraDeviceMark)
{
	ImgMainProcess *CameraSend = (ImgMainProcess *)sender();
	for (int i = 0; i < DeviceMarkPointTree.size(); i++)
	{
		if (CameraSend->CameraDeviceStr == DeviceMarkPointTree[i].CameraDeviceStr)
		{
			if (DeviceCheckBox[i] == true)
			{
				for (int j = 0; j < DeviceMarkPointTree.size(); j++)
				{
					DeviceCheckBox[j] = false;
				}
				CheckCount = 0;
				break;
			}
			else
			{
				CheckCount++;
				DeviceCheckBox[i] = true;
			}
			DeviceMarkPointTree[i].MarkPoint.swap(*CameraDeviceMark);
		}
	}
	if(CheckCount == CameraParamentTree.size())
	{
		for (int j = 0; j < DeviceMarkPointTree.size(); j++)
		{
			DeviceCheckBox[j] = false;
		}
		CheckCount = 0;
		PointFrameCount++;
	}
}


void cvPoint3D2Matrix(vector<vector<Matrix<double, 1, 3>>> &MatrixTree, vector<CirquePointtypedef> MarkPoint)
{
	vector<Matrix<double, 1, 3>> temp;
	for (int i = 0; i < MarkPoint.size(); i++)
	{
		Matrix<double, 1, 3> m;
		m << MarkPoint[i].CirquePoint.x, MarkPoint[i].CirquePoint.y, MarkPoint[i].CirquePoint.z;
		temp.push_back(m);
	}
	MatrixTree.push_back(temp);
}

extern void GetRigidTrans3D(vector<Point3d> srcPoints, vector<Point3d> dstPoints, int pointsNum, TRigidTrans3D& transform);

void Measure3DPointThread::AddWorldPoint(std::vector<Point3d> &q, std::vector<Point3d> &t)
{
	RealworldPoint3D.swap(q);
	selectPoint3D.swap(t);
}

void Measure3DPointThread::ReceiveCameraChange(std::vector<std::vector<Eigen::Matrix<double, 1, 3>>>  *q, std::vector<cv::Point3d> *t)
{
	ReCheckCount--;
	if (RealworldPoint3D.size() == t->size() && t->size() == selectPoint3D.size() && RealworldPoint3D.size() >= 4)
	{
		if (ReCheckCount == 9)
		{
			WandCalbrateProcess::GetCameraParamentFromFile("./parament/CameraParament.yml", ExternCameraParaMat);
			Matrix<double, 4, 4> TransMatrix;
			Matrix<double, 4, 4> oriMatrix;

			TRigidTrans3D TRigidTrans;
			GetRigidTrans3D(RealworldPoint3D, selectPoint3D, selectPoint3D.size(), TRigidTrans);
			TransMatrix.setZero();
			for (int i = 0; i < 3; i++)
			{
				for (int j = 0; j < 3; j++)
				{
					TransMatrix(i, j) = TRigidTrans.matR[i * 3 + j];
				}
			}
			TransMatrix(0, 3) = TRigidTrans.X;
			TransMatrix(1, 3) = TRigidTrans.Y;
			TransMatrix(2, 3) = TRigidTrans.Z;
			TransMatrix(3, 3) = 1;

			for (int i = 0; i < ExternCameraParaMat.size(); i++)
			{
				oriMatrix.setZero();
				Matrix<double, 3, 4> temp;
				cv2eigen(ExternCameraParaMat[i].CameraOuterMat, temp);
				oriMatrix.block(0, 0, 3, 4) = temp;
				oriMatrix(3, 3) = 1;

				oriMatrix = oriMatrix * TransMatrix.inverse();
				temp = oriMatrix.block(0, 0, 3, 4);
				eigen2cv(temp, ExternCameraParaMat[i].CameraOuterMat);
			}
		}
		
		for (int i = 0; i < ExternCameraParaMat.size(); i++)
		{
			vector<Point2d> PointCameraOri;     /////原始相机坐标系
			PointCameraOri.clear();
			cv::Mat rvecCorrect = cv::Mat::zeros(3, 1, CV_64FC1);
			cv::Mat tvecCorrect = (Mat_<double>(3, 1) <<
				ExternCameraParaMat[i].CameraOuterMat.at<double>(0, 3), ExternCameraParaMat[i].CameraOuterMat.at<double>(1, 3), ExternCameraParaMat[i].CameraOuterMat.at<double>(2, 3));

			cv::Mat rvecMat;
			Matrix<double, 3, 4> TransMattemp;
			cv::cv2eigen(ExternCameraParaMat[i].CameraOuterMat, TransMattemp);
			cv::eigen2cv(Matrix<double, 3, 3>(TransMattemp.block(0, 0, 3, 3)), rvecMat);
			cv::Rodrigues(rvecMat, rvecCorrect);

			for (int j = 0; j < selectPoint3D.size() && j < q->size(); j++)
			{
				for (int k = 0; k < t->size(); k++)
				{
					double dd = MyMathLib::CalTowPointDistance((*t)[k], selectPoint3D[j]);
					if (abs(dd) < 100)
					{
						PointCameraOri.push_back(Point2d((*q)[k][i](0, 0), (*q)[k][i](0, 1)));
						break;
					}
				}
			}
			if (PointCameraOri.size() != selectPoint3D.size())
			{
				qDebug() << "重建失败" << endl;
				disconnect(this, SIGNAL(SendCameraChange(std::vector<std::vector<Eigen::Matrix<double, 1, 3>>>  *, std::vector<cv::Point3d> *)),
					this, SLOT(ReceiveCameraChange(std::vector<std::vector<Eigen::Matrix<double, 1, 3>>>  *, std::vector<cv::Point3d> *)));
				return;
			}
			/*
			for (int k = 0; k < t->size(); k++)
			{
				PointCameraOri.push_back(Point2d((*q)[i].MarkPoint[k].CirquePoint.x, (*q)[i].MarkPoint[k].CirquePoint.y));
			}*/
			//旋转矩阵转轴角
			//////代入PnP初始化上述参量进行LM优化
		//	if(ReCheckCount == 9)
		//		cv::solvePnP(RealworldPoint3D, PointCameraOri, ExternCameraParaMat[i].CameraInterMat, ExternCameraParaMat[i].CameraDisMat,
		//			rvecCorrect, tvecCorrect, false);
		//	else
			cv::solvePnP(RealworldPoint3D, PointCameraOri, ExternCameraParaMat[i].CameraInterMat, ExternCameraParaMat[i].CameraDisMat,
					rvecCorrect, tvecCorrect, true, cv::SOLVEPNP_ITERATIVE);

			cv::Rodrigues(rvecCorrect, rvecMat);
			Eigen::Matrix<double, 3, 3> RvecMatrix;
			Eigen::Matrix<double, 3, 1> TvecMatrix;

			cv::cv2eigen(rvecMat, RvecMatrix);
			cv::cv2eigen(tvecCorrect, TvecMatrix);
			TransMattemp.block(0, 0, 3, 3) = RvecMatrix;
			TransMattemp.col(3) = TvecMatrix;
			cv::eigen2cv(TransMattemp, ExternCameraParaMat[i].CameraOuterMat);
		}
	}
	if (ReCheckCount == 0)
	{
		WandCalbrateProcess::WriteCameraParamentToFile("./parament/CameraParament.yml", ExternCameraParaMat);
		qDebug() << "重建完成" << endl;
		disconnect(this, SIGNAL(SendCameraChange(std::vector<std::vector<Eigen::Matrix<double, 1, 3>>>  *, std::vector<cv::Point3d> *)),
			this, SLOT(ReceiveCameraChange(std::vector<std::vector<Eigen::Matrix<double, 1, 3>>>  *, std::vector<cv::Point3d> *)));
	}
}


void Measure3DPointThread::run()
{
	forever
	{
		if (PointFrameCount > CalFrameCount)
		{
			WorldPoint3D.clear();
			vector<vector<Matrix<double, 1, 3>>> PointTree;
			vector<Matrix<double, 3, 3>> CameraInterMatrix;
			vector<Matrix<double, 3, 4>> CameraOuterMatrix;
			int MatchSize = 0;
			int cameraSize = CameraInterMatrix.size();
			for (int i = 0; i < DeviceMarkPointTree.size(); i++)
			{
				Matrix<double, 3, 3> interTemp;
				Matrix<double, 3, 4> outerTemp;
				cv::cv2eigen(CameraParamentTree[i].CameraParament.CameraInterMat, interTemp);
				cv::cv2eigen(CameraParamentTree[i].CameraParament.CameraOuterMat, outerTemp);
				CameraInterMatrix.push_back(interTemp);
				CameraOuterMatrix.push_back(outerTemp);
				cvPoint3D2Matrix(PointTree, DeviceMarkPointTree[i].MarkPoint);
				/////畸变矫正
				MyDistortProcess::DistortCorrectMatrix(PointTree[i], CameraParamentTree[i].CameraParament.CameraInterMat, CameraParamentTree[i].CameraParament.CameraDisMat);
			}
			Calculator3D::CalPointMatch(PointTree, CameraOuterMatrix, CameraInterMatrix, MatchSize, pointsMatchList);

			for (int i = 0; i < CameraOuterMatrix.size(); i++)
			{
				CameraOuterMatrix[i] = CameraInterMatrix[i] * CameraOuterMatrix[i];
			}

			for (int i = 0; i < pointsMatchList.size(); i++)
			{
				vector<Matrix<double, 1, 3>> SinglePoint;
				Matrix<double, 1, 3> globalsite;
				globalsite.setZero();
				SinglePoint = pointsMatchList[i];
				Calculator3D::CalPointGlobalSite(CameraOuterMatrix, SinglePoint, globalsite);
				cv::Point3d pt(globalsite(0, 0), globalsite(0, 1), globalsite(0, 2));
				WorldPoint3D.push_back(pt);
			}
			emit SendWorldPoint(&WorldPoint3D);
			emit SendCameraChange(&pointsMatchList, &WorldPoint3D);
			CalFrameCount++;
		}
	}
}


void Measure3DPointThread::stop()
{

}