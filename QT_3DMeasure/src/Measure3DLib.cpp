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
			for (int i = 0; i < DeviceMarkPointTree.size(); i++)
			{
				Matrix<double, 3, 3> interTemp;
				Matrix<double, 3, 4> outerTemp;
				cv::cv2eigen(CameraParamentTree[i].CameraParament.CameraInterMat, interTemp);
				cv::cv2eigen(CameraParamentTree[i].CameraParament.CameraOuterMat, outerTemp);
				CameraInterMatrix.push_back(interTemp);
				CameraOuterMatrix.push_back(outerTemp);
				cvPoint3D2Matrix(PointTree, DeviceMarkPointTree[i].MarkPoint);
				/////»û±ä½ÃÕý
				MyDistortProcess::DistortCorrectMatrix(PointTree[i], CameraParamentTree[i].CameraParament.CameraInterMat, CameraParamentTree[i].CameraParament.CameraDisMat);
			}
			Calculator3D::CalPointMatch(PointTree, CameraOuterMatrix, MatchSize);
			
			for (int i = 0; i < MatchSize; i++)
			{
				vector<Matrix<double, 1, 3>> SinglePoint;
				Matrix<double, 1, 3> globalsite;
				globalsite.setZero();
				for (int j = 0; j < PointTree.size(); j++)
				{
					MyMathLib::PointTransfer1x3(PointTree[j][i], CameraInterMatrix[j]);
					SinglePoint.push_back(PointTree[j][i]);
				}
				Calculator3D::CalPointGlobalSite(CameraOuterMatrix, SinglePoint, globalsite);
				cv::Point3d pt(globalsite(0, 0), globalsite(0, 1), globalsite(0, 2));
				WorldPoint3D.push_back(pt);
			}
			emit SendWorldPoint(&WorldPoint3D);
			CalFrameCount++;
			cout << CalFrameCount << endl;
		}
	}
}


void Measure3DPointThread::stop()
{

}