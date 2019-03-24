#include "qt_3dmeasure.h"
#include "WandCalibrate.h"
#include "FileLib.h"
#include "MyMathLib.h"
#include "Calculate3D.h"
#include "SolvePnP_Matrix.h"
#include <opencv2/core/eigen.hpp>
#include "qmath.h"
#include "ChineseDebug.h"

VTK_MODULE_INIT(vtkRenderingFreeType);
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingOpenGL2);

void QT_3DMeasure::actionDeleteCache_click()
{
	QString DirString;
	QString QWorkPath = QDir::currentPath();
	DirString = QWorkPath + "/" + ExternFileDirectory + "/" + DirString;

	DeleteDirectory(DirString);
	QDir di;
	di.mkpath(DirString);
}

void QT_3DMeasure::actionSetCameraZero_click()
{
	vector<CameraParamenttypedef> ExternCameraParaMat;
	
	if (WandCalbrateProcess::GetCameraParamentFromFile("./parament/CameraParament.yml", ExternCameraParaMat) == true)
	{
		int i = 0;

		Matrix<double, 3, 4> CameraOuterMatrix;
		cv::cv2eigen(ExternCameraParaMat[i].CameraOuterMat, CameraOuterMatrix);

		Matrix3d RvcMat = CameraOuterMatrix.block(0, 0, 3, 3);
		Matrix<double, 3, 1> EulerMat = RvcMat.eulerAngles(2, 1, 0);
		Matrix<double, 1, 3> SpinMat = 180 / M_PI * (EulerMat.transpose());
		Matrix<double, 3, 4> TransMat_Real;

		Matrix<double, 4, 1> T = CameraZeroCal(CameraOuterMatrix, T);
		Matrix<double, 4, 1> F = CameraFocusCal(CameraOuterMatrix, F, 250);

		uiRenderer->ResetCamera();
		uiVTKCamera->SetViewAngle(19);

		uiVTKCamera->SetFocalPoint(F(0, 0) / 10, F(1, 0) / 10, F(2, 0) / 10);
		uiVTKCamera->SetPosition(T(0, 0) / 10, T(1, 0) / 10, T(2, 0) / 10);


		cv::Mat camRot = cv::Mat::zeros(3, 1, CV_64FC1);
		cv::Mat rvecMat;
		cv::eigen2cv(Matrix<double, 3, 3>(TransMat_Real.block(0, 0, 3, 3).inverse()), rvecMat);
		cv::Rodrigues(rvecMat, camRot);

		Matrix<double, 3, 1> ViewUpMat;
		ViewUpMat << 0, 1, 0;

		ViewUpMat = -RvcMat.inverse() * ViewUpMat;


		uiVTKCamera->SetViewUp(ViewUpMat(0, 0), ViewUpMat(1, 0), ViewUpMat(2, 0));//设视角位置 

		uiRenderer->ResetCameraClippingRange();
	}
}

void QT_3DMeasure::action3DMeasure_click()
{
	if (Measure3DTree.size() > 0 || ImgProcessThreadTree.size() == 0)
		return;
	else
	{
		int i = 0;
		Measure3DFullCount = 0;
		vector<CameraParamenttypedef> ExternCameraParaMat;
		WandCalbrateProcess::GetCameraParamentFromFile("./parament/CameraParament.yml", ExternCameraParaMat);
		if (ExternCameraParaMat.size() != ImgProcessThreadTree.size())
		{
			qDebug() << "参数错误" << endl;
			return;
		}
		Measure3DPointThread * Measure3DPointThreadtemp = new Measure3DPointThread(this);
		connect(Measure3DPointThreadtemp, SIGNAL(SendWorldPoint(std::vector<cv::Point3d> *)),
			this, SLOT(WorldPointRender(std::vector<cv::Point3d> *)), Qt::QueuedConnection);
		for (i = 0; i < ExternCameraParaMat.size(); i++)
		{
			Measure3DPointThreadtemp->AddCameraParament(ExternCameraParaMat[i], ImgProcessThreadTree[i]->CameraDeviceStr);
			connect(ImgProcessThreadTree[i], SIGNAL(SendCirquePoint(std::vector<CirquePointtypedef> *)),
				Measure3DPointThreadtemp, SLOT(FromCameraMark(std::vector<CirquePointtypedef> *)), Qt::QueuedConnection);
		}
		Measure3DPointThreadTree.push_back(Measure3DPointThreadtemp);
		for (int i = 0; i < Measure3DPointThreadTree.size(); i++)
		{
			for (int j = 0; j < Measure3DPointThreadTree[i]->CameraParamentTree.size(); j++)
			{
				cout << Measure3DPointThreadTree[i]->CameraParamentTree[j].CameraParament.CameraOuterMat << endl;
			}

			Measure3DPointThreadTree[i]->start();
		}

	}
}

void QT_3DMeasure::actionOpenFile_click()
{
	QString fileName = QFileDialog::getOpenFileName(this, tr("Open File"), "", "STL File(*.stl)");
	if (fileName.isEmpty())
		return;
	vtkSmartPointer<vtkSTLReader> reader = vtkSmartPointer<vtkSTLReader>::New();
	reader->SetFileName(fileName.toStdString().c_str());
	reader->Update(); // Visualize

	vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInputConnection(reader->GetOutputPort());
	vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
	actor->SetMapper(mapper);
	uiRenderer->AddActor(actor);

	actor->GetProperty()->SetColor(0, 0, 0);
	vtkSmartPointer<vtkTransform> trans =
		vtkSmartPointer<vtkTransform>::New();
	trans->PostMultiply();
	trans->Scale(0.1, 0.1, 0.1);

	trans->RotateX(90);
	trans->RotateZ(180);
	trans->Translate(0, -8.5, -200);
	actor->SetUserTransform(trans);
	actor->InitPathTraversal();
	uiRenderWindow->Render();
}

//////打开输出到文件
void QT_3DMeasure::actionToFileOpen_click()
{
	if (ImgSaveTree.size() > 0)
		return;
	int CameraCount = USBCamera_Num;
	if (USBCamera_Num <= 0)
	{
		Messageprompt("Scan Devices First!");
		return;
	}
	QString QWorkPath = QDir::currentPath();
	for (int i = 0; i < USBCamera_Num; i++)
	{
		QString DirString;
		DirString = QWorkPath + "/" + ExternFileDirectory + "/" + QString::fromStdString(CameraThreadTree[i]->CameraDeviceStr);
		QDir di;
		if (di.exists(DirString))
		{
			qDebug() << DirString << "存在" << endl;
		}
		else
		{
			if (di.mkpath(DirString))
				qDebug() << DirString << "创建成功" << endl;
			else
			{
				qDebug() << "创建失败" << endl;
				return;
			}
		}

		DirString = DirString + "/CameraCache";
		if (di.exists(DirString))
		{
			qDebug() << DirString << "存在" << endl;
		}
		else
		{
			if (di.mkpath(DirString))
				qDebug() << DirString << "创建成功" << endl;
			else
			{
				qDebug() << "创建失败" << endl;
				return;
			}
		}

		ImgSaveThread * ImgSavetemp = new ImgSaveThread(DirString);
		if (i < CameraThreadTree.size())
		{
			connect(CameraThreadTree[i], SIGNAL(captured(cv::Mat *)),
				ImgSavetemp, SLOT(ImageSaveProcess(cv::Mat *)), Qt::BlockingQueuedConnection);
			ImgSaveTree.push_back(ImgSavetemp);
		}
		else
		{
			qDebug() << "未打开相机" << endl;
			return;
		}
	}
	if (ImgSaveTree.size() > 0)
		ui.actionToFileOpen->setEnabled(false);
}

void QT_3DMeasure::actionToFileClose_click()
{
	if (ImgSaveTree.size() > 0)
	{
		for (int i = 0; i < ImgSaveTree.size(); i++)
		{
			CameraThreadTree[i]->disconnect(ImgSaveTree[i]);
		}
		ImgSaveTree.clear();
		vector<ImgSaveThread*>().swap(ImgSaveTree);
		ui.actionToFileOpen->setEnabled(true);
	}
	else
		qDebug() << "未执行保存" << endl;
}

void QT_3DMeasure::actionImportCamera_click()
{
	int i = 0;
	vector<CameraParamenttypedef> ExternCameraParaMat;

	if (WandCalbrateProcess::GetCameraParamentFromFile("./parament/CameraParament.yml", ExternCameraParaMat) == true)
	{
		while (uiCameraSorceTree.size() < ExternCameraParaMat.size())
		{
			uiCameraSorceTree.push_back(RenderAddCamera(uiCameraSTLSorce));
		}
		for (i = 0; i < uiCameraSorceTree.size(); i++)
		{
			if (i < ExternCameraParaMat.size())
			{
				uiCameraSorceTree[i]->SetVisibility(1);
				CameraSiteSet(ExternCameraParaMat[i], uiCameraSorceTree[i]);
				cout << ExternCameraParaMat[i].CameraOuterMat << endl << endl;
			}
			else
				uiCameraSorceTree[i]->SetVisibility(0);
		}
	}
}

void QT_3DMeasure::actionCloseCamera_click()
{
	if (CameraThreadTree.size() > 0)
	{
		for (int i = 0; i < ImgProcessThreadTree.size(); i++)
		{
			ImgProcessThreadTree[i]->CloseCal();
		}
		for (int i = 0; i < CameraThreadTree.size(); i++)
		{
			CameraThreadTree[i]->disconnect();
			CameraThreadTree[i]->stop();
			while (!CameraThreadTree[i]->wait(100))
			{
				QCoreApplication::processEvents();
			}
			CameraThreadTree[i]->release();
			CameraThreadTree[i]->destroyed();
			CameraStop(CameraHandle[i]);
			CameraUnInit(CameraHandle[i]);
		}
		CameraThreadTree.clear();

		for (int i = 0; i < ImgProcessThreadTree.size(); i++)
		{
			ImgProcessThreadTree[i]->release();
		}

		std::vector<CaptureThread *>().swap(CameraThreadTree);
		std::vector<ImgMainProcess *>().swap(ImgProcessThreadTree);
		std::vector<ImgViewItem *>().swap(ImgViewItemTree);

		ui.actionOpenCamera->setEnabled(true);
	}
	else
	{
		qDebug() << "未打开相机" << endl;
	}
}

void QT_3DMeasure::actionOpenCamera_click()
{
	int CameraOpen = -1;
	if (USBCamera_Num > 0 && USBCamera_Num <= MAXCAMERANUM && CameraThreadTree.size() == 0)
	{
		int g_hCamera0, g_hCamera1;
		ImgViewItemTree.clear();
		CameraThreadTree.clear();
		ImgProcessThreadTree.clear();

		for (int index = 0; index < USBCamera_Num; index++)
		{
			QString CameraQStr;
			CameraQStr.sprintf("Camera_%03d", index);
			CameraOpen = CameraInit(&tCameraEnumList[index], -1, -1, &g_hCamera0);
			if (CameraOpen != CAMERA_STATUS_SUCCESS)
			{
				CameraIsOpened(&tCameraEnumList[index], &CameraOpen);
				if (CameraOpen)
					qDebug() << "Camera " << g_hCamera0 << " Is Open" << endl;
				return;
			}

			CameraPlay(g_hCamera0);
			CameraSetIspOutFormat(g_hCamera0, CAMERA_MEDIA_TYPE_MONO8);


			CaptureThread * CameraThreadtemp = new CaptureThread(this, g_hCamera0, CameraQStr.toStdString());
			CameraThreadTree.push_back(CameraThreadtemp);

			ImgMainProcess * ImgProcessThreadtemp = new ImgMainProcess(CameraQStr.toStdString());
			ImgProcessThreadTree.push_back(ImgProcessThreadtemp);
			ImgProcessThreadtemp->OpenCal();

			ImgViewItem * ImgViewtemp = new ImgViewItem(ui.graphicsView_1, index , CameraQStr.toStdString());
			uiRenderScene->addItem(ImgViewtemp->img_item);
			ImgViewItemTree.push_back(ImgViewtemp);

			connect(CameraThreadtemp, SIGNAL(captured(cv::Mat *)),
				ImgProcessThreadtemp, SLOT(ImgProcessThread(cv::Mat *)), Qt::BlockingQueuedConnection);

			connect(ImgProcessThreadtemp, SIGNAL(SendShowRGBImg(cv::Mat *)),
				ImgViewtemp, SLOT(ImgViewProcess(cv::Mat *)), Qt::QueuedConnection);

			CameraHandle[index] = g_hCamera0;
			CameraThreadtemp->start();
			CameraThreadtemp->stream();

		}

		if (CameraThreadTree.size() > 0)
			ui.actionOpenCamera->setEnabled(false);
	}
	else
	{
		Messageprompt("没有USB设备");
	}
}

void QT_3DMeasure::actionScanUSB_click()
{
	int	iCameraCounts = 4;
	int	iStatus = -1;
	CameraEnumerateDevice(tCameraEnumList, &iCameraCounts);
	int num = iCameraCounts;
	if (num == 0)
	{
		Messageprompt("No Devices!");
	}
	else
	{
		USBCamera_Num = num;

		int counter = ui.CameralistView->count();
		for (int index = 0; index < counter; index++)
		{
			QListWidgetItem *item = ui.CameralistView->takeItem(0);
			delete item;
		}
		QStringList * sl = new QStringList();

		for (int i = 0; i < num; i++)
		{
			sl->append(QString::fromStdString(tCameraEnumList[i].acProductName));
		}
		ui.CameralistView->addItems(*sl);
	}
}

