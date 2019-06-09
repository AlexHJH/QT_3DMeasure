#include "qt_3dmeasure.h"
#include "WandCalibrate.h"
#include "FileLib.h"
#include "MyMathLib.h"
#include "Calculate3D.h"
#include "SolvePnP_Matrix.h"
#include "qmath.h"
#include "ChineseDebug.h"

VTK_MODULE_INIT(vtkRenderingFreeType);
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingOpenGL2);

vtkStandardNewMacro(PointPickerInteractorStyle);

extern QT_3DMeasure *MainWindowStatic;
QT_3DMeasure::QT_3DMeasure(QWidget *parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);
	MainWindowStatic = this;
	connect(ui.actionCalibrate, SIGNAL(triggered(bool)), this, SLOT(actionCalibrate_click()));
	connect(ui.actionOpenFile, SIGNAL(triggered(bool)), this, SLOT(actionOpenFile_click()));
	connect(ui.actionScanUSB, SIGNAL(triggered(bool)), this, SLOT(actionScanUSB_click()));
	connect(ui.actionOpenCamera, SIGNAL(triggered(bool)), this, SLOT(actionOpenCamera_click()));
	connect(ui.actionCloseCamera, SIGNAL(triggered(bool)), this, SLOT(actionCloseCamera_click()));
	connect(ui.actionToFileOpen, SIGNAL(triggered(bool)), this, SLOT(actionToFileOpen_click()));
	connect(ui.actionToFileClose, SIGNAL(triggered(bool)), this, SLOT(actionToFileClose_click()));
	connect(ui.action3DMeasure, SIGNAL(triggered(bool)), this, SLOT(action3DMeasure_click()));
	connect(ui.actionCalibrateFromFile, SIGNAL(triggered(bool)), this, SLOT(actionCalibrateFromFile_click()));
	connect(ui.actionImportCamera, SIGNAL(triggered(bool)), this, SLOT(actionImportCamera_click()));
	connect(ui.actionSetCameraZero, SIGNAL(triggered(bool)), this, SLOT(actionSetCameraZero_click()));
	connect(ui.actionDeleteCache, SIGNAL(triggered(bool)), this, SLOT(actionDeleteCache_click()));
	connect(ui.actionAddWorldSite, SIGNAL(triggered(bool)), this, SLOT(actionAddWorldSite_click()));
	connect(ui.actionRebuildCamera, SIGNAL(triggered(bool)), this, SLOT(actionRebuildCamera_click()));
	connect(ui.actionCameraParaChange, SIGNAL(triggered(bool)), this, SLOT(actionCameraParaChange_click()));

	statusBar()->setStyleSheet(QString("QStatusBar::item{border: 0px}")); // 设置不显示label的边框
	statusBar()->setSizeGripEnabled(false); //设置是否显示右边的大小控制点
	
	LengthLabel = new QLabel("L: ", this);
	PointLocationLabel = new QLabel("X: , Y: , Z:", this);
	PointLocationLabel->setAlignment(Qt::AlignCenter);
	PointLocationLabel->setMinimumSize(PointLocationLabel->sizeHint());
	statusBar()->addWidget(LengthLabel);
	statusBar()->addPermanentWidget(PointLocationLabel); //现实永久信息
	uiRenderWindow = vtkSmartPointer<vtkRenderWindow>::New();
	uiRenderer = vtkSmartPointer<vtkRenderer>::New();
	 
	spereRenderer = vtkSmartPointer<vtkRenderer>::New();

	ui.qvtkWidget3D->SetRenderWindow(uiRenderWindow);
	uiRenderWindow->SetSize(30 * 1000 / VTKWidgtZoomRatio, 30 * 1000 / VTKWidgtZoomRatio);

	uiRenderWindow->SetNumberOfLayers(2);
	uiRenderWindow->AddRenderer(uiRenderer);

	vtkSmartPointer<vtkAxesActor> uiAxes = vtkSmartPointer<vtkAxesActor>::New();
	uiAxes->SetAxisLabels(0);
	uiAxes->SetTotalLength(200 / VTKWidgtZoomRatio, 200/ VTKWidgtZoomRatio, 200 / VTKWidgtZoomRatio);
	uiRenderer->AddActor(uiAxes);
	double GirdColor[3] = { 0.5, 0.5, 0.5 };
	uiRenderer->SetBackground(0.5, 0.5, 0.5);
	//uiRenderer->AddActor(RenderGird(200 / VTKWidgtZoomRatio, GirdColor, uiRenderWindow));
	uiRenderer->ResetCamera();
	uiRenderWindow->Render();

	uiVTKCamera = vtkCamera::New();
	uiRenderer->SetActiveCamera(uiVTKCamera);
	uiVTKCamera->ComputeViewPlaneNormal();
	uiVTKCamera->SetPosition(0, uiRenderWindow->GetSize()[1] / 6, -uiRenderWindow->GetSize()[0] / 2);//设观察对象位

	uiRenderScene = new QGraphicsScene;
	ui.graphicsView_1->setScene(uiRenderScene);
	USBCamera_Num = 0;

	uiCameraSTLSorce = GetSTLFileObject("./obj/Camera.stl");
	GetWandCalbrateInit("./parament/InterParament.yml", cameraPara);
	ExternFileDirectory = "ExternalCalibration_Temp";

	Matrix<double, 4, 3> WandPoint;
	WandPoint << 0, 0, 0,
		150, 0, 0,
		600, 0, 0,
		0, 0, 300;
	double site[3];
	double CirColor[3] = { 0.9, 0.9, 0.9 };
	for (int k = 0; k < 4; k++)
	{
		vtkSmartPointer<vtkSphereSource> sphereSource = vtkSmartPointer<vtkSphereSource>::New();
		site[0] = WandPoint(k, 0) / VTKWidgtZoomRatio;
		site[1] = WandPoint(k, 1) / VTKWidgtZoomRatio;
		site[2] = WandPoint(k, 2) / VTKWidgtZoomRatio;
		uiRenderer->AddActor(RenderSphere(sphereSource, site, CirColor));
		uiRenderWindow->Render();
	}
	vtkSmartPointer<vtkCellPicker> pointPicker =
		vtkSmartPointer<vtkCellPicker>::New();

	ui.qvtkWidget3D->GetInteractor()->SetPicker(pointPicker);
	VTKPointPickstyle = vtkSmartPointer<PointPickerInteractorStyle>::New();
	ui.qvtkWidget3D->GetInteractor()->SetInteractorStyle(VTKPointPickstyle);
	InitTimer();
	actionScanUSB_click();
}

void QT_3DMeasure::setUi()
{
	setWindowState(Qt::WindowMaximized);
	int screenWidth = QApplication::desktop()->geometry().width();
	int screenHeight = QApplication::desktop()->geometry().height() - 140;
	int lastHeight = 0;
	ui.controlGroup->setGeometry(0, 0, screenWidth * 0.15, screenHeight);
	lastHeight += 0;
	ui.CameralistView->setGeometry(0, lastHeight, ui.controlGroup->geometry().width(), screenHeight * 0.1);
	lastHeight += ui.CameralistView->geometry().height();
	ui.selectPointGroup->setGeometry(0, lastHeight, ui.controlGroup->geometry().width(), screenHeight * 0.2);
	ui.pointListWidget->setGeometry(0, 20, ui.selectPointGroup->geometry().width(), ui.selectPointGroup->geometry().height() - 20);
	lastHeight += ui.selectPointGroup->geometry().height();
	ui.worldPointGroup->setGeometry(0, lastHeight, ui.controlGroup->geometry().width(), screenHeight * 0.2);
	ui.pointListWidgetInput->setGeometry(0, 20, ui.worldPointGroup->geometry().width(), ui.worldPointGroup->geometry().height() - 20);
	lastHeight += ui.worldPointGroup->geometry().height();
	ui.textBrowser->setGeometry(0, lastHeight, ui.controlGroup->geometry().width(), screenHeight - lastHeight);
	ui.qvtkWidget3D->setGeometry(ui.CameralistView->geometry().width(), 0, (screenWidth - ui.CameralistView->geometry().width()) * 0.8, screenHeight);
	ui.graphicsView_1->setGeometry(ui.qvtkWidget3D->geometry().width() + ui.CameralistView->geometry().width(), 0, screenWidth -
		ui.qvtkWidget3D->geometry().width() - ui.controlGroup->geometry().width(), screenHeight);
	qDebug() << "初始化成功" << endl;
}


QTimer *m_timer = new QTimer;
void QT_3DMeasure::InitTimer()
{
	if (NULL == m_timer)
		m_timer = new QTimer;
	//设置定时器是否为单次触发。默认为 false 多次触发
	m_timer->setSingleShot(false);
	//定时器触发信号槽
	connect(m_timer, SIGNAL(timeout()), this, SLOT(TimerTimeOut()));
	//启动或重启定时器, 并设置定时器时间：毫秒
	m_timer->start(20);
}

void QT_3DMeasure::TimerTimeOut()
{
	//判断定时器是否运行
	if (m_timer->isActive())
		m_timer->stop();   //停止定时器
						   //执行定时器触发时需要处理的业务
	if (VTKPointPickstyle->SelectActorTree.size() > 0)
	{
		if (VTKPointPickstyle->SelectActorTree.size() != ui.pointListWidget->count())
		{
			ui.pointListWidget->clear();
			VTKPointPickstyle->SelectPointSiteTree.clear();
			for (int i = 0; i < VTKPointPickstyle->SelectActorTree.size(); i++)
			{
				QString ls;
				double * pMCenter = VTKPointPickstyle->SelectActorTree[i]->GetCenter();
				Point3d pt;
				pt.x = pMCenter[0] * VTKWidgtZoomRatio;
				pt.y = pMCenter[1] * VTKWidgtZoomRatio;
				pt.z = pMCenter[2] * VTKWidgtZoomRatio;
				VTKPointPickstyle->SelectPointSiteTree.push_back(pt);
				ls.sprintf("X: %.2f, Y: %.2f, Z: %.2f", pt.x, pt.y, pt.z);
				QListWidgetItem *item = new QListWidgetItem;
				item->setText(ls);
				ui.pointListWidget->addItem(item);
			}
		}
		

		int lastone = VTKPointPickstyle->SelectActorTree.size() - 1;
		if (VTKPointPickstyle->SelectActorTree[lastone]->GetVisibility() == 1)
		{
			double * pMCenter = VTKPointPickstyle->SelectActorTree[lastone]->GetCenter();
			QString ls;
			ls.sprintf("X: %.2f, Y: %.2f, Z: %.2f", pMCenter[0] * VTKWidgtZoomRatio, pMCenter[1] * VTKWidgtZoomRatio, pMCenter[2] * VTKWidgtZoomRatio);
			PointLocationLabel->setText(ls);
			if (VTKPointPickstyle->SelectActorTree.size() > 1)
			{
				int nextone = VTKPointPickstyle->SelectActorTree.size() - 2;
				if (VTKPointPickstyle->SelectActorTree[nextone]->GetVisibility() == 1)
				{
					double * pMCenterNext = VTKPointPickstyle->SelectActorTree[nextone]->GetCenter();
					double Distance = VTKWidgtZoomRatio * sqrt(pow(pMCenterNext[0] - pMCenter[0], 2) + pow(pMCenterNext[2] - pMCenter[2], 2) + pow(pMCenterNext[1] - pMCenter[1], 2));
					QString lengths;
					lengths.sprintf("L: %.2f", Distance);
					LengthLabel->setText(lengths);
				}
				else
				{
					LengthLabel->setText("L: ");
				}
			}
			else
			{
				LengthLabel->setText("L: ");
			}
		}
		else
		{
			PointLocationLabel->setText("X: , Y: , Z:");
		}
	}
	else
	{
		ui.pointListWidget->clear();
		PointLocationLabel->setText("X: , Y: , Z:");
	}
	m_timer->start();
}

QT_3DMeasure::~QT_3DMeasure()
{
	if (CameraThreadTree.size() > 0)
	{
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
	}
}


void QT_3DMeasure::actionCalibrateFromFile_click()
{
	CalibrateFromFile();
}

void QT_3DMeasure::CalibrateFromFile()
{
	int i, CameraNum;
	QString QWorkPath = QDir::currentPath();
	std::string CameraFileNametype("Camera_");
	vector<QString> MatchFileCa;
	vector<QString> FileQCamera;
	GetAllFileFolder(QWorkPath + "/" + ExternFileDirectory, FileQCamera);
	CameraNum = FileQCamera.size();

	WandCalibrateTree.clear();
	vector<WandCalbrateProcess *>().swap(WandCalibrateTree);
	if (CameraNum == 0)
	{
		qDebug() << "无文件" << endl;
	}
	else
	{
		for (i = 0; i <= (FileQCamera.size() - 1); i++)
		{
			int m = CameraFileNametype.compare(0, CameraFileNametype.size() - 1, FileQCamera[i].toStdString(), 0, CameraFileNametype.size() - 1);
			if (FileQCamera[i].size() != (CameraFileNametype.size() + 3) && m != 0)
				FileQCamera.pop_back();
			else
			{
				MatchFileCa.push_back(FileQCamera[i]);
			}
		}
		for (i = 0; i < MatchFileCa.size(); i++)
		{
			
			WandCalbrateProcess * CameraExternCal = new WandCalbrateProcess(cameraPara, (QWorkPath + "/" + ExternFileDirectory + "/" + MatchFileCa[i]).toStdString(), MatchFileCa[i].toStdString());
			CameraExternCal->WandSolveMat();
			WandCalibrateTree.push_back(CameraExternCal);
		}
	}
	WandCalbrateProcess::CameraCalbrateAll(WandCalibrateTree);
	WriteCameraParamentToFile("./parament/CameraParament.yml", WandCalibrateTree);
	qDebug() << "标定完成" << endl;
}

void QT_3DMeasure::CalibrateFullImgConfirm(bool flag)
{
	if (flag == true)
	{
		CalibrateFullCount++;
	}
	if (CalibrateFullCount == CalibrateTree.size())
	{
		vector<vector<Matrix<double, 4, 3>>> MarkData;
		for (int i = 0; i < CalibrateTree.size(); i++)
		{
			ImgProcessThreadTree[i]->disconnect(CalibrateTree[i]);
		}
		int MinSize = 1000000;
		int i, j, k;
		int CameraNum = CalibrateTree.size();
		vector<int> BadPoint;
		for (i = 0; i < CameraNum; i++)
		{
			if (CalibrateTree[i]->MarkSiteBuff.size() < MinSize)
			{
				MinSize = CalibrateTree[i]->MarkSiteBuff.size();
			}
		}

		for (i = 0; i < MinSize; i++)
		{
			for (j = 0; j < CameraNum; j++)
			{
				if (CalibrateTree[j]->MarkSiteBuff[i](0, 2) < 0)
				{
					break;
				}
			}
			if (j != CameraNum)
			{
				BadPoint.push_back(i);
			}
		}

		for (i = 0; i < CameraNum; i++)
		{
			vector<Matrix<double, 4, 3>> MarkDatatemp;
			k = 0;
			int Bat_p = -1;
			for (j = 0; j < MinSize; j++)
			{
				Matrix<double, 4, 3> Mattemp;
				if (k < BadPoint.size())
					Bat_p = BadPoint[k];
				if (j != Bat_p)
				{
					Mattemp = CalibrateTree[i]->MarkSiteBuff[j];
					Mattemp(0, 2) = 1;
					MarkDatatemp.push_back(Mattemp);
				}
				else if (k < BadPoint.size())
					k++;
			}
			MarkData.push_back(MarkDatatemp);
		}
		for (i = 0; i < CameraNum; i++)
		{
			CalibrateTree[i]->MarkSiteBuff = MarkData[i];
			CalibrateTree[i]->ResultToFile();
		}
		CalibrateTree.clear();
		CalibrateFromFile();
		qDebug() << "标定结束" << endl;
	}
}

void QT_3DMeasure::actionCalibrate_click()
{
	vector<string> FileCamera;
	int CameraNum = 0;
	int i = 0;
	char CalDataFileDirectory[_MAX_PATH];

	qDebug() << "开始进行标定" << endl;

	QString QWorkPath = QDir::currentPath();

	qDebug() << "工作目录:" << QWorkPath << endl;


	QString InterParaDirSorce = QWorkPath + "/parament/InterParament.yml";

	if (!QFile::exists(InterParaDirSorce))
	{
		qDebug() << "内参文件不存在" << endl;
		return;
	}
	/////时序同步

	CalibrateFullCount = 0;

	for (int i = 0; i < CameraThreadTree.size(); i++)
		CameraThreadTree[i]->pause();              /////停止相机数据采集

/******************************************************************************************/
	for (int i = 0; i < USBCamera_Num; i++)
	{
		if (i < CameraThreadTree.size())
		{
			QString DirString;
			DirString = QWorkPath + "/" + ExternFileDirectory + "/" + QString::fromStdString(ImgProcessThreadTree[i]->CameraDeviceStr);
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
			CalibrateThread * CalibrateThreadtemp = new CalibrateThread(cameraPara, DirString.toStdString(), i);
			connect(ImgProcessThreadTree[i], SIGNAL(SendCirquePoint(std::vector<CirquePointtypedef> *)),
				CalibrateThreadtemp, SLOT(CalibrateImgProcess(std::vector<CirquePointtypedef> *)), Qt::QueuedConnection);

			connect(CalibrateThreadtemp, SIGNAL(ResultFull(bool)),
				this, SLOT(CalibrateFullImgConfirm(bool)), Qt::QueuedConnection);
			CalibrateTree.push_back(CalibrateThreadtemp);
		}
		else
		{
			qDebug() << "未打开相机" << endl;
			return;
		}
	}
	for (int i = 0; i < CameraThreadTree.size(); i++)
		CameraThreadTree[i]->stream();
	/////时序同步
}

//************************************
// Method:    Messageprompt
// FullName:  QT_ForTest::Messageprompt
// Access:    private 
// Returns:   void
// Qualifier:
// Parameter: QString str
//************************************
void QT_3DMeasure::Messageprompt(QString str)
{
	QMessageBox::about(NULL, "提示", str);
}


ImgSaveThread::ImgSaveThread(QString dir)
{
	FileDir = dir;
	FrameCount = 0;
}

void ImgSaveThread::ImageSaveProcess(cv::Mat * img)
{
	QString SaveDir;
	SaveDir.sprintf("/Pic_%06d.bmp", FrameCount);
	SaveDir = FileDir + SaveDir;
	imwrite(SaveDir.toStdString(), *img);
	FrameCount++;
}

Measure3DThread::Measure3DThread(CameraParamenttypedef &camera)
{
	CameraParament = camera;
/*	CameraParament.CameraOuterMat = camera.CameraOuterMat.clone();
	CameraParament.CameraInterMat = camera.CameraInterMat.clone();
	CameraParament.CameraDisMat = camera.CameraDisMat.clone();
	CameraParament.ReProjectErr = camera.ReProjectErr;*/
}

void Measure3DThread::Point2DTransferPoint3D(std::vector<CirquePointtypedef> *pointMark)
{
	emit SendPointCalEnd();
}
