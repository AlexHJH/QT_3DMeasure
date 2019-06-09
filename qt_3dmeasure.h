#ifndef QT_3DMEASURE_H
#define QT_3DMEASURE_H

#include "ui_cameraParaChange.h"
#include "ui_WorldPointInput.h"


#include <vtkAutoInit.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkPolyData.h>
#include <vtkSphereSource.h>
#include <vtkAxesActor.h>
#include <vtkTransform.h>
#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>
#include <vtkActor.h>
#include <vtkConeSource.h>
#include <vtkRenderer.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkLight.h>
#include <vtkCamera.h>
#include <vtkActor2D.h>
#include <vtkRendererCollection.h>
#include <vtkSTLReader.h>
#include <vtkPointPicker.h>
#include <vtkCellPicker.h>
#include <vtkPropPicker.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkObjectFactory.h>
#include <vtkIterativeClosestPointTransform.h>
#include <vtkLandmarkTransform.h>
#include <vtkMatrix4x4.h>



#include "opencv.hpp"
#include "ui_qt_3dmeasure.h"
#include "usbcamera.h"
#include "ImgLib.h"
#include "ImgView.h"
#include "WandCalibrate.h"
#include "Measure3DLib.h"

using namespace cv;
using namespace std;


#define  MAXCAMERANUM       20
#define  VTKWidgtZoomRatio  1.0




class PointPickerInteractorStyle : public vtkInteractorStyleTrackballCamera
{
public:
	static PointPickerInteractorStyle* New();
	vtkTypeMacro(PointPickerInteractorStyle, vtkInteractorStyleTrackballCamera);
	virtual void OnLeftButtonDown();
	virtual void OnRightButtonDown();
public:
	std::vector<vtkActor *> SelectActorTree;
	std::vector<Point3d> SelectPointSiteTree;
	std::vector<Point3d> WorldPointSiteTree;
private:
	std::vector<std::vector<double>> SelectActorOricolorTree;
};


class ImgSaveThread : public QObject
{
	Q_OBJECT
public:
	ImgSaveThread(QString dir);

public slots:
	void ImageSaveProcess(cv::Mat *);

private:
	QString FileDir;
	long long FrameCount;
};

class cameraParaChange : public QMainWindow, public WandCalbrateProcess
{
	Q_OBJECT
public:
	cameraParaChange(WandCheckDatatypedef, QWidget *parent = 0);
	~cameraParaChange();
public slots:
	void savePara_click();
	void push_click();
public:
	WandCheckDatatypedef cameraPara;
signals:
	void sendCameraPara(WandCheckDatatypedef *);
private:
	Ui_cameraParaChange ui;
};

class InputWorldPoint : public QMainWindow
{
	Q_OBJECT
public:
	InputWorldPoint(QWidget *parent = 0);
	~InputWorldPoint();
public slots:
void confirm_click();
void delete_click();
void savePoint_click();
void loadPoint_click();
signals:
void senditem(vector<Point3d> *);
void deleteitem();
private:
	Ui_wordPointInput ui;
	
};

class Measure3DThread : public QObject
{
	Q_OBJECT
public:
	Measure3DThread(CameraParamenttypedef &camera);
signals:
	void SendPointCalEnd();
public slots:
	void Point2DTransferPoint3D(std::vector<CirquePointtypedef> *);
public:
	CameraParamenttypedef CameraParament;
public:
	vector<Matrix<double, 1, 3>> CirqueCenterMat;
};

class QT_3DMeasure : public QMainWindow, public WandCalbrateProcess
{
	Q_OBJECT

public:
	QT_3DMeasure(QWidget *parent = 0);
	~QT_3DMeasure();
public:
	void setUi();
private slots:
	void actionCalibrate_click();
	void actionOpenFile_click();
	void actionScanUSB_click();
	void actionOpenCamera_click();
	void actionToFileOpen_click();
	void actionToFileClose_click();
	void actionCloseCamera_click();
	void action3DMeasure_click();
	void actionCalibrateFromFile_click();
	void actionImportCamera_click();
	void actionSetCameraZero_click();
	void actionDeleteCache_click();
	void actionAddWorldSite_click();
	void CalibrateFullImgConfirm(bool flag);
	void WorldPointRender(std::vector<cv::Point3d> *);
	void TimerTimeOut();
	void receiveWorldItem(vector<Point3d> *);
	void receiveDeleteItem();
	void actionRebuildCamera_click();
	void actionCameraParaChange_click();
public slots:
	void receiveCameraPara(WandCheckDatatypedef *);
private:
	void Messageprompt(QString str);
	vtkSmartPointer<vtkActor> RenderGird(double interval, double * color, vtkSmartPointer<vtkRenderWindow> renderWindow);
	vtkSmartPointer<vtkActor> RenderSphere(vtkSmartPointer<vtkSphereSource> sphereSource, double * Center);
	vtkSmartPointer<vtkSTLReader> GetSTLFileObject(string FileName);
	void CameraSiteSet(CameraParamenttypedef transMat, vtkSmartPointer<vtkActor> actor);
	vtkSmartPointer<vtkActor> RenderAddCamera(vtkSmartPointer<vtkSTLReader> reader);
	void CalibrateFromFile();
	vtkSmartPointer<vtkActor> RenderSphere(vtkSmartPointer<vtkSphereSource> sphereSource, double * Center, double * Color);
	void InitTimer();
public:
	Ui::QT_3DMeasureClass ui;
private:
	tSdkCameraDevInfo  tCameraEnumList[MAXCAMERANUM];
	int CameraHandle[MAXCAMERANUM];
	int USBCamera_Num;
	
	vtkSmartPointer<vtkRenderer> uiRenderer;
	vtkSmartPointer<vtkRenderer> spereRenderer;
	vtkSmartPointer<vtkRenderWindow> uiRenderWindow;
	vtkCamera *uiVTKCamera;


	QGraphicsScene * uiRenderScene;
	vector<ImgViewItem *> ImgViewItemTree;
	vector<ImgMainProcess *> ImgProcessThreadTree;
	vector<CaptureThread *>  CameraThreadTree;
	vector<Measure3DPointThread *> Measure3DPointThreadTree;
	vector<ImgSaveThread *> ImgSaveTree;
	vector<CalibrateThread *> CalibrateTree;
	vector<Measure3DThread *> Measure3DTree;
	vector<WandCalbrateProcess *> WandCalibrateTree;
	vector<vtkSmartPointer<vtkActor>> uiCameraSorceTree;
	vector<vtkSmartPointer<vtkActor>> sphereSourceActorTree;
	vector<vtkSmartPointer<vtkSphereSource>> sphereSourceTree;
	vtkSmartPointer<vtkSTLReader> uiCameraSTLSorce;
	vtkSmartPointer<PointPickerInteractorStyle> VTKPointPickstyle;
	int CalibrateFullCount;
	int Measure3DFullCount;
	QString ExternFileDirectory;

	QLabel *PointLocationLabel;
	QLabel *LengthLabel;
	WandCheckDatatypedef cameraPara;
};

#endif // QT_3DMEASURE_H
