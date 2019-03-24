#include "qt_3dmeasure.h"
#include "WandCalibrate.h"
#include "FileLib.h"
#include "MyMathLib.h"
#include "Calculate3D.h"
#include "SolvePnP_Matrix.h"
#include <opencv2/core/eigen.hpp>


VTK_MODULE_INIT(vtkRenderingFreeType);
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingOpenGL2);

void PointPickerInteractorStyle::OnLeftButtonDown()
{
	int* clickPos = this->Interactor->GetEventPosition();
	vtkSmartPointer<vtkPropPicker>  picker =
		vtkSmartPointer<vtkPropPicker>::New();
	picker->Pick(clickPos[0], clickPos[1], 0, this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer());
	vtkActor* pA = (vtkActor*)picker->GetActor();
	bool IsPAFind = false;
	if (pA != NULL)
	{
		for (vector<vtkActor *>::iterator it = SelectActorTree.begin(); it != SelectActorTree.end(); it++)
		{
			if (*it == pA)
			{
				IsPAFind = true;
				break;
			}
		}
		if (IsPAFind == false)
		{
			SelectActorTree.push_back(pA);
			double * OriColor;
			OriColor = pA->GetProperty()->GetColor();

			vector<double> colortemp;
			colortemp.push_back(OriColor[0]);
			colortemp.push_back(OriColor[1]);
			colortemp.push_back(OriColor[2]);

			SelectActorOricolorTree.push_back(colortemp);
			pA->GetProperty()->SetColor(1, 0, 0);
		}
	}
	vtkInteractorStyleTrackballCamera::OnLeftButtonDown();
};

void PointPickerInteractorStyle::OnRightButtonDown()
{
	int* clickPos = this->Interactor->GetEventPosition();
	vtkSmartPointer<vtkPropPicker>  picker =
		vtkSmartPointer<vtkPropPicker>::New();
	picker->Pick(clickPos[0], clickPos[1], 0, this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer());
	vtkActor* pA = (vtkActor*)picker->GetActor();
	if (pA != NULL)
	{
		vector<vector<double>>::iterator colorit = SelectActorOricolorTree.begin();
		for (vector<vtkActor *>::iterator it = SelectActorTree.begin(); it != SelectActorTree.end();)
		{
			if (*it == pA)
			{
				double colortemp[3];
				colortemp[0] = (*colorit)[0];
				colortemp[1] = (*colorit)[1];
				colortemp[2] = (*colorit)[2];

				pA->GetProperty()->SetColor(colortemp);
				it = SelectActorTree.erase(it);
				colorit = SelectActorOricolorTree.erase(colorit);
				continue;
			}
			else
			{
				++it;
				++colorit;
			}
		}
		if (SelectActorTree.size() == 0 && SelectActorOricolorTree.size() == 0)
		{
			vector<vtkActor *>().swap(SelectActorTree);
			vector<vector<double>>().swap(SelectActorOricolorTree);
		}
	}
	vtkInteractorStyleTrackballCamera::OnRightButtonDown();
}


vtkSmartPointer<vtkActor> QT_3DMeasure::RenderSphere(vtkSmartPointer<vtkSphereSource> sphereSource, double * Center, double * Color)
{
	sphereSource->SetCenter(Center);//设置球的中心
	sphereSource->SetRadius(1.5);//设置球的半径
	sphereSource->SetThetaResolution(30);//设置球表面精度，值越大球的光滑程度越高
	vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();//定义局部PolydataMapper对象
	mapper->SetInputConnection(sphereSource->GetOutputPort());
	vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();//定义局部actor对象
	actor->SetMapper(mapper);
	actor->GetProperty()->SetColor(Color);//设置球的颜色,这里可以为每个点赋不同的颜色或者是随机颜色
	return actor;
}

vtkSmartPointer<vtkActor> QT_3DMeasure::RenderSphere(vtkSmartPointer<vtkSphereSource> sphereSource, double * Center)
{
	sphereSource->SetCenter(Center);//设置球的中心
	sphereSource->SetRadius(1.5);//设置球的半径
	sphereSource->SetThetaResolution(30);//设置球表面精度，值越大球的光滑程度越高
	vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();//定义局部PolydataMapper对象
	mapper->SetInputConnection(sphereSource->GetOutputPort());
	vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();//定义局部actor对象
	actor->SetMapper(mapper);
	actor->GetProperty()->SetColor(0.4, 0.4, 0.4);//设置球的颜色,这里可以为每个点赋不同的颜色或者是随机颜色
	return actor;
}

vtkSmartPointer<vtkActor> QT_3DMeasure::RenderGird(double interval, double * color, vtkSmartPointer<vtkRenderWindow> renderWindow)
{
	vtkSmartPointer<vtkPolyDataMapper> v1mmMapper =
		vtkSmartPointer<vtkPolyDataMapper>::New();
	vtkSmartPointer<vtkPolyData> v1mmPolyData =
		vtkSmartPointer<vtkPolyData>::New();
	vtkSmartPointer<vtkCellArray> v1mmLines =
		vtkSmartPointer<vtkCellArray>::New();
	vtkSmartPointer<vtkPoints> v1mmPoints =
		vtkSmartPointer<vtkPoints>::New();

	v1mmPolyData->SetPoints(v1mmPoints);
	v1mmPolyData->SetLines(v1mmLines);
	v1mmMapper->SetInputData(v1mmPolyData);

	vtkSmartPointer<vtkActor> v1mmLinesActor =
		vtkSmartPointer<vtkActor>::New();
	v1mmLinesActor->SetMapper(v1mmMapper);
	v1mmLinesActor->GetProperty()->SetColor(color);
	v1mmLinesActor->GetProperty()->SetLineWidth(1);
	v1mmLinesActor->DragableOff();
	v1mmLinesActor->PickableOff();

	//--------------------start generate------------------
	v1mmPoints->Initialize();
	v1mmLines->Initialize();
	double width = renderWindow->GetActualSize()[0];
	double height = renderWindow->GetActualSize()[1];

	double x, y;
#if 1
	//--------------------vertical lines------------------
	for (x = -width / 2; x <= width / 2; x += interval)
	{
		double linePoint1[3] = { x, 0, -height / 2 };
		double linePoint2[3] = { x, 0, height / 2 };
		vtkIdType pointId1 = v1mmPoints->InsertNextPoint(linePoint1);
		vtkIdType pointId2 = v1mmPoints->InsertNextPoint(linePoint2);
		vtkIdType lineIds[2] = { pointId1, pointId2 };
		v1mmLines->InsertNextCell(2, lineIds);
	}
	//--------------------horizontal lines----------------
	for (y = -height / 2; y <= height / 2; y += interval)
	{
		double linePoint1[3] = { -width / 2, 0, y };
		double linePoint2[3] = { width / 2, 0, y };
		vtkIdType pointId1 = v1mmPoints->InsertNextPoint(linePoint1);
		vtkIdType pointId2 = v1mmPoints->InsertNextPoint(linePoint2);
		vtkIdType lineIds[2] = { pointId1, pointId2 };
		v1mmLines->InsertNextCell(2, lineIds);
	}
#endif
	v1mmPolyData->Modified();
	return v1mmLinesActor;
}


vtkSmartPointer<vtkSTLReader> QT_3DMeasure::GetSTLFileObject(string FileName)
{
	vtkSmartPointer<vtkSTLReader> reader = vtkSmartPointer<vtkSTLReader>::New();
	reader->SetFileName(FileName.c_str());
	reader->Update();
	return reader;
}


/*
旋转坐标系转换到相机表示
*/
void QT_3DMeasure::CameraSiteSet(CameraParamenttypedef transMat, vtkSmartPointer<vtkActor> actor)
{
	Matrix<double, 3, 4> TransMat_Real;
	cv::cv2eigen(transMat.CameraOuterMat, TransMat_Real);
	Matrix<double, 1, 3> SpinMat = WandCalbrateProcess::SpinMattoDegree(TransMat_Real.block(0, 0, 3, 3));
	Matrix<double, 4, 1> T = CameraZeroCal(TransMat_Real, T);

	vtkSmartPointer<vtkTransform> trans =
		vtkSmartPointer<vtkTransform>::New();

	trans->PostMultiply(); 
	trans->Translate(0, 9, 0);
	trans->Scale(0.1, 0.1, 0.1);
	trans->RotateX(90);


	trans->RotateX(SpinMat(0, 0));
	trans->RotateY(SpinMat(0, 1));
	trans->RotateZ(SpinMat(0, 2));
	trans->Translate(T(0, 0) / VTKWidgtZoomRatio, T(1, 0) / VTKWidgtZoomRatio, T(2, 0) / VTKWidgtZoomRatio);
	actor->SetUserTransform(trans);
	actor->InitPathTraversal();
}

vtkSmartPointer<vtkActor> QT_3DMeasure::RenderAddCamera(vtkSmartPointer<vtkSTLReader> reader)
{
	vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInputConnection(reader->GetOutputPort());
	vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
	actor->SetMapper(mapper);
	uiRenderer->AddActor(actor);
	actor->GetProperty()->SetColor(0, 0, 0);
	return actor;
}

void QT_3DMeasure::WorldPointRender(std::vector<cv::Point3d> * ptree)
{
	while (sphereSourceTree.size() < ptree->size())
	{
		vtkSmartPointer<vtkSphereSource> sphereSource = vtkSmartPointer<vtkSphereSource>::New();
		double ci[3] = { 0 , 0, 0 };
		double clo[3] = { 0.3, 0.4, 0.6 };
		sphereSourceActorTree.push_back(RenderSphere(sphereSource, ci, clo));
		sphereSourceTree.push_back(sphereSource);
		uiRenderer->AddActor(sphereSourceActorTree[sphereSourceActorTree.size() - 1]);
	}

	for (int i = 0; i < sphereSourceTree.size(); i++)
	{
		if (i < ptree->size())
		{
			sphereSourceActorTree[i]->SetVisibility(1);
			sphereSourceTree[i]->SetCenter((*ptree)[i].x / VTKWidgtZoomRatio, (*ptree)[i].y / VTKWidgtZoomRatio, (*ptree)[i].z / VTKWidgtZoomRatio);
		}
		else
			sphereSourceActorTree[i]->SetVisibility(0);
	}

	uiRenderWindow->Render();
}
