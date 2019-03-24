#include "ImgLib.h"
#include "MyMathLib.h"
#include "qmath.h"


ImgMainProcess::ImgMainProcess(string str)
{
	RoundnessLimit = 0.1;
	Segthreshold = 200;
	CirqueCountLimitMax = 200;
	CirqueCountLimitMin = 20;
	FrameCount = 0;
	DealFrameCount = 0;
	ResizeMatK = 2;
	ExternSizeK = 0.2;
	CalRunning = true;
	SegmentType = CV_THRESHOLD;
	CameraDeviceStr = str;
}

void ImgMainProcess::release()
{
	free(RGBShowImg);
	CopyMainImg.release();
	FrameCount = 0;
}

void ImgMainProcess::ImgProcessCacheInit(cv::Mat * OriImg)
{
	OriMainImg = OriImg;
	///保证copy大小
	if (FrameCount == 0)
	{
		RGBShowImg = new cv::Mat(OriMainImg->size(), CV_32SC3);
	}
	CirquePointTree.clear();
	FrameCount++;
}

////预处理
void ImgMainProcess::ImgSegmentProcess()
{
	cv::resize(*OriMainImg, CopyMainImg, cv::Size(OriMainImg->cols / ResizeMatK, OriMainImg->rows / ResizeMatK));
	if (SegmentType == CV_THRESHOLD)
	{
		cv::threshold(CopyMainImg, CopyMainImg, Segthreshold, 255, THRESH_BINARY);
	}
	else if (SegmentType == CV_OTSU)
	{
		cv::threshold(CopyMainImg, CopyMainImg, Segthreshold, 255, THRESH_OTSU);
	}
	else
	{
		cv::threshold(CopyMainImg, CopyMainImg, Segthreshold, 255, THRESH_BINARY);
	}
}

///////从闭合区域内找圆
bool ImgMainProcess::FindCirqueFromContour(cv::Point3d &p, std::vector<std::vector<cv::Point>> contours,
	int index, double roundlimit, double& roundness, double& rave)
{
	int contourlength = contours[index].size();
	double avg_px = 0, avg_py = 0;
	int i = 0;
	double rtc = 0;
	double * r = new double[contourlength];
	cv::Point2d pt;
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
		r[i] = sqrt((p.x - contours[index][i].x) * (p.x - contours[index][i].x)
			+ (p.y - contours[index][i].y) * (p.y - contours[index][i].y));
		rave += r[i];
	}
	rave = rave / contourlength;
	p.z = rave;

	for (i = 0; i < contourlength; i++)
	{
		rtc += abs(r[i] - rave);
	}
	rtc = rtc / contourlength / rave;
	delete[] r;
	roundness = rtc;
	if (rtc > roundlimit)
		return false;
	return true;
}


Point3d GetCentroidsBinary(Mat Input_img)
{
	int i, j;
	Point3d out;

	double sumX = 0, sumY = 0, sum = 0;
	for (i = 0; i < Input_img.rows; i++)
	{
		for (j = 0; j < Input_img.cols; j++)
		{
			sumX += Input_img.at<unsigned char>(i, j) * j;
			sumY += Input_img.at<unsigned char>(i, j) * i;
			sum += Input_img.at<unsigned char>(i, j);
		}
	}
	out.x = sumX / sum;
	out.y = sumY / sum;
	out.z = 0;
	return out;
}

/////从图上找到圆心生成圆心集
void ImgMainProcess::ImgFindContoursProcess()
{
	int index = 0;
	Point3d CenterPoint;
	double roundnesstemp = 0;
	double radiustemp = 0;
	CirquePointtypedef CirqueSingleData;
	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;
	vector<cv::Point3f> CirqueCenter;
	vector<vector<cv::Point>> CirqueContours;

	cv::findContours(CopyMainImg, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE);

	for (index = 0; index < contours.size(); index++)
	{
		if (contours[index].size() >(CopyMainImg.cols / 200 * M_PI) && contours[index].size() < (CopyMainImg.cols / 20 * M_PI))
		{
			if (FindCirqueFromContour(CenterPoint, contours, index, RoundnessLimit, roundnesstemp, radiustemp) == true)
			{
				Rect OriRect = cv::boundingRect(Mat(contours[index]));

				OriRect.width *= ResizeMatK;
				OriRect.height *= ResizeMatK;
				OriRect.x = ResizeMatK * OriRect.x - OriRect.width * ExternSizeK;
				OriRect.y = ResizeMatK * OriRect.y - OriRect.height * ExternSizeK;
				OriRect.width *= (1 + ExternSizeK * 2);
				OriRect.height *= (1 + ExternSizeK * 2);

				if (OriRect.x < 0)
					OriRect.x = 0;
				if (OriRect.y < 0)
					OriRect.y = 0;
				if (OriRect.x + OriRect.width > OriMainImg->cols)
				{
					OriRect.width = OriMainImg->cols - OriRect.x;
				}
				if (OriRect.y + OriRect.height > OriMainImg->rows)
				{
					OriRect.height = OriMainImg->rows - OriRect.y;
				}
				Mat CirqueMat = OriMainImg->operator()(OriRect);

				cv::threshold(CirqueMat, CirqueMat, 0, 255, cv::THRESH_OTSU);

				CenterPoint = GetCentroidsBinary(CirqueMat);
				CenterPoint.x += OriRect.x;
				CenterPoint.y += OriRect.y;

				////满足第一圆率的圆心集
				CirqueSingleData.CirquePoint = CenterPoint;
				CirqueSingleData.Roundness = roundnesstemp * ResizeMatK;
				CirqueSingleData.CirqueRadius = radiustemp * ResizeMatK;
				CirquePointTree.push_back(CirqueSingleData);
			}
		}
	}
	emit SendCirquePoint(&CirquePointTree);
}


void ImgMainProcess::CloseCal()
{
	CalRunning = false;
}

void ImgMainProcess::OpenCal()
{
	CalRunning = true;
}

void ImgMainProcess::ImgProcessThread(cv::Mat * Input)
{
	if (CalRunning == false)
	{
		return;
	}
	//OriMainImg = Input;
	// 	clock_t start, finish;
	// 	double totaltime;
	// 	start = clock();
	ImgProcessCacheInit(Input);

	//FrameCount++;
	/////缓存拷贝
	// 	finish = clock();
	// 	totaltime = (double)(finish - start) / CLOCKS_PER_SEC;
	// 	qDebug() << "\n此程序的运行时间为" << totaltime << "秒！" << endl;

	ImgSegmentProcess();  ////分割

	ImgFindContoursProcess();

	/////二值化之后的结果
	cv::cvtColor(*OriMainImg, *RGBShowImg, COLOR_GRAY2BGR);
	for (vector<CirquePointtypedef>::iterator it = CirquePointTree.begin(); it != CirquePointTree.end(); it++)
	{
		cv::Point pt;
		pt.x = it->CirquePoint.x;// / ResizeMatK;
		pt.y = it->CirquePoint.y;// / ResizeMatK;
		MyMathLib::drawCross(*RGBShowImg, pt, Scalar(0, 0, 255), it->CirqueRadius * 5, 10);
	}
	emit SendShowRGBImg(RGBShowImg);
}
