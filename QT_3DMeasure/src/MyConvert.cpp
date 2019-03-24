#include "qt_3dmeasure.h"
#include "MyConvert.h"



cv::Mat MyConverter::Matrix3ftoCvMat(const Eigen::Matrix3f &m)
{
	cv::Mat cvMat(3, 3, CV_32F);
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			cvMat.at<float>(i, j) = m(i, j);

	return cvMat.clone();
}

Eigen::Matrix<float, 3, 3> MyConverter::CvMattoMatrix3f(const cv::Mat &cvMat3)
{
	Eigen::Matrix<float, 3, 3> M;

	M << cvMat3.at<float>(0, 0), cvMat3.at<float>(0, 1), cvMat3.at<float>(0, 2),
		cvMat3.at<float>(1, 0), cvMat3.at<float>(1, 1), cvMat3.at<float>(1, 2),
		cvMat3.at<float>(2, 0), cvMat3.at<float>(2, 1), cvMat3.at<float>(2, 2);

	return M;
}

cv::Mat MyConverter::Matrix43dtoCvMat(const Eigen::Matrix<double, 4, 3> &m)
{
	cv::Mat cvMat(4, 3, CV_64F);
	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 3; j++)
			cvMat.at<double>(i, j) = m(i, j);

	return cvMat.clone();
}

Eigen::Matrix<double, 3, 1> MyConverter::CvMat31toMatrix3d(const cv::Mat &cvMat3)
{
	Eigen::Matrix<double, 3, 1> M;
	M << cvMat3.at<double>(0, 0), cvMat3.at<double>(1, 0), cvMat3.at<double>(2, 0);
	return M;
}

cv::Mat MyConverter::Matrix3dtoCvMat(const Eigen::Matrix3d &m)
{
	cv::Mat cvMat(3, 3, CV_64F);
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			cvMat.at<double>(i, j) = m(i, j);

	return cvMat.clone();
}

Eigen::Matrix<double, 4, 3> MyConverter::CvMattoMatrix43d(const cv::Mat &cvMat43)
{
	Eigen::Matrix<double, 4, 3> M;

	M << cvMat43.at<double>(0, 0), cvMat43.at<double>(0, 1), cvMat43.at<double>(0, 2),
		cvMat43.at<double>(1, 0), cvMat43.at<double>(1, 1), cvMat43.at<double>(1, 2),
		cvMat43.at<double>(2, 0), cvMat43.at<double>(2, 1), cvMat43.at<double>(2, 2),
		cvMat43.at<double>(3, 0), cvMat43.at<double>(3, 1), cvMat43.at<double>(3, 2);

	return M;
}

Eigen::Matrix<double, 3, 3> MyConverter::CvMattoMatrix3d(const cv::Mat &cvMat3)
{
	Eigen::Matrix<double, 3, 3> M;

	M << cvMat3.at<double>(0, 0), cvMat3.at<double>(0, 1), cvMat3.at<double>(0, 2),
		cvMat3.at<double>(1, 0), cvMat3.at<double>(1, 1), cvMat3.at<double>(1, 2),
		cvMat3.at<double>(2, 0), cvMat3.at<double>(2, 1), cvMat3.at<double>(2, 2);

	return M;
}

//************************************
// Method:    cvMat2QImage
// FullName:  QT_ForTest::cvMat2QImage
// Access:    private 
// Returns:   QT_NAMESPACE::QImage
// Qualifier:
// Parameter: cv::Mat & mat
//************************************
QImage MyConverter::cvMat2QImage(cv::Mat& mat)
{
	// 8-bits unsigned, NO. OF CHANNELS = 1
	if (mat.type() == CV_8UC1)
	{
		QImage image(mat.cols, mat.rows, QImage::Format_Indexed8);
		// Set the color table (used to translate colour indexes to qRgb values)
		image.setColorCount(256);
		for (int i = 0; i < 256; i++)
		{
			image.setColor(i, qRgb(i, i, i));
		}
		// Copy input Mat
		uchar *pSrc = mat.data;
		for (int row = 0; row < mat.rows; row++)
		{
			uchar *pDest = image.scanLine(row);
			memcpy(pDest, pSrc, mat.cols);
			pSrc += mat.step;
		}
		return image;
	}
	else if (mat.type() == CV_16UC1)
	{
		cv::Mat mat8u(mat.size(), CV_8UC1);
		for (int i = 0; i < mat.rows; i++)
		{
			for (int j = 0; j < mat.cols; j++)
			{
				int n = 255 * mat.at<ushort>(i, j) / 65535; //tt1
				mat8u.at<uchar>(i, j) = n;
			}
		}
		QImage image(mat8u.cols, mat8u.rows, QImage::Format_Indexed8);
		// Set the color table (used to translate colour indexes to qRgb values)
		image.setColorCount(256);
		for (int i = 0; i < 256; i++)
		{
			image.setColor(i, qRgb(i, i, i));
		}
		// Copy input Mat
		uchar *pSrc = mat8u.data;
		for (int row = 0; row < mat8u.rows; row++)
		{
			uchar *pDest = image.scanLine(row);
			memcpy(pDest, pSrc, mat8u.cols);
			pSrc += mat8u.step;
		}
		return image;
	}
	// 8-bits unsigned, NO. OF CHANNELS = 3
	else if (mat.type() == CV_8UC3)
	{
		// Copy input Mat
		const uchar *pSrc = (const uchar*)mat.data;
		// Create QImage with same dimensions as input Mat
		QImage image(pSrc, mat.cols, mat.rows, mat.step, QImage::Format_RGB888);
		return image.rgbSwapped();
	}
	else if (mat.type() == CV_8UC4)
	{
		qDebug() << "CV_8UC4";
		// Copy input Mat
		const uchar *pSrc = (const uchar*)mat.data;
		// Create QImage with same dimensions as input Mat
		QImage image(pSrc, mat.cols, mat.rows, mat.step, QImage::Format_ARGB32);
		return image.copy();
	}
	else
	{
		qDebug() << "ERROR: Mat could not be converted to QImage.";
		return QImage();
	}
}


//************************************
// Method:    QImage2cvMat
// FullName:  QT_ForTest::QImage2cvMat
// Access:    private 
// Returns:   cv
// Qualifier:
// Parameter: QImage image
//************************************
cv::Mat MyConverter::QImage2cvMat(QImage image)
{
	cv::Mat mat;
	qDebug() << image.format();
	switch (image.format())
	{
	case QImage::Format_ARGB32:
	case QImage::Format_RGB32:
	case QImage::Format_ARGB32_Premultiplied:
		mat = cv::Mat(image.height(), image.width(), CV_8UC4, (void*)image.constBits(), image.bytesPerLine());
		break;
	case QImage::Format_RGB888:
		mat = cv::Mat(image.height(), image.width(), CV_8UC3, (void*)image.constBits(), image.bytesPerLine());
		cv::cvtColor(mat, mat, CV_BGR2RGB);
		break;
	case QImage::Format_Indexed8:
		mat = cv::Mat(image.height(), image.width(), CV_8UC1, (void*)image.constBits(), image.bytesPerLine());
		break;
	}
	return mat;
}