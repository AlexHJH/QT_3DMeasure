#ifndef Convert_h
#define Convert_h


#include <io.h>
#include <string>
#include <vector>
#include "opencv.hpp"
#include <QDebug>
#include <QString>
#include <QTreeWidget>
#include <QFrame>
#include "FileLib.h"
#include <Dense>
#include <QObject>

namespace MyConverter
{
	cv::Mat Matrix3ftoCvMat(const Eigen::Matrix3f &m);
	Eigen::Matrix<float, 3, 3> CvMattoMatrix3f(const cv::Mat &cvMat3);


	cv::Mat Matrix3dtoCvMat(const Eigen::Matrix3d &m);
	Eigen::Matrix<double, 3, 3> CvMattoMatrix3d(const cv::Mat &cvMat3);

	cv::Mat Matrix43dtoCvMat(const Eigen::Matrix<double, 4, 3> &m);
	Eigen::Matrix<double, 4, 3> CvMattoMatrix43d(const cv::Mat &cvMat43);

	Eigen::Matrix<double, 3, 1> CvMat31toMatrix3d(const cv::Mat &cvMat3);

	QImage cvMat2QImage(cv::Mat& mat);
	cv::Mat QImage2cvMat(QImage image);
};

#endif // Convert_h
