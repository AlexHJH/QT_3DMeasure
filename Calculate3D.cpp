#include "Calculate3D.h"
#include "MyConvert.h"
#include "MyMathLib.h"
#include "qmath.h"
#include <opencv2/core/eigen.hpp>
#include <limits.h>

void CalPolarEquation(Matrix<double, 3, 4> Camera1ExternMat, Matrix<double, 3, 4> Camera2ExternMat,
	Matrix<double, 3, 3> Camera1InterMat, Matrix<double, 3, 3> Camera2InterMat, Matrix<double, 3, 1> Pointcal)
{
	//////Pointcal为原始像素坐标
	Pointcal = Camera1InterMat * Pointcal;
	Matrix<double, 3, 3> R1 = Camera1ExternMat.block(0, 0, 3, 3);
	Matrix<double, 3, 3> R2 = Camera2ExternMat.block(0, 0, 3, 3);
	Matrix<double, 3, 1> T1 = Camera1ExternMat.block(0, 3, 3, 1);
	Matrix<double, 3, 1> T2 = Camera2ExternMat.block(0, 3, 3, 1);

	Matrix<double, 3, 3> Mcc = R2 * R1.inverse();
	Matrix<double, 3, 1> MccUc = Mcc * Pointcal;
}

class GetDistanceOf2linesIn3D
{
public:
	//输入直线A的两个点，以便获得A的方程
	void SetLineA(double A1x, double A1y, double A1z, double A2x, double A2y, double A2z)
	{
		a1_x = A1x;
		a1_y = A1y;
		a1_z = A1z;

		a2_x = A2x;
		a2_y = A2y;
		a2_z = A2z;
	}

	//输入直线B的两个点，以便获得B的方程
	void SetLineB(double B1x, double B1y, double B1z, double B2x, double B2y, double B2z)
	{
		b1_x = B1x;
		b1_y = B1y;
		b1_z = B1z;

		b2_x = B2x;
		b2_y = B2y;
		b2_z = B2z;
	}

	//用SetLineA、SetLineB输入A、B方程后
	//调用本函数解出结果
	void GetDistance()
	{
		//方法来自：http://blog.csdn.net/pi9nc/article/details/11820545

		double d1_x = a2_x - a1_x;
		double d1_y = a2_y - a1_y;
		double d1_z = a2_z - a1_z;

		double d2_x = b2_x - b1_x;
		double d2_y = b2_y - b1_y;
		double d2_z = b2_z - b1_z;

		double e_x = b1_x - a1_x;
		double e_y = b1_y - a1_y;
		double e_z = b1_z - a1_z;


		double cross_e_d2_x, cross_e_d2_y, cross_e_d2_z;
		cross(e_x, e_y, e_z, d2_x, d2_y, d2_z, cross_e_d2_x, cross_e_d2_y, cross_e_d2_z);
		double cross_e_d1_x, cross_e_d1_y, cross_e_d1_z;
		cross(e_x, e_y, e_z, d1_x, d1_y, d1_z, cross_e_d1_x, cross_e_d1_y, cross_e_d1_z);
		double cross_d1_d2_x, cross_d1_d2_y, cross_d1_d2_z;
		cross(d1_x, d1_y, d1_z, d2_x, d2_y, d2_z, cross_d1_d2_x, cross_d1_d2_y, cross_d1_d2_z);

		double t1, t2;
		t1 = dot(cross_e_d2_x, cross_e_d2_y, cross_e_d2_z, cross_d1_d2_x, cross_d1_d2_y, cross_d1_d2_z);
		t2 = dot(cross_e_d1_x, cross_e_d1_y, cross_e_d1_z, cross_d1_d2_x, cross_d1_d2_y, cross_d1_d2_z);
		double dd = norm(cross_d1_d2_x, cross_d1_d2_y, cross_d1_d2_z);
		t1 /= dd*dd;
		t2 /= dd*dd;

		//得到最近的位置
		PonA_x = (a1_x + (a2_x - a1_x)*t1);
		PonA_y = (a1_y + (a2_y - a1_y)*t1);
		PonA_z = (a1_z + (a2_z - a1_z)*t1);

		PonB_x = (b1_x + (b2_x - b1_x)*t2);
		PonB_y = (b1_y + (b2_y - b1_y)*t2);
		PonB_z = (b1_z + (b2_z - b1_z)*t2);

		distance = norm(PonB_x - PonA_x, PonB_y - PonA_y, PonB_z - PonA_z);
	}



	double PonA_x;//两直线最近点之A线上的点的x坐标
	double PonA_y;//两直线最近点之A线上的点的y坐标
	double PonA_z;//两直线最近点之A线上的点的z坐标
	double PonB_x;//两直线最近点之B线上的点的x坐标
	double PonB_y;//两直线最近点之B线上的点的y坐标
	double PonB_z;//两直线最近点之B线上的点的z坐标
	double distance;//两直线距离
private:
	//直线A的第一个点
	double a1_x;
	double a1_y;
	double a1_z;
	//直线A的第二个点
	double a2_x;
	double a2_y;
	double a2_z;

	//直线B的第一个点
	double b1_x;
	double b1_y;
	double b1_z;

	//直线B的第二个点
	double b2_x;
	double b2_y;
	double b2_z;


	//点乘
	double dot(double ax, double ay, double az, double bx, double by, double bz) { return ax*bx + ay*by + az*bz; }
	//向量叉乘得到法向量，最后三个参数为输出参数
	void cross(double ax, double ay, double az, double bx, double by, double bz, double& x, double& y, double& z)
	{
		x = ay*bz - az*by;
		y = az*bx - ax*bz;
		z = ax*by - ay*bx;
	}
	//向量取模
	double norm(double ax, double ay, double az) { return sqrt(dot(ax, ay, az, ax, ay, az)); }
};

Eigen::Matrix<double, 3, 3> AntisymmetricMatrix(Eigen::Matrix<double, 3, 1> TFunda)
{
	Eigen::Matrix<double, 3, 3> TFundaTrans;                                              /////反对称矩阵
	TFundaTrans.setZero();
	TFundaTrans(0, 1) = -TFunda(2, 0);
	TFundaTrans(0, 2) = TFunda(1, 0);
	TFundaTrans(1, 0) = TFunda(2, 0);
	TFundaTrans(1, 2) = -TFunda(0, 0);
	TFundaTrans(2, 0) = -TFunda(1, 0);
	TFundaTrans(2, 1) = TFunda(0, 0);
	return TFundaTrans;
}

//////从0转换到1
Eigen::Matrix<double, 3, 3> CalFundmentalMatrix(Eigen::Matrix<double, 3, 4> CameraExternMatrix[2], Eigen::Matrix<double, 3, 3> CameraInterMatrix[2])
{
	Eigen::Matrix<double, 3, 3> RMatrixTrans;
	RMatrixTrans = CameraExternMatrix[1].block(0, 0, 3, 3) * (CameraExternMatrix[0].block(0, 0, 3, 3)).inverse();
	Eigen::Matrix<double, 3, 1> TMatrixTrans;
	TMatrixTrans = CameraExternMatrix[1].col(3) - RMatrixTrans * CameraExternMatrix[0].col(3);
	Eigen::Matrix<double, 3, 3> TAntisymmetricMatrix;
	TAntisymmetricMatrix = AntisymmetricMatrix(TMatrixTrans);   ///反对称矩阵
	Eigen::Matrix<double, 3, 3> FundmentalMatrix;
	FundmentalMatrix = (CameraInterMatrix[1].inverse().transpose()) * TAntisymmetricMatrix * RMatrixTrans * (CameraInterMatrix[0].inverse());
	return FundmentalMatrix;
}

typedef struct
{
	uint64_t	 Pointindex0;
	uint64_t	 Pointindex1;
	double	     Length;
}PointMatchtypedef;

typedef struct
{
	vector<uint64_t> Pointindex;
	double		     Length;
}MutiPointMatchypedef;

typedef struct
{
	Eigen::Matrix<double, 1, 3> pointMatrix;
	double fundaLength;
	double distance;
}MatrixMatchtypedef;

bool MyPointLengthCompare(const PointMatchtypedef a, const PointMatchtypedef b)
{
	return a.Length < b.Length;
}

bool MyMultiPointMatchCompare(const MutiPointMatchypedef a, const MutiPointMatchypedef b)
{
	return a.Length < b.Length;
}


bool MyMatrixMatchCompare(const MatrixMatchtypedef a, MatrixMatchtypedef b)
{
	if (abs(a.fundaLength - b.fundaLength) < 20)
	{
		return a.distance < b.distance;
	}
	else
		return a.fundaLength < b.fundaLength;
}

#if 0
void Calculator3D::CalPointMatch(std::vector<std::vector<Eigen::Matrix<double, 1, 3>>> &Pointstree, std::vector<Eigen::Matrix<double, 3, 4>> CameraExternMatrix,
	std::vector<Matrix<double, 3, 3>> CameraInterMatrix, int &size, std::vector<std::vector<bool>> &PointMatchTree)
{
	int CameraSize = CameraInterMatrix.size();
	//std::vector<std::vector<bool>> PointMatchTree;
	PointMatchTree.resize(CameraSize);
	for (int i = 0; i < CameraSize; i++)
	{
		PointMatchTree[i].resize(Pointstree[i].size());
	}
	for (int i = 0; i < CameraSize; i++)
	{
		for (int j = i + 1; j < CameraSize; j++)
		{
			Eigen::Matrix<double, 3, 4> CameraExternMatrixTemp[2];
			Eigen::Matrix<double, 3, 3> CameraInterMatrixTemp[2];
			CameraExternMatrixTemp[0] = CameraExternMatrix[i];
			CameraExternMatrixTemp[1] = CameraExternMatrix[j];
			CameraInterMatrixTemp[0] = CameraInterMatrix[i];
			CameraInterMatrixTemp[1] = CameraInterMatrix[j];
			Eigen::Matrix<double, 3, 3> FundmentalMatrix = CalFundmentalMatrix(CameraExternMatrixTemp, CameraInterMatrixTemp);
			for (int k = 0; k < Pointstree[i].size(); k++)
			{
				Matrix<double, 3, 1> ParamentMatrix = FundmentalMatrix * (Pointstree[i][k].transpose());  ///第I个相机第K个点的极线方程
				for (int m = 0; m < Pointstree[j].size(); m++)
				{
					double Length = abs(Pointstree[j][m] * ParamentMatrix) / sqrt(pow(ParamentMatrix(0, 0), 2) + pow(ParamentMatrix(1, 0), 2));
					if (Length < 200 && (!PointMatchTree[i][k] || !PointMatchTree[j][m]))
					{
						PointMatchTree[i][k] = true;
						PointMatchTree[j][m] = true;
					}
				}
			}
		}
	}
	/*
	for (int i = 0; i < PointMatchTree.size(); i++)
	{
		int j = 0;
		for (vector<Eigen::Matrix<double, 1, 3>>::iterator it = Pointstree[i].begin(); it != Pointstree[i].end();)
		{
			if (!PointMatchTree[i][j])
			{
				it = Pointstree[i].erase(it);
			}
			else
			{
				++it;
			}
			j++;
		}
	}*/

	int minPoint = INT_MAX;
	for (int i = 0; i < Pointstree.size(); i++)
	{
		if (Pointstree[i].size() < minPoint)
		{
			minPoint = Pointstree[i].size();
		}
	}
	size = minPoint;
	return;
}
#else
void Calculator3D::CalPointMatch(std::vector<std::vector<Eigen::Matrix<double, 1, 3>>> &Pointstree, std::vector<Eigen::Matrix<double, 3, 4>> CameraExternMatrix,
	std::vector<Matrix<double, 3, 3>> CameraInterMatrix, int &size, std::vector<std::vector<Eigen::Matrix<double, 1, 3>>> &pointsMatchList)
{
	int CameraSize = CameraInterMatrix.size();
	int i, j, k;
	pointsMatchList.clear();
	pointsMatchList.resize(Pointstree[0].size());
	for (i = 0; i < pointsMatchList.size(); i++)
	{
		pointsMatchList[i].clear();
		pointsMatchList[i].push_back(Pointstree[0][i]);
	}
	for (j = 0; j < pointsMatchList.size(); j++)
	{
		for (i = 1; i < CameraSize; i++)
		{
			Eigen::Matrix<double, 3, 4> CameraExternMatrixTemp[2];
			Eigen::Matrix<double, 3, 3> CameraInterMatrixTemp[2];
			CameraExternMatrixTemp[0] = CameraExternMatrix[0];
			CameraExternMatrixTemp[1] = CameraExternMatrix[i];
			CameraInterMatrixTemp[0] = CameraInterMatrix[0];
			CameraInterMatrixTemp[1] = CameraInterMatrix[i];
			Eigen::Matrix<double, 3, 3> FundmentalMatrix = CalFundmentalMatrix(CameraExternMatrixTemp, CameraInterMatrixTemp);
			vector<MatrixMatchtypedef> fundmentalDistanceList;
			for (k = 0; k < Pointstree[i].size(); k++)
			{
				Matrix<double, 3, 1> ParamentMatrix = FundmentalMatrix * (pointsMatchList[j][0].transpose());  ///第0个相机第j个点的极线方程
				double length = abs(Pointstree[i][k] * ParamentMatrix) / sqrt(pow(ParamentMatrix(0, 0), 2) + pow(ParamentMatrix(1, 0), 2));
				if (length < 200)
				{
					MatrixMatchtypedef MatrixMatchtemp;
					MatrixMatchtemp.pointMatrix = Pointstree[i][k];
					MatrixMatchtemp.fundaLength = length;
					MatrixMatchtemp.distance = MyMathLib::CalTowPointDistance(Pointstree[i][k], pointsMatchList[j][0]);
					fundmentalDistanceList.push_back(MatrixMatchtemp);
				}
			}
			if (fundmentalDistanceList.size() > 0)
			{
				sort(fundmentalDistanceList.begin(), fundmentalDistanceList.end(), MyMatrixMatchCompare);
				pointsMatchList[j].push_back(fundmentalDistanceList[0].pointMatrix);
			}
		}
	}
	for (std::vector<std::vector<Eigen::Matrix<double, 1, 3>>>::iterator it = pointsMatchList.begin(); it != pointsMatchList.end();)
	{
		if (it->size() < CameraSize)
		{
			it = pointsMatchList.erase(it);
		}
		else
		{
			++it;
		}
	}
	int minPoint = pointsMatchList.size();
	size = minPoint;
	return;
}
#endif

/*
void Calculator3D::CalPointMatch(std::vector<std::vector<Eigen::Matrix<double, 1, 3>>> &Pointstree, std::vector<Eigen::Matrix<double, 3, 4>> CameraExternMatrix,
	std::vector<Matrix<double, 3, 3>> CameraInterMatrix, int &size)
{
	
	////达到这样一个效果。最终输出的点对是匹配的
	if (Pointstree.size() != CameraExternMatrix.size() ||
		CameraExternMatrix.size() != CameraInterMatrix.size() ||
		CameraInterMatrix.size() != Pointstree.size())
	{
		size = 0;
		return;
	}
	//////核对相机数量
	int CameraSize = CameraInterMatrix.size();
	vector<PointMatchtypedef> PointMatchTree;
	for (int i = 0; i < CameraSize; i++)
	{
		for (int j = i + 1; j < CameraSize; j++)
		{
			Eigen::Matrix<double, 3, 4> CameraExternMatrixTemp[2];
			Eigen::Matrix<double, 3, 3> CameraInterMatrixTemp[2];
			CameraExternMatrixTemp[0] = CameraExternMatrix[i];
			CameraExternMatrixTemp[1] = CameraExternMatrix[j];
			CameraInterMatrixTemp[0] = CameraInterMatrix[i];
			CameraInterMatrixTemp[1] = CameraInterMatrix[j];
			Eigen::Matrix<double, 3, 3> FundmentalMatrix = CalFundmentalMatrix(CameraExternMatrixTemp, CameraInterMatrixTemp);
			for (int k = 0; k < Pointstree[i].size(); k++)
			{
				Matrix<double, 3, 1> ParamentMatrix = FundmentalMatrix * (Pointstree[i][k].transpose());  ///第I个相机第K个点的极线方程
				for (int m = 0; m < Pointstree[j].size(); m++)
				{
					double Length = abs(Pointstree[j][m] * ParamentMatrix) / sqrt(pow(ParamentMatrix(0, 0), 2) + pow(ParamentMatrix(1, 0), 2));
					PointMatchtypedef PointMatchtemp;
					PointMatchtemp.Pointindex0 = 0;
					PointMatchtemp.Pointindex1 = 0;
					PointMatchtemp.Pointindex0 |= (int64_t)i << 32;
					PointMatchtemp.Pointindex1 |= (int64_t)j << 32;
					PointMatchtemp.Pointindex0 |= (int32_t)k;
					PointMatchtemp.Pointindex1 |= (int32_t)m;
					PointMatchtemp.Length = Length;
					PointMatchTree.push_back(PointMatchtemp);
				}
			}
		}
	}
	sort(PointMatchTree.begin(), PointMatchTree.end(), MyPointLengthCompare);
	int minPoint = INT_MAX;
	for (int i = 0; i < Pointstree.size(); i++)
	{
		if (Pointstree[i].size() < minPoint)
		{
			minPoint = Pointstree[i].size();
		}
	}
	size = minPoint;
	return;
	///////////
	int MatchCountMax = 0;
	if (minPoint > 1)
	{
		MatchCountMax = MyMathLib::combinator(CameraSize, 2) * MyMathLib::combinator(minPoint, 2);
	}
	else
		MatchCountMax = 1;
	///////最大匹配对数
	vector<MutiPointMatchypedef> FinalMatchPoint;
	for (int i = 0; i < PointMatchTree.size() && i < MatchCountMax; i++)
	{
		vector<Eigen::Matrix<double, 1, 3>> Ptemp(CameraSize);
		///现在规定一定要全部的相机都找到
		if (PointMatchTree[i].Length > 200)
		{
			break;
		}
		if (FinalMatchPoint.size() < 1)
		{
			MutiPointMatchypedef mt;
			mt.Length = 0;
			mt.Pointindex.push_back(PointMatchTree[i].Pointindex0);
			mt.Pointindex.push_back(PointMatchTree[i].Pointindex1);
			mt.Length += PointMatchTree[i].Length;
			FinalMatchPoint.push_back(mt);
		}
		else
		{
			bool isMatched = false;
			for (int j = 0; j < FinalMatchPoint.size(); j++)
			{
				for (int k = 0; k < FinalMatchPoint[j].Pointindex.size(); k++)
				{
					if (FinalMatchPoint[j].Pointindex[k] == PointMatchTree[i].Pointindex0)                    //////遍历所有元素如果有节点相等则相加
					{
						FinalMatchPoint[j].Pointindex.push_back(PointMatchTree[i].Pointindex1);
						FinalMatchPoint[j].Length += PointMatchTree[i].Length;
						isMatched = true;
						break;
					}
					else if (FinalMatchPoint[j].Pointindex[k] == PointMatchTree[i].Pointindex1)
					{
						FinalMatchPoint[j].Pointindex.push_back(PointMatchTree[i].Pointindex0);
						FinalMatchPoint[j].Length += PointMatchTree[i].Length;
						isMatched = true;
						break;
					}
				}
				if (isMatched)
				{
					break;
				}
			}
			if (isMatched == false)
			{
				//////现有的所有节点均不满足
				MutiPointMatchypedef mt;
				mt.Length = 0;
				mt.Pointindex.push_back(PointMatchTree[i].Pointindex0);
				mt.Pointindex.push_back(PointMatchTree[i].Pointindex1);
				mt.Length += PointMatchTree[i].Length;
				FinalMatchPoint.push_back(mt);
			}
		}
	}
	for (int i = 0; i < FinalMatchPoint.size(); i++)
	{
		FinalMatchPoint[i].Length /= FinalMatchPoint[i].Pointindex.size();
	}
	sort(FinalMatchPoint.begin(), FinalMatchPoint.end(), MyMultiPointMatchCompare);
	std::vector<std::vector<Eigen::Matrix<double, 1, 3>>> Pointstreetemp(CameraSize);
	for (int i = 0; i < CameraSize; i++)
	{
		Pointstreetemp[i].resize(FinalMatchPoint.size());
	}
	int FinalMinMatch = 0;
	for (int i = 0; i < FinalMatchPoint.size(); i++)
	{
		if (FinalMatchPoint[i].Pointindex.size() == CameraSize)
		{
			for (int k = 0; k < CameraSize; k++)
			{
				int CameraIndex = FinalMatchPoint[i].Pointindex[k] >> 32;
				int PointIndex = FinalMatchPoint[i].Pointindex[k] & 0x00000000FFFFFFFF;
				Pointstreetemp[CameraIndex][FinalMinMatch] = Pointstree[CameraIndex][PointIndex];			
			}
			FinalMinMatch++;
		}
	}
	Pointstreetemp.swap(Pointstree);
	size = FinalMinMatch;
	return;

}
*/

void Calculator3D::CalPointGlobalSite(const std::vector<Eigen::Matrix<double, 3, 4>> CameraExternMat,
	const std::vector<Eigen::Matrix<double, 1, 3>> Pointstree, Eigen::Matrix<double, 1, 3> &output)
{
	Eigen::Matrix<double, Dynamic, 3> A;
	Eigen::Matrix<double, Dynamic, 1> B;
	Eigen::Matrix<double, 3, 1> T;
	A.resize(2 * CameraExternMat.size(), 3);
	B.resize(2 * CameraExternMat.size(), 1);
	A.setZero();
	B.setZero();
	output.setZero();

	if (CameraExternMat.size() < 2 ||
		CameraExternMat.size() != Pointstree.size())
	{
		///////////输入大小错误，不匹配
		qDebug() << "Camera Parament Size Error" << endl;
		return;
	}

	for (int i = 0; i < CameraExternMat.size(); i++)
	{
		A(2 * i + 0, 0) = Pointstree[i](0, 0) * CameraExternMat[i](2, 0) - CameraExternMat[i](0, 0);
		A(2 * i + 0, 1) = Pointstree[i](0, 0) * CameraExternMat[i](2, 1) - CameraExternMat[i](0, 1);
		A(2 * i + 0, 2) = Pointstree[i](0, 0) * CameraExternMat[i](2, 2) - CameraExternMat[i](0, 2);

		A(2 * i + 1, 0) = Pointstree[i](0, 1) * CameraExternMat[i](2, 0) - CameraExternMat[i](1, 0);
		A(2 * i + 1, 1) = Pointstree[i](0, 1) * CameraExternMat[i](2, 1) - CameraExternMat[i](1, 1);
		A(2 * i + 1, 2) = Pointstree[i](0, 1) * CameraExternMat[i](2, 2) - CameraExternMat[i](1, 2);

		B(2 * i + 0, 0) = CameraExternMat[i](0, 3) - Pointstree[i](0, 0) * CameraExternMat[i](2, 3);
		B(2 * i + 1, 0) = CameraExternMat[i](1, 3) - Pointstree[i](0, 1) * CameraExternMat[i](2, 3);
	}

	T = (A.transpose() * A).inverse() * (A.transpose() * B);
	output = T.transpose();
}

void CalPointGlobalSite(Matrix<double, 3, 4> Camera1ExternMat, Matrix<double, 3, 4> Camera2ExternMat,
	Matrix<double, 1, 3> Point1, Matrix<double, 1, 3> Point2, Eigen::Matrix<double, 1, 3> &output)
{
	Matrix<double, 1, 3> Result;
	Matrix<double, 4, 3> A;
	Matrix<double, 4, 1> B;

	Matrix<double, 3, 1> T;
	A.setZero();
	B.setZero();

	A(0, 0) = Point1(0, 0) * Camera1ExternMat(2, 0) - Camera1ExternMat(0, 0);
	A(0, 1) = Point1(0, 0) * Camera1ExternMat(2, 1) - Camera1ExternMat(0, 1);
	A(0, 2) = Point1(0, 0) * Camera1ExternMat(2, 2) - Camera1ExternMat(0, 2);

	A(1, 0) = Point1(0, 1) * Camera1ExternMat(2, 0) - Camera1ExternMat(1, 0);
	A(1, 1) = Point1(0, 1) * Camera1ExternMat(2, 1) - Camera1ExternMat(1, 1);
	A(1, 2) = Point1(0, 1) * Camera1ExternMat(2, 2) - Camera1ExternMat(1, 2);

	A(2, 0) = Point2(0, 0) * Camera2ExternMat(2, 0) - Camera2ExternMat(0, 0);
	A(2, 1) = Point2(0, 0) * Camera2ExternMat(2, 1) - Camera2ExternMat(0, 1);
	A(2, 2) = Point2(0, 0) * Camera2ExternMat(2, 2) - Camera2ExternMat(0, 2);

	A(3, 0) = Point2(0, 1) * Camera2ExternMat(2, 0) - Camera2ExternMat(1, 0);
	A(3, 1) = Point2(0, 1) * Camera2ExternMat(2, 1) - Camera2ExternMat(1, 1);
	A(3, 2) = Point2(0, 1) * Camera2ExternMat(2, 2) - Camera2ExternMat(1, 2);


	B(0, 0) = Camera1ExternMat(0, 3) - Point1(0, 0) * Camera1ExternMat(2, 3);
	B(1, 0) = Camera1ExternMat(1, 3) - Point1(0, 1) * Camera1ExternMat(2, 3);
	B(2, 0) = Camera2ExternMat(0, 3) - Point2(0, 0) * Camera2ExternMat(2, 3);
	B(3, 0) = Camera2ExternMat(1, 3) - Point2(0, 1) * Camera2ExternMat(2, 3);

	T = (A.transpose() * A).inverse() * (A.transpose() * B);
	Result = T.transpose();

	output = Result;
}