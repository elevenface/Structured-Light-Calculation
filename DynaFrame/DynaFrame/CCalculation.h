#ifndef _CCALCULATION_H_
#define _CCALCULATION_H_

#include "CSensorV.h"
#include "CDecodeGray.h"
#include "CDecodePhase.h"


class CCalculation
{
private:
	// 传感器控制类
	CSensor * m_sensor;

	// 解码器
	CDecodeGray * m_decodeGrayv;	// 格雷码解码
	CDecodePhase * m_decodePhasev;	// PS解码

	// 计算出的空间坐标
	Mat * m_bwPic;				// 转化成的二值图
	Mat * m_stripPic;			// 识别出的条码
	Mat * m_ProjectorU;			// 生成的具体坐标
	Mat * m_xMat;
	Mat * m_yMat;
	Mat * m_zMat;

	// 标定的矩阵
	Mat m_camMatrix;
	Mat m_proMatrix;
	Mat m_R;
	Mat m_T;
	Mat m_C;
	Mat m_P;
	double m_cA;
	double m_cB;
	Mat m_cC;
	Mat m_cD;
	string m_paraPath;
	string m_paraName;
	string m_paraSuffix;

	// 输出点云的参数
	string m_pcPath;
	string m_pcFisrtName;
	string m_pcSucceedName;
	string m_pcSuffix;

	// 用于FloodFill的全局临时矩阵
	Mat m_tempMat;

	// 常用功能的小函数
	bool ReleaseSpace();				// 释放空间
	bool Gray2BwPic(int fN);				// 进行图像二值化
	int FloodFill(int h, int w, uchar from, uchar to);	// 填充函数
	bool RecoFirstStripPic();				// 首帧的条码识别
	bool RecoOtherStripPic(int fN);			// 首帧的动态条码识别
	bool FillFirstProjectorU();				// 首帧的解码
	bool FillOtherProjectorU(int fN);		// 其他帧的动态解码
	bool FillCoordinate(int i);				// 计算坐标

public:
	CCalculation();
	~CCalculation();
	bool Init();				// 初始化
	bool CalculateFirst();		// 计算首帧
	bool CalculateOther();		// 计算动态帧
	bool Result(std::string fileName, int i);	// 记录结果
};


#endif