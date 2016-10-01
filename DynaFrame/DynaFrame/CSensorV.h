#ifndef _CSENSOR_H_
#define _CSENSOR_H_

#include <string>
#include <opencv2/opencv.hpp>
#include <strstream>
#include "StaticParameters.h"
#include "GlobalFunction.h"
#include "CStorage.h"

using namespace std;
using namespace cv;

// ������ģ�顣
class CSensor
{
private:
	// ��ǰ��ȡ������أ�
	int m_dataNum;
	int m_nowNum;
	string m_filePath;
	string m_fileName;
	string m_fileSuffix;

	// �������ݣ�
	string m_groupDataPath;	// һ�����ݵ�path
	string m_iFramePath;
	string m_cFramePath;
	string m_vGrayName;
	string m_vPhaseName;
	string m_dynaName;
	string m_dataFileSuffix;

	// �洢�����ͼ��
	Mat * m_dataMats;

public:
	CSensor();
	~CSensor();

	// ��ʼ��������
	bool InitSensor();

	// �رմ�����
	bool CloseSensor();

	// ��ȡͼ��
	// groupNum == 0����ȡvGray����
	// groupNum == 1����ȡvPhase����
	// groupNum == 2����ȡdynaMat����
	bool LoadDatas(int groupNum);

	// �ͷ��Ѷ�ȡͼ��
	bool UnloadDatas();

	// ����ͶӰ��ͶӰ��ͼ��
	bool SetProPicture(int nowNum);
	
	// ��ȡ���ͼ��
	Mat GetCamPicture();
};

#endif