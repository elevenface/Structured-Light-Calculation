#ifndef _CCALCULATION_H_
#define _CCALCULATION_H_

#include "CSensorV.h"
#include "CDecodeGray.h"
#include "CDecodePhase.h"


class CCalculation
{
private:
	// ������������
	CSensor * m_sensor;

	// ������
	CDecodeGray * m_decodeGrayv;	// ���������
	CDecodePhase * m_decodePhasev;	// PS����

	// ������Ŀռ�����
	Mat * m_bwPic;				// ת���ɵĶ�ֵͼ
	Mat * m_stripPic;			// ʶ���������
	Mat * m_ProjectorU;			// ���ɵľ�������
	Mat * m_xMat;
	Mat * m_yMat;
	Mat * m_zMat;

	// �궨�ľ���
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

	// ������ƵĲ���
	string m_pcPath;
	string m_pcFisrtName;
	string m_pcSucceedName;
	string m_pcSuffix;

	// ����FloodFill��ȫ����ʱ����
	Mat m_tempMat;

	// ���ù��ܵ�С����
	bool ReleaseSpace();				// �ͷſռ�
	bool Gray2BwPic(int fN);				// ����ͼ���ֵ��
	int FloodFill(int h, int w, uchar from, uchar to);	// ��亯��
	bool RecoFirstStripPic();				// ��֡������ʶ��
	bool RecoOtherStripPic(int fN);			// ��֡�Ķ�̬����ʶ��
	bool FillFirstProjectorU();				// ��֡�Ľ���
	bool FillOtherProjectorU(int fN);		// ����֡�Ķ�̬����
	bool FillCoordinate(int i);				// ��������

public:
	CCalculation();
	~CCalculation();
	bool Init();				// ��ʼ��
	bool CalculateFirst();		// ������֡
	bool CalculateOther();		// ���㶯̬֡
	bool Result(std::string fileName, int i);	// ��¼���
};


#endif