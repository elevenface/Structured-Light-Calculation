#ifndef _CCALCULATION_H_
#define _CCALCULATION_H_

#include "CSensorV.h"
#include "CDecodeGray.h"
#include "CDecodePhase.h"
#include "CVisualization.h"


class CCalculation
{
private:
	// ������������
	CSensor * m_sensor;

	// ������
	CDecodeGray * m_decodeGrayv;	// ���������
	CDecodePhase * m_decodePhasev;	// PS����

	// ������Ŀռ�����
	//Mat * m_bwPic;				// ת���ɵĶ�ֵͼ
	//Mat * m_stripPic;			// ʶ���������

	// ÿ֡��¼�µ���ʱ��Ϣ
	Mat * m_stripW;
	Mat * m_stripB;

	// �������P
	Mat * m_ProjectorU;

	// �仯��
	Mat * m_deltaP;
	Mat * m_deltaZ;

	// ÿ֡�ĵ�����Ϣ
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
	string m_resPath;
	string m_pcPath;
	string m_pcFisrtName;
	string m_pcSucceedName;
	string m_pcSuffix;

	// ����FloodFill��ȫ����ʱ����
	Mat m_tempMat;

	// ���ù��ܵ�С����
	bool ReleaseSpace();				// �ͷſռ�
	int FloodFill(int h, int w, uchar from, uchar to);	// ��亯��

	//bool Gray2BwPic(int fN);				// ����ͼ���ֵ��
	
	//bool RecoFirstStripPic();				// ��֡������ʶ��
	//bool RecoOtherStripPic(int fN);			// ��֡�Ķ�̬����ʶ��
	
	// ��֡P����Ľ���
	bool FillFirstProjectorU();				// ��֡�Ľ���
	
	// ����֡��deltaP�Ľ���
	bool FillOtherDeltaProU(int fN);

	//bool FillOtherProjectorU(int fN);		// ����֡�Ķ�̬����
	
	// ��������
	bool FillCoordinate(int i);

	// ʶ��ͼ�е�������ֵ
	bool StripRegression(int fN);

public:
	CCalculation();
	~CCalculation();
	bool Init();				// ��ʼ��
	bool CalculateFirst();		// ������֡
	bool CalculateOther();		// ���㶯̬֡
	bool Result(std::string fileName, int i);	// ��¼���
};


#endif