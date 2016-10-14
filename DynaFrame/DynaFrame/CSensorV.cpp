#include "CSensorV.h"


// ������ģ�顣
// ����ģ�⴫�����������ݡ�
// Ϊ�����������ص������Ѿ����߲ɼ��ã���ģ������ģ�⴫����ģ��������ݵĶ��롣

CSensor::CSensor()
{
	this->m_dataNum = 0;
	this->m_nowNum = 0;
	this->m_filePath = "";
	this->m_fileName = "";
	this->m_fileSuffix = "";

	this->m_dataMats = 0;

}

CSensor::~CSensor()
{
	if (this->m_dataMats != NULL)
	{
		delete[]this->m_dataMats;
		this->m_dataMats = NULL;
	}
}

// ��ʼ�����������趨һЩ����
// ��Ҫ�Ƕ�ȡ���ļ�·������
bool CSensor::InitSensor()
{
	bool status = true;

	this->m_groupDataPath = DATA_PATH + "Results\\2\\";
	this->m_iFramePath = "iFrame\\";
	this->m_cFramePath = "cFrame\\";
	this->m_vGrayName = "vGrayCam";
	this->m_vPhaseName = "vPhaseCam";
	this->m_dynaName = "dynaCam";
	this->m_dataFileSuffix = ".bmp";

	return status;
}

// �رմ�����
bool CSensor::CloseSensor()
{
	bool status = true;
	
	this->UnloadDatas();

	return status;
}

// ��ȡͼ��
// groupNum == 0����ȡvGray����
// groupNum == 1����ȡvPhase����
// groupNum == 2����ȡdynaMat����
bool CSensor::LoadDatas(int groupNum)
{

	// ���״̬�Ƿ�Ϸ�
	if (this->m_dataMats != NULL)
	{
		this->UnloadDatas();
	}

	// ����ѡ�ѡ�������ļ�·��
	if (groupNum == 0)
	{
		this->m_dataNum = GRAY_V_NUMDIGIT * 2;
		this->m_nowNum = 0;
		this->m_filePath = this->m_groupDataPath + this->m_iFramePath;
		this->m_fileName = this->m_vGrayName;
		this->m_fileSuffix = this->m_dataFileSuffix;
	}
	else if (groupNum == 1)
	{
		this->m_dataNum = PHASE_NUMDIGIT;
		this->m_nowNum = 0;
		this->m_filePath = this->m_groupDataPath + this->m_iFramePath;
		this->m_fileName = this->m_vPhaseName;
		this->m_fileSuffix = this->m_dataFileSuffix;
	}
	else if (groupNum == 2)
	{
		this->m_dataNum = DYNAFRAME_MAXNUM;
		this->m_nowNum = 0;
		this->m_filePath = this->m_groupDataPath + this->m_cFramePath;
		this->m_fileName = this->m_dynaName;
		this->m_fileSuffix = this->m_dataFileSuffix;
	}
	else
	{
		return false;
	}

	// ����ռ�
	this->m_dataMats = new Mat[this->m_dataNum];

	// ��ȡ
	for (int i = 0; i < this->m_dataNum; i++)
	{
		Mat tempMat;
		string idx2Str;
		strstream ss;
		ss << i ;
		ss >> idx2Str;
		
		tempMat = imread(this->m_filePath
			+ this->m_fileName
			+ idx2Str
			+ this->m_fileSuffix, CV_LOAD_IMAGE_GRAYSCALE);
		tempMat.copyTo(this->m_dataMats[i]);

		/*cout << this->m_filePath
			+ this->m_fileName
			+ idx2Str
			+ this->m_fileSuffix << endl;*/

		if (tempMat.empty())
		{
			ErrorHandling("CSensor::LoadPatterns::<Read>, imread error: "
				+ this->m_filePath
				+ this->m_fileName
				+ idx2Str
				+ this->m_fileSuffix);
		}
	}

	return true;
}

// �ͷ��Ѷ�ȡͼ��
bool CSensor::UnloadDatas()
{
	if (this->m_dataMats != NULL)
	{
		delete[]this->m_dataMats;
		this->m_dataMats = NULL;
	}
	
	this->m_dataNum = 0;
	this->m_nowNum = 0;
	this->m_filePath = "";
	this->m_fileName = "";
	this->m_fileSuffix = "";

	return true;
}

// ����ͶӰ��ͶӰ��ͼ��
bool CSensor::SetProPicture(int nowNum)
{
	bool status = true;

	// �������Ƿ�Ϸ�
	if (nowNum >= this->m_dataNum)
	{
		status = false;
		return status;
	}

	this->m_nowNum = nowNum;

	return status;
}

// ��ȡ���ͼ��
Mat CSensor::GetCamPicture()
{
	bool status = true;

	Mat tempMat;
	this->m_dataMats[this->m_nowNum].copyTo(tempMat);

	return tempMat;
}