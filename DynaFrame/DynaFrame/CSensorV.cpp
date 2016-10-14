#include "CSensorV.h"


// 传感器模块。
// 用于模拟传感器传入数据。
// 为方便起见，相关的数据已经离线采集好，该模块则是模拟传感器模块进行数据的读入。

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

// 初始化传感器：设定一些参数
// 主要是读取的文件路径参数
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

// 关闭传感器
bool CSensor::CloseSensor()
{
	bool status = true;
	
	this->UnloadDatas();

	return status;
}

// 读取图案
// groupNum == 0：读取vGray数据
// groupNum == 1：读取vPhase数据
// groupNum == 2：读取dynaMat数据
bool CSensor::LoadDatas(int groupNum)
{

	// 检查状态是否合法
	if (this->m_dataMats != NULL)
	{
		this->UnloadDatas();
	}

	// 根据选项，选择读入的文件路径
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

	// 申请空间
	this->m_dataMats = new Mat[this->m_dataNum];

	// 读取
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

// 释放已读取图案
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

// 设置投影仪投影的图像
bool CSensor::SetProPicture(int nowNum)
{
	bool status = true;

	// 检查参数是否合法
	if (nowNum >= this->m_dataNum)
	{
		status = false;
		return status;
	}

	this->m_nowNum = nowNum;

	return status;
}

// 获取相机图像
Mat CSensor::GetCamPicture()
{
	bool status = true;

	Mat tempMat;
	this->m_dataMats[this->m_nowNum].copyTo(tempMat);

	return tempMat;
}