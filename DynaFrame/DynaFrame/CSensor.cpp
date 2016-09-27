#include "head.h"
#include <strstream>
using namespace cv;
using namespace std;

// ������ģ�顣
// ����ģ�⴫�����������ݡ�
// ����ͼƬ����֮ǰ�Ѿ����߲ɼ��ã���������ֻ��Ҫ���ļ��ж����ļ����ɡ�

CSensor::CSensor()
{
	//this->m_frameNum = FRAME_NUMBER;
	this->m_ProPicNum[0] = GRAY_V_NUMDIGIT * 2;
	this->m_ProPicNum[1] = PHASE_NUMDIGIT;
	this->m_ProPicNum[2] = MAX_FRAME_NUM;

	//this->m_chessFilePath = "Data/20160510_Rotation/";
	this->m_chessFilePath = "Data/20160409_Face2/";
	//this->m_chessFilePath = "Data/20160229_StaticScene/";
	this->m_chessFileFrameNum = "";
	this->m_chessFileCamPath = "FirstFrame/";
	this->m_chessFileCamName = "CameraMat";
	
	this->m_chessFileProPath[0] = "FirstFrame/vGray/";
	this->m_chessFileProPath[1] = "FirstFrame/vPhase/";
	this->m_chessFileProPath[2] = "OtherFrames/";
	this->m_chessFileProName[0] = "vGrayMat";
	this->m_chessFileProName[1] = "vPhaseMat";
	this->m_chessFileProName[2] = "DynaMat";

	this->m_chessFileSuffix = ".bmp";
}

CSensor::~CSensor()
{

}

// ���õ�ǰ�궨��֡����
// �趨֮����ܽ��ж�ȡ��Ĭ���Ǵ�0��ʼ��
//bool CSensor::SetChessFrame(int frame)
//{
//	// �жϲ����Ƿ�Ϸ�
//	if ((frame<0) || (frame >= this->m_frameNum))
//	{
//		ErrorHandling("SetChessFrame->int frame is not valid");
//		return false;
//	}
//
//	strstream ss;
//	ss << frame+1 << '/';
//	ss >> this->m_chessFileFrameNum;
//	return true;
//}

// ��ȡ���ͼ��
cv::Mat CSensor::GetCamFrame()
{
	Mat tempMat;
	tempMat = imread(this->m_chessFilePath
		+ this->m_chessFileFrameNum
		+ this->m_chessFileCamPath
		+ this->m_chessFileCamName
		+ this->m_chessFileSuffix, CV_LOAD_IMAGE_GRAYSCALE);
	Mat convertMat;
	tempMat.copyTo(convertMat);
	//resize(tempMat, convertMat, Size(CAMERA_RESLINE, CAMERA_RESROW));
	if (tempMat.empty())
	{
		ErrorHandling("GetCamFrame->imread error.");
	}
	return convertMat;
}

// ��ȡͶӰ��ͼ��������
int CSensor::GetPicNum(int patternIdx)
{
	if ((patternIdx < 0) || (patternIdx>=3))
	{
		ErrorHandling("GetPicNum->Parameter Error.");
		return false;
	}
	return this->m_ProPicNum[patternIdx];
}

// ��ȡͶӰ��ͼ��
cv::Mat CSensor::GetProFrame(int patternIdx, int picIdx)
{
	// ȷ�������Ϸ�
	if ((patternIdx < 0) || (patternIdx>=3))
	{
		ErrorHandling("CSensor.GetProFrame-> <patternIdx> Parameter Error.");
	}
	if ((picIdx < 0) || (picIdx>=this->m_ProPicNum[patternIdx]))
	{
		ErrorHandling("CSensor.GetProFrame-> <picIdx> Parameter Error.");
	}

	// ��ȡ
	strstream ss;
	string picNum;
	ss << picIdx;
	ss >> picNum;
	Mat tempMat;
	tempMat = imread(this->m_chessFilePath
		+ this->m_chessFileFrameNum
		+ this->m_chessFileProPath[patternIdx]
		+ this->m_chessFileProName[patternIdx]
		+ picNum
		+ this->m_chessFileSuffix, CV_LOAD_IMAGE_GRAYSCALE);
	Mat convertMat;
	tempMat.copyTo(convertMat);
	resize(tempMat, convertMat, Size(CAMERA_RESLINE, CAMERA_RESROW));
	if (tempMat.empty())
	{
		ErrorHandling("GetProFrame->imread error.");
	}
	return convertMat;
}