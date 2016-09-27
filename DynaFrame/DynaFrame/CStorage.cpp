#include "head.h"
#include <strstream>

CStorage::CStorage()
{
	this->m_matFilePath = "";
	this->m_matFileName = "";
	this->m_matFileSuffix = "";
	this->m_storagePath = "";
}

CStorage::~CStorage()
{

}

bool CStorage::Store(cv::Mat * pictures, int num)
{
	using namespace cv;

	// �жϲ����Ƿ�Ϸ�
	if (num <= 0)
		return false;

	bool status = true;

	// �洢
	if (num == 1)
	{
		status = imwrite(this->m_storagePath, *pictures);
		if (!status)
		{
			// ����Ŀ¼
			string temp = this->m_matFilePath;
			for (int c = 0; c < temp.length(); c++)
			{
				if (temp[c] == '/')
					temp[c] = '\\';
			}
			system((string("mkdir ") + temp).c_str());
			status = imwrite(this->m_storagePath, *pictures);
		}
	}
	else
	{
		for (int i = 0; i < num; i++)
		{
			std::string tempNum;
			std::strstream ss;
			ss << i;
			ss >> tempNum;
			status = imwrite(this->m_storagePath, pictures[i]);
			if (!status)
			{
				// ����Ŀ¼
				string temp = this->m_matFilePath;
				for (int c = 0; c < temp.length(); c++)
				{
					if (temp[c] == '/')
						temp[c] = '\\';
				}
				system((string("mkdir ") + temp).c_str());
				status = imwrite(this->m_storagePath, pictures[i]);
			}
		}
	}

	if (!status)
	{
		ErrorHandling("CStorage.Store->imwrite Error.");
	}

	return true;
}

// �趨�洢Ŀ¼
bool CStorage::SetMatFileName(std::string matFilePath,
	std::string matFileName,
	std::string matFileSuffix)
{
	using namespace std;

	// �������
	this->m_matFilePath = matFilePath;
	this->m_matFileName = matFileName;
	this->m_matFileSuffix = matFileSuffix;
	this->m_storagePath = matFilePath + matFileName + matFileSuffix;

	//// stringת��
	//string temp = matFilePath;
	//for (int i = 0; i < temp.length(); i++)
	//{
	//	if (temp[i] == '/')
	//		temp[i] = '\\';
	//}

	//// ����Ŀ¼
	//system((string("mkdir ") + temp).c_str());
	
	return true;
}