#include "CCalculation.h"

using namespace cv;
using namespace std;

CVisualization myDebug("Debug");

CCalculation::CCalculation()
{
	this->m_sensor = NULL;
	this->m_decodeGrayv = NULL;
	this->m_decodePhasev = NULL;
	this->m_stripW = NULL;
	this->m_stripB = NULL;
	this->m_ProjectorU = NULL;
	this->m_xMat = NULL;
	this->m_yMat = NULL;
	this->m_zMat = NULL;
}

CCalculation::~CCalculation()
{
	this->ReleaseSpace();
}

bool CCalculation::ReleaseSpace()
{
	if (this->m_sensor != NULL)
	{
		delete(this->m_sensor);
		this->m_sensor = NULL;
	}
	if (this->m_decodeGrayv != NULL)
	{
		delete(this->m_decodeGrayv);
		this->m_decodeGrayv = NULL;
	}
	if (this->m_decodePhasev != NULL)
	{
		delete(this->m_decodePhasev);
		this->m_decodePhasev = NULL;
	}
	if (this->m_stripW != NULL)
	{
		delete(this->m_stripW);
		this->m_stripW = NULL;
	}
	if (this->m_stripB != NULL)
	{
		delete(this->m_stripB);
		this->m_stripB = NULL;
	}
	if (this->m_ProjectorU != NULL)
	{
		delete(this->m_ProjectorU);
		this->m_ProjectorU = NULL;
	}
	if (this->m_xMat != NULL)
	{
		delete(this->m_xMat);
		this->m_xMat = NULL;
	}
	if (this->m_yMat != NULL)
	{
		delete(this->m_yMat);
		this->m_yMat = NULL;
	}
	if (this->m_zMat != NULL)
	{
		delete(this->m_zMat);
		this->m_zMat = NULL;
	}

	return true;
}

bool CCalculation::Init()
{
	// ȷ�������Ϸ�
	if ((this->m_sensor != NULL))
		return false;
	if ((this->m_decodeGrayv != NULL) || (this->m_decodePhasev != NULL))
		return false;

	// �趨��ز���
	this->m_paraPath = "";
	this->m_paraName = "parameters";
	this->m_paraSuffix = ".yml";
	this->m_pcPath = "PointCloud\\";
	this->m_pcFisrtName = "iFrame";
	this->m_pcSucceedName = "cFrame";
	this->m_pcSuffix = ".txt";

	// ������Ԫ��
	this->m_sensor = new CSensor;
	this->m_sensor->InitSensor();
	this->m_decodeGrayv = new CDecodeGray;
	this->m_decodePhasev = new CDecodePhase;

	// ����ռ�:uxyz
	this->m_xMat = new Mat[DYNAFRAME_MAXNUM];
	this->m_yMat = new Mat[DYNAFRAME_MAXNUM];
	this->m_zMat = new Mat[DYNAFRAME_MAXNUM];
	this->m_stripW = new Mat[DYNAFRAME_MAXNUM];
	this->m_stripB = new Mat[DYNAFRAME_MAXNUM];
	this->m_ProjectorU = new Mat[DYNAFRAME_MAXNUM];
	this->m_deltaP = new Mat[DYNAFRAME_MAXNUM];

	for (int i = 0; i < DYNAFRAME_MAXNUM; i++)
	{
		this->m_stripW[i].create(CAMERA_RESROW, CAMERA_RESLINE, CV_32FC1);
		this->m_stripB[i].create(CAMERA_RESROW, CAMERA_RESLINE, CV_32FC1);
		this->m_ProjectorU[i].create(CAMERA_RESROW, CAMERA_RESLINE, CV_64FC1);
		this->m_deltaP[i].create(CAMERA_RESROW, CAMERA_RESLINE, CV_32FC1);

		this->m_xMat[i].create(CAMERA_RESROW, CAMERA_RESLINE, CV_64FC1);
		this->m_yMat[i].create(CAMERA_RESROW, CAMERA_RESLINE, CV_64FC1);
		this->m_zMat[i].create(CAMERA_RESROW, CAMERA_RESLINE, CV_64FC1);
	}
	
	// ����궨��ϵͳ����
	FileStorage fs(DATA_PATH 
		+ this->m_paraPath 
		+ this->m_paraName 
		+ this->m_paraSuffix, FileStorage::READ);
	fs["CamMat"] >> this->m_camMatrix;
	fs["ProMat"] >> this->m_proMatrix;
	fs["R"] >> this->m_R;
	fs["T"] >> this->m_T;
	fs.release();

	// ���C
	this->m_C.create(3, 4, CV_64FC1);
	this->m_C.setTo(0);
	this->m_camMatrix.copyTo(this->m_C.colRange(0, 3));
	//cout << this->m_C << endl;
	
	// ���P
	Mat temp;
	temp.create(3, 4, CV_64FC1);
	this->m_R.copyTo(temp.colRange(0, 3));
	this->m_T.copyTo(temp.colRange(3, 4));
	this->m_P = this->m_proMatrix * temp;
	/*cout << this->m_proMatrix << endl;
	cout << temp << endl;
	cout << this->m_P << endl;*/

	// ���ABCD����
	this->m_cA = this->m_C.at<double>(0, 0) * this->m_C.at<double>(1, 1) * this->m_P.at<double>(0, 3);
	this->m_cB = this->m_C.at<double>(0, 0) * this->m_C.at<double>(1, 1) * this->m_P.at<double>(2, 3);
	this->m_cC.create(CAMERA_RESROW, CAMERA_RESLINE, CV_64FC1);
	this->m_cD.create(CAMERA_RESROW, CAMERA_RESLINE, CV_64FC1);
	for (int u = 0; u < CAMERA_RESLINE; u++)
	{
		for (int v = 0; v < CAMERA_RESROW; v++)
		{
			this->m_cC.at<double>(v, u) = (u - this->m_C.at<double>(0, 2))*this->m_C.at<double>(1, 1) * this->m_P.at<double>(0, 0)
				+ (v - this->m_C.at<double>(1, 2))*this->m_C.at<double>(0, 0) * this->m_P.at<double>(0, 1)
				+ this->m_C.at<double>(0, 0) * this->m_C.at<double>(1, 1) * this->m_P.at<double>(0, 2);
			this->m_cD.at<double>(v, u) = (u - this->m_C.at<double>(0, 2))*this->m_C.at<double>(1, 1) * this->m_P.at<double>(2, 0)
				+ (v - this->m_C.at<double>(1, 2))*this->m_C.at<double>(0, 0) * this->m_P.at<double>(2, 1)
				+ this->m_C.at<double>(0, 0) * this->m_C.at<double>(1, 1) * this->m_P.at<double>(2, 2);
		}
	}

	return true;
}

bool CCalculation::CalculateFirst()
{
	bool status = true;

	// �жϲ����Ƿ�Ϸ�
	if ((this->m_sensor == NULL))
		return false;
	if ((this->m_decodeGrayv == NULL) || (this->m_decodePhasev == NULL))
		return false;
	if (this->m_C.empty() || this->m_P.empty())
		return false;

	cout << "Begin calculate first frame." << endl;

	// ������ֵ����ProjectorU
	status = this->FillFirstProjectorU();

	// ����ProjectorU�Ͳ�������ռ�����
	status = this->FillCoordinate(0);

	// �����������
	/*printf("Begin writing...");
	status = this->Result(DATA_PATH 
		+ this->m_pcPath 
		+ this->m_pcFisrtName 
		+ this->m_pcSuffix, 0);
	printf("Finished FirstFrame PointCloud.\n");*/

	// ���е�һ֡��׼��������ʶ��ǰ֡����Ϣ
	this->StripRegression(0);

	cout << "First frame finished." << endl;
	
	return true;
}

bool CCalculation::CalculateOther()
{
	bool status = true;

	// �жϲ����Ƿ�Ϸ�
	if ((this->m_sensor == NULL))
		return false;
	if ((this->m_decodeGrayv == NULL) || (this->m_decodePhasev == NULL))
		return false;
	if (this->m_C.empty() || this->m_P.empty())
		return false;

	// ��֡���м��㣺
	for (int frameNum = 1; frameNum < DYNAFRAME_MAXNUM; frameNum += 1)
	{
		string idx2str;
		stringstream ss;
		ss << frameNum;
		ss >> idx2str;

		cout << "Frame:" << frameNum << endl;

		// ʶ��ǰ֡������������
		this->StripRegression(frameNum);

		// ������������������deltaP����䣺
		this->FillOtherDeltaProU(frameNum);

		// ������ʱ��deltaP���л�ԭ��
		this->FillCoordinate(frameNum);

		myDebug.Show(this->m_zMat[frameNum], 100, true, 0.5, true, DATA_PATH + "DepthFrame" + idx2str + ".jpg");

		// ��������
		this->Result(DATA_PATH
			+ this->m_pcPath
			+ this->m_pcSucceedName
			+ idx2str
			+ this->m_pcSuffix, frameNum);
	}

	return true;
}

// �洢����¼���
bool CCalculation::Result(string fileName, int i)
{
	fstream file;
	file.open(fileName, ios::out);
	if (!file)
	{
		ErrorHandling("CCalculation::Result() OpenFile Error:" + fileName);
		return false;
	}
	for (int u = 0; u < CAMERA_RESLINE; u++)
	{
		for (int v = 0; v < CAMERA_RESROW; v++)
		{
			// ����zֵ����ɸѡ
			double valZ = this->m_zMat[i].at<double>(v, u);
			if ((valZ < FOV_MIN_DISTANCE) || (valZ > FOV_MAX_DISTANCE))
			{
				continue;
			}

			// ���
			file << this->m_xMat[i].at<double>(v, u) << ' ';
			file << this->m_yMat[i].at<double>(v, u) << ' ';
			file << this->m_zMat[i].at<double>(v, u) << endl;
			//printf("(%f, %f, %f)\n", this->m_xMat.at<double>(v, u), this->m_yMat.at<double>(v, u), this->m_zMat.at<double>(v, u));
		}
	}

	file.close();
	return true;
}


//bool CCalculation::RecoFirstStripPic()
//{
//	// ɸѡʶ������
//	int thredhold = 15;
//
//	//��ʼ����Ӧ��Mat
//	this->m_stripPic[0].setTo(0);
//
//	int startIdx = 0;
//	int dis = 5;
//	for (startIdx = 0; startIdx < 1280; startIdx += dis * 2)
//	{
//		//printf("For idx:%d\n", startIdx);
//
//		// ȡ����Ӧλ�ú�ɫ�Ĳ��֣�����ʶ��
//		this->m_tempMat.create(CAMERA_RESROW, CAMERA_RESLINE, CV_8UC1);
//		for (int h = 0; h < CAMERA_RESROW; h++)
//		{
//			for (int w = 0; w < CAMERA_RESLINE; w++)
//			{
//				double value = this->m_ProjectorU[0].at<double>(h, w);
//				if ((value >= startIdx) && (value <= startIdx + dis))
//				{
//					this->m_tempMat.at<uchar>(h, w) = 0;
//				}
//				else
//				{
//					this->m_tempMat.at<uchar>(h, w) = 1;
//				}
//			}
//		}
//
//		if (startIdx == 600)
//		{
//			//printf("Hi\n");
//			//this->m_tempMat.setTo(0);
//			//myDebug.Show(this->m_tempMat, 0, true);
//		}
//
//		// ����floodfill��ȥ����С������
//		for (int h = 0; h < CAMERA_RESROW; h++)
//		{
//			for (int w = 0; w < CAMERA_RESLINE; w++)
//			{
//				if (this->m_tempMat.at<uchar>(h, w) == 0)
//				{
//					uchar fillValue = 2;
//					int result = this->FloodFill(h, w, 0, 2);
//					if (result > thredhold)	// ���ڣ�����
//					{
//						fillValue = 3;
//					}
//					else
//					{
//						fillValue = 1;	// С�ڣ��򲻱���
//					}
//					this->FloodFill(h, w, 2, fillValue);
//				}
//			}
//		}
//		this->m_tempMat = this->m_tempMat - 2;
//
//		//myDebug.Show(this->m_tempMat, 0, true);
//
//		// ���������ʶ�����������
//		Mat tempMat16;
//		this->m_tempMat.convertTo(tempMat16, CV_16UC1);
//		this->m_stripPic[0] = this->m_stripPic[0] + tempMat16 * (startIdx + dis);
//		//myDebug.Show(this->m_stripPic[0], 20, true);
//	}
//
//	myDebug.Show(this->m_stripPic[0], 100, true);
//
//	return true;
//}
//
//bool CCalculation::RecoOtherStripPic(int fN)
//{
//	bool status = true;
//
//	// ��ʼ��stripPic
//	this->m_stripPic[fN].setTo(0);
//
//	for (int h = 0; h < CAMERA_RESROW; h++)
//	{
//		for (int w = 0; w < CAMERA_RESLINE; w++)
//		{
//			uchar bwVal = this->m_bwPic[fN].at<uchar>(h, w);
//			if (bwVal == 0)	// ��ɫ
//			{
//				// ͳ���ܱߵ���ֵ
//				ushort count[2][9];
//				for (int i = 0; i < 9; i++)
//				{
//					count[0][i] = count[1][i] = 0;
//				}
//				int existVal = 0;
//				for (int i = -1; i <= 1; i++)
//				{
//					for (int j = -1; j <= 1; j++)
//					{
//						if ((i == 0) && (j == 0))
//						{
//							continue;
//						}
//						if ((h + i >= 0) && (h + i < CAMERA_RESROW) && (w + j >= 0) && (w + j < CAMERA_RESLINE))
//						{
//							ushort val = this->m_stripPic[fN - 1].at<ushort>(h + i, w + j);
//							if (val == 0)	// ����ط�����
//							{
//								continue;
//							}
//							// ����ֵ�ӵ�ͳ��������
//							int p = 0;
//							for (p = 0; p < existVal; p++)
//							{
//								if (count[0][p] == val)
//								{
//									count[1][p]++;
//									break;
//								}
//							}
//							// ���û���ҵ������½�һ����
//							if (p == existVal)
//							{
//								count[0][existVal] = val;
//								count[1][existVal]++;
//								existVal++;
//							}
//						}
//					}
//				}
//
//				// �����
//				ushort maxTime = 0;
//				ushort maxVal = 0;
//				for (int p = 0; p < existVal; p++)
//				{
//					if (count[1][p] > maxTime)
//					{
//						maxTime = count[1][p];
//						maxVal = count[0][p];
//					}
//				}
//
//				// ��������
//				if (maxTime == 0)	// ��Χȫ�ǿհ�
//				{
//					this->m_stripPic[fN].at<ushort>(h, w) = 0;
//				}
//				else	// ��ȫ�ǿհ�
//				{
//					this->m_stripPic[fN].at<ushort>(h, w) = maxVal;
//				}
//			}
//		}
//	}
//
//	//myDebug.Show(this->m_stripPic[fN], 100, true);
//
//	return status;
//}

// ��ɵ�һ֡��vGray��vPhase���빤������ԭ��P_0
// ԭ���ͬ�ڱ궨�еĹ���
bool CCalculation::FillFirstProjectorU()
{
	bool status = true;

	// ������ʱ����
	Mat sensorMat;
	Mat vGrayMat;
	Mat vPhaseMat;
	Mat vProjectorMat;

	// Grayv����
	this->m_sensor->LoadDatas(0);
	this->m_decodeGrayv->SetNumDigit(GRAY_V_NUMDIGIT, true);
	this->m_decodeGrayv->SetMatFileName("Patterns/", "vGrayCode.txt");
	for (int i = 0; i < GRAY_V_NUMDIGIT * 2; i++)
	{
		this->m_sensor->SetProPicture(i);
		sensorMat = this->m_sensor->GetCamPicture();
		this->m_decodeGrayv->SetMat(i, sensorMat);
	}
	this->m_decodeGrayv->Decode();
	vGrayMat = this->m_decodeGrayv->GetResult();

	// Phasev����
	this->m_sensor->LoadDatas(1);
	int v_pixPeriod = PROJECTOR_RESLINE / (1 << GRAY_V_NUMDIGIT - 1);
	this->m_decodePhasev->SetNumMat(PHASE_NUMDIGIT, v_pixPeriod);
	for (int i = 0; i < PHASE_NUMDIGIT; i++)
	{
		this->m_sensor->SetProPicture(i);
		sensorMat = this->m_sensor->GetCamPicture();
		this->m_decodePhasev->SetMat(i, sensorMat);
	}
	this->m_decodePhasev->Decode();
	vPhaseMat = this->m_decodePhasev->GetResult();

	// �ϲ�
	int vGrayNum = 1 << GRAY_V_NUMDIGIT;
	int vGrayPeriod = PROJECTOR_RESLINE / vGrayNum;
	for (int h = 0; h < CAMERA_RESROW; h++)
	{
		for (int w = 0; w < CAMERA_RESLINE; w++)
		{
			double grayVal = vGrayMat.at<double>(h, w);
			double phaseVal = vPhaseMat.at<double>(h, w);
			if ((int)(grayVal / vGrayPeriod) % 2 == 0)
			{
				if (phaseVal >(double)v_pixPeriod * 0.75)
				{
					vPhaseMat.at<double>(h, w) = phaseVal - v_pixPeriod;
				}
			}
			else
			{
				if (phaseVal < (double)v_pixPeriod * 0.25)
				{
					vPhaseMat.at<double>(h, w) = phaseVal + v_pixPeriod;
				}
				vPhaseMat.at<double>(h, w) = vPhaseMat.at<double>(h, w) - 0.5 * v_pixPeriod;
			}
		}
	}
	vProjectorMat = vGrayMat + vPhaseMat;

	vProjectorMat.copyTo(this->m_ProjectorU[0]);

	return true;
}

// ����strip��Ϣ����ɺ�����deltaP�����
bool CCalculation::FillOtherDeltaProU(int fN)
{
	bool status = true;

	for (int h = 0; h < CAMERA_RESROW; h++)
	{
		for (int w = 0; w < CAMERA_RESLINE; w++)
		{
			float f0W = this->m_stripW[fN - 1].at<float>(h, w);
			float f0B = this->m_stripB[fN - 1].at<float>(h, w);
			float f1W = this->m_stripW[fN].at<float>(h, w);
			float f1B = this->m_stripB[fN].at<float>(h, w);

			bool f0Wleft = f0W < f0B;
			bool f1Wleft = f1W < f1B;

			// ����˳��δ��������ݶ��������ж�
			if (f0Wleft == f1Wleft)
			{
				this->m_deltaP[fN].at<float>(h, w) = ((f0W + f0B) - (f1W + f1B)) / 2;
			}
			else
			{
				if (f0Wleft && !f1Wleft)
				{
					this->m_deltaP[fN].at<float>(h, w) = f0B - f1B;
				}
				else
				{
					this->m_deltaP[fN].at<float>(h, w) = f0W - f1W;
				}
			}

			this->m_ProjectorU[fN].at<double>(h, w) =
				this->m_ProjectorU[fN - 1].at<double>(h, w)
				+ this->m_deltaP[fN].at<float>(h, w);
		}
	}

	return status;
}

// ����ϵͳ������Լ�������p��z�ļ���
bool CCalculation::FillCoordinate(int i)
{
	double min = 60;
	double max = 0;

	// ����ip����������z
	for (int u = 0; u < CAMERA_RESLINE; u++)
	{
		for (int v = 0; v < CAMERA_RESROW; v++)
		{
			double z = 0;
			// �����û��ֵ�Ĳ��֣�z����ʱ����
			if (this->m_ProjectorU[i].at<double>(v, u) == 0)
			{
				z = 0;
				continue;
			}
			// ��ֵ�Ĳ��֣��������
			else
			{
				z = -(this->m_cA - this->m_cB*this->m_ProjectorU[i].at<double>(v, u))
					/ (this->m_cC.at<double>(v, u) - this->m_cD.at<double>(v, u)*this->m_ProjectorU[i].at<double>(v, u));
			}

			// ���ӻ����֣��ռ������Сֵ
			if (z < min)
			{
				min = z;
			}
			if (z > max)
			{
				max = z;
			}

			// �ж�z�Ƿ��ںϷ���Χ��
			if ((z < FOV_MIN_DISTANCE) || (z > FOV_MAX_DISTANCE))
			{
				z = 0;
			}
			
			this->m_zMat[i].at<double>(v, u) = z;
		}
	}

	// ��z���в�ֵ��Ŀǰû�в�ֵ��
	//if (i > DYNAFRAME_MAXNUM + 1)
	//{
	//	for (int h = 0; h < CAMERA_RESROW; h++)
	//	{
	//		int from = -1;
	//		for (int w = 0; w < CAMERA_RESLINE; w++)
	//		{
	//			// ���ȸ��ݵ�ǰ��ֵ�������ж�
	//			double val = this->m_zMat[i].at<double>(h, w);
	//			// ��ֵ��С��0��from��ʱû�е�ʱ������
	//			if ((val <= 0) && (from < 0))
	//			{
	//				continue;
	//			}
	//			// ��ֵ�Ǵ���0��from��ʱû�е�ʱ�򣬼�¼�����׼����ֵ
	//			else if ((val > 0) && (from < 0))
	//			{
	//				from = w;
	//			}
	//			// ��ֵ��С��0��from�Ѿ��е�ʱ������Ѱ��֮���
	//			else if ((val <= 0) && (from >= 0))
	//			{
	//				continue;
	//			}
	//			// ��ֵ�Ǵ���0��from�Ѿ��е�ʱ�򣬶�֮���ֵ���в�ֵ
	//			else if ((val > 0) && (from >= 0))
	//			{
	//				double val1 = this->m_zMat[i].at<double>(h, from);
	//				double val2 = this->m_zMat[i].at<double>(h, w);
	//				for (int x = from + 1; x < w; x++)
	//				{
	//					double val3 = val1 + (val2 - val1) * (x - from) / (w - from);
	//					this->m_zMat[i].at<double>(h, x) = val3;
	//				}
	//				from = w;
	//			}
	//			else
	//			{
	//				ErrorHandling("CCalculation::Fillcoordinate(), Error on val & from.\n");
	//			}
	//		}
	//	}
	//}

	// ����z���x��y
	for (int u = 0; u < CAMERA_RESLINE; u++)
	{
		for (int v = 0; v < CAMERA_RESROW; v++)
		{
			// x = z * u_c/f_u
			double z = this->m_zMat[i].at<double>(v, u);
			double uc = u - this->m_C.at<double>(0, 2);
			double vc = v - this->m_C.at<double>(1, 2);
			double fu = this->m_C.at<double>(0, 0);
			double fv = this->m_C.at<double>(1, 1);
			this->m_xMat[i].at<double>(v, u) = z * uc / fu;
			this->m_yMat[i].at<double>(v, u) = z * vc / fv;
			
			//printf("(%f, %f, %f)\n", this->m_xMat.at<double>(v, u), this->m_yMat.at<double>(v, u), this->m_zMat.at<double>(v, u));
		}
	}
	
	Mat temp;
	Mat zTemp = (this->m_zMat[i] - min) / (max - min) * 255;
	zTemp.convertTo(temp, CV_16U);
	//imwrite("test.bmp", temp);

	//myDebug.Show(this->m_zMat[i], 100, true, 0.5);

	return true;
}

// ʶ��ͼ�е�������ֵ��
// �����ж�������֡�ı仯�����
bool CCalculation::StripRegression(int fN)
{
	this->m_sensor->LoadDatas(2);
	this->m_sensor->SetProPicture(fN);
	Mat tempMat, CamMat;
	tempMat = this->m_sensor->GetCamPicture();
	tempMat.copyTo(CamMat);

	// ���stripB��stripW
	this->m_stripB[fN].setTo(0);
	this->m_stripW[fN].setTo(0);
	for (int h = RECO_WINDOW_SIZE / 2; h < CAMERA_RESROW - RECO_WINDOW_SIZE / 2; h++)
	{
		for (int w = RECO_WINDOW_SIZE / 2; w < CAMERA_RESLINE - RECO_WINDOW_SIZE / 2; w++)
		{
			Mat color;
			color = CamMat.rowRange(Range(h - RECO_WINDOW_SIZE / 2, h + RECO_WINDOW_SIZE / 2 + 1)).colRange(Range(w - RECO_WINDOW_SIZE / 2, w + RECO_WINDOW_SIZE / 2 + 1));
			
			// ��ȡÿ�еĺ�
			vector<float> SumValue;
			for (int wc = 0; wc < RECO_WINDOW_SIZE; wc++)
			{
				float sum = 0;
				for (int hc = 0; hc < RECO_WINDOW_SIZE; hc++)
				{
					sum += (float)color.at<uchar>(hc, wc);
				}
				SumValue.push_back(sum);
			}

			// ��ȡ�����Сֵ
			float max = SumValue[0];
			float maxIdx = 0;
			float min = SumValue[0];
			float minIdx = 0;
			for (int i = 0; i < RECO_WINDOW_SIZE; i++)
			{
				if (SumValue[i] > max)
				{
					max = SumValue[i];
					maxIdx = i;
				}
				if (SumValue[i] < min)
				{
					min = SumValue[i];
					minIdx = i;
				}
			}

			// �洢��StripB��W��
			this->m_stripB[fN].at<float>(h, w) = minIdx - RECO_WINDOW_SIZE / 2;
			this->m_stripW[fN].at<float>(h, w) = maxIdx - RECO_WINDOW_SIZE / 2;
		}
	}

	return true;
}