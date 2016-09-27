#include "head.h"
#include <strstream>
#include <fstream>

using namespace cv;
using namespace std;

CVisualization myDebug("Debug");

CCalculation::CCalculation()
{
	this->m_sensor = NULL;
	this->m_decodeGrayv = NULL;
	this->m_decodePhasev = NULL;
	this->m_bwPic = NULL;
	this->m_stripPic = NULL;
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
	if (this->m_bwPic != NULL)
	{
		delete(this->m_bwPic);
		this->m_bwPic = NULL;
	}
	if (this->m_stripPic != NULL)
	{
		delete(this->m_stripPic);
		this->m_stripPic = NULL;
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
	// 确保参数合法
	if ((this->m_sensor != NULL))
		return false;
	if ((this->m_decodeGrayv != NULL) || (this->m_decodePhasev != NULL))
		return false;

	// 创建新元素
	this->m_sensor = new CSensor;
	this->m_decodeGrayv = new CDecode_Gray;
	this->m_decodePhasev = new CDecode_Phase;

	// 申请空间:uxyz
	this->m_xMat = new Mat[MAX_FRAME_NUM];
	this->m_yMat = new Mat[MAX_FRAME_NUM];
	this->m_zMat = new Mat[MAX_FRAME_NUM];
	this->m_bwPic = new Mat[MAX_FRAME_NUM];
	this->m_stripPic = new Mat[MAX_FRAME_NUM];
	this->m_ProjectorU = new Mat[MAX_FRAME_NUM];
	for (int i = 0; i < MAX_FRAME_NUM; i++)
	{
		this->m_bwPic[i].create(CAMERA_RESROW, CAMERA_RESLINE, CV_8UC1);
		this->m_stripPic[i].create(CAMERA_RESROW, CAMERA_RESLINE, CV_16UC1);
		this->m_ProjectorU[i].create(CAMERA_RESROW, CAMERA_RESLINE, CV_64FC1);
		this->m_xMat[i].create(CAMERA_RESROW, CAMERA_RESLINE, CV_64FC1);
		this->m_yMat[i].create(CAMERA_RESROW, CAMERA_RESLINE, CV_64FC1);
		this->m_zMat[i].create(CAMERA_RESROW, CAMERA_RESLINE, CV_64FC1);
	}
	
	// 导入标定参数
	FileStorage fs("Result.yml", FileStorage::READ);
	fs["CamMat"] >> this->m_camMatrix;
	fs["ProMat"] >> this->m_proMatrix;
	fs["R"] >> this->m_R;
	fs["T"] >> this->m_T;
	fs.release();

	// 填充C
	this->m_C.create(3, 4, CV_64FC1);
	this->m_C.setTo(0);
	this->m_camMatrix.copyTo(this->m_C.colRange(0, 3));
	//cout << this->m_C << endl;
	
	// 填充P
	Mat temp;
	temp.create(3, 4, CV_64FC1);
	this->m_R.copyTo(temp.colRange(0, 3));
	this->m_T.copyTo(temp.colRange(3, 4));
	this->m_P = this->m_proMatrix * temp;
	/*cout << this->m_proMatrix << endl;
	cout << temp << endl;
	cout << this->m_P << endl;*/

	// 填充ABCD参数
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

	// 判断参数是否合法
	if ((this->m_sensor == NULL))
		return false;
	if ((this->m_decodeGrayv == NULL) || (this->m_decodePhasev == NULL))
		return false;
	if (this->m_C.empty() || this->m_P.empty())
		return false;

	// 根据数值计算ProjectorU
	status = this->FillFirstProjectorU();

	// 根据ProjectorU和参数计算空间坐标
	status = this->FillCoordinate(0);

	// 输出、保存结果
	printf("Begin writing...");
	status = this->Result("PointCloud/FirstFramePC.txt", 0);
	printf("Finished FirstFrame PointCloud.\n");
	fstream file;
	file.open("rawZmat.txt", ios::out);
	for (int h = 0; h < CAMERA_RESROW; h++)
	{
		for (int w = 0; w < CAMERA_RESLINE; w++)
		{
			file << this->m_zMat[0].at<double>(h, w) << ' ';
		}
		file << endl;
	}
	file.close();

	// 识别第一帧的条纹
	this->Gray2BwPic(0);
	this->RecoFirstStripPic();

	return true;
}

bool CCalculation::CalculateOther()
{
	bool status = true;

	// 判断参数是否合法
	if ((this->m_sensor == NULL))
		return false;
	if ((this->m_decodeGrayv == NULL) || (this->m_decodePhasev == NULL))
		return false;
	if (this->m_C.empty() || this->m_P.empty())
		return false;

	// 逐帧计算。设定起始帧
	this->m_ProjectorU[0] = this->m_ProjectorU[0];

	// 逐帧推算，时间序列
	for (int frameNum = 1; frameNum < MAX_FRAME_NUM; frameNum += 1)
	{
		string strNum;
		stringstream ss;
		ss << frameNum;
		ss >> strNum;
		
		// 图像二值化
		if (status)
		{
			status = this->Gray2BwPic(frameNum);
			//imwrite("BwImg.bmp", this->m_bwPic[1]);
		}

		// 先识别出新图片的条纹
		if (status)
		{
			status = this->RecoOtherStripPic(frameNum);
			/*fstream file;
			file.open("StrImg.txt", ios::out);
			for (int h = 0; h < CAMERA_RESROW; h++)
			{
				for (int w = 0; w < CAMERA_RESLINE; w++)
				{
					file << this->m_stripPic[1].at<ushort>(h, w) << ' ';
				}
				file << endl;
			}
			file.close();*/
		}

		// 根据条纹，填充Projector的坐标
		if (status)
		{
			status = this->FillOtherProjectorU(frameNum);
			/*fstream file;
			file.open("ProU.txt", ios::out);
			for (int h = 0; h < CAMERA_RESROW; h++)
			{
				for (int w = 0; w < CAMERA_RESLINE; w++)
				{
					file << this->m_ProjectorU[1].at<double>(h, w) << ' ';
				}
				file << endl;
			}
			file.close();*/
		}

		/*for (int h = 0; h < CAMERA_RESROW; h++)
		{
			for (int w = 0; w < CAMERA_RESLINE; w++)
			{
				if ((this->m_ProjectorU[1].at<double>(h, w)>288) && (this->m_ProjectorU[1].at<double>(h, w) < 297))
				{
					continue;
				}
				else
				{
					this->m_ProjectorU[1].at<double>(h, w) = 0;
				}
			}
		}

		for (int h = 0; h < CAMERA_RESROW; h++)
		{
			for (int w = 0; w < CAMERA_RESLINE; w++)
			{
				if ((this->m_ProjectorU[0].at<double>(h, w)>=290) && (this->m_ProjectorU[0].at<double>(h, w) <= 295))
				{
					this->m_ProjectorU[0].at<double>(h, w) = 1;
				}
				else
				{
					this->m_ProjectorU[0].at<double>(h, w) = 0;
				}
			}
		}
		while (true)
		{
			myDebug.Show(this->m_ProjectorU[0], 00, true);
			myDebug.Show(this->m_ProjectorU[1], 00, true);
		}*/

		// 计算三维坐标
		if (status)
		{
			status = FillCoordinate(frameNum);
			/*fstream file;
			file.open("Zmat.txt", ios::out);
			for (int h = 0; h < CAMERA_RESROW; h++)
			{
				for (int w = 0; w < CAMERA_RESLINE; w++)
				{
					file << this->m_zMat[1].at<double>(h, w) << ' ';
				}
				file << endl;
			}
			file.close();*/
		}

		// 输出点云
		if (status)
		{
			printf("Begin writing...");
			status = this->Result("PointCloud/OtherFramesPC" + strNum + ".txt", frameNum);
			printf("Finished %d frame PointCloud.\n", frameNum);
		}

		//system("PAUSE");
	}

	return true;
}

bool CCalculation::Result(string fileName, int i)
{
	fstream file;
	file.open(fileName, ios::out);
	if (!file)
	{
		ErrorHandling("CCalculation::Result() OpenFile Error.\n");
		return false;
	}
	for (int u = 0; u < CAMERA_RESLINE; u++)
	{
		for (int v = 0; v < CAMERA_RESROW; v++)
		{
			// 根据z值进行筛选
			double valZ = this->m_zMat[i].at<double>(v, u);
			if ((valZ < 23.5) || (valZ > 31))
			{
				continue;
			}

			// 输出
			file << this->m_xMat[i].at<double>(v, u) << ' ';
			file << this->m_yMat[i].at<double>(v, u) << ' ';
			file << this->m_zMat[i].at<double>(v, u) << endl;
			//printf("(%f, %f, %f)\n", this->m_xMat.at<double>(v, u), this->m_yMat.at<double>(v, u), this->m_zMat.at<double>(v, u));
		}
	}

	file.close();
	return true;
}

bool CCalculation::RecoFirstStripPic()
{
	// 筛选识别条纹
	int thredhold = 15;

	//初始化相应的Mat
	this->m_stripPic[0].setTo(0);

	int startIdx = 0;
	int dis = 5;
	for (startIdx = 0; startIdx < 1280; startIdx += dis * 2)
	{
		//printf("For idx:%d\n", startIdx);

		// 取出对应位置黑色的部分，进行识别
		this->m_tempMat.create(CAMERA_RESROW, CAMERA_RESLINE, CV_8UC1);
		for (int h = 0; h < CAMERA_RESROW; h++)
		{
			for (int w = 0; w < CAMERA_RESLINE; w++)
			{
				double value = this->m_ProjectorU[0].at<double>(h, w);
				if ((value >= startIdx) && (value <= startIdx + dis))
				{
					this->m_tempMat.at<uchar>(h, w) = 0;
				}
				else
				{
					this->m_tempMat.at<uchar>(h, w) = 1;
				}
			}
		}

		if (startIdx == 600)
		{
			//printf("Hi\n");
			//this->m_tempMat.setTo(0);
			//myDebug.Show(this->m_tempMat, 0, true);
		}

		// 进行floodfill，去除过小的区域
		for (int h = 0; h < CAMERA_RESROW; h++)
		{
			for (int w = 0; w < CAMERA_RESLINE; w++)
			{
				if (this->m_tempMat.at<uchar>(h, w) == 0)
				{
					uchar fillValue = 2;
					int result = this->FloodFill(h, w, 0, 2);
					if (result > thredhold)	// 大于，则保留
					{
						fillValue = 3;
					}
					else
					{
						fillValue = 1;	// 小于，则不保留
					}
					this->FloodFill(h, w, 2, fillValue);
				}
			}
		}
		this->m_tempMat = this->m_tempMat - 2;

		//myDebug.Show(this->m_tempMat, 0, true);

		// 将结果加在识别出的条纹中
		Mat tempMat16;
		this->m_tempMat.convertTo(tempMat16, CV_16UC1);
		this->m_stripPic[0] = this->m_stripPic[0] + tempMat16 * (startIdx + dis);
		//myDebug.Show(this->m_stripPic[0], 20, true);
	}

	myDebug.Show(this->m_stripPic[0], 100, true);

	return true;
}

bool CCalculation::RecoOtherStripPic(int fN)
{
	bool status = true;

	// 初始化stripPic
	this->m_stripPic[fN].setTo(0);

	for (int h = 0; h < CAMERA_RESROW; h++)
	{
		for (int w = 0; w < CAMERA_RESLINE; w++)
		{
			uchar bwVal = this->m_bwPic[fN].at<uchar>(h, w);
			if (bwVal == 0)	// 黑色
			{
				// 统计周边的数值
				ushort count[2][9];
				for (int i = 0; i < 9; i++)
				{
					count[0][i] = count[1][i] = 0;
				}
				int existVal = 0;
				for (int i = -1; i <= 1; i++)
				{
					for (int j = -1; j <= 1; j++)
					{
						if ((i == 0) && (j == 0))
						{
							continue;
						}
						if ((h + i >= 0) && (h + i < CAMERA_RESROW) && (w + j >= 0) && (w + j < CAMERA_RESLINE))
						{
							ushort val = this->m_stripPic[fN - 1].at<ushort>(h + i, w + j);
							if (val == 0)	// 空余地方不管
							{
								continue;
							}
							// 将数值加到统计数组中
							int p = 0;
							for (p = 0; p < existVal; p++)
							{
								if (count[0][p] == val)
								{
									count[1][p]++;
									break;
								}
							}
							// 如果没有找到，就新建一个数
							if (p == existVal)
							{
								count[0][existVal] = val;
								count[1][existVal]++;
								existVal++;
							}
						}
					}
				}

				// 找最大
				ushort maxTime = 0;
				ushort maxVal = 0;
				for (int p = 0; p < existVal; p++)
				{
					if (count[1][p] > maxTime)
					{
						maxTime = count[1][p];
						maxVal = count[0][p];
					}
				}

				// 分类讨论
				if (maxTime == 0)	// 周围全是空白
				{
					this->m_stripPic[fN].at<ushort>(h, w) = 0;
				}
				else	// 不全是空白
				{
					this->m_stripPic[fN].at<ushort>(h, w) = maxVal;
				}
			}
		}
	}

	//myDebug.Show(this->m_stripPic[fN], 100, true);

	return status;
}

bool CCalculation::FillFirstProjectorU()
{
	bool status = true;

	// 设置传感器传入对应帧的信息
	//this->m_sensor->SetChessFrame(frameIdx);
	Mat sensorMat;

	// Grayv解码
	Mat vGrayMat;
	this->m_decodeGrayv->SetNumDigit(GRAY_V_NUMDIGIT, true);
	this->m_decodeGrayv->SetMatFileName("Pattern/", "GrayCode.txt");
	sensorMat = this->m_sensor->GetCamFrame();
	this->m_decodeGrayv->SetCamMat(sensorMat);
	for (int i = 0; i < GRAY_V_NUMDIGIT * 2; i++)		// Projector
	{
		sensorMat = this->m_sensor->GetProFrame(0, i);//Gray_v的图片编号为0
		this->m_decodeGrayv->SetMat(i, sensorMat);
	}
	this->m_decodeGrayv->Decode();
	vGrayMat = this->m_decodeGrayv->GetResult();

	// Phasev解码
	Mat vPhaseMat;
	int v_pixPeriod = PROJECTOR_RESLINE / (1 << GRAY_V_NUMDIGIT);
	this->m_decodePhasev->SetNumMat(PHASE_NUMDIGIT, v_pixPeriod);
	sensorMat = this->m_sensor->GetCamFrame();
	for (int i = 0; i < PHASE_NUMDIGIT; i++)
	{
		sensorMat = this->m_sensor->GetProFrame(1, i);//Phase_v的图像编号为1
		this->m_decodePhasev->SetMat(i, sensorMat);
	}
	this->m_decodePhasev->Decode();
	vPhaseMat = this->m_decodePhasev->GetResult();

	// 合并
	Mat vGrayMatDouble;
	vGrayMat.convertTo(vGrayMatDouble, vPhaseMat.type());
	this->m_ProjectorU[0] = vGrayMatDouble + vPhaseMat;

	// 平滑滤波
	/*Mat temp = this->m_ProjectorU;
	blur(temp, this->m_ProjectorU, Size(3, 3), Point(-1, -1));*/

	//myDebug.Show(this->m_ProjectorU[0], 100, true);

	//imwrite("Coordinate.bmp", this->m_ProjectorU[0]);

	return true;
}

bool CCalculation::FillOtherProjectorU(int fN)
{
	bool status = true;

	// 初始化相应的值
	this->m_ProjectorU[fN].setTo(0);
	Mat Labels;
	Labels.create(256, CAMERA_RESROW, CV_16SC1);
	Labels.setTo(-1);
	
	// 填充Labels
	for (int h = 0; h < CAMERA_RESROW; h++)
	{
		for (int w = 0; w < CAMERA_RESLINE; w++)
		{
			ushort val = this->m_stripPic[fN].at<ushort>(h, w);
			if (val == 0)
			{
				continue;
			}
			else if (Labels.at<short>(val / 5 - 1, h) < 0)
			{
				Labels.at<short>(val / 5 - 1, h) = w;
				Labels.at<short>(val / 5, h) = w;
			}
			else
			{
				Labels.at<short>(val / 5, h) = w;
			}
		}
	}

	// 分段线性拟合条纹，给对应位置赋值
	for (int s = 0; s < 256; s++)
	{
		int from = 0;
		while ((from < CAMERA_RESROW) && (Labels.at<short>(s, from) < 0))
		{
			from++;
		}
		int end = CAMERA_RESROW - 1;
		while ((end >= 0) && (Labels.at<short>(s, end) < 0))
		{
			end--;
		}
		// 未出现的条纹，跳过
		if (from > end)
		{
			continue;
		}

		// 如果条纹中某些位置无值，或距离上、下位置过远，则进行填充
		for (int h = from + 1; h <= end; h++)
		{
			if (Labels.at<short>(s, h) < 0)
			{
				Labels.at<short>(s, h) = Labels.at<short>(s, h - 1);
			}
		}

		for (int h = from + 1; h < end; h++)
		{
			short valH = Labels.at<short>(s, h);
			short valHup = Labels.at<short>(s, h - 1);
			short valHdown = Labels.at<short>(s, h + 1);
			if ((abs(valH - valHup) > 1) && (abs(valH - valHdown) > 1))
			{
				Labels.at<short>(s, h) = (valHup + valHdown) / 2;
			}
		}
		
		// 对于已出现的条纹，间隔为5的进行线性拟合并填充ProU
		int dis = 5;
		for (int hIdx = from; hIdx <= end - dis; hIdx += dis)
		{
			// 起始点和终止点
			double h1 = hIdx;
			double h2 = hIdx + dis;
			double w1 = Labels.at<short>(s, hIdx);
			double w2 = Labels.at<short>(s, hIdx + dis);

			// 根据线性公式，计算结果，并填充之间的格点
			for (int h = h1; h < h2; h++)
			{
				double w = (w1 - w2) / (h1 - h2) * (h - h1) + w1;
				double i_w = floor(w);
				this->m_ProjectorU[fN].at<double>(h, i_w) = (double)s * 5 - (w - i_w);
			}
		}

	}
	

	//for (int h = 0; h < CAMERA_RESROW; h++)
	//{
	//	// 寻找起始点位置和终止点位置
	//	int FromTo[256];
	//	for (int i = 0; i < 256; i++)
	//	{
	//		FromTo[i] = 0;
	//	}
	//	for (int w = 0; w < CAMERA_RESLINE; w++)
	//	{
	//		ushort val = this->m_stripPic[fN].at<ushort>(h, w);
	//		if (val == 0)
	//		{
	//			continue;
	//		}
	//		else if (FromTo[val/5 - 1] == 0)
	//		{
	//			FromTo[val / 5 - 1] = w;
	//			FromTo[val / 5] = w;
	//		}
	//		else
	//		{
	//			FromTo[val / 5] = w;
	//		}
	//	}

	//	// 填充相应的位置
	//	for (int i = 0; i < 256; i += 2)
	//	{
	//		this->m_ProjectorU[fN].at<double>(h, FromTo[i]) = (double)i * 5 + 1;
	//		this->m_ProjectorU[fN].at<double>(h, FromTo[i + 1]) = (double)(i + 1) * 5;
	//	}

	//	// 插值
	//	//int from = -1;
	//	//for (int w = 0; w < CAMERA_RESLINE; w++)
	//	//{
	//	//	double val = this->m_ProjectorU[fN].at<double>(h, w);
	//	//	if (val == 0)
	//	//	{
	//	//		continue;
	//	//	}
	//	//	else
	//	//	{
	//	//		if (from < 0)
	//	//		{
	//	//			from = w;
	//	//		}
	//	//		else
	//	//		{
	//	//			if (from < w - 1)	// 相距比较远，需要插值
	//	//			{
	//	//				double val1 = this->m_ProjectorU[fN].at<double>(h, from);
	//	//				double val2 = this->m_ProjectorU[fN].at<double>(h, w);
	//	//				for (int i = from + 1; i < w; i++)
	//	//				{
	//	//					double val3 = val1 + (val2 - val1) * (i - from) / (w - from);
	//	//					this->m_ProjectorU[fN].at<double>(h, i) = val3;
	//	//				}
	//	//			}
	//	//			from = w;
	//	//		}
	//	//	}
	//	//}
	//}
	
	myDebug.Show(this->m_ProjectorU[fN], 100, true);
	
	return true;
}

bool CCalculation::Gray2BwPic(int fN)
{
	bool status = true;

	// 首先获取灰度图
	Mat GreyMat = this->m_sensor->GetProFrame(2, fN);

	// 进行二值化：与均值滤波比较
	Mat AverageMat, bwMat;
	blur(GreyMat, AverageMat, Size(5, 5));
	bwMat.create(AverageMat.size(), CV_8UC1);
	for (int h = 0; h < CAMERA_RESROW; h++)
	{
		for (int w = 0; w < CAMERA_RESLINE; w++)
		{
			if (GreyMat.at<uchar>(h, w) < AverageMat.at<uchar>(h, w))
			{
				bwMat.at<uchar>(h, w) = 0;
			}
			else
			{
				bwMat.at<uchar>(h, w) = 1;
			}
		}
	}

	// 保存二值图
	this->m_bwPic[fN] = bwMat;
	stringstream ss;
	ss << fN;
	string strNum;
	ss >> strNum;

	// Debug输出部分
	/*imwrite("DebugPic/BwPic/BwImg" + strNum + ".bmp", bwMat*255);
	imshow("Camera", bwMat*255);
	waitKey(1000);*/

	return true;
}

int CCalculation::FloodFill(int h, int w, uchar from, uchar to)
{
	int result;
	this->m_tempMat.at<uchar>(h, w) = to;
	result = 1;

	for (int i = -1; i <= 1; i++)
	{
		for (int j = -1; j <= 1; j++)
		{
			if ((i == 0) && (j == 0))
			{
				continue;
			}
			if ((h + i >= 0) && (h + i < CAMERA_RESROW) && (w + j >= 0) && (w + j < CAMERA_RESLINE))
			{
				if (this->m_tempMat.at<uchar>(h + i, w + j) == from)
				{
					result = result + FloodFill(h + i, w + j, from, to);
				}
			}
		}
	}

	return result;
}

bool CCalculation::FillCoordinate(int i)
{
	double min = 60;
	double max = 0;

	// 根据ip坐标求得深度z
	for (int u = 0; u < CAMERA_RESLINE; u++)
	{
		for (int v = 0; v < CAMERA_RESROW; v++)
		{
			double z = 0;
			// 如果是没有值的部分，z就暂时不填
			if (this->m_ProjectorU[i].at<double>(v, u) == 0)
			{
				z = 0;
				continue;
			}
			// 有值的部分，计算深度
			else
			{
				z = -(this->m_cA - this->m_cB*this->m_ProjectorU[i].at<double>(v, u))
					/ (this->m_cC.at<double>(v, u) - this->m_cD.at<double>(v, u)*this->m_ProjectorU[i].at<double>(v, u));
			}
			
			if (z < min)
			{
				min = z;
			}
			if (z > max)
			{
				max = z;
			}

			// 如果不在范围内，则把z置为0
			if ((z < 10) || (z > 40))
			{
				z = 0;
			}
			
			this->m_zMat[i].at<double>(v, u) = z;
		}
	}

	// 对z进行插值
	if (i > 101)
	{
		for (int h = 0; h < CAMERA_RESROW; h++)
		{
			int from = -1;
			for (int w = 0; w < CAMERA_RESLINE; w++)
			{
				// 首先根据当前的值，进行判断
				double val = this->m_zMat[i].at<double>(h, w);
				// 当值是小于0且from暂时没有的时候，跳过
				if ((val <= 0) && (from < 0))
				{
					continue;
				}
				// 当值是大于0且from暂时没有的时候，记录下起点准备插值
				else if ((val > 0) && (from < 0))
				{
					from = w;
				}
				// 当值是小于0且from已经有的时候，跳过寻找之后的
				else if ((val <= 0) && (from >= 0))
				{
					continue;
				}
				// 当值是大于0且from已经有的时候，对之间的值进行插值
				else if ((val > 0) && (from >= 0))
				{
					double val1 = this->m_zMat[i].at<double>(h, from);
					double val2 = this->m_zMat[i].at<double>(h, w);
					for (int x = from + 1; x < w; x++)
					{
						double val3 = val1 + (val2 - val1) * (x - from) / (w - from);
						this->m_zMat[i].at<double>(h, x) = val3;
					}
					from = w;
				}
				else
				{
					ErrorHandling("CCalculation::Fillcoordinate(), Error on val & from.\n");
				}
			}
		}
	}

	// 根据z求得x和y
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

	myDebug.Show(this->m_zMat[i], 20, true);

	return true;
}