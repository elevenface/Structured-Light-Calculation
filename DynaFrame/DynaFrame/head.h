// OpenCV
#include <opencv2/opencv.hpp>

// 其他常用库
#include <vector>
#include <direct.h>

#define VISUAL_DEBUG false
#define FRAME_NUMBER 1

// 一些常量
static int PROJECTOR_RESLINE = 1280;
static int PROJECTOR_RESROW = 800;
static int CAMERA_RESLINE = 640;	// 1280->640
static int CAMERA_RESROW = 512;	// 1024->512
static int PC_BIASLINE = 1366;
static int PC_BIASROW = 0;
static int GRAY_V_NUMDIGIT = 6;
//static int GRAY_H_NUMDIGIT = 5;
static int PHASE_NUMDIGIT = 4;
static int MAX_FRAME_NUM = 100;
static int SHOW_PICTURE_TIME = 500;		// 默认Visual图片的持续时间

// 错误处理函数
int ErrorHandling(std::string message);

// 传感器模块。模拟
class CSensor
{
private:
	int m_ProPicNum[4];				// 各部分Pattern的总计图案数

	std::string m_chessFilePath;	// 存储文件的总路径
	std::string m_chessFileFrameNum;	// 当前标定的帧数
	std::string m_chessFileCamPath;	// 用于相机标定的文件路径
	std::string m_chessFileCamName;	// 用于相机标定的文件名
	std::string m_chessFileProPath[3];	// 用于投影仪标定的文件路径
	std::string m_chessFileProName[3];	// 用于投影仪标定的文件名
	std::string m_chessFileSuffix;	// 文件后缀名

public:
	CSensor();
	~CSensor();
	//bool SetChessFrame(int frame);	// 设定当前标定的帧数	
	cv::Mat GetCamFrame();			// 获取相机图像
	int GetPicNum(int patternIdx);	// 获取投影仪图案的总数
	cv::Mat GetProFrame(int patternIdx, int picIdx);	// 获取投影仪图像
};

// 数据存储模块。存储中间数据做存档
class CStorage
{
private:
	std::string m_matFilePath;
	std::string m_matFileName;
	std::string m_matFileSuffix;
	std::string m_storagePath;		// 用于存储的最终路径。debug用。
public:
	CStorage();
	~CStorage();
	bool Store(cv::Mat *pictures, int num);		// 存储图片。
	
	bool SetMatFileName(std::string matFilePath,	// 设定存储路径并创建
		std::string matFileName,
		std::string matFileSuffix);
};

// 可视化模块，用于debug。自动创建销毁窗口。
class CVisualization
{
private:
	std::string m_winName;		// 窗口名称
public:
	CVisualization(std::string winName);
	~CVisualization();
	int Show(cv::Mat pic, int time, bool norm = false);
};

// 格雷码解码器。解码已经编写好的格雷码。
// 输入为一组灰度图，输出为一张灰度图，每一点存储的是projector中的坐标。
// 使用前需要分别调用4个Set函数传参。
class CDecode_Gray
{
private:
	int m_numDigit;			// 位数
	int m_grayCodeSize;		// 总共的格雷码数目
	short * m_gray2bin;		// 格雷码到二进制码的转换
	std::string m_codeFilePath;	// 存储格雷码的文件路径
	std::string m_codeFileName;	// 存储格雷码的文件名
	int resRow;				// 图像的行分辨率
	int resLine;			// 图像的列分辨率
	bool m_vertical;		// 设定格雷码方向

	cv::Mat m_camPicture;	// 摄像机原图
	cv::Mat * m_grePicture;	// 输入的灰度图
	cv::Mat * m_binPicture;	// 加工后的二值图
	cv::Mat m_result;		// 结果

	CVisualization * m_visual;	// 用于显示中间结果

	bool AllocateSpace();		// 为输入的矩阵、short类型内容申请空间
	bool DeleteSpace();			// 删除所有空间
	bool Grey2Bin();			// 将灰度图加工为二值图，以便进一步处理
	bool CountResult();			// 根据二值图统计结果
	bool Visualize();			// 显示中间结果

public:
	CDecode_Gray();
	~CDecode_Gray();

	bool Decode();
	cv::Mat GetResult();

	bool SetCamMat(cv::Mat pic);							// 输入相应的相机图片
	bool SetMat(int num, cv::Mat pic);						// 输入相应灰度图
	bool SetNumDigit(int numDigit, bool ver);				// 设置格雷码位数
	bool SetMatFileName(std::string codeFilePath,
		std::string codeFileName);			// 设置存储格雷码的文件名
};

// 相移解码器。解码已经编写好的相移码。
// 输入为一组灰度图，输出为一张灰度图，每一点存储的是projector中的偏移坐标。
class CDecode_Phase
{
private:
	int m_numMat;		// 图片数目。默认为4。
	int m_pixPeroid;	// 像素周期。
	int m_resRow;				// 图像的行分辨率
	int m_resLine;			// 图像的列分辨率

	cv::Mat * m_grePicture;	// 输入的灰度图
	cv::Mat m_result;		// 结果

	CVisualization * m_visual;	// 用于显示中间结果

	bool AllocateSpace();		// 为输入的矩阵申请空间
	bool DeleteSpace();			// 删除所有空间
	bool CountResult();			// 根据归一化灰度图统计结果
	bool Visualize();			// 显示中间结果

public:
	CDecode_Phase();
	~CDecode_Phase();

	bool Decode();
	cv::Mat GetResult();

	bool SetMat(int num, cv::Mat pic);						// 输入相应灰度图
	bool SetNumMat(int numDigit, int pixperiod);			// 设置参数
};

// 计算类。用于计算点云。
class CCalculation
{
private:
	// 传感器控制类
	CSensor * m_sensor;
	// 解码器
	CDecode_Gray * m_decodeGrayv;	// 格雷码解码
	//CDecode_Gray * m_decodeGrayh;
	CDecode_Phase * m_decodePhasev;	// PS解码
	//CDecode_Phase * m_decodePhaseh;
	// 计算出的空间坐标
	cv::Mat * m_bwPic;				// 转化成的二值图
	cv::Mat * m_stripPic;			// 识别出的条码
	cv::Mat * m_ProjectorU;			// 生成的具体坐标
	cv::Mat * m_xMat;
	cv::Mat * m_yMat;
	cv::Mat * m_zMat;
	// 标定的矩阵
	cv::Mat m_camMatrix;
	cv::Mat m_proMatrix;
	cv::Mat m_R;
	cv::Mat m_T;
	cv::Mat m_C;
	cv::Mat m_P;
	double m_cA;
	double m_cB;
	cv::Mat m_cC;
	cv::Mat m_cD;
	// 用于FloodFill的全局临时矩阵
	cv::Mat m_tempMat;

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