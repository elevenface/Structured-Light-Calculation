// OpenCV
#include <opencv2/opencv.hpp>

// �������ÿ�
#include <vector>
#include <direct.h>

#define VISUAL_DEBUG false
#define FRAME_NUMBER 1

// һЩ����
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
static int SHOW_PICTURE_TIME = 500;		// Ĭ��VisualͼƬ�ĳ���ʱ��

// ��������
int ErrorHandling(std::string message);

// ������ģ�顣ģ��
class CSensor
{
private:
	int m_ProPicNum[4];				// ������Pattern���ܼ�ͼ����

	std::string m_chessFilePath;	// �洢�ļ�����·��
	std::string m_chessFileFrameNum;	// ��ǰ�궨��֡��
	std::string m_chessFileCamPath;	// ��������궨���ļ�·��
	std::string m_chessFileCamName;	// ��������궨���ļ���
	std::string m_chessFileProPath[3];	// ����ͶӰ�Ǳ궨���ļ�·��
	std::string m_chessFileProName[3];	// ����ͶӰ�Ǳ궨���ļ���
	std::string m_chessFileSuffix;	// �ļ���׺��

public:
	CSensor();
	~CSensor();
	//bool SetChessFrame(int frame);	// �趨��ǰ�궨��֡��	
	cv::Mat GetCamFrame();			// ��ȡ���ͼ��
	int GetPicNum(int patternIdx);	// ��ȡͶӰ��ͼ��������
	cv::Mat GetProFrame(int patternIdx, int picIdx);	// ��ȡͶӰ��ͼ��
};

// ���ݴ洢ģ�顣�洢�м��������浵
class CStorage
{
private:
	std::string m_matFilePath;
	std::string m_matFileName;
	std::string m_matFileSuffix;
	std::string m_storagePath;		// ���ڴ洢������·����debug�á�
public:
	CStorage();
	~CStorage();
	bool Store(cv::Mat *pictures, int num);		// �洢ͼƬ��
	
	bool SetMatFileName(std::string matFilePath,	// �趨�洢·��������
		std::string matFileName,
		std::string matFileSuffix);
};

// ���ӻ�ģ�飬����debug���Զ��������ٴ��ڡ�
class CVisualization
{
private:
	std::string m_winName;		// ��������
public:
	CVisualization(std::string winName);
	~CVisualization();
	int Show(cv::Mat pic, int time, bool norm = false);
};

// ������������������Ѿ���д�õĸ����롣
// ����Ϊһ��Ҷ�ͼ�����Ϊһ�ŻҶ�ͼ��ÿһ��洢����projector�е����ꡣ
// ʹ��ǰ��Ҫ�ֱ����4��Set�������Ρ�
class CDecode_Gray
{
private:
	int m_numDigit;			// λ��
	int m_grayCodeSize;		// �ܹ��ĸ�������Ŀ
	short * m_gray2bin;		// �����뵽���������ת��
	std::string m_codeFilePath;	// �洢��������ļ�·��
	std::string m_codeFileName;	// �洢��������ļ���
	int resRow;				// ͼ����зֱ���
	int resLine;			// ͼ����зֱ���
	bool m_vertical;		// �趨�����뷽��

	cv::Mat m_camPicture;	// �����ԭͼ
	cv::Mat * m_grePicture;	// ����ĻҶ�ͼ
	cv::Mat * m_binPicture;	// �ӹ���Ķ�ֵͼ
	cv::Mat m_result;		// ���

	CVisualization * m_visual;	// ������ʾ�м���

	bool AllocateSpace();		// Ϊ����ľ���short������������ռ�
	bool DeleteSpace();			// ɾ�����пռ�
	bool Grey2Bin();			// ���Ҷ�ͼ�ӹ�Ϊ��ֵͼ���Ա��һ������
	bool CountResult();			// ���ݶ�ֵͼͳ�ƽ��
	bool Visualize();			// ��ʾ�м���

public:
	CDecode_Gray();
	~CDecode_Gray();

	bool Decode();
	cv::Mat GetResult();

	bool SetCamMat(cv::Mat pic);							// ������Ӧ�����ͼƬ
	bool SetMat(int num, cv::Mat pic);						// ������Ӧ�Ҷ�ͼ
	bool SetNumDigit(int numDigit, bool ver);				// ���ø�����λ��
	bool SetMatFileName(std::string codeFilePath,
		std::string codeFileName);			// ���ô洢��������ļ���
};

// ���ƽ������������Ѿ���д�õ������롣
// ����Ϊһ��Ҷ�ͼ�����Ϊһ�ŻҶ�ͼ��ÿһ��洢����projector�е�ƫ�����ꡣ
class CDecode_Phase
{
private:
	int m_numMat;		// ͼƬ��Ŀ��Ĭ��Ϊ4��
	int m_pixPeroid;	// �������ڡ�
	int m_resRow;				// ͼ����зֱ���
	int m_resLine;			// ͼ����зֱ���

	cv::Mat * m_grePicture;	// ����ĻҶ�ͼ
	cv::Mat m_result;		// ���

	CVisualization * m_visual;	// ������ʾ�м���

	bool AllocateSpace();		// Ϊ����ľ�������ռ�
	bool DeleteSpace();			// ɾ�����пռ�
	bool CountResult();			// ���ݹ�һ���Ҷ�ͼͳ�ƽ��
	bool Visualize();			// ��ʾ�м���

public:
	CDecode_Phase();
	~CDecode_Phase();

	bool Decode();
	cv::Mat GetResult();

	bool SetMat(int num, cv::Mat pic);						// ������Ӧ�Ҷ�ͼ
	bool SetNumMat(int numDigit, int pixperiod);			// ���ò���
};

// �����ࡣ���ڼ�����ơ�
class CCalculation
{
private:
	// ������������
	CSensor * m_sensor;
	// ������
	CDecode_Gray * m_decodeGrayv;	// ���������
	//CDecode_Gray * m_decodeGrayh;
	CDecode_Phase * m_decodePhasev;	// PS����
	//CDecode_Phase * m_decodePhaseh;
	// ������Ŀռ�����
	cv::Mat * m_bwPic;				// ת���ɵĶ�ֵͼ
	cv::Mat * m_stripPic;			// ʶ���������
	cv::Mat * m_ProjectorU;			// ���ɵľ�������
	cv::Mat * m_xMat;
	cv::Mat * m_yMat;
	cv::Mat * m_zMat;
	// �궨�ľ���
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
	// ����FloodFill��ȫ����ʱ����
	cv::Mat m_tempMat;

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