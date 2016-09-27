#include "head.h"

using namespace cv;
using namespace std;

int ErrorHandling(std::string message)
{
	cout << "An Error Occurs:" << message << endl;
	system("PAUSE");
	return 0;
}

int main()
{
	/*Mat A = imread("DynaMat.jpg");
	cvtColor(A, A, CV_BGR2GRAY);
	vector<vector<Point>> contours;
	findContours(A, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	Mat B;
	Canny(A, B, 2, 4);

	namedWindow("Res");
	imshow("Res", B);
	waitKey(0);
	destroyWindow("Res");
	return 0;*/

	/*float valA[] = { 1, 1, -1, -1, -1, -1, 1, 1, -1,
		1, -1, -1, -1, 1, 1, 1, 1, -1,
		-1, -1, -1, -1, 1, 1, 1, 1, -1,
		-1, -1, 1, 1, 1, 1, 1, 1, -1,
		-1, -1, 1, 1, 1, 1, 1, 1, -1,
		1, 1, 1, -1, -1, -1, 1, 1, -1,
		1, 1, 1, -1, -1, -1, 1, 1, -1,
		1, 1, -1, -1, -1, 1, 1, 1, -1,
		-1, -1, -1, -1, 1, 1, 1, 1, -1};
	Mat A(9, 9, CV_32FC1, valA);
	Mat B;
	Mat C;
	flip(Mat::eye(9, 9, CV_32FC1), C, 1);
	filter2D(A, B, -1, C);

	cout << A << endl;
	cout << B << endl;
	cout << C << endl;
	system("PAUSE");*/

	// 创建类：
	CCalculation myCalculation;		// 标定类
	myCalculation.Init();
	myCalculation.CalculateFirst();
	myCalculation.CalculateOther();

	/*CCalibration calibration;
	calibration.Init();
	calibration.Calibrate();*/
	/*CCamera Camera;
	bool Running = true;
	if (Running)
	{
		printf("Begin Initalizing Camera...\n");
		Running = Camera.InitCamera();
	}
	if (Running)
	{
		printf("Begin Capture Camera Shot...\n");
		Mat M;
		while (true)
		{
			Running = Camera.getPicture(M);
			if (Running)
			{
				namedWindow("CameraImage");
				imshow("CameraImage", M);
				waitKey(10);
			}
		}
	}
	if (Running)
	{
		printf("Begin Closing Camera...\n");
		Running = Camera.CloseCamera();
	}*/

	system("PAUSE");
	return 0;
}