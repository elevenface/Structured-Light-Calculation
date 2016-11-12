#include "depthMapUtils.h"
#include <opencv2\imgproc\imgproc.hpp>
#include <fstream>
#include <iomanip>
void convertDepthImgtoPointcloud(const cv::Mat& depth, std::vector<Eigen::Vector3f>& cloud, std::vector<bool>& validState,const cv::Matx33f&  K)
{
	const double fx = K(0, 0);
	const double fy = K(1, 1);
	const double cx = K(0, 2);
	const double cy = K(1, 2);

	cloud.resize(depth.cols*depth.rows);
	validState.resize(cloud.size());
	////#pragma omp parallel for

	

	for (int i = 0; i < depth.rows; i++)
	{
		//std::cout << sizeof(ushort) << std::endl;
		const ushort* depth_prt = reinterpret_cast<const ushort*>(depth.ptr(i));
		//////#pragma omp parallel for
		for (int j = 0; j < depth.cols; j++)
		{
			int cloudID = (depth.cols*i) + j;
			if (depth_prt[j] == 0)
			{
				validState[cloudID] = false;
			}
			else
			{
				cloud[cloudID].z() = -(double)depth_prt[j]/1000;
				cloud[cloudID].x() = -(j - cx) * cloud[cloudID].z() / fx;
				cloud[cloudID].y() = (i - cy) * cloud[cloudID].z() / fy;
				validState[cloudID] = true;
			}
		}
	}
}



int savePointCloud(std::string fileName, const std::vector<Eigen::Vector3f>& cloud, std::vector<Eigen::Vector3f>& normal, const std::vector<bool>& validState)
{

	bool need_validate = true;
	if (validState.empty())
		need_validate = false;


	std::ofstream file(fileName);
	for (int i = 0; i < cloud.size(); i++)
	{
		if (need_validate&&validState[i] == false)
			continue;
		file << cloud[i](0) << " " << cloud[i](1) << " " << cloud[i](2) << " " << normal[i](0) << " " << normal[i](1) << " " << normal[i](2) << std::endl;
	}
	file.close();
	return cloud.size();
}

int savePointCloud(std::string fileName,const std::vector<Eigen::Vector3f>& cloud, const std::vector<bool>& validState, cv::Mat& color)
{

	cv::Mat rgbImage;
	if (color.empty())
	{
		rgbImage = cv::Mat(cloud.size(), 1, CV_8UC3, cv::Scalar(255, 255, 255));
	}
	else if (color.channels() == 1)
		cv::cvtColor(color, rgbImage, CV_GRAY2RGB);
	else
		color.copyTo(rgbImage);

	bool need_validate = true;
	if (validState.empty())
		need_validate = false;


	cv::Vec3b* rgbData = (cv::Vec3b*)rgbImage.data;
	std::ofstream file(fileName);
	for (int i = 0; i < cloud.size(); i++)
	{
		if (need_validate&&validState[i] == false)
			continue;
		if (color.empty())
			file<<std::fixed<<std::setprecision(7) << cloud[i](0) << " " << cloud[i](1) << " " << cloud[i](2)  << std::endl;
		else
			file << std::fixed << std::setprecision(7) << cloud[i](0) << " " << cloud[i](1) << " " << cloud[i](2) << " " << (int)rgbData[i][0] << " " << (int)rgbData[i][1] << " " << (int)rgbData[i][2] << std::endl;
	}
	file.close();
	return cloud.size();
}


void computeNormal(const std::vector<Eigen::Vector3f>& cloud, std::vector<bool>& validState, std::vector<Eigen::Vector3f>& normal, const int height, const int width)
{
	normal.resize(cloud.size());
	////#pragma omp parallel for
	for (int h = 0; h < height; h++)
	{
		for (int w = 0; w < width; w++)
		{
			int idx = h*width + w;
			int idx_right = idx + 1;
			int idx_down = idx + width;
			if (h == height-1 || w == width-1||!(validState[idx] && validState[idx_right] && validState[idx_down]))
			{
				validState[idx] = false;
				continue;
			}
			const Eigen::Vector3f& center_point = cloud[idx];
			const Eigen::Vector3f& right_point = cloud[idx_right];
			const Eigen::Vector3f& down_point = cloud[idx_down];
			
			normal[idx] = (down_point - center_point).cross(right_point - center_point);
			normal[idx].normalize();
		}
	}

}


void convertToLuminanceMap(const std::vector<Eigen::Vector3f>& cloud, const std::vector<Eigen::Vector3f>& normal, const std::vector<bool>& validState, const int height, const int width, Eigen::Vector3f cameraPosition, cv::Mat& luminanceMap)
{
	luminanceMap = cv::Mat::zeros(cv::Size(width, height), CV_32F);
	float* luminanceMapData = (float*)luminanceMap.data;

	
	//************light parameters
	float I_amb = 60;
	float I_diff = 150;
	float I_spec = 50;
	float n_s = 0.2;
	Eigen::Vector3f lightPosition(0, 0, 0);
	//******************

	for (int h = 0; h < height ; h++)
	{
		for (int w = 0; w < width ; w++)
		{
			int idx = h*width + w;
			if (validState[idx] == false)
				continue;

			Eigen::Vector3f ray = (lightPosition - cloud[idx]).normalized();
			Eigen::Vector3f spec_ray = (2 * normal[idx].dot(ray))*normal[idx] - ray;
			Eigen::Vector3f view_ray = (cameraPosition - cloud[idx]).normalized();

			float intensity_diff = I_diff*abs(normal[idx].dot(ray));
			float intensity_amb = I_amb;
			float intensity_spec = (view_ray.dot(spec_ray));
			intensity_spec = intensity_spec>0 ? I_spec*pow(intensity_spec,n_s) : 0;
			float intensity = intensity_amb + intensity_diff + intensity_spec;

			if (intensity>255)intensity = 255;

			luminanceMapData[idx] = intensity;
		}
	}

	
	luminanceMap.convertTo(luminanceMap, CV_8U);
}


void renderDepthMap(const cv::Mat& depth,const cv::Matx33f&  K, cv::Mat& luminanceMap)
{
	cv::Mat depth32F,filteredDepthMap;
	
	std::vector<Eigen::Vector3f> cloud,filteredCloud, normal;
	std::vector<bool> validationOfDepth;
	int height = depth.rows;
	int width = depth.cols;


	
	depth.convertTo(depth32F, CV_32FC1,1e-3);
	cv::bilateralFilter(depth32F, filteredDepthMap, 3, 10, 25);
	filteredDepthMap.convertTo(filteredDepthMap,CV_16U,1e3);

	convertDepthImgtoPointcloud(filteredDepthMap, filteredCloud, validationOfDepth, K);
	computeNormal(filteredCloud, validationOfDepth, normal, height, width);

	convertDepthImgtoPointcloud(depth, cloud, validationOfDepth, K);
	convertToLuminanceMap(cloud, normal, validationOfDepth, height, width, Eigen::Vector3f(1, 1, 1), luminanceMap);
}



void normalizeDepthImage(cv::Mat& depth, cv::Mat& disp)
{

	ushort* depthData = (ushort*)depth.data;
	int width = depth.cols;
	int height = depth.rows;

	static ushort max = *std::max_element(depthData, depthData + width*height);
	static ushort min = *std::min_element(depthData, depthData + width*height);

	disp = cv::Mat(depth.size(), CV_8U);
	uchar* dispData = disp.data;


	for (int i = 0; i < height; i++)
		for (int j = 0; j < width; j++)
		{
			dispData[i*width + j] = (((double)(depthData[i*width + j] - min)) / ((double)(max - min))) * 255;
		}

}




void normalizeInfraredImage(cv::Mat& depth, cv::Mat& disp)
{

	ushort* depthData = (ushort*)depth.data;
	int width = depth.cols;
	int height = depth.rows;

	static ushort max = *std::max_element(depthData, depthData + width*height);
	static ushort min = *std::min_element(depthData, depthData + width*height);

	disp = cv::Mat(depth.size(), CV_8U);
	uchar* dispData = disp.data;


	for (int i = 0; i < height; i++)
		for (int j = 0; j < width; j++)
		{
			dispData[i*width + j] = (((double)(depthData[i*width + j] - min)) / ((double)(max - min))) * 255;
		}
}





void normalize64FImage(cv::Mat& depth, cv::Mat& disp)
{

	double* depthData = (double*)depth.data;
	int width = depth.cols;
	int height = depth.rows;

	static double max = (*std::max_element(depthData, depthData + width*height))*0.01;
	static double min = *std::min_element(depthData, depthData + width*height);

	disp = cv::Mat(depth.size(), CV_8U);
	uchar* dispData = disp.data;


	for (int i = 0; i < height; i++)
		for (int j = 0; j < width; j++)
		{
			dispData[i*width + j] = (((double)(depthData[i*width + j] - min)) / ((double)(max - min))) * 255;
			if (dispData[i*width + j] > 255)dispData[i*width + j] = 255;
		}

}