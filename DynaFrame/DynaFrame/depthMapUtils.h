#include <opencv2\highgui\highgui.hpp>
#include <opencv2\core\core.hpp>
#include <Eigen\core>
#include <Eigen\geometry>


void convertDepthImgtoPointcloud(const cv::Mat& depth, std::vector<Eigen::Vector3f>& cloud, std::vector<bool>& validState, const cv::Matx33f&  K);
void computeNormal(const std::vector<Eigen::Vector3f>& cloud, std::vector<bool>& validState, std::vector<Eigen::Vector3f>& normal, const int height, const int width);
void convertToLuminanceMap(const std::vector<Eigen::Vector3f>& cloud, const std::vector<Eigen::Vector3f>& normal, const std::vector<bool>& validState, const int height, const int width, Eigen::Vector3f cameraPosition, cv::Mat& luminanceMap);
void renderDepthMap(const cv::Mat& depth, const cv::Matx33f&  K, cv::Mat& luminanceMap);
int savePointCloud(std::string fileName, const std::vector<Eigen::Vector3f>& cloud, std::vector<Eigen::Vector3f>& normal, const std::vector<bool>& validState = std::vector<bool>());
int savePointCloud(std::string fileName, const std::vector<Eigen::Vector3f>& cloud, const std::vector<bool>& validState = std::vector<bool>(), cv::Mat& color = cv::Mat());

void normalizeDepthImage(cv::Mat& depth, cv::Mat& disp);
void normalizeInfraredImage(cv::Mat& depth, cv::Mat& disp);
void normalize64FImage(cv::Mat& depth, cv::Mat& disp);
