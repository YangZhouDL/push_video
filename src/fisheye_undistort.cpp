#include <iostream>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

using namespace std;
using namespace cv;

int main(int argc, char *argv[])
{
    Mat K = (cv::Mat_<double>(3, 3) << 286.1809997558594, 0.0, 421.6383972167969,
         0.0, 286.3576965332031, 403.9013977050781,
         0.0, 0.0, 1.0);
    Mat D = (cv::Mat_<double>(4, 1) << -0.008326118811964989, 0.04620290920138359, -0.04403631016612053, 0.00837636087089777);

    Mat raw_image = imread("/home/redwall/catkin_ws/src/push_video/image/raw.jpg");
    cout << raw_image.cols  << " " << raw_image.rows << endl;
    int width = 848;
    int height = 800;
    Mat map1, map2;
    Mat undistortImg;
    cv::Size imageSize(width, height);
    cv::fisheye::initUndistortRectifyMap(K, D, cv::Mat(), K, imageSize, CV_16SC2, map1, map2);
    // cout << "map1:" << map1 << "\nmap2:" << map2 << endl;
    remap(raw_image, undistortImg, map1, map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
    imwrite("/home/redwall/catkin_ws/src/push_video/image/cpp_processed.jpg", undistortImg);

    return 0;
}