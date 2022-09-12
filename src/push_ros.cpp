#include <push_rtmp.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

Mat frame;
cv_bridge::CvImageConstPtr cv_ptr;

PushRTMP::Ptr push_rtmp;

string node_arg;
string node_name;
int width;
int height;

Mat map1, map2;

void initRectify()
{
    Mat K = (cv::Mat_<double>(3, 3) << 286.1809997558594, 0.0, 421.6383972167969,
             0.0, 286.3576965332031, 403.9013977050781,
             0.0, 0.0, 1.0);
    Mat D = (cv::Mat_<double>(4, 1) << -0.008326118811964989, 0.04620290920138359, -0.04403631016612053, 0.00837636087089777);

    cv::Size imageSize(width, height);
    cv::fisheye::initUndistortRectifyMap(K, D, cv::Mat(), K, imageSize, CV_16SC2, map1, map2);
}

void imageCallback(const sensor_msgs::Image::ConstPtr &img)
{
    // cout << img->width << "x" << img->height << " " << img->encoding << endl;
    try
    {
        if (node_arg == "zedm")
        {
            cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGRA8);
            cv::resize(cv_ptr->image, frame, cv::Size(width, height));
        }
        else if (node_arg == "realsense_dis")
        {
            cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
            frame = cv_ptr->image;
        }
        else if (node_arg == "realsense_undis")
        {
            cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
            remap(cv_ptr->image, frame, map1, map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
        }
        else
        {
            ROS_WARN("Node argument mismatch!");
        }
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_brige exception : %s", e.what());
        return;
    }
    // cout << cv_ptr->image.cols << "x" << cv_ptr->image.rows << " channels:" << cv_ptr->image.channels() << endl;
    imshow(node_name, frame);
    waitKey(1);
    push_rtmp->startPush(frame.data, frame.elemSize());
}

int main(int argc, char *argv[])
{
    node_arg = argv[1];

    string subscribe_topic;
    string push_addr;

    if (node_arg == "zedm")
    {
        node_name = "/push_zedm";
        subscribe_topic = "/zedm/zed_node/rgb/image_rect_color";
        push_addr = "rtmp://1.116.137.21:7788/zedm";
    }
    else if (node_arg == "realsense_dis")
    {
        node_name = "/push_realsense_dis";
        subscribe_topic = "/camera/fisheye1/image_raw";
        push_addr = "rtmp://1.116.137.21:7789/realsense_distort";
    }
    else if (node_arg == "realsense_undis")
    {
        node_name = "/push_realsense_undis";
        subscribe_topic = "/camera/fisheye1/image_raw";
        push_addr = "rtmp://1.116.137.21:7790/realsense_undistort";
    }
    else
    {
        ROS_ERROR("Wrong node argument!");
    }

    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    int fps;
    string pixel_type;
    ros::param::get(node_name + "/width", width);
    ros::param::get(node_name + "/height", height);
    ros::param::get(node_name + "/fps", fps);
    ros::param::get(node_name + "/pixel_type", pixel_type);
    // cout << width << " " << height << " " << fps << endl;

    push_rtmp = make_shared<PushRTMP>(width, height, fps);
    if (!push_rtmp->initRTMP(push_addr, pixel_type))
    {
        ROS_ERROR("Failed to initialize PushRTMP!");
        return -1;
    }

    if (node_arg == "realsense_undis")
    {
        initRectify();
    }

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe(subscribe_topic, 1, imageCallback);

    ros::spin();

    return 0;
}