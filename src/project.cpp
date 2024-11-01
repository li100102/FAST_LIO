#include <iostream>
#include <thread>

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CompressedImage.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <Eigen/Eigen>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <livox_ros_driver/CustomMsg.h>

using namespace std;
using namespace Eigen;

inline cv::Scalar depthToRGB(double depth) {
    if (depth < 0.0f) depth = 0.0f;
    if (depth > 1.0f) depth = 1.0f;

    int r, g, b;

    if (depth < 0.125f) {
        r = 0;
        g = 0;
        b = 127 + (depth / 0.125f) * 128;
    } else if (depth < 0.375f) {
        r = 0;
        g = (depth - 0.125f) / 0.25f * 255;
        b = 255;
    } else if (depth < 0.625f) {
        r = (depth - 0.375f) / 0.25f * 255;
        g = 255;
        b = 255 - (depth - 0.375f) / 0.25f * 255;
    } else if (depth < 0.875f) {
        r = 255;
        g = 255 - (depth - 0.625f) / 0.25f * 255;
        b = 0;
    } else {
        r = 255 - (depth - 0.875f) / 0.125f * 127;
        g = 0;
        b = 0;
    }

    return cv::Scalar(b, g, r);  // Note: OpenCV uses BGR format by default
}

void saveCustomMsgToPCD(const livox_ros_driver::CustomMsg::ConstPtr& pc_msg, const std::string& filename, const pair<double, double> depth_range)
{
    const double &min_x = depth_range.first, &max_x = depth_range.second;
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    cloud.width = pc_msg->points.size();
    cloud.height = 1;
    cloud.is_dense = false;
    cloud.points.resize(cloud.width * cloud.height);

    for (size_t i = 0; i < pc_msg->points.size(); ++i)
    {
        cloud.points[i].x = pc_msg->points[i].x;
        cloud.points[i].y = pc_msg->points[i].y;
        cloud.points[i].z = pc_msg->points[i].z;
        double depth = (pc_msg->points[i].x - min_x) / (max_x - min_x); // calculate color index based on x value
        cv::Scalar rgb = depthToRGB(depth);
        cloud.points[i].r = rgb[2];
        cloud.points[i].g = rgb[1];
        cloud.points[i].b = rgb[0];
    }
    pcl::io::savePCDFileBinary(filename, cloud);
}

inline void draw_point(cv::Mat& img, const Vector3d& p, const double depth=0)
{
    cv::Point2d p2d(p(0) / p(2), p(1) / p(2));
    cv::circle(img, p2d, 2, depthToRGB(depth), -1);
}

void callback(const sensor_msgs::CompressedImageConstPtr& img_msg, const livox_ros_driver::CustomMsg::ConstPtr& pc_msg)
{
    Matrix3d R, K;   
    Vector3d t(0.0025563, 0.0567484, -0.0512149);
    R <<    0.00298088,-0.999728,-0.0231416,
            -0.00504636,0.0231263,-0.99972,
            0.999983,0.00309683,-0.00497605;
    K <<    1453.72, 0.0,      1172.18,
            0.0,     1453.28,  1041.78,
            0.0,     0.0,      1.0;
    // Convert compressed image to OpenCV Mat
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Access the OpenCV Mat
    cv::Mat img = cv_ptr->image;

    // get max and min x value
    float max_x = -1e9, min_x = 1e9;
    for (const auto _p: pc_msg->points)
    {
        if (_p.x < 1e-8) continue;
        max_x = max(max_x, _p.x);
        min_x = min(min_x, _p.x);
    }

    for (const auto _p: pc_msg->points)
    {
        if (_p.x < 1e-8) continue;
        Vector3d p(_p.x, _p.y, _p.z);
        Vector3d p_ = K * (R * p + t);
        double depth = (_p.x - min_x) / (max_x - min_x); // calculate color index based on x value
        draw_point(img, p_, depth);
    }
    // show image
    cv::Mat resized_img;
    cv::resize(img, resized_img, cv::Size(918, 768));
    cv::imshow("image", resized_img);
    auto key = cv::waitKey(1);
    if (key == 'q')
        exit(0);
    else if (key == 's')
    {
        thread t(saveCustomMsgToPCD, pc_msg, "projection.pcd", make_pair(min_x, max_x));
        t.detach();
        cv::imwrite("projection.png", img);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "projection_node");
    ros::NodeHandle nh;

    // Subscribe to compressed image topic
    message_filters::Subscriber<sensor_msgs::CompressedImage> img_sub(nh, "/left_camera/image/compressed", 1);

    // Subscribe to point cloud 2 topic
    message_filters::Subscriber<livox_ros_driver::CustomMsg> pc_sub(nh, "/livox/lidar", 1);

    // Synchronize the two topics by time
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, livox_ros_driver::CustomMsg> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), img_sub, pc_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    ros::spin();

    return 0;
}
