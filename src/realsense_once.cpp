#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <tfpose_ros/Poses.h>
#include <tfpose_ros/Pose.h>
#include <tfpose_ros/Keypoint.h>
#include <std_msgs/String.h>
#include "../include/tfpose_ros/realsence_once.h"


TFPoseRealsenseOnce::TFPoseRealsenseOnce()
{
    image_pub = n.advertise<sensor_msgs::Image>("/tfpose/input", 1);
    pose_result_pub = n.advertise<tfpose_ros::Poses>("/tfpose_ros/result", 1);
    point_cloud_data_sub =
        n.subscribe("/camera/depth_registered/points", 1, &TFPoseRealsenseOnce::point_cloud_data_callback, this);
    pose_output_sub = n.subscribe("/tfpose/output", 1, &TFPoseRealsenseOnce::pose_callback, this);
    shutter_sub = n.subscribe("/tfpose_ros/shutter", 1, &TFPoseRealsenseOnce::shutter_callback, this);
}

TFPoseRealsenseOnce::~TFPoseRealsenseOnce()
{
    printf("Shutdown class of 'TFPoseRealsenseOnce'\n");
}

void TFPoseRealsenseOnce::point_cloud_data_callback(const boost::shared_ptr<const sensor_msgs::PointCloud2> &input)
{
    int height = (int) input->height;
    int width = (int) input->width;
    double z;

    color = cv::Mat::zeros(cv::Size(width, height), CV_8UC3);
    depth = cv::Mat::zeros(cv::Size(width, height), CV_64F);

    //pcl変換
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2(pcl_pc2, *temp_cloud);

    pc = *temp_cloud;

    //展開
    for (int w = 0; w < width; ++w) {
        for (int h = 0; h < height; ++h) {
            color.at<cv::Vec3b>(h, w)[0] = (int) temp_cloud->points[width * h + w].b;
            color.at<cv::Vec3b>(h, w)[1] = (int) temp_cloud->points[width * h + w].g;
            color.at<cv::Vec3b>(h, w)[2] = (int) temp_cloud->points[width * h + w].r;
            z = temp_cloud->points[width * h + w].z;
            depth.at<double>(h, w) = z;
        }
    }

    depth.convertTo(view_depth, CV_8UC1, 255.0 / 4.0);

    cv::namedWindow("depth", CV_WINDOW_NORMAL);
    cv::imshow("depth", view_depth);

    cv::namedWindow("color", CV_WINDOW_NORMAL);
    cv::imshow("color", color);

    cv::waitKey(1);
}

void TFPoseRealsenseOnce::shutter_callback(const std_msgs::String::ConstPtr &msg)
{
    /*
     * シャッター。受け取ったら画像とpcを保存
     */
    take_pc = pc;
    take_color = color;
    image_pub.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", color));
    printf("画像送信\n");
}

void TFPoseRealsenseOnce::pose_callback(const tfpose_ros::Poses::ConstPtr &data)
{
    tfpose_ros::Poses new_poses;
    int image_x, image_y;

    for (auto pose : data->poses) {
        tfpose_ros::Pose new_pose;
        for (auto key : pose.keypoints) {
            image_x = key.image_position.x;
            image_y = key.image_position.y;
            cv::Point3d result;
            //zの取得
            result = get_real_point_data(&take_pc, image_x, cv::Point(image_x, image_y));
            std::cout << result << '\n';
            tfpose_ros::Keypoint new_key;
            new_key.part = key.part;
            new_key.image_position.x = image_x;
            new_key.image_position.y = image_y;
            new_key.position.x = key.position.x;
            new_key.position.y = key.position.y;
            new_key.position.z = result.z;
            new_key.score = key.score;
            new_pose.keypoints.push_back(new_key);
        }
        new_poses.poses.push_back(new_pose);
    }
    pose_result_pub.publish(new_poses);
}

cv::Point3d TFPoseRealsenseOnce::get_real_point_data(pcl::PointCloud<pcl::PointXYZRGB> *data,
                                                     int width,
                                                     cv::Point image)
{
    double z = depth.at<double>(image.y, image.x);
    if (!(z >= 0.4 && z <= 4.0)) return cv::Point3d(0.0, 0.0, 0.0);
    auto point = data->points[width * image.y + image.x];
    return cv::Point3d(point.x, point.y, point.z);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tfpose_ros_realsense");
    TFPoseRealsenseOnce realsense;
    ros::spin();
}