#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/String.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include "ros_posenet/Keypoint.h"
#include "ros_posenet/Poses.h"
#include "ros_posenet/Pose.h"
#include "../include/tfpose_ros/camera.h"

Camera::Camera(ros::NodeHandle *n)
{
    printf("Start class of 'Camera'\n");
    this->color_image_sub = n->subscribe("/usb_cam/image_raw", 1, &Camera::image_callback, this);
    this->output_sub = n->subscribe("/tfpose_ros/output", 1, &Camera::poses_callback, this);
    this->input_pub = n->advertise<sensor_msgs::Image>("/trpose_ros/input", 1);
    this->posenet_result_pub = n->advertise<ros_posenet::Poses>("/tfpose_ros/result", 1);
}

Camera::~Camera()
{
    printf("Shutdown class of 'Camera'\n");
}

void Camera::image_callback(const sensor_msgs::Image::ConstPtr &msg)
{
    this->input_pub.publish(msg);
}

void Camera::poses_callback(const std_msgs::String::ConstPtr &msg)
{
    std::string err;
    auto json = json11::Json::parse(msg->data, err);
    ros_posenet::Poses poses;

    if (json["poses"].array_items().empty())
        printf("error\n");
    else {
        for (auto &p : json["poses"].array_items()) {
            ros_posenet::Pose pose;
            for (auto &k : p["keypoints"].array_items()) {
                if (std::stod(k["score"].dump()) > 0.5) {
                    ros_posenet::Keypoint key;
                    key.position.x = std::stod(k["position"]["x"].dump());
                    key.position.y = std::stod(k["position"]["y"].dump());
                    key.position.z = 0;
                    key.score = std::stod(k["score"].dump());
                    key.part = k["part"].dump();
                    key.part.erase(remove(key.part.begin(), key.part.end(), '"'), key.part.end());
                    pose.keypoints.push_back(key);
                }
            }
            poses.poses.push_back(pose);
        }
        this->posenet_result_pub.publish(poses);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "camera");
    ros::NodeHandle n;
    Camera camera(&n);
    ros::spin();
    return 0;
}