//
// Created by migly on 19/08/04.
//

#ifndef REALSENSE_ONCE_H
#define REALSENSE_ONCE_H

class TFPoseRealsenseOnce
{
public:
    TFPoseRealsenseOnce();
    ~TFPoseRealsenseOnce();

    cv::Mat color;
    cv::Mat take_color;
    cv::Mat depth;
    cv::Mat view_depth;
    pcl::PointCloud<pcl::PointXYZRGB> pc;
    pcl::PointCloud<pcl::PointXYZRGB> take_pc;

private:
    ros::NodeHandle n;
    ros::Publisher image_pub;
    ros::Publisher pose_result_pub;
    ros::Subscriber point_cloud_data_sub;
    ros::Subscriber pose_output_sub;
    ros::Subscriber shutter_sub;

    void point_cloud_data_callback(const boost::shared_ptr<const sensor_msgs::PointCloud2> &input);
    cv::Point3d get_real_point_data(pcl::PointCloud<pcl::PointXYZRGB> *data, int width, cv::Point image);
    void shutter_callback(const std_msgs::String_<std::allocator<void>>::ConstPtr &msg);
    void pose_callback(const tfpose_ros::Poses_<std::allocator<void>>::ConstPtr &data);
};


#endif //REALSENSE_ONCE_H
