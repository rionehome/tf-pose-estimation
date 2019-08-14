//
// Created by migly on 19/08/04.
//

#ifndef CAMERA_H
#define CAMERA_H

class Camera
{
public:
    explicit Camera(ros::NodeHandle *n);
    ~Camera();

private:
    ros::Subscriber color_image_sub;
    ros::Subscriber output_sub;
    ros::Publisher input_pub;
    ros::Publisher posenet_result_pub;

    void image_callback(const sensor_msgs::Image_<std::allocator<void>>::ConstPtr &msg);
    void poses_callback(const std_msgs::String_<std::allocator<void>>::ConstPtr &msg);
};


#endif //CAMERA_H
