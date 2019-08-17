#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <tfpose_ros/Persons.h>
#include <tfpose_ros/Person.h>
#include <tfpose_ros/BodyPartElm.h>

class TFPose_Realsense
{
public:
    TFPose_Realsense();

    ~TFPose_Realsense();

    cv::Mat color;
    cv::Mat depth;
    cv::Mat view_depth;
    pcl::PointCloud<pcl::PointXYZRGB> pc;


private:
    ros::NodeHandle n;
    ros::Publisher realsense_image;
    ros::Publisher pose_estimator_3d;
    ros::Subscriber point_cloud_data;
    ros::Subscriber pose_estimator_2d;

    void point_cloud_data_callback(const boost::shared_ptr<const sensor_msgs::PointCloud2> &input)
    {

        int height = (int) input->height;
        int width = (int) input->width;
        double x, y, z;

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
                x = temp_cloud->points[width * h + w].x;
                y = temp_cloud->points[width * h + w].y;
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

        realsense_image.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", color));

    }

    void pose_estimator_2d_callback(const tfpose_ros::Persons::ConstPtr &data)
    {

        tfpose_ros::Persons persons;
        int image_x, image_y;

        persons.image_w = data->image_w;
        persons.image_h = data->image_h;
        persons.header = data->header;

        for (auto person : data->persons) {
            tfpose_ros::Person new_person;
            for (auto body : person.body_part) {
                image_x = (int) (body.x * persons.image_w + 0.5);
                image_y = (int) (body.y * persons.image_h + 0.5);
                cv::Point3d result;
                int i = 1;
                while (search_around(i++, cv::Point(image_x, image_y), color.cols, &result));
                tfpose_ros::BodyPartElm new_body;
                new_body.part_id = body.part_id;
                new_body.x = body.x;
                new_body.y = body.y;
                new_body.z = result.z;
                new_body.confidence = body.confidence;
                new_person.body_part.push_back(new_body);
            }
            persons.persons.push_back(new_person);
        }
        pose_estimator_3d.publish(persons);
    }

    cv::Point3d get_real_point_data(pcl::PointCloud<pcl::PointXYZRGB> *data, int width, cv::Point image)
    {
        double z = depth.at<double>(image.y, image.x);
        if (!(z >= 0.4 && z <= 4.0)) return cv::Point3d(0.0, 0.0, 0.0);
        auto point = data->points[width * image.y + image.x];
        return cv::Point3d(point.x, point.y, point.z);
    }

    bool search_around(int area, cv::Point image_center, int width, cv::Point3d *result)
    {
        //areaは奇数に限る
        if (area % 2 == 0) return true;
        if (area == 1) {
            *result = get_real_point_data(&pc, width, image_center);
            return result->z == 0.0;
        }
        cv::Point min, max;
        cv::Point3d sum, tmp;
        double count = 0;

        min.x = image_center.x - (int) (area / 2);
        min.y = image_center.y - (int) (area / 2);
        max.x = image_center.x + (int) (area / 2);
        max.y = image_center.y + (int) (area / 2);

        for (int y = min.y; y < max.y; ++y) {
            for (int x = min.x; x < max.x; ++x) {
                tmp = get_real_point_data(&pc, width, cv::Point(x, y));
                if (tmp.z == 0.0) continue;
                sum.x += tmp.x;
                sum.y += tmp.y;
                sum.z += tmp.z;
                count++;
            }
        }

        if (count == 0) return true;

        result->x = sum.x / count;
        result->y = sum.y / count;
        result->z = sum.z / count;

        return false;
    }

};

TFPose_Realsense::TFPose_Realsense()
{
    realsense_image = n.advertise<sensor_msgs::Image>("/tf_pose/realsense_image", 1);
    pose_estimator_3d = n.advertise<tfpose_ros::Persons>("/pose_estimator/pose_3d", 1);
    point_cloud_data = n.subscribe("/camera/depth_registered/points", 1, &TFPose_Realsense::point_cloud_data_callback,
                                   this);
    pose_estimator_2d = n.subscribe("/pose_estimator/pose", 1, &TFPose_Realsense::pose_estimator_2d_callback, this);
}

TFPose_Realsense::~TFPose_Realsense()
{
    printf("Shutdown class of 'TFPose_Realsense'\n");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tfpose_ros_realsense");
    TFPose_Realsense realsense;
    ros::spin();
}