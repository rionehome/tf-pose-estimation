#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>


class TFPose_Kinect {
public:
	TFPose_Kinect();
	~TFPose_Kinect();
private:
	ros::NodeHandle n;
	ros::Publisher cloud_image;
	ros::Subscriber point_cloud_data;

	void point_cloud_data_callback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input) {

		pcl::PCLPointCloud2 pcl_pc2;
		pcl_conversions::toPCL(*input, pcl_pc2);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::fromPCLPointCloud2(pcl_pc2, *temp_cloud);
		//do stuff with temp_cloud here
		printf("%d\n", temp_cloud->points[10000].r );

	}

};

TFPose_Kinect::TFPose_Kinect() {
	//ros::Publisher pub_points = n.advertise<std_msgs::Float64MultiArray>("/ros_kinect/points", 1000);

	point_cloud_data = n.subscribe("/camera/depth_registered/points", 1000, &TFPose_Kinect::point_cloud_data_callback, this);

}

TFPose_Kinect::~TFPose_Kinect() {

}




int main(int argc, char** argv) {

	ros::init(argc, argv, "ros_kinect");

	TFPose_Kinect kinect;

	while (ros::ok()) {
		ros::spinOnce();
	}

}