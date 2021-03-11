//Ver1.0.0 2020/03/11 k-trash

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>

int main(int argc, char *argv[]){
	ros::init(argc, argv, "imu_pose_pub");
	ros::NodeHandle nh;

	ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("marker", 1);

	ros::Subscriber imu_sub = nh.subscribe("/imu/data_raw", 10, imuCallback);

	ros::spin();

	return 0;
}
