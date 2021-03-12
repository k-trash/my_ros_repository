//Ver1.1.0 2020/03/12 k-trash
//Add gyro

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>

#include <cmath>

void imuCallback(const sensor_msgs::Imu& imu_msg_);
void convertRpyQuat(double roll_, double pitch_, double yaw_);

geometry_msgs::Quaternion marker_quat;

int main(int argc, char *argv[]){
	ros::init(argc, argv, "imu_pose_pub");
	ros::NodeHandle nh;

	ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("marker", 1);

	ros::Subscriber imu_sub = nh.subscribe("/imu/data_raw", 10, imuCallback);

	ros::Rate loop_rate(10);
	while(ros::ok()){
		visualization_msgs::Marker marker;
		marker.header.frame_id = "/world";
		marker.header.stamp = ros::Time::now();
		marker.ns = "basic_shapes";
		marker.id = 0;

		marker.type = visualization_msgs::Marker::CUBE;
		marker.action = visualization_msgs::Marker::ADD;
		marker.lifetime = ros::Duration();

		marker.scale.x = 0.5;
		marker.scale.y = 0.5;
		marker.scale.z = 0.1;
		marker.pose.position.x = 0;
		marker.pose.position.y = 0;
		marker.pose.position.z = 0;
		marker.pose.orientation.x = marker_quat.x;
		marker.pose.orientation.y = marker_quat.y;
		marker.pose.orientation.z = marker_quat.z;
		marker.pose.orientation.w = marker_quat.w;

		marker.color.r = 0.0f;
		marker.color.g = 1.0f;
		marker.color.b = 0.0f;
		marker.color.a = 2.0f;

		marker_pub.publish(marker);

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

void imuCallback(const sensor_msgs::Imu& imu_msg_){
	static double pre_time = ros::Time::now().toSec();

	static double roll = 0.0f, pitch = 0.0f, yaw = 0.0f;

	double d_time;
	double gyr_x, gyr_y, gyr_z;
	double d_roll, d_pitch,d_yaw;

	d_time = imu_msg_.header.stamp.toSec() - pre_time;
	pre_time = imu_msg_.header.stamp.toSec();

	gyr_x = imu_msg_.angular_velocity.x * d_time;
	gyr_y = imu_msg_.angular_velocity.y * d_time;
	gyr_z = -1*imu_msg_.angular_velocity.z * d_time;

	d_roll = gyr_x + gyr_y*sin(roll)*tan(pitch) + gyr_z*cos(roll)*tan(pitch);
	d_pitch = gyr_y*cos(roll) - gyr_z*sin(roll);
	d_yaw = gyr_y*sin(roll)/cos(pitch) + gyr_z*cos(roll)/cos(pitch);

	roll += d_roll;
	pitch += d_pitch;
	yaw += d_yaw;

	double acc_x = imu_msg_.linear_acceleration.x;
	double acc_y = imu_msg_.linear_acceleration.y;
	double acc_z = imu_msg_.linear_acceleration.z;


	roll = roll*0.95 - 0.05*atan(acc_y/acc_z);
	pitch = pitch*0.95 - 0.05*atan(-acc_x/sqrt(acc_y*acc_y+acc_z*acc_z));

	convertRpyQuat(roll, pitch, yaw);
}

void convertRpyQuat(double roll_, double pitch_, double yaw_){
	tf::Quaternion quat = tf::createQuaternionFromRPY(roll_, pitch_, yaw_);
	quaternionTFToMsg(quat, marker_quat);
}
