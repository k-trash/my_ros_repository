//Ver1.1.0 2020/03/07 k-trash
//change to ros node

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"

#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

int openSerial(const char *devise_name_);

int main(int argc, char *argv[]){
	ros::init(argc, argv, "cpp_serial_test");
	ros::NodeHandle nh;

	int ret_dev = 0;
	int rec_write = 0;
	int recv_size = 0;
	char cap_write[1] = {'0'};
	char device_name[] = "/dev/ttyACM0";
	unsigned char recv_data[256] = {0};

	sensor_msgs::Imu imu;
	
	ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("imu_data", 10);

	ret_dev = openSerial(device_name);

	if(ret_dev<0){
		std::cout << "Couldn't open" << std::endl;
		ros::shutdown();
	}

	ros::Rate loop_rate(10);
	while(ros::ok()){
		rec_write = write(ret_dev, cap_write, 1);

		if(rec_write == 0){
			std::cout << "Couldn't write" << std::endl;
			ros::shutdown();
		}
		recv_size = read(ret_dev, recv_data, sizeof(recv_data));
		if(recv_size > 0){
			if(recv_size == 18){
				imu.header.frame_id = "imu_link";
				imu.header.stamp = ros::Time::now();
				imu.linear_acceleration.x = int16_t(recv_data[0]<<8 | recv_data[1]);
				imu.linear_acceleration.y = int16_t(recv_data[2]<<8 | recv_data[3]);
				imu.linear_acceleration.z = int16_t(recv_data[4]<<8 | recv_data[5]);
				imu.angular_velocity.x = int16_t(recv_data[6]<<8 | recv_data[7]);
				imu.angular_velocity.y = int16_t(recv_data[8]<<8 | recv_data[9]);
				imu.angular_velocity.z = int16_t(recv_data[10]<<8 | recv_data[11]);
				std::cout << std::endl;
			}else{
				std::cout << "Read fail" << std::endl;
			}
		}
		imu_pub.publish(imu);

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

int openSerial(const char *device_name_){
	int fd_1 = open(device_name_, O_RDWR | O_NOCTTY | O_NONBLOCK);
	fcntl(fd_1, F_SETFL, 0);

	struct termios conf_tio;
	tcgetattr(fd_1, &conf_tio);

	speed_t BAUDRATE = B115200;
	cfsetispeed(&conf_tio, BAUDRATE);
	cfsetospeed(&conf_tio, BAUDRATE);

	conf_tio.c_lflag &= ~(ECHO | ICANON);

	conf_tio.c_cc[VMIN] = 0;
	conf_tio.c_cc[VTIME] = 0;

	tcsetattr(fd_1, TCSANOW, &conf_tio);

	return fd_1;
}
