//Ver1.1.1 2020/03/07 k-trash
//change magnet publish

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"

#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

//#define OFFSET
//#define MAGNET

int openSerial(const char *devise_name_);

int main(int argc, char *argv[]){
	ros::init(argc, argv, "cpp_serial_test");
	ros::NodeHandle nh;

	int max[3] = {0};
	int min[3] = {0};

	int ret_dev = 0;
	int rec_write = 0;
	int recv_size = 0;
	char cap_write[1] = {'0'};
	char device_name[] = "/dev/ttyACM0";
	unsigned char recv_data[256] = {0};

	sensor_msgs::Imu imu;
	sensor_msgs::MagneticField mag;

	ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("imu/data_raw", 10);

	#ifdef MAGNET
		ros::Publisher mag_pub = nh.advertise<sensor_msgs::MagneticField>("imu/mag", 10);
	#endif

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
				imu.angular_velocity.x = int16_t(recv_data[6]<<8 | recv_data[7]) + 40;
				imu.angular_velocity.y = int16_t(recv_data[8]<<8 | recv_data[9]) - 40;
				imu.angular_velocity.z = int16_t(recv_data[10]<<8 | recv_data[11]) - 25;
				
				#ifdef MAGNET
					mag.header.frame_id = "imu_link";
					mag.header.stamp = ros::Time::now();
					mag.magnetic_field.x = int16_t(recv_data[12]<<8 | recv_data[13]) + 200;
					mag.magnetic_field.y = int16_t(recv_data[14]<<8 | recv_data[15]) + 100;
					mag.magnetic_field.z = int16_t(recv_data[16]<<8 | recv_data[17]);
				#endif

				#ifdef OFFSET
					if(max[0] < mag.magnetic_field.x)	max[0] = mag.magnetic_field.x;
					if(min[0] > mag.magnetic_field.x)	min[0] = mag.magnetic_field.x;
					if(max[1] < mag.magnetic_field.y)	max[1] = mag.magnetic_field.y;
					if(min[1] > mag.magnetic_field.y)	min[1] = mag.magnetic_field.y;
					if(max[2] < mag.magnetic_field.z)	max[2] = mag.magnetic_field.z;
					if(min[2] > mag.magnetic_field.z)	min[2] = mag.magnetic_field.z;

					for(int i=0;i<3;i++){
						std::cout << max[i] << '\t' << min[i] << '\t';
					}
					std::cout << std::endl;
				#endif
			}else{
				std::cout << "Read fail" << std::endl;
			}
		}
		imu_pub.publish(imu);
		
		#ifdef MAGNET
			mag_pub.publish(mag);
		#endif

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
