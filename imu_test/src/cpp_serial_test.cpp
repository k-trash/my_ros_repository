//Ver1.0.0 2020/03/05 k-trash

//#include <ros/ros.h>
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <thread>
#include <chrono>

int openSerial(const char *devise_name_);

int main(int argc, char *argv[]){
	//ros::init(argc, argv, "cpp_serial_test");
	//ros::NodeHandle nh;

	int ret_dev = 0;
	int rec_write = 0;
	int recv_size = 0;
	int state = 0;
	int next_index = 0;
	char cap_write[1] = {'0'};
	char device_name[] = "/dev/ttyACM0";
	static unsigned char buffer_data[256] = {0};
	static int buffer_size = 0;
	unsigned char recv_data[256] = {0};

	ret_dev = openSerial(device_name);

	if(ret_dev<0){
		std::cout << "Couldn't open" << std::endl;
		return 1;
	}

	while(true){
		rec_write = write(ret_dev, cap_write, 1);

		if(rec_write == 0){
			std::cout << "Couldn't write" << std::endl;
			return 1;
		}
		std::cout << "Wrote\t";
		recv_size = read(ret_dev, recv_data, sizeof(recv_data));
		if(recv_size > 0){
			if(recv_size == 12){
				for(int i=0;i<6;i++){
					std::cout << int16_t(recv_data[i<<1]<<8 | recv_data[(i<<1)+1]) << '\t';
				}
				std::cout << std::endl;
			}else{
				std::cout << "Read fail" << std::endl;
			}
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
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
