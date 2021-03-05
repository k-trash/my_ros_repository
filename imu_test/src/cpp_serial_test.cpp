//Ver1.0.0 2020/03/05 k-trash

#include <iostream>
#include <cunistd>
#include <cfcntl>
#include <ctermios>

int openSerial(const char *devise_name_);

int main(int argc, char *argv[]){
	int ret_dev = 0;
	int recv_size = 0;
	int state = 0;
	int next_index = 0;
	char device_name[] = "/dev/ttyACM0";
	static unsigned char buffer_data[256] = {0};
	static int buffer_size = 0;
	unsigned char recv_data[256] = {0};

	ret_dev = openSerial(device_name);

	if(ret_dev<0){
		std::cout << "Serial Fail\n" << std::endl;
		return 0;
	}

	while(true){
		recv_size = read(ret_dev, recv_data, sizeof(recv_data));
		if(recv_size > 0){
			for(int i=0;i<recv_size;i++){
				buffer_data[buffer_size+i] = recv_data[i];
			}
			buffer_size += recv_size;

			state = 0;
			next_index = 0;
			for(int i=0;i<buffer_size;i++){
				if(state == 0 || state == 1){
					if(buffer_data[i] == 
