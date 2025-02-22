#include <iostream>
#include <termios.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "../include/serial.h"
using namespace std;

Serial::Serial(Baud_Rate_e br, Word_Length_e wl, Stop_Bit_e sb, Parity_e p, Hardware_Flow_Control_e hfc)
{
	this->init.baud_rate = br;
	this->init.word_length = wl;
	this->init.stop_bit = sb;
	this->init.parity = p;
	this->init.hw_flow_ctl = hfc;
}

Serial::~Serial(void)
{
    close(this->fd);
	//cout<<"close succcessful"<<endl;
}

bool Serial::sOpen(const char* dev_name)
{
	this->fd = open(dev_name, O_RDWR | O_NOCTTY | O_NDELAY);
	if(fd < 0)
		return false;
	else
		fcntl(fd, F_SETFL, 1);

	struct termios opt;
    if(tcgetattr(fd, &opt) < 0)
		return false;

	opt.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);	//using raw mode
	opt.c_oflag &= ~OPOST;	//no output process
	switch(this->init.baud_rate)
	{
		case BR115200:
			cfsetispeed(&opt, B115200);
    		cfsetospeed(&opt, B115200);
			break;
                case BR9600:
                      cfsetispeed(&opt, B9600);
                      cfsetospeed(&opt, B9600);
		      break;
                default:
			return false;
	}
	
	switch(this->init.word_length)
    {
        case WORDLENGTH_8B:
			opt.c_cflag &= ~CSIZE;
    		opt.c_cflag |= CS8;
            break;
        default:
            return false;
    }
	
	switch(this->init.stop_bit)
    {
        case STOPBITS_1:
			opt.c_cflag &= ~CSTOPB;
            break;
        default:
            return false;
    }

    switch(this->init.parity)
    {
        case PARITY_NONE:
          	 opt.c_cflag &= ~PARENB;
			 break;
        default:
            return false;
    }

	switch(this->init.hw_flow_ctl)
    {
        case HWCONTROL_NONE:
            break;
        default:
            return false;
    }

	tcflush(fd, TCIOFLUSH);

	if(tcsetattr(this->fd, TCSANOW, &opt) < 0)
    {
        return false;
    }
	
	//cout<<"open successful"<<endl;
	return true;
}

bool Serial::sClose(void)
{
	if(close(this->fd) < 0)
		return false;

	return true;
}

bool Serial::sSendData(float x_offset, float y_offset, float distance, unsigned char shoot_flag)
{
	unsigned char data[14];
    data[0] = 0xFF;

    Float_Uchar_u temp;
    temp.data_f = x_offset;
    data[1] = temp.data_uc[0];
    data[2] = temp.data_uc[1];
    data[3] = temp.data_uc[2];
    data[4] = temp.data_uc[3];

    temp.data_f = y_offset;
    data[5] = temp.data_uc[0];
    data[6] = temp.data_uc[1];
    data[7] = temp.data_uc[2];
    data[8] = temp.data_uc[3];
	
    temp.data_f = distance;
    data[9] = temp.data_uc[0];
    data[10] = temp.data_uc[1];
    data[11] = temp.data_uc[2];
    data[12] = temp.data_uc[3];

    data[13] = shoot_flag;

    if(write(this->fd, data, 14) == 14)
        return true;
    else
        return false;

}

bool Serial::sReadData(void)
{
	unsigned char data[9];
	if(read(this->fd, data, 9) != 9)
        return false;

    Float_Uchar_u temp;
    float x,y;
    temp.data_uc[0] = data[1];
    temp.data_uc[1] = data[2];
    temp.data_uc[2] = data[3];
    temp.data_uc[3] = data[4];
	x = temp.data_f;

    temp.data_uc[0] = data[5];
    temp.data_uc[1] = data[6];
    temp.data_uc[2] = data[7];
    temp.data_uc[3] = data[8];
	y = temp.data_f;
	cout<<"x = "<<x<<" y = "<<y<<endl;
}







