#ifndef __SERIAL_H
#define __SERIAL_H

typedef enum{
	BR115200,
        BR9600
}Baud_Rate_e;

typedef enum{
	WORDLENGTH_8B
}Word_Length_e;

typedef enum{
	STOPBITS_1
}Stop_Bit_e;

typedef enum{
	PARITY_NONE
}Parity_e;

typedef enum{
	HWCONTROL_NONE
}Hardware_Flow_Control_e;

typedef struct
{
	Baud_Rate_e baud_rate;
	Word_Length_e word_length;
	Stop_Bit_e stop_bit;
	Parity_e parity;
	Hardware_Flow_Control_e hw_flow_ctl;
}Serial_Init_t;

class Serial
{
public:
	int fd;
	Serial_Init_t init;

	Serial(Baud_Rate_e br, Word_Length_e wl, Stop_Bit_e sb, Parity_e p, Hardware_Flow_Control_e hfc);	//init serial
	~Serial(void);	//close serial
	
	bool sOpen(const char* dev_name);	
	bool sClose(void);
	bool sSendData(float x_offset, float y_offset, float distance, unsigned char shoot_flag);
	bool sReadData(void);
};

typedef union
{
    float data_f; 
    unsigned char data_uc[4];
}Float_Uchar_u;

#endif
