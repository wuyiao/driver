#ifndef __gsv2001_core_h
#define __gsv2001_core_h

#include "av_config.h"
#include "av_main.h"
#include "uapi.h"

//i2c select 
#define I2C_MODE0  0X0
#define I2C_MODE1  0X1
#define I2C_MODE2  0X2
#define I2C_MODE3  0X3

#define MAX_GSV2015_CLIENT	8 

typedef struct _gsv2015_core_dev {
        struct i2c_client *client;
}gsv2015_core_dev;

typedef struct tagVIDINFO 
{
    int iWidth;
    int iHeight;
    int iInterlaced;
    int iFps;
	int input_color_cpace;
	int output_mode;
    unsigned int mode;
	unsigned char audio_frequency;
	unsigned char have_audio;
	
}VIDINFO,*PVIDINFO;

//GSV2001 Command
#define GSV2001_CMD		0x2001
#define GET_RXA_INPUT_5V_STATUS		(GSV2001_CMD+0)

AvRet ManI2cRead(uint32 devAddress, uint32 regAddress, uint8 *i2cdata, uint16 count, uint8 index, uint8 Flag16bit);
AvRet ManI2cWrite(uint32 devAddress, uint32 regAddress, uint8 *i2cdata, uint16 count, uint8 index, uint8 Flag16bit);

extern gsv2015_core_dev g_gsv2015_dev;
extern struct i2c_client *gsv2015_client[MAX_GSV2015_CLIENT];

extern void select_i2c_mode(int mode);

#endif
