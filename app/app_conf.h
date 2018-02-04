/*
********************************************************************************
                                计时器
文件名：app_conf.h
版本：
程序员：Tank_CG
********************************************************************************
说明：
********************************************************************************
*/

#ifndef  __APP_CONF_H
#define  __APP_CONF_H


/*
********************************************************************************
                               头文件        
********************************************************************************
*/
#include "stm32f10x.h"
#include "stm32f10x_iwdg.h"

/*
********************************************************************************
                               函数申明       
********************************************************************************
*/

/*
********************************************************************************
                                    数据类型声明
********************************************************************************
*/

#ifndef uint8
typedef unsigned char uint8;
#endif

#ifndef uint16
typedef unsigned short int uint16;
#endif

#ifndef uint32
typedef unsigned  int uint32;
#endif







/*
********************************************************************************
                                    宏定义
********************************************************************************
*/

/*
@ 两个RTK传输数据有时差，所以选RTK1作为参考，RTK2与RTK1同一时间的位置通过推算
*/
#define RTK1_REV_LEN              125     //接收RTK数据缓存大小
#define RTK2_REV_LEN              250     //接收RTK数据缓存大小，两组。
#define IMU_REV_LEN               64      //接收陀螺仪数据缓存大小
#define PI                        3.1416 //

//利用union特性转换
typedef union{
    
    double data_double;
    uint8   data_byte[8];
 }byteToDouble;

typedef union{
    
    uint32  data_uint32;
    uint8   data_byte[4];
 }byteToUint32;

typedef enum 
{
	RTK_SINGLE,
	RTK_FLOAT,
	RTK_FIX,
	RTK_RECKON
}RTK_State;

typedef struct
{
	unsigned short wn;//GPS周数
	unsigned int tow;//周内毫秒数
}GPSTime;

typedef struct
{
	RTK_State rtkState;
	double longitude;//角度制
	double latitude;//角度制
	double altitude;//单位米
	GPSTime time;
}GPSFrame;

extern uint8 UART1_reviceBuf[RTK1_REV_LEN];
extern uint8 UART2_reviceBuf[RTK2_REV_LEN];
extern uint8 UART5_reviceBuf[IMU_REV_LEN];

extern volatile uint8 UART2_dataNewFlag;//需要保存两组数据，0表示还没接收数据，1表示第一组是最新数据，2表示第二组是最新数据

extern volatile uint8 UART1_reviceFlag;
extern volatile uint8 UART2_reviceFlag;
extern volatile uint8 UART5_reviceFlag;



/*
********************************************************************************

                               函数声明
********************************************************************************
*/

void WDG_Config(uint8_t WDG_time);

#endif 