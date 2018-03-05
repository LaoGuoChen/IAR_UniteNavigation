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

#define   RTK_DATA_LEN              125     //接收RTK数据缓存大小
#define   IMU_REV_LEN               64      //接收陀螺仪数据缓存大小
#define   PI                        3.14159265358 //
#define  INM_LON_LAT_SCALE          100000000.0 //经纬度转换成定点小数的比例系数

#define RTK_POS_LLH_P      53
#define RTK_T_TIME         100 //RTK发数据时间间隔

//利用union特性转换
typedef union{
    
    double data_double;
    uint8   data_byte[8];
 }byteToDouble;

typedef union{
    
    float   data_float;
    uint8   data_byte[4];
 }byteToFloat;

 typedef union{
    
    uint16_t  data_uint16;
    uint8     data_byte[2];
   
 }byteToUint16;

typedef union{
    
    int32_t  data_int32;
    uint8    data_byte[4];
 }byteToInt32;

typedef union{
    
    uint32_t  data_uint32;
    uint8    data_byte[4];
 }byteToUint32;



typedef struct
{
  uint8_t       rtk_state;
  byteToInt32   longitude;//定点小数形式：毫弧度*INM_LON_LAT_SCALE
  byteToInt32   latitude;//定点小数形式：毫弧度*INM_LON_LAT_SCALE
  byteToFloat   altitude;
  byteToFloat   roll;
  byteToFloat   pitch;
  byteToFloat   yaw;
  byteToUint16  gps_weeks;
  byteToUint32  gps_ms;
  
}INM_Data;




extern uint8 UART1_reviceBuf[RTK_DATA_LEN];
extern uint8 UART2_reviceBuf[RTK_DATA_LEN*2];
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