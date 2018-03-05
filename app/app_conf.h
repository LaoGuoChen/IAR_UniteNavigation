/*
********************************************************************************
                                ��ʱ��
�ļ�����app_conf.h
�汾��
����Ա��Tank_CG
********************************************************************************
˵����
********************************************************************************
*/

#ifndef  __APP_CONF_H
#define  __APP_CONF_H


/*
********************************************************************************
                               ͷ�ļ�        
********************************************************************************
*/
#include "stm32f10x.h"
#include "stm32f10x_iwdg.h"

/*
********************************************************************************
                               ��������       
********************************************************************************
*/

/*
********************************************************************************
                                    ������������
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
                                    �궨��
********************************************************************************
*/

/*
@ ����RTK����������ʱ�����ѡRTK1��Ϊ�ο���RTK2��RTK1ͬһʱ���λ��ͨ������

*/

#define   RTK_DATA_LEN              125     //����RTK���ݻ����С
#define   IMU_REV_LEN               64      //�������������ݻ����С
#define   PI                        3.14159265358 //
#define  INM_LON_LAT_SCALE          100000000.0 //��γ��ת���ɶ���С���ı���ϵ��

#define RTK_POS_LLH_P      53
#define RTK_T_TIME         100 //RTK������ʱ����

//����union����ת��
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
  byteToInt32   longitude;//����С����ʽ��������*INM_LON_LAT_SCALE
  byteToInt32   latitude;//����С����ʽ��������*INM_LON_LAT_SCALE
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

extern volatile uint8 UART2_dataNewFlag;//��Ҫ�����������ݣ�0��ʾ��û�������ݣ�1��ʾ��һ�����������ݣ�2��ʾ�ڶ�������������

extern volatile uint8 UART1_reviceFlag;
extern volatile uint8 UART2_reviceFlag;
extern volatile uint8 UART5_reviceFlag;



/*
********************************************************************************

                               ��������
********************************************************************************
*/

void WDG_Config(uint8_t WDG_time);

#endif 