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
#define RTK1_REV_LEN              125     //����RTK���ݻ����С
#define RTK2_REV_LEN              250     //����RTK���ݻ����С�����顣
#define IMU_REV_LEN               64      //�������������ݻ����С
#define PI                        3.1416 //

//����union����ת��
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
	unsigned short wn;//GPS����
	unsigned int tow;//���ں�����
}GPSTime;

typedef struct
{
	RTK_State rtkState;
	double longitude;//�Ƕ���
	double latitude;//�Ƕ���
	double altitude;//��λ��
	GPSTime time;
}GPSFrame;

extern uint8 UART1_reviceBuf[RTK1_REV_LEN];
extern uint8 UART2_reviceBuf[RTK2_REV_LEN];
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