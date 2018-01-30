/**
******************************************************************************
main.c

����������STM32F107VC
�ļ�����    main.c
�̼��汾��  3.5
����Ա��    TK_CG
������      �������
******************************************************************************
*/

/*
********************************************************************************
ͷ�ļ�
********************************************************************************
*/


#include <stdio.h>
#include <math.h>
#include "app_conf.h"
#include "stm32f10x.h"
#include "delay.h"
#include "bsp_uart1.h"
#include "bsp_uart2.h"
#include "bsp_uart3.h"
#include "bsp_uart5.h"
#include "bsp_timer.h"
#include "crc_citt.h"
#include "io_output.h"


/*
********************************************************************************
��������
********************************************************************************
*/

/*
********************************************************************************
printf������ӡ�ַ���USART
********************************************************************************
*/

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch) 1
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif 

PUTCHAR_PROTOTYPE
{
  
  ITM_SendChar(ch);
  
  return ch;
  
}

/*
********************************************************************************
�궨��
********************************************************************************
*/
//RTK������������λ�����

#define GPS_POS_LLH_P      53

#define RTK_T_MS           100         //RTK������ʱ����ms



/*
********************************************************************************
ȫ�ֱ���
********************************************************************************
*/
uint8 UART1_reviceBuf[RTK1_REV_LEN];
uint8 UART2_reviceBuf[RTK2_REV_LEN];
uint8 UART5_reviceBuf[IMU_REV_LEN];

volatile uint8 UART2_dataNewFlag;

volatile uint8 UART1_reviceFlag;
volatile uint8 UART2_reviceFlag;
volatile uint8 UART5_reviceFlag;


/*
*******************************************************************************
�ֲ�����
*******************************************************************************
*/



byteToUint32 gpsa_time_ms;    //a��ʱ��
byteToUint32 gpsbo_time_ms;   //b����һ��ʱ��
byteToUint32 gpsbn_time_ms;   //b������ʱ��

byteToDouble gpsu_position[3]; //��Ϻ��gpsλ�� 0γ�� 1���� 2����
byteToDouble gpsa_position[3]; //RTK���A��gpsλ�� 0γ�� 1���� 2����
byteToDouble gpsbn_position[3]; //RTK���B���µ�gpsλ�� 0γ�� 1���� 2����
byteToDouble gpsbo_position[3]; //RTK���B��һ���gpsλ�� 0γ�� 1���� 2����

byteToDouble gpsb_position[3]; //B��������λ�� 0����1γ��2����

static uint8 rtk_flag;         //˫��λ��Ч��־0��Ч��bit0=1,RTK-A��Ч��bit1=1,RTK-B��λ��Ч
static uint16 rtk_data_p;        //����ָ��     
static uint8 gps_data_flag;            // ���ݱ�־��1���� 0���ݴ���


static int change_ms;     //RTK��A�������һ�����ݵ�ʱ��



static uint8 uart2_sp=0;

/*
@�� (x, y) �ڵ�һ����, 0 < �� < PI/2.        ��������
@�� (x, y) �ڵڶ����� PI/2 < �ȡ�PI.         ��������
@�� (x, y) �ڵ�������, -PI < �� < -PI/2.     ���Ϸ���
@�� (x, y) �ڵ�������, -PI/2 < �� < 0.       ���Ϸ���
*/
byteToDouble dir_radian;   //���򣬻��ȱ�ʾ

/*
********************************************************************************
���� ����
********************************************************************************
*/
static uint8 CheckData2(void);
static uint8 CheckData1(void);
static void sendRTKData(void);
static void getRTK_GPSA(void);
static void getRTK_GPSB(void);

/*
********************************************************************************
mian����
********************************************************************************
*/
int main(void)
{ 
  
  Delay_Ms(50);
  UART1_Init();
  UART2_Init();
  UART3_Init();
  UART5_Init();
  
  
  UART2_dataNewFlag = 0;
  UART1_reviceFlag = 0;
  UART2_reviceFlag = 0;
  UART5_reviceFlag = 0;
  
 // printf("start....");
  
  while(1)
  {
    
    //uart2����У�飬�����ݳ������ʱ����ֹ���ݱ��ض�
    if(1 == UART2_reviceFlag)
    {
      uart2_sp = UART2_dataNewFlag;  //��ǰ��������������λ��
      CheckData2();
      UART2_reviceFlag=0;
    }		
    
    //A��Ϊ�ο���A���յ����ݿ�ʼ����RTK��ϵ���λ�� ,B��������������ݺ�A���־����1

    if(1 == UART1_reviceFlag)
    {
      uint8_t checkFlag;
      uart2_sp = UART2_dataNewFlag;  //��ǰ��������������λ��
      checkFlag=1;
      //����У��
      if(0 == CheckData1())
      {
        checkFlag=0;
      }
      if(0 == CheckData2())
      {
        checkFlag=0;
      }
      
      if(checkFlag)
      {
        getRTK_GPSA();   
        getRTK_GPSB();//����A����B ��˳������       
        
        
        //     printf("GPS_1 ʱ�䣺%d γ�ȣ�%f ���ȣ�%f ���Σ�%f ��־��%d\n",gpsa_time_ms.data_uint32,gpsa_position[0].data_double,
        //           gpsa_position[1].data_double,gpsa_position[2].data_double,rtk_flag);
        
        //     printf("GPS_2 ʱ�䣺%d γ�ȣ�%f ���ȣ�%f ���Σ�%f ��־��%d\n",gpsbn_time_ms.data_uint32,gpsbn_position[0].data_double,
        //             gpsbn_position[1].data_double,gpsbn_position[2].data_double,rtk_flag);
        
        //       printf("GPS_3 ʱ�䣺%d γ�ȣ�%f ���ȣ�%f ���Σ�%f ��־��%d\n",gpsbo_time_ms.data_uint32,gpsbo_position[0].data_double,
        //             gpsbo_position[1].data_double,gpsbo_position[2].data_double,rtk_flag);
        //    printf("GPS_4 γ�ȣ�%f ���ȣ�%f ���Σ�%f ��־��%d\n",gpsb_position[0].data_double,
        //           gpsb_position[1].data_double,gpsb_position[2].data_double,rtk_flag);
        
        printf("GPS_5 ��־��%d �Ƕȣ�%f γ�ȣ�%f ���ȣ�%f ���Σ�%f \n",rtk_flag,dir_radian.data_double,gpsu_position[0].data_double,
               gpsu_position[1].data_double,gpsu_position[2].data_double);
        printf("GPS_Time ʱ��%d\n",change_ms);
        
        UART1_reviceFlag = 0;
        UART2_reviceFlag = 0;
        
        sendRTKData();
        
        
      }else //У��ʧ��
      {
        uint8 err[41];
        uint8 i;
        UART1_reviceFlag = 0;
        UART2_reviceFlag = 0;
        
        for(i=0;i<41;i++)
        {
          err[i] = 0x55;
        }
        UART3_sendData(41,err);
        
      }
      
    }
    
    
  }
  
}
/*
********************************************************************************
static void getRTK_GPSA(void)

������     ��ȡRTK A������
������     ��
����ֵ��    ��
********************************************************************************
*/
static void getRTK_GPSA(void)
{
  uint8 i;
  
  //A��ת������
  rtk_data_p = GPS_POS_LLH_P + 39; //��λ��־��ַ
  
  if((UART1_reviceBuf[rtk_data_p] & 0x07) != 0)
  {
    rtk_flag = 0x01;
  }else
  {
    rtk_flag = 0x00;
  }
  
  rtk_data_p = GPS_POS_LLH_P + 6; //ʱ���ַ
  gpsa_time_ms.data_byte[0] = UART1_reviceBuf[rtk_data_p];
  gpsa_time_ms.data_byte[1] = UART1_reviceBuf[rtk_data_p+1];
  gpsa_time_ms.data_byte[2] = UART1_reviceBuf[rtk_data_p+2];
  gpsa_time_ms.data_byte[3] = UART1_reviceBuf[rtk_data_p+3];
  
  rtk_data_p = GPS_POS_LLH_P + 10; //γ��
  
  for(i=0;i<8;i++)
  {
    gpsa_position[0].data_byte[i] = UART1_reviceBuf[rtk_data_p+i];
  }
  
  rtk_data_p = GPS_POS_LLH_P + 18; //����
  for(i=0;i<8;i++)
  {
    gpsa_position[1].data_byte[i] = UART1_reviceBuf[rtk_data_p+i];
  }
  
  rtk_data_p = GPS_POS_LLH_P + 26; //����
  for(i=0;i<8;i++)
  {
    gpsa_position[2].data_byte[i] = UART1_reviceBuf[rtk_data_p+i];
  }
  
  gps_data_flag = 1;
  
  if(((gpsa_position[0].data_double > 90) && (gpsa_position[0].data_double < 0))
     || ((gpsa_position[1].data_double > 180) && (gpsa_position[1].data_double < 0)))
  {
    gpsa_position[0].data_double = 0;
    gpsa_position[1].data_double = 0;
    gpsa_position[2].data_double = 0;
    gps_data_flag = 0;
  }
  
  
}

/*
********************************************************************************
static void getRTK_GPSB(void)

������     ת��RTK B������
������     ��
����ֵ��    ��
********************************************************************************
*/

static void getRTK_GPSB(void)
{
  uint8 i;
  
  double x,y;
  
  //B������ת��
  if(1 == uart2_sp)
  {
    //���RTK״̬
    if((UART2_reviceBuf[GPS_POS_LLH_P + 39] & 0x07) != 0 && (UART2_reviceBuf[GPS_POS_LLH_P + GPS_DATA_LEN*2 + 39] & 0x07) != 0)
    {
      rtk_flag &= ~0x02;
      rtk_flag |= 0x02;
    }else
    {
      rtk_flag &= ~0x02;
    }   
    
    //--------------------------������--------------------------//
    rtk_data_p = GPS_POS_LLH_P + 6;    //ʱ��
    gpsbn_time_ms.data_byte[0] = UART2_reviceBuf[rtk_data_p];
    gpsbn_time_ms.data_byte[1] = UART2_reviceBuf[rtk_data_p+1];
    gpsbn_time_ms.data_byte[2] = UART2_reviceBuf[rtk_data_p+2];
    gpsbn_time_ms.data_byte[3] = UART2_reviceBuf[rtk_data_p+3];
    
    rtk_data_p = GPS_POS_LLH_P + 10; //γ��
    for(i=0;i<8;i++)
    {
      gpsbn_position[0].data_byte[i] = UART2_reviceBuf[rtk_data_p+i];
    }
    
    rtk_data_p = GPS_POS_LLH_P + 18; //����
    for(i=0;i<8;i++)
    {
      gpsbn_position[1].data_byte[i] = UART2_reviceBuf[rtk_data_p+i];
    }
    
    rtk_data_p = GPS_POS_LLH_P + 26; //����
    for(i=0;i<8;i++)
    {
      gpsbn_position[2].data_byte[i] = UART2_reviceBuf[rtk_data_p+i];
    }
    
    //-----------------------����------------------------//
    rtk_data_p = GPS_POS_LLH_P + GPS_DATA_LEN*2  + 6;   //ʱ��
    gpsbo_time_ms.data_byte[0] = UART2_reviceBuf[rtk_data_p];
    gpsbo_time_ms.data_byte[1] = UART2_reviceBuf[rtk_data_p+1];
    gpsbo_time_ms.data_byte[2] = UART2_reviceBuf[rtk_data_p+2];
    gpsbo_time_ms.data_byte[3] = UART2_reviceBuf[rtk_data_p+3];
    
    rtk_data_p = GPS_POS_LLH_P + GPS_DATA_LEN*2 + 10; //γ��
    for(i=0;i<8;i++)
    {
      gpsbo_position[0].data_byte[i] = UART2_reviceBuf[rtk_data_p+i];
    }
    
    rtk_data_p = GPS_POS_LLH_P + GPS_DATA_LEN*2 + 18; //����
    for(i=0;i<8;i++)
    {
      gpsbo_position[1].data_byte[i] = UART2_reviceBuf[rtk_data_p+i];
    }
    
    rtk_data_p = GPS_POS_LLH_P + GPS_DATA_LEN*2 + 26; //����
    for(i=0;i<8;i++)
    {
      gpsbo_position[2].data_byte[i] = UART2_reviceBuf[rtk_data_p+i];
    }
    
    
    
  }else if(2 == uart2_sp)
  {
    //���RTK״̬
    if((UART2_reviceBuf[GPS_POS_LLH_P+39] & 0x07) != 0 && (UART2_reviceBuf[GPS_POS_LLH_P + GPS_DATA_LEN + 39] & 0x07) != 0)
    {
      rtk_flag &= ~0x02;
      rtk_flag |= 0x02;
    }else
    {
      rtk_flag &= ~0x02;
    }   
    
    
    //--------------------------������--------------------------//
    rtk_data_p = GPS_POS_LLH_P + GPS_DATA_LEN + 6;   //ʱ��    
    gpsbn_time_ms.data_byte[0] = UART2_reviceBuf[rtk_data_p];
    gpsbn_time_ms.data_byte[1] = UART2_reviceBuf[rtk_data_p+1];
    gpsbn_time_ms.data_byte[2] = UART2_reviceBuf[rtk_data_p+2];
    gpsbn_time_ms.data_byte[3] = UART2_reviceBuf[rtk_data_p+3];
    
    rtk_data_p = GPS_POS_LLH_P + GPS_DATA_LEN + 10; //γ��
    for(i=0;i<8;i++)
    {
      gpsbn_position[0].data_byte[i] = UART2_reviceBuf[rtk_data_p+i];
    }
    
    rtk_data_p = GPS_POS_LLH_P + GPS_DATA_LEN + 18; //����
    for(i=0;i<8;i++)
    {
      gpsbn_position[1].data_byte[i] = UART2_reviceBuf[rtk_data_p+i];
    }
    
    rtk_data_p = GPS_POS_LLH_P + GPS_DATA_LEN + 26; //����
    for(i=0;i<8;i++)
    {
      gpsbn_position[2].data_byte[i] = UART2_reviceBuf[rtk_data_p+i];
    }
    
    //--------------------------����--------------------------//
    rtk_data_p = GPS_POS_LLH_P + 6;         //ʱ��
    gpsbo_time_ms.data_byte[0] = UART2_reviceBuf[rtk_data_p];
    gpsbo_time_ms.data_byte[1] = UART2_reviceBuf[rtk_data_p+1];
    gpsbo_time_ms.data_byte[2] = UART2_reviceBuf[rtk_data_p+2];
    gpsbo_time_ms.data_byte[3] = UART2_reviceBuf[rtk_data_p+3];
    
    rtk_data_p = GPS_POS_LLH_P + 10; //γ��
    for(i=0;i<8;i++)
    {
      gpsbo_position[0].data_byte[i] = UART2_reviceBuf[rtk_data_p+i];
    }
    
    rtk_data_p = GPS_POS_LLH_P + 18; //����
    for(i=0;i<8;i++)
    {
      gpsbo_position[1].data_byte[i] = UART2_reviceBuf[rtk_data_p+i];
    }
    
    rtk_data_p = GPS_POS_LLH_P + 26; //����
    for(i=0;i<8;i++)
    {
      gpsbo_position[2].data_byte[i] = UART2_reviceBuf[rtk_data_p+i];
    }
    
  }else if(3 == uart2_sp)
  {
    //���RTK״̬
    if((UART2_reviceBuf[GPS_POS_LLH_P + GPS_DATA_LEN +39] & 0x07) != 0 && (UART2_reviceBuf[GPS_POS_LLH_P + GPS_DATA_LEN*2 + 39] & 0x07) != 0)
    {
      rtk_flag &= ~0x02;
      rtk_flag |= 0x02;
    }else
    {
      rtk_flag &= ~0x02;
    }   
    
    //--------------------------������--------------------------//
    rtk_data_p = GPS_POS_LLH_P + GPS_DATA_LEN*2 + 6;   //ʱ��    
    
    gpsbn_time_ms.data_byte[0] = UART2_reviceBuf[rtk_data_p];
    gpsbn_time_ms.data_byte[1] = UART2_reviceBuf[rtk_data_p+1];
    gpsbn_time_ms.data_byte[2] = UART2_reviceBuf[rtk_data_p+2];
    gpsbn_time_ms.data_byte[3] = UART2_reviceBuf[rtk_data_p+3];
    
    rtk_data_p = GPS_POS_LLH_P + GPS_DATA_LEN*2 + 10; //γ��
    for(i=0;i<8;i++)
    {
      gpsbn_position[0].data_byte[i] = UART2_reviceBuf[rtk_data_p+i];
    }
    
    rtk_data_p = GPS_POS_LLH_P + GPS_DATA_LEN*2 + 18; //����
    for(i=0;i<8;i++)
    {
      gpsbn_position[1].data_byte[i] = UART2_reviceBuf[rtk_data_p+i];
    }
    
    rtk_data_p = GPS_POS_LLH_P + GPS_DATA_LEN*2 + 26; //����
    for(i=0;i<8;i++)
    {
      gpsbn_position[2].data_byte[i] = UART2_reviceBuf[rtk_data_p+i];
    }
    
    //--------------------------����--------------------------//
    rtk_data_p = GPS_POS_LLH_P  + GPS_DATA_LEN + 6;         //ʱ��
    gpsbo_time_ms.data_byte[0] = UART2_reviceBuf[rtk_data_p];
    gpsbo_time_ms.data_byte[1] = UART2_reviceBuf[rtk_data_p+1];
    gpsbo_time_ms.data_byte[2] = UART2_reviceBuf[rtk_data_p+2];
    gpsbo_time_ms.data_byte[3] = UART2_reviceBuf[rtk_data_p+3];
    
    rtk_data_p = GPS_POS_LLH_P  + GPS_DATA_LEN  + 10; //γ��
    for(i=0;i<8;i++)
    {
      gpsbo_position[0].data_byte[i] = UART2_reviceBuf[rtk_data_p+i];
    }
    
    rtk_data_p = GPS_POS_LLH_P  + GPS_DATA_LEN  + 18; //����
    for(i=0;i<8;i++)
    {
      gpsbo_position[1].data_byte[i] = UART2_reviceBuf[rtk_data_p+i];
    }
    
    rtk_data_p = GPS_POS_LLH_P  + GPS_DATA_LEN  + 26; //����
    for(i=0;i<8;i++)
    {
      gpsbo_position[2].data_byte[i] = UART2_reviceBuf[rtk_data_p+i];
    }
    
  }
  
  //����ʱ���
  change_ms = gpsa_time_ms.data_uint32 - gpsbn_time_ms.data_uint32;
  
  if((gpsbn_position[0].data_double > 90 && gpsbn_position[0].data_double < 0)
     || (gpsbn_position[1].data_double > 180 && gpsbn_position[1].data_double < 0))
  {
    gpsbn_position[0].data_double = 0;
    gpsbn_position[1].data_double = 0;
    gpsbn_position[2].data_double = 0;
    gps_data_flag = 0;
    change_ms = 0;
    
  }
  
  if((gpsbo_position[0].data_double > 90 && gpsbo_position[0].data_double < 0)
     || (gpsbo_position[1].data_double > 180 && gpsbo_position[1].data_double < 0))
  {
    gpsbo_position[0].data_double = 0;
    gpsbo_position[1].data_double = 0;
    gpsbo_position[2].data_double = 0;
    gps_data_flag = 0;
    change_ms = 0;
  }
  
  if(change_ms > 100 || change_ms < 0)
  {
    gps_data_flag=0;
  }
  
  
  if(1 == gps_data_flag)
  {
    //����B��ǰ��γ�Ⱥ��μ���
    gpsb_position[0].data_double = gpsbn_position[0].data_double + ((gpsbn_position[0].data_double - gpsbo_position[0].data_double)/RTK_T_MS)*change_ms; //γ��
    gpsb_position[1].data_double = gpsbn_position[1].data_double + ((gpsbn_position[1].data_double*cos(gpsbn_position[0].data_double*PI/180) 
                                                                     - gpsbo_position[1].data_double*cos(gpsbo_position[0].data_double*PI/180))/RTK_T_MS)*change_ms;                    //����    
    gpsb_position[2].data_double = gpsbn_position[2].data_double + ((gpsbn_position[2].data_double - gpsbo_position[2].data_double)/RTK_T_MS)*change_ms; //����
    
    //  
    //������ϵ���λ��
    for(i=0;i<3;i++)
    {
      gpsu_position[i].data_double = (gpsb_position[i].data_double + gpsa_position[i].data_double)/2;
    }
    
    //���㷽λ��Ĭ��Aָ��B������A�ں�B��ǰ��Ŀǰֻ�����ڱ�����
    //ʹ��atan2
    
    y = gpsb_position[0].data_double - gpsa_position[0].data_double;
    x = gpsb_position[1].data_double*cos(gpsb_position[0].data_double*PI/180) - gpsa_position[1].data_double*cos(gpsa_position[0].data_double*PI/180);
    dir_radian.data_double = atan2(y,x)*180/PI;
  }else
  {
    dir_radian.data_double = 0;
    for(i=0;i<3;i++)
    {
      gpsu_position[i].data_double = 0;
    }
  }
}

/*
********************************************************************************
static uint8 CheckData2(void)

������     У��RTK����,A��
������     ��
����ֵ��    ��
********************************************************************************
*/
static uint8 CheckData1(void)
{
  uint8 flag;
  uint16_t crc1;
  flag=1;
  crc1=0;
  crc1 =  crc16_ccitt_table(&UART1_reviceBuf[GPS_POS_LLH_P+1], 39, crc1);
  
  if(crc1 != (UART1_reviceBuf[GPS_POS_LLH_P+40] | (UART1_reviceBuf[GPS_POS_LLH_P+41]<<8)))
  {
    flag = 0;
    //��ʱ���������ݱ��ضϵ����  
    Delay_Ms(15);
 //   printf("crcAУ��ʧ��\n");
  }else
  {
     printf("crcAУ��ɹ�\n");
  }
  
  //���������ж�
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);  
  
  return flag;
  
}	
/*
********************************************************************************
static uint8 CheckData2(void)

������     У��RTK���� ��B��
������     ��
����ֵ��    ��
********************************************************************************
*/
static uint8 CheckData2(void)
{
  uint16 crc2;
 // int i;
  uint8 flag;
  flag=1;
  crc2=0;
  
  //����У��
  if(1 ==  uart2_sp)
  {
    crc2 =  crc16_ccitt_table(&UART2_reviceBuf[GPS_POS_LLH_P+1], 39, crc2);
    if(crc2 != (UART2_reviceBuf[GPS_POS_LLH_P+40] |(UART2_reviceBuf[GPS_POS_LLH_P+41]<<8)))
    {
      //��ʱ���������ݱ��ضϵ����
      Delay_Ms(15);
      flag=0;
      /*
      //��ӡ���յ�������
      for(i=GPS_POS_LLH_P;i<GPS_POS_LLH_P+42;i++)
      {
        printf(" %x",UART2_reviceBuf[GPS_DATA_LEN+i]);
        
      }
      printf("\n");
      printf("У��ֵ1��%x %x\n",(uint8_t)crc2,(uint8_t)(crc2>>8));
      */
      
    }else
    {
    //   printf("crc1У��ɹ�\n");
    }    
    
  }else if(2 ==  uart2_sp)
  {
    crc2 =  crc16_ccitt_table(&UART2_reviceBuf[GPS_POS_LLH_P+GPS_DATA_LEN+1], 39, crc2);
    if(crc2 != (UART2_reviceBuf[GPS_POS_LLH_P+GPS_DATA_LEN+40] |(UART2_reviceBuf[GPS_POS_LLH_P+GPS_DATA_LEN+41]<<8)))
    {
      //��ʱ���������ݱ��ضϵ����
      Delay_Ms(15);   
      flag=0;
      /*
      //��ӡ���յ�������
      for(i=GPS_POS_LLH_P+GPS_DATA_LEN;i<GPS_POS_LLH_P+GPS_DATA_LEN+42;i++)
      {
        printf(" %x",UART2_reviceBuf[GPS_DATA_LEN+i]);
        
      }
      printf("\n");
      printf("У��ֵ2��%x %x\n",(uint8_t)crc2,(uint8_t)(crc2>>8));
*/
    }else
    {
 //      printf("crc2У��ɹ�\n");
    }
    
   
    
  }else if(3 ==  uart2_sp)
  {
    crc2 =  crc16_ccitt_table(&UART2_reviceBuf[GPS_POS_LLH_P+GPS_DATA_LEN*2+1], 39, crc2);
    
    if(crc2 != (UART2_reviceBuf[GPS_POS_LLH_P+GPS_DATA_LEN*2+40] |(UART2_reviceBuf[GPS_POS_LLH_P+GPS_DATA_LEN*2+41]<<8)))
    {
      //��ʱ���������ݱ��ضϵ����
      Delay_Ms(15);     
      flag=0;
      
      /*
      //��ӡ���յ�������
      for(i=GPS_POS_LLH_P+GPS_DATA_LEN*2;i<GPS_POS_LLH_P+GPS_DATA_LEN*2+42;i++)
      {
        printf(" %x",UART2_reviceBuf[GPS_DATA_LEN+i]);
        
      }
      printf("\n");
      printf("У��ֵ3��%x %x\n",(uint8_t)crc2,(uint8_t)(crc2>>8));
*/
    }else
    {
  //     printf("crc3У��ɹ�\n");
    }
    
    
  }
  
  
  //���������ж�
  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);  	
  
  return flag;
  
}	

/*
********************************************************************************
static void sendRTKData(void)

������     ����RTK���ݵ�����
������     ��
����ֵ��    ��
********************************************************************************
*/
static void sendRTKData(void)
{
  uint8 sendData[50];
  uint8 i,j,fra,send_p;
  uint16_t check;
  
  send_p = 0;
  sendData[send_p++] = 0x55;
  sendData[send_p++] = 0;       
  sendData[send_p++] = rtk_flag;
  sendData[send_p++] = gpsa_time_ms.data_byte[0];
  sendData[send_p++] = gpsa_time_ms.data_byte[1];
  sendData[send_p++] = gpsa_time_ms.data_byte[2];
  sendData[send_p++] = gpsa_time_ms.data_byte[3];
  
  
  for(j=0;j<3;j++)
  {
    for(i=0;i<8;i++)
    {
      sendData[send_p++] = gpsu_position[j].data_byte[i];
    }
  }
  for(i=0;i<8;i++)
  {
    sendData[send_p++] = dir_radian.data_byte[i];
  }
  sendData[1] = send_p-1;   //��Ч���ݳ���,����У��λ��֡ͷ
  
  check=0;
  check = crc16_ccitt(&sendData[1], sendData[1]);
  
  //У��λ
  sendData[send_p++] = check & 0xff;
  sendData[send_p]   = (check>>8) & 0xff;
  
  
  //    UART3_sendData(sendData[1]+3,sendData);
  
  fra=(sendData[1]+3)/20;
  
  for(i=0;i< fra;i++)
  {
    UART3_sendData(20,&sendData[20*i]);
    Delay_Ms(3);
  }
  Delay_Ms(3);
  UART3_sendData(sendData[1]+3-fra*20,&sendData[fra*20]);
 // printf("֡����%d\n",sendData[1]+3);
}







