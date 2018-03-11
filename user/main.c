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



/*
********************************************************************************
                                   ȫ�ֱ���
********************************************************************************
*/
uint8 UART1_reviceBuf[RTK_DATA_LEN];
uint8 UART2_reviceBuf[RTK_DATA_LEN*2];
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

uint8 rtk_flag;         //˫��λ��Ч��־1��Ч��0 FIX
uint8 rtk_data_p;        //����ָ��      

uint32 change_ms;     //RTK��A�������һ�����ݵ�ʱ��

INM_Data inm_data;//��ϵ���֡����



/*�����壬������uart1��ӦrtkA��uart2��ӦrtkB��BA����Ϊ��ͷ����BA���ͼx�����н�Ϊ0ʱ��Ӧ��������x��������Ϊ������
uart1-A uart2->B
@�� (x, y) �ڵ�һ����, 0 < �� < 90.        ��������
@�� (x, y) �ڵڶ����� 90< �ȡ�180         ��������
@�� (x, y) �ڵ�������, -180< �� < -90.     ���Ϸ���
@�� (x, y) �ڵ�������, -90 < �� < 0       ���Ϸ���
*/
byteToDouble dir_radian;   //���򣬻��ȱ�ʾ


/*
********************************************************************************
                                   ��������
********************************************************************************
*/
//�����������ǻ��ڴ��ļ��µı���������˳������
static void getRTK_GPS(void);
static void sendRTK_Data(void);
/*
********************************************************************************
                                   mian����
********************************************************************************
*/
void main()
{ 

  
  UART1_Init();
  UART2_Init();
  UART3_Init();
 // UART5_Init();
 // OutPutInit();
  
  
  UART2_dataNewFlag = 0;
  UART1_reviceFlag = 0;
  UART2_reviceFlag = 0;
  UART5_reviceFlag = 0;
 
  //��ʼ�����Ź� ι��ʱ��4s
  WDG_Config(4);
 
  
        
  while(1)
  {      
    
   #if 1 
    //A��Ϊ�ο���A���յ����ݿ�ʼ����RTK��ϵ���λ��
    if(1 == UART1_reviceFlag && UART2_dataNewFlag != 0 && 1 == UART2_reviceFlag)
    {
        IWDG_ReloadCounter();  //ι��
       
        uint16_t crc1=0;
        uint16_t crc2=0;
        uint16_t crc3=0;
        uint8_t checkFlag = 1;
        //����У��
        crc1 =  CRC16_ccitt_table(&UART1_reviceBuf[RTK_POS_LLH_P+1], 39, crc1);
        crc2 =  CRC16_ccitt_table(&UART2_reviceBuf[RTK_POS_LLH_P+1], 39, crc2);
        crc3 =  CRC16_ccitt_table(&UART2_reviceBuf[RTK_POS_LLH_P+RTK_DATA_LEN+1], 39, crc3);

      //  printf("У��ֵ1��%x %x %x %x\n",(uint8_t)crc1,(uint8_t)(crc1>>8),UART1_reviceRTK_POS_LLH_PLH_P+40],UART1_reviceRTK_POS_LLH_PLH_P+41]);
      //  printf("У��ֵ2��%x %x %x %x\n",(uint8_t)crc2,(uint8_t)(crc2>>8),UART2_reviceRTK_POS_LLH_PLH_P+40],UART2_reviceRTK_POS_LLH_PLH_P+41]);
      //  printf("У��ֵ3��%x %x %x %x\n",(uint8_t)crc3,(uint8_t)(crc3>>8),UART2_reviceRTK_POS_LLH_PLH_P+GPS_DATA_LEN+40],UART2_reviceRTK_POS_LLH_PLH_P+GPS_DATA_LEN+41]);
            
        // if(!((UART1_reviceBuf[RTK_POS_LLH_P+40] == (uint8_t)crc1) && (UART1_reviceBuf[RTK_POS_LLH_P+41] == (uint8_t)(crc1>>8))))
        if(crc1 != (UART1_reviceBuf[RTK_POS_LLH_P+40] | (UART1_reviceBuf[RTK_POS_LLH_P+41]<<8)))
        {
          checkFlag = 0;
          //��ʱ���������ݱ��ضϵ����
      //    USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);   
          Delay_Ms(13);
     //     USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);   
       //   printf("crc1У��ʧ��\n");
         
           
        }
        if(crc2 != (UART2_reviceBuf[RTK_POS_LLH_P+40] |(UART2_reviceBuf[RTK_POS_LLH_P+41]<<8)))
        {
          checkFlag = 0;
          //��ʱ���������ݱ��ضϵ����
       //   USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);   
          Delay_Ms(13);
       //   USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);   
       //   printf("crc2У��ʧ��\n");
           UART2_dataNewFlag=0;
        }
        if(crc3 != (UART2_reviceBuf[RTK_POS_LLH_P+RTK_DATA_LEN+40] | (UART2_reviceBuf[RTK_POS_LLH_P+RTK_DATA_LEN+41]<<8)))
        {
       //   printf("crc3У��ʧ��\n");
            //��ʱ���������ݱ��ضϵ����
      //    USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);   
          Delay_Ms(13);
     //     USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); 
          checkFlag = 0;
          UART2_dataNewFlag=0;
          
        }
        /*
        printf("У��ֵ��%x %x %x\n",crc1,crc2,crc3);
        
        for(int i=RTK_POS_LLH_P;i<RTK_POS_LLH_P+42;i++)
        {
          printf(" %x",UART1_reviceBuf[i]);
          
        }
        
        printf("\n");
        for(int i=RTK_POS_LLH_P;i<RTK_POS_LLH_P+42;i++)
        {
          printf(" %x",UART2_reviceBuf[i]);
          
        }
        printf("\n");
        for(int i=RTK_POS_LLH_P;i<RTK_POS_LLH_P+42;i++)
        {
          printf(" %x",UART2_reviceBuf[GPS_DATA_LEN+i]);
          
        }
        printf("\n");
*/
       
        
        if(1 == checkFlag)
        {
          //��ȡRTK����
          getRTK_GPS();
          //��ȡIMU����
          
          //תΪ֡����
          inm_data.rtk_state =  rtk_flag;
          inm_data.longitude.data_int32 = (int32_t)((gpsu_position[1].data_double*PI/180)*INM_LON_LAT_SCALE); //����
          inm_data.latitude.data_int32  = (int32_t)((gpsu_position[0].data_double*PI/180)*INM_LON_LAT_SCALE);//γ��
          
          inm_data.altitude.data_float  = (float)(gpsu_position[2].data_double); //����
          inm_data.roll.data_float = 0;   //
          inm_data.pitch.data_float =0;    //
          inm_data.yaw.data_float = (float)(dir_radian.data_double*PI/180);    
          inm_data.gps_ms.data_uint32=gpsa_time_ms.data_uint32;//��ʱ��
          inm_data.gps_weeks.data_uint16 = 0;                 //�ܺ���       
                
   //     printf("GPS_1 ʱ�䣺%d γ�ȣ�%f ���ȣ�%f ���Σ�%f ��־��%d\n",gpsa_time_ms.data_uint32,gpsa_position[0].data_double,
  //            gpsa_position[1].data_double,gpsa_position[2].data_double,rtk_flag);
        
   //     printf("GPS_2 ʱ�䣺%d γ�ȣ�%f ���ȣ�%f ���Σ�%f ��־��%d\n",gpsbn_time_ms.data_uint32,gpsbn_position[0].data_double,
   //            gpsbn_position[1].data_double,gpsbn_position[2].data_double,rtk_flag);
        
  //      printf("GPS_3 ʱ�䣺%d γ�ȣ�%f ���ȣ�%f ���Σ�%f ��־��%d\n",gpsbo_time_ms.data_uint32,gpsbo_position[0].data_double,
  //             gpsbo_position[1].data_double,gpsbo_position[2].data_double,rtk_flag);
  //     printf("GPS_4 γ�ȣ�%f ���ȣ�%f ���Σ�%f ��־��%d\n",gpsb_position[0].data_double,
  //             gpsb_position[1].data_double,gpsb_position[2].data_double,rtk_flag);
        
  //      printf("GPS_5 ����=%d γ��=%d ���Σ�%f �Ƕ�=%f ʱ��=%d ��־=%d ʱ���=%d\n",inm_data.longitude.data_int32,inm_data.latitude.data_int32,
  //              inm_data.altitude.data_float,inm_data.yaw.data_float,inm_data.gps_ms.data_uint32,inm_data.rtk_state,change_ms);
        
     //   printf("GPS_5 ��־��%d �Ƕȣ�%f γ�ȣ�%f ���ȣ�%f ���Σ�%f \n",rtk_flag,dir_radian.data_double,gpsu_position[0].data_double,
     //          gpsu_position[1].data_double,gpsu_position[2].data_double);
        
        
        sendRTK_Data();
 
        UART1_reviceFlag = 0;
        UART2_reviceFlag = 0;
        
      }else //У��ʧ��
      {
        
        UART1_reviceFlag = 0;
        UART2_reviceFlag = 0;
        uint8 err[42];
        for(int i=0;i<41;i++)
        {
            err[i] = 0xfe;
        }

      //  UART3_sendData(41,err);
        
      }
      
    }
#endif
  }

  
}


/*
********************************************************************************
void getRTKB_GPS()

������     ������ϵ���ģ������
������     ��
����ֵ��   ��
********************************************************************************
*/
static void getRTK_GPS(void){
  
  uint8 rtk_a_fix,rtk_b_fix1,rtk_b_fix2;
  
  rtk_a_fix = UART1_reviceBuf[RTK_POS_LLH_P + 39];
  rtk_b_fix1 = UART2_reviceBuf[RTK_POS_LLH_P + 39];
  rtk_b_fix2 = UART2_reviceBuf[RTK_POS_LLH_P  + RTK_DATA_LEN + 39];
  
  //�ж��Ƿ�FIX
  if((rtk_a_fix & 0x07) == 0x01 && (rtk_b_fix1 & 0x07) == 0x01 && (rtk_b_fix2 & 0x07) == 0x01)
  {
    rtk_flag = 0;
  }else{
    rtk_flag=1;
  }

//  print("fix=%x fix1=%x fix2=%x", rtk_a_fix, rtk_b_fix1, rtk_b_fix2);
  
  //ת������
  rtk_data_p = RTK_POS_LLH_P + 6; //ʱ���ַ
  
  gpsa_time_ms.data_byte[0] = UART1_reviceBuf[rtk_data_p];
  gpsa_time_ms.data_byte[1] = UART1_reviceBuf[rtk_data_p+1];
  gpsa_time_ms.data_byte[2] = UART1_reviceBuf[rtk_data_p+2];
  gpsa_time_ms.data_byte[3] = UART1_reviceBuf[rtk_data_p+3];
  
  rtk_data_p = RTK_POS_LLH_P + 10; //γ��
  for(uint8 i=0;i<8;i++)
  {
    gpsa_position[0].data_byte[i] = UART1_reviceBuf[rtk_data_p+i];
  }
  
  rtk_data_p = RTK_POS_LLH_P + 18; //����
  for(uint8 i=0;i<8;i++)
  {
    gpsa_position[1].data_byte[i] = UART1_reviceBuf[rtk_data_p+i];
  }
  
  rtk_data_p = RTK_POS_LLH_P + 26; //����
  for(uint8 i=0;i<8;i++)
  {
    gpsa_position[2].data_byte[i] = UART1_reviceBuf[rtk_data_p+i];
  }
  
  
  uint8 gps_data_flag;
  gps_data_flag = 1;
  
  if(((gpsa_position[0].data_double > 90) && (gpsa_position[0].data_double < 0))
     || ((gpsa_position[1].data_double > 180) && (gpsa_position[1].data_double < 0)))
  {
    gpsa_position[0].data_double = 0;
    gpsa_position[1].data_double = 0;
    gpsa_position[2].data_double = 0;
    gps_data_flag = 0;
  }
  
  
  if(2 == UART2_dataNewFlag)
  {
    //--------------------------������--------------------------//
    rtk_data_p = RTK_POS_LLH_P + 6;    //ʱ��
    gpsbn_time_ms.data_byte[0] = UART2_reviceBuf[rtk_data_p];
    gpsbn_time_ms.data_byte[1] = UART2_reviceBuf[rtk_data_p+1];
    gpsbn_time_ms.data_byte[2] = UART2_reviceBuf[rtk_data_p+2];
    gpsbn_time_ms.data_byte[3] = UART2_reviceBuf[rtk_data_p+3];
    
    rtk_data_p = RTK_POS_LLH_P + 10; //γ��
    for(uint8 i=0;i<8;i++)
    {
      gpsbn_position[0].data_byte[i] = UART2_reviceBuf[rtk_data_p+i];
    }
    
    rtk_data_p = RTK_POS_LLH_P + 18; //����
    for(uint8 i=0;i<8;i++)
    {
      gpsbn_position[1].data_byte[i] = UART2_reviceBuf[rtk_data_p+i];
    }
    
    rtk_data_p = RTK_POS_LLH_P + 26; //����
    for(uint8 i=0;i<8;i++)
    {
      gpsbn_position[2].data_byte[i] = UART2_reviceBuf[rtk_data_p+i];
    }
    
    //-----------------------����------------------------//
    rtk_data_p = RTK_POS_LLH_P + RTK_DATA_LEN  + 6;   //ʱ��
    gpsbo_time_ms.data_byte[0] = UART2_reviceBuf[rtk_data_p];
    gpsbo_time_ms.data_byte[1] = UART2_reviceBuf[rtk_data_p+1];
    gpsbo_time_ms.data_byte[2] = UART2_reviceBuf[rtk_data_p+2];
    gpsbo_time_ms.data_byte[3] = UART2_reviceBuf[rtk_data_p+3];
    
    rtk_data_p = RTK_POS_LLH_P + RTK_DATA_LEN + 10; //γ��
    for(uint8 i=0;i<8;i++)
    {
      gpsbo_position[0].data_byte[i] = UART2_reviceBuf[rtk_data_p+i];
    }
    
    rtk_data_p = RTK_POS_LLH_P + RTK_DATA_LEN + 18; //����
    for(uint8 i=0;i<8;i++)
    {
      gpsbo_position[1].data_byte[i] = UART2_reviceBuf[rtk_data_p+i];
    }
    
    rtk_data_p = RTK_POS_LLH_P + RTK_DATA_LEN + 26; //����
    for(uint8 i=0;i<8;i++)
    {
      gpsbo_position[2].data_byte[i] = UART2_reviceBuf[rtk_data_p+i];
    }
    
  }else
  {
    //--------------------------������--------------------------//
    rtk_data_p = RTK_POS_LLH_P + RTK_DATA_LEN + 6;   //ʱ��    
    gpsbn_time_ms.data_byte[0] = UART2_reviceBuf[rtk_data_p];
    gpsbn_time_ms.data_byte[1] = UART2_reviceBuf[rtk_data_p+1];
    gpsbn_time_ms.data_byte[2] = UART2_reviceBuf[rtk_data_p+2];
    gpsbn_time_ms.data_byte[3] = UART2_reviceBuf[rtk_data_p+3];
    
    rtk_data_p = RTK_POS_LLH_P + RTK_DATA_LEN + 10; //γ��
    for(uint8 i=0;i<8;i++)
    {
      gpsbn_position[0].data_byte[i] = UART2_reviceBuf[rtk_data_p+i];
    }
    
    rtk_data_p = RTK_POS_LLH_P + RTK_DATA_LEN + 18; //����
    for(uint8 i=0;i<8;i++)
    {
      gpsbn_position[1].data_byte[i] = UART2_reviceBuf[rtk_data_p+i];
    }
    
    rtk_data_p = RTK_POS_LLH_P + RTK_DATA_LEN + 26; //����
    for(uint8 i=0;i<8;i++)
    {
      gpsbn_position[2].data_byte[i] = UART2_reviceBuf[rtk_data_p+i];
    }
    
    //--------------------------����--------------------------//
    rtk_data_p = RTK_POS_LLH_P + 6;         //ʱ��
    gpsbo_time_ms.data_byte[0] = UART2_reviceBuf[rtk_data_p];
    gpsbo_time_ms.data_byte[1] = UART2_reviceBuf[rtk_data_p+1];
    gpsbo_time_ms.data_byte[2] = UART2_reviceBuf[rtk_data_p+2];
    gpsbo_time_ms.data_byte[3] = UART2_reviceBuf[rtk_data_p+3];
    
    rtk_data_p = RTK_POS_LLH_P + 10; //γ��
    for(uint8 i=0;i<8;i++)
    {
      gpsbo_position[0].data_byte[i] = UART2_reviceBuf[rtk_data_p+i];
    }
    
    rtk_data_p = RTK_POS_LLH_P + 18; //����
    for(uint8 i=0;i<8;i++)
    {
      gpsbo_position[1].data_byte[i] = UART2_reviceBuf[rtk_data_p+i];
    }
    
    rtk_data_p = RTK_POS_LLH_P + 26; //����
    for(uint8 i=0;i<8;i++)
    {
      gpsbo_position[2].data_byte[i] = UART2_reviceBuf[rtk_data_p+i];
    }
    
  }
  
  change_ms = gpsa_time_ms.data_uint32 - gpsbn_time_ms.data_uint32;
  
  //ʱ���
  if(change_ms>100)
  {
    gps_data_flag = 0;
   // printf("ʱ���������change_ms=%d",change_ms);
  }
 //��γ�ȳ�����Χ
  if((gpsbn_position[0].data_double > 90 && gpsbn_position[0].data_double < 0)
     || (gpsbn_position[1].data_double > 180 && gpsbn_position[1].data_double < 0))
  {
    gps_data_flag = 0;

  }
   //��γ�ȳ�����Χ
  if((gpsbo_position[0].data_double > 90 && gpsbo_position[0].data_double < 0)
     || (gpsbo_position[1].data_double > 180 && gpsbo_position[1].data_double < 0))
  {
    gps_data_flag = 0;

  }
  
  
  
  if(1 == gps_data_flag)
  {
    //����B��ǰ��γ�Ⱥ���
    gpsb_position[0].data_double = gpsbn_position[0].data_double + ((gpsbn_position[0].data_double - gpsbo_position[0].data_double)/RTK_T_TIME )*change_ms; //γ��
  //  gpsb_position[1].data_double = gpsbn_position[1].data_double + ((gpsbn_position[1].data_double*cos(gpsbn_position[0].data_double*PI/180) 
  //                                                                   - gpsbo_position[1].data_double*cos(gpsbo_position[0].data_double*PI/180))/RTK_T_TIME )*change_ms;//����     
    gpsb_position[1].data_double = gpsbn_position[1].data_double + ((gpsbn_position[1].data_double - gpsbo_position[1].data_double)/RTK_T_TIME )*change_ms; 
    gpsb_position[2].data_double = gpsbn_position[2].data_double + ((gpsbn_position[2].data_double - gpsbo_position[2].data_double)/RTK_T_TIME )*change_ms; //����
    
    //  
    //������ϵ���λ��
    for(uint8 i=0;i<3;i++)
    {
      gpsu_position[i].data_double = (gpsb_position[i].data_double + gpsa_position[i].data_double)/2;
    }
    
    //���㷽λ��Ĭ��Bָ��A������B�ں�A��ǰ��
    //ʹ��atan2
    
    double x,y;
    y = gpsa_position[0].data_double - gpsb_position[0].data_double;
    x = (gpsa_position[1].data_double-gpsb_position[1].data_double)*cos(gpsa_position[0].data_double*PI/180);
    dir_radian.data_double = atan2(y,x)*180/PI;
  }else
  {
    dir_radian.data_double = 0;
    for(uint8 i=0;i<3;i++)
    {
      gpsu_position[i].data_double = 0;
    }
  }
  
  
}

/*
********************************************************************************
                   sendRTK_Data()

������    ����RTK����
������     ��
����ֵ��   ��
********************************************************************************
*/

static void sendRTK_Data(void){
  
  
  
  uint8 sendData[50];
  uint8 i;
  
  uint8 send_p = 0;
  sendData[send_p++] = 0x55;
  //sendData[send_p++] = 0;    
/*  
  sendData[send_p++] = rtk_flag;
  sendData[send_p++] = gpsa_time_ms.data_byte[0];
  sendData[send_p++] = gpsa_time_ms.data_byte[1];
  sendData[send_p++] = gpsa_time_ms.data_byte[2];
  sendData[send_p++] = gpsa_time_ms.data_byte[3];
  
  for(uint8 j=0;j<3;j++)
  {
    for(uint8 i=0;i<8;i++)
    {
      sendData[send_p++] = gpsu_position[j].data_byte[i];
    }
  }
  for(uint8 i=0;i<8;i++)
  {
    sendData[send_p++] = dir_radian.data_byte[i];
  }
*/
  sendData[send_p++] = inm_data.rtk_state;
  //����
  for(i=0;i<4;i++)
  {
    sendData[send_p++] =inm_data.longitude.data_byte[i];
  }
  //γ��
  for(i=0;i<4;i++)
  {
    sendData[send_p++] = inm_data.latitude.data_byte[i];
  }
  //����
   for(i=0;i<4;i++)
  {
    sendData[send_p++] = inm_data.altitude.data_byte[i];
  }
  //roll;
   for(i=0;i<4;i++)
  {
    sendData[send_p++] =  inm_data.roll.data_byte[i];
  }

  // pitch;
   for(i=0;i<4;i++)
  {
    sendData[send_p++] = inm_data.pitch.data_byte[i];
  }
 //   yaw;
   for(i=0;i<4;i++)
  {
    sendData[send_p++] = inm_data.yaw.data_byte[i];
  }
  //��
  for(i=0;i<2;i++)
  {
    sendData[send_p++] = inm_data.gps_weeks.data_byte[i];
  }
  //�ܺ���
  for(i=0;i<4;i++)
  {
    sendData[send_p++] = inm_data.gps_ms.data_byte[i];
  }
   
  
  //sendData[1] = send_p-1;   //��Ч���ݳ���,����У��λ��֡ͷ
  
  int len = send_p-1;   //��Ч���ݳ���,����У��λ��֡ͷ
  
  
  uint16_t check=0;
  
  //check = CRC16_ccitt(&sendData[1], sendData[1]);
  check = CRC16_ccitt(&sendData[1], len);
  //У��λ
  sendData[send_p++] = check & 0xff;
  sendData[send_p]   = (check>>8) & 0xff;
  
  uint8_t fra=(sendData[1]+3)/20;
  
  for(int i=0;i< fra;i++)
  {
    UART3_sendData(20,&sendData[20*i]);
    Delay_Ms(3);
  }
  Delay_Ms(3);
  UART3_sendData(len+3-fra*20,&sendData[fra*20]);
}





