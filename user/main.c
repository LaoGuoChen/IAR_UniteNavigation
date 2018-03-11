/**
******************************************************************************
                              main.c

                        开发环境：STM32F107VC
文件名：    main.c
固件版本：  3.5
程序员：    TK_CG
描述：      程序入口
******************************************************************************
*/

/*
********************************************************************************
                                   头文件
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
                                  变量定义
********************************************************************************
*/

/*
********************************************************************************
                                   printf（）打印字符到USART
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
                                    宏定义
********************************************************************************
*/



/*
********************************************************************************
                                   全局变量
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
                                 局部变量
*******************************************************************************
*/



byteToUint32 gpsa_time_ms;    //a组时间
byteToUint32 gpsbo_time_ms;   //b组上一组时间
byteToUint32 gpsbn_time_ms;   //b组最新时间

byteToDouble gpsu_position[3]; //组合后的gps位置 0纬度 1经度 2海拔
byteToDouble gpsa_position[3]; //RTK编号A的gps位置 0纬度 1经度 2海拔
byteToDouble gpsbn_position[3]; //RTK编号B最新的gps位置 0纬度 1经度 2海拔
byteToDouble gpsbo_position[3]; //RTK编号B上一组的gps位置 0纬度 1经度 2海拔

byteToDouble gpsb_position[3]; //B编号推算的位置 0经度1纬度2海拔

uint8 rtk_flag;         //双定位有效标志1无效，0 FIX
uint8 rtk_data_p;        //数据指针      

uint32 change_ms;     //RTK的A组离最近一组数据的时间

INM_Data inm_data;//组合导航帧数据



/*方向定义，板子上uart1对应rtkA，uart2对应rtkB，BA向量为车头方向，BA与地图x正方夹角为0时对应正东方（x周正方向为东方）
uart1-A uart2->B
@当 (x, y) 在第一象限, 0 < θ < 90.        东北方向
@当 (x, y) 在第二象限 90< θ≤180         西北方向
@当 (x, y) 在第三象限, -180< θ < -90.     西南方向
@当 (x, y) 在第四象限, -90 < θ < 0       东南方向
*/
byteToDouble dir_radian;   //方向，弧度表示


/*
********************************************************************************
                                   函数申明
********************************************************************************
*/
//函数变量都是基于此文件下的变量，调用顺序不能乱
static void getRTK_GPS(void);
static void sendRTK_Data(void);
/*
********************************************************************************
                                   mian函数
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
 
  //初始化看门狗 喂狗时间4s
  WDG_Config(4);
 
  
        
  while(1)
  {      
    
   #if 1 
    //A组为参考，A接收到数据开始计算RTK组合导航位置
    if(1 == UART1_reviceFlag && UART2_dataNewFlag != 0 && 1 == UART2_reviceFlag)
    {
        IWDG_ReloadCounter();  //喂狗
       
        uint16_t crc1=0;
        uint16_t crc2=0;
        uint16_t crc3=0;
        uint8_t checkFlag = 1;
        //数据校验
        crc1 =  CRC16_ccitt_table(&UART1_reviceBuf[RTK_POS_LLH_P+1], 39, crc1);
        crc2 =  CRC16_ccitt_table(&UART2_reviceBuf[RTK_POS_LLH_P+1], 39, crc2);
        crc3 =  CRC16_ccitt_table(&UART2_reviceBuf[RTK_POS_LLH_P+RTK_DATA_LEN+1], 39, crc3);

      //  printf("校验值1：%x %x %x %x\n",(uint8_t)crc1,(uint8_t)(crc1>>8),UART1_reviceRTK_POS_LLH_PLH_P+40],UART1_reviceRTK_POS_LLH_PLH_P+41]);
      //  printf("校验值2：%x %x %x %x\n",(uint8_t)crc2,(uint8_t)(crc2>>8),UART2_reviceRTK_POS_LLH_PLH_P+40],UART2_reviceRTK_POS_LLH_PLH_P+41]);
      //  printf("校验值3：%x %x %x %x\n",(uint8_t)crc3,(uint8_t)(crc3>>8),UART2_reviceRTK_POS_LLH_PLH_P+GPS_DATA_LEN+40],UART2_reviceRTK_POS_LLH_PLH_P+GPS_DATA_LEN+41]);
            
        // if(!((UART1_reviceBuf[RTK_POS_LLH_P+40] == (uint8_t)crc1) && (UART1_reviceBuf[RTK_POS_LLH_P+41] == (uint8_t)(crc1>>8))))
        if(crc1 != (UART1_reviceBuf[RTK_POS_LLH_P+40] | (UART1_reviceBuf[RTK_POS_LLH_P+41]<<8)))
        {
          checkFlag = 0;
          //延时，处理数据被截断的情况
      //    USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);   
          Delay_Ms(13);
     //     USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);   
       //   printf("crc1校验失败\n");
         
           
        }
        if(crc2 != (UART2_reviceBuf[RTK_POS_LLH_P+40] |(UART2_reviceBuf[RTK_POS_LLH_P+41]<<8)))
        {
          checkFlag = 0;
          //延时，处理数据被截断的情况
       //   USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);   
          Delay_Ms(13);
       //   USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);   
       //   printf("crc2校验失败\n");
           UART2_dataNewFlag=0;
        }
        if(crc3 != (UART2_reviceBuf[RTK_POS_LLH_P+RTK_DATA_LEN+40] | (UART2_reviceBuf[RTK_POS_LLH_P+RTK_DATA_LEN+41]<<8)))
        {
       //   printf("crc3校验失败\n");
            //延时，处理数据被截断的情况
      //    USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);   
          Delay_Ms(13);
     //     USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); 
          checkFlag = 0;
          UART2_dataNewFlag=0;
          
        }
        /*
        printf("校验值：%x %x %x\n",crc1,crc2,crc3);
        
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
          //获取RTK数据
          getRTK_GPS();
          //获取IMU数据
          
          //转为帧数据
          inm_data.rtk_state =  rtk_flag;
          inm_data.longitude.data_int32 = (int32_t)((gpsu_position[1].data_double*PI/180)*INM_LON_LAT_SCALE); //经度
          inm_data.latitude.data_int32  = (int32_t)((gpsu_position[0].data_double*PI/180)*INM_LON_LAT_SCALE);//纬度
          
          inm_data.altitude.data_float  = (float)(gpsu_position[2].data_double); //海拔
          inm_data.roll.data_float = 0;   //
          inm_data.pitch.data_float =0;    //
          inm_data.yaw.data_float = (float)(dir_radian.data_double*PI/180);    
          inm_data.gps_ms.data_uint32=gpsa_time_ms.data_uint32;//周时间
          inm_data.gps_weeks.data_uint16 = 0;                 //周毫秒       
                
   //     printf("GPS_1 时间：%d 纬度：%f 经度：%f 海拔：%f 标志：%d\n",gpsa_time_ms.data_uint32,gpsa_position[0].data_double,
  //            gpsa_position[1].data_double,gpsa_position[2].data_double,rtk_flag);
        
   //     printf("GPS_2 时间：%d 纬度：%f 经度：%f 海拔：%f 标志：%d\n",gpsbn_time_ms.data_uint32,gpsbn_position[0].data_double,
   //            gpsbn_position[1].data_double,gpsbn_position[2].data_double,rtk_flag);
        
  //      printf("GPS_3 时间：%d 纬度：%f 经度：%f 海拔：%f 标志：%d\n",gpsbo_time_ms.data_uint32,gpsbo_position[0].data_double,
  //             gpsbo_position[1].data_double,gpsbo_position[2].data_double,rtk_flag);
  //     printf("GPS_4 纬度：%f 经度：%f 海拔：%f 标志：%d\n",gpsb_position[0].data_double,
  //             gpsb_position[1].data_double,gpsb_position[2].data_double,rtk_flag);
        
  //      printf("GPS_5 经度=%d 纬度=%d 海拔：%f 角度=%f 时间=%d 标志=%d 时间差=%d\n",inm_data.longitude.data_int32,inm_data.latitude.data_int32,
  //              inm_data.altitude.data_float,inm_data.yaw.data_float,inm_data.gps_ms.data_uint32,inm_data.rtk_state,change_ms);
        
     //   printf("GPS_5 标志：%d 角度：%f 纬度：%f 经度：%f 海拔：%f \n",rtk_flag,dir_radian.data_double,gpsu_position[0].data_double,
     //          gpsu_position[1].data_double,gpsu_position[2].data_double);
        
        
        sendRTK_Data();
 
        UART1_reviceFlag = 0;
        UART2_reviceFlag = 0;
        
      }else //校验失败
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

描述：     计算组合导航模块数据
参数：     无
返回值：   无
********************************************************************************
*/
static void getRTK_GPS(void){
  
  uint8 rtk_a_fix,rtk_b_fix1,rtk_b_fix2;
  
  rtk_a_fix = UART1_reviceBuf[RTK_POS_LLH_P + 39];
  rtk_b_fix1 = UART2_reviceBuf[RTK_POS_LLH_P + 39];
  rtk_b_fix2 = UART2_reviceBuf[RTK_POS_LLH_P  + RTK_DATA_LEN + 39];
  
  //判断是否FIX
  if((rtk_a_fix & 0x07) == 0x01 && (rtk_b_fix1 & 0x07) == 0x01 && (rtk_b_fix2 & 0x07) == 0x01)
  {
    rtk_flag = 0;
  }else{
    rtk_flag=1;
  }

//  print("fix=%x fix1=%x fix2=%x", rtk_a_fix, rtk_b_fix1, rtk_b_fix2);
  
  //转化数据
  rtk_data_p = RTK_POS_LLH_P + 6; //时间地址
  
  gpsa_time_ms.data_byte[0] = UART1_reviceBuf[rtk_data_p];
  gpsa_time_ms.data_byte[1] = UART1_reviceBuf[rtk_data_p+1];
  gpsa_time_ms.data_byte[2] = UART1_reviceBuf[rtk_data_p+2];
  gpsa_time_ms.data_byte[3] = UART1_reviceBuf[rtk_data_p+3];
  
  rtk_data_p = RTK_POS_LLH_P + 10; //纬度
  for(uint8 i=0;i<8;i++)
  {
    gpsa_position[0].data_byte[i] = UART1_reviceBuf[rtk_data_p+i];
  }
  
  rtk_data_p = RTK_POS_LLH_P + 18; //经度
  for(uint8 i=0;i<8;i++)
  {
    gpsa_position[1].data_byte[i] = UART1_reviceBuf[rtk_data_p+i];
  }
  
  rtk_data_p = RTK_POS_LLH_P + 26; //海拔
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
    //--------------------------最新组--------------------------//
    rtk_data_p = RTK_POS_LLH_P + 6;    //时间
    gpsbn_time_ms.data_byte[0] = UART2_reviceBuf[rtk_data_p];
    gpsbn_time_ms.data_byte[1] = UART2_reviceBuf[rtk_data_p+1];
    gpsbn_time_ms.data_byte[2] = UART2_reviceBuf[rtk_data_p+2];
    gpsbn_time_ms.data_byte[3] = UART2_reviceBuf[rtk_data_p+3];
    
    rtk_data_p = RTK_POS_LLH_P + 10; //纬度
    for(uint8 i=0;i<8;i++)
    {
      gpsbn_position[0].data_byte[i] = UART2_reviceBuf[rtk_data_p+i];
    }
    
    rtk_data_p = RTK_POS_LLH_P + 18; //经度
    for(uint8 i=0;i<8;i++)
    {
      gpsbn_position[1].data_byte[i] = UART2_reviceBuf[rtk_data_p+i];
    }
    
    rtk_data_p = RTK_POS_LLH_P + 26; //海拔
    for(uint8 i=0;i<8;i++)
    {
      gpsbn_position[2].data_byte[i] = UART2_reviceBuf[rtk_data_p+i];
    }
    
    //-----------------------旧组------------------------//
    rtk_data_p = RTK_POS_LLH_P + RTK_DATA_LEN  + 6;   //时间
    gpsbo_time_ms.data_byte[0] = UART2_reviceBuf[rtk_data_p];
    gpsbo_time_ms.data_byte[1] = UART2_reviceBuf[rtk_data_p+1];
    gpsbo_time_ms.data_byte[2] = UART2_reviceBuf[rtk_data_p+2];
    gpsbo_time_ms.data_byte[3] = UART2_reviceBuf[rtk_data_p+3];
    
    rtk_data_p = RTK_POS_LLH_P + RTK_DATA_LEN + 10; //纬度
    for(uint8 i=0;i<8;i++)
    {
      gpsbo_position[0].data_byte[i] = UART2_reviceBuf[rtk_data_p+i];
    }
    
    rtk_data_p = RTK_POS_LLH_P + RTK_DATA_LEN + 18; //经度
    for(uint8 i=0;i<8;i++)
    {
      gpsbo_position[1].data_byte[i] = UART2_reviceBuf[rtk_data_p+i];
    }
    
    rtk_data_p = RTK_POS_LLH_P + RTK_DATA_LEN + 26; //海拔
    for(uint8 i=0;i<8;i++)
    {
      gpsbo_position[2].data_byte[i] = UART2_reviceBuf[rtk_data_p+i];
    }
    
  }else
  {
    //--------------------------最新组--------------------------//
    rtk_data_p = RTK_POS_LLH_P + RTK_DATA_LEN + 6;   //时间    
    gpsbn_time_ms.data_byte[0] = UART2_reviceBuf[rtk_data_p];
    gpsbn_time_ms.data_byte[1] = UART2_reviceBuf[rtk_data_p+1];
    gpsbn_time_ms.data_byte[2] = UART2_reviceBuf[rtk_data_p+2];
    gpsbn_time_ms.data_byte[3] = UART2_reviceBuf[rtk_data_p+3];
    
    rtk_data_p = RTK_POS_LLH_P + RTK_DATA_LEN + 10; //纬度
    for(uint8 i=0;i<8;i++)
    {
      gpsbn_position[0].data_byte[i] = UART2_reviceBuf[rtk_data_p+i];
    }
    
    rtk_data_p = RTK_POS_LLH_P + RTK_DATA_LEN + 18; //经度
    for(uint8 i=0;i<8;i++)
    {
      gpsbn_position[1].data_byte[i] = UART2_reviceBuf[rtk_data_p+i];
    }
    
    rtk_data_p = RTK_POS_LLH_P + RTK_DATA_LEN + 26; //海拔
    for(uint8 i=0;i<8;i++)
    {
      gpsbn_position[2].data_byte[i] = UART2_reviceBuf[rtk_data_p+i];
    }
    
    //--------------------------旧组--------------------------//
    rtk_data_p = RTK_POS_LLH_P + 6;         //时间
    gpsbo_time_ms.data_byte[0] = UART2_reviceBuf[rtk_data_p];
    gpsbo_time_ms.data_byte[1] = UART2_reviceBuf[rtk_data_p+1];
    gpsbo_time_ms.data_byte[2] = UART2_reviceBuf[rtk_data_p+2];
    gpsbo_time_ms.data_byte[3] = UART2_reviceBuf[rtk_data_p+3];
    
    rtk_data_p = RTK_POS_LLH_P + 10; //纬度
    for(uint8 i=0;i<8;i++)
    {
      gpsbo_position[0].data_byte[i] = UART2_reviceBuf[rtk_data_p+i];
    }
    
    rtk_data_p = RTK_POS_LLH_P + 18; //经度
    for(uint8 i=0;i<8;i++)
    {
      gpsbo_position[1].data_byte[i] = UART2_reviceBuf[rtk_data_p+i];
    }
    
    rtk_data_p = RTK_POS_LLH_P + 26; //海拔
    for(uint8 i=0;i<8;i++)
    {
      gpsbo_position[2].data_byte[i] = UART2_reviceBuf[rtk_data_p+i];
    }
    
  }
  
  change_ms = gpsa_time_ms.data_uint32 - gpsbn_time_ms.data_uint32;
  
  //时间差
  if(change_ms>100)
  {
    gps_data_flag = 0;
   // printf("时间差计算错误change_ms=%d",change_ms);
  }
 //经纬度超出范围
  if((gpsbn_position[0].data_double > 90 && gpsbn_position[0].data_double < 0)
     || (gpsbn_position[1].data_double > 180 && gpsbn_position[1].data_double < 0))
  {
    gps_data_flag = 0;

  }
   //经纬度超出范围
  if((gpsbo_position[0].data_double > 90 && gpsbo_position[0].data_double < 0)
     || (gpsbo_position[1].data_double > 180 && gpsbo_position[1].data_double < 0))
  {
    gps_data_flag = 0;

  }
  
  
  
  if(1 == gps_data_flag)
  {
    //推算B当前经纬度海拔
    gpsb_position[0].data_double = gpsbn_position[0].data_double + ((gpsbn_position[0].data_double - gpsbo_position[0].data_double)/RTK_T_TIME )*change_ms; //纬度
  //  gpsb_position[1].data_double = gpsbn_position[1].data_double + ((gpsbn_position[1].data_double*cos(gpsbn_position[0].data_double*PI/180) 
  //                                                                   - gpsbo_position[1].data_double*cos(gpsbo_position[0].data_double*PI/180))/RTK_T_TIME )*change_ms;//经度     
    gpsb_position[1].data_double = gpsbn_position[1].data_double + ((gpsbn_position[1].data_double - gpsbo_position[1].data_double)/RTK_T_TIME )*change_ms; 
    gpsb_position[2].data_double = gpsbn_position[2].data_double + ((gpsbn_position[2].data_double - gpsbo_position[2].data_double)/RTK_T_TIME )*change_ms; //海拔
    
    //  
    //计算组合导航位置
    for(uint8 i=0;i<3;i++)
    {
      gpsu_position[i].data_double = (gpsb_position[i].data_double + gpsa_position[i].data_double)/2;
    }
    
    //计算方位，默认B指向A，车身B在后，A在前。
    //使用atan2
    
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

描述：    发送RTK数据
参数：     无
返回值：   无
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
  //经度
  for(i=0;i<4;i++)
  {
    sendData[send_p++] =inm_data.longitude.data_byte[i];
  }
  //纬度
  for(i=0;i<4;i++)
  {
    sendData[send_p++] = inm_data.latitude.data_byte[i];
  }
  //海拔
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
  //周
  for(i=0;i<2;i++)
  {
    sendData[send_p++] = inm_data.gps_weeks.data_byte[i];
  }
  //周毫秒
  for(i=0;i<4;i++)
  {
    sendData[send_p++] = inm_data.gps_ms.data_byte[i];
  }
   
  
  //sendData[1] = send_p-1;   //有效数据长度,不含校验位和帧头
  
  int len = send_p-1;   //有效数据长度,不含校验位和帧头
  
  
  uint16_t check=0;
  
  //check = CRC16_ccitt(&sendData[1], sendData[1]);
  check = CRC16_ccitt(&sendData[1], len);
  //校验位
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





