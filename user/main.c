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
//RTK数据流个数据位置起点

#define GPS_POS_LLH_P      53
#define GPS_DATA_LEN       125  //存两组RTK数据流数据间隔

#define GPS_T_TIME         //RTK发数据时间间隔


/*
********************************************************************************
                                   全局变量
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

uint8 rtk_flag;         //双定位有效标志0无效，bit0=1,RTK-A有效，bit1=1,RTK-B定位有效
uint8 rtk_data_p;        //数据指针      

uint32 gpsb_Tms;      //RTK的B组两个数据间隔时间
uint32 change_ms;     //RTK的A组离最近一组数据的时间


/*
@当 (x, y) 在第一象限, 0 < θ < 90.        东北方向
@当 (x, y) 在第二象限 90< θ≤180         西北方向
@当 (x, y) 在第三象限, -180 < θ < -90.     西南方向
@当 (x, y) 在第四象限, -90 < θ < 0       东南方向
*/
byteToDouble dir_radian;   //方向，弧度表示
        
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
    if(1 == UART1_reviceFlag && UART2_dataNewFlag != 0)
    {
        IWDG_ReloadCounter();  //喂狗
       
        uint16_t crc1=0;
        uint16_t crc2=0;
        uint16_t crc3=0;
        uint8_t checkFlag = 1;
        //数据校验
        crc1 =  crc16_ccitt_table(&UART1_reviceBuf[GPS_POS_LLH_P+1], 39, crc1);
        crc2 =  crc16_ccitt_table(&UART2_reviceBuf[GPS_POS_LLH_P+1], 39, crc2);
        crc3 =  crc16_ccitt_table(&UART2_reviceBuf[GPS_POS_LLH_P+GPS_DATA_LEN+1], 39, crc3);

      //  printf("校验值1：%x %x %x %x\n",(uint8_t)crc1,(uint8_t)(crc1>>8),UART1_reviceBuf[GPS_POS_LLH_P+40],UART1_reviceBuf[GPS_POS_LLH_P+41]);
      //  printf("校验值2：%x %x %x %x\n",(uint8_t)crc2,(uint8_t)(crc2>>8),UART2_reviceBuf[GPS_POS_LLH_P+40],UART2_reviceBuf[GPS_POS_LLH_P+41]);
      //  printf("校验值3：%x %x %x %x\n",(uint8_t)crc3,(uint8_t)(crc3>>8),UART2_reviceBuf[GPS_POS_LLH_P+GPS_DATA_LEN+40],UART2_reviceBuf[GPS_POS_LLH_P+GPS_DATA_LEN+41]);
            
        // if(!((UART1_reviceBuf[GPS_POS_LLH_P+40] == (uint8_t)crc1) && (UART1_reviceBuf[GPS_POS_LLH_P+41] == (uint8_t)(crc1>>8))))
        if(crc1 != (UART1_reviceBuf[GPS_POS_LLH_P+40] | (UART1_reviceBuf[GPS_POS_LLH_P+41]<<8)))
        {
          checkFlag = 0;
          //延时，处理数据被截断的情况
      //    USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);   
          Delay_Ms(13);
     //     USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);   
          printf("crc1校验失败\n");
         
           
        }
        if(crc2 != (UART2_reviceBuf[GPS_POS_LLH_P+40] |(UART2_reviceBuf[GPS_POS_LLH_P+41]<<8)))
        {
          checkFlag = 0;
          //延时，处理数据被截断的情况
       //   USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);   
          Delay_Ms(13);
       //   USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);   
          printf("crc2校验失败\n");
           UART2_dataNewFlag=0;
        }
        if(crc3 != (UART2_reviceBuf[GPS_POS_LLH_P+GPS_DATA_LEN+40] | (UART2_reviceBuf[GPS_POS_LLH_P+GPS_DATA_LEN+41]<<8)))
        {
          printf("crc3校验失败\n");
            //延时，处理数据被截断的情况
      //    USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);   
          Delay_Ms(13);
     //     USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); 
          checkFlag = 0;
          UART2_dataNewFlag=0;
          
        }
        /*
        printf("校验值：%x %x %x\n",crc1,crc2,crc3);
        
        for(int i=GPS_POS_LLH_P;i<GPS_POS_LLH_P+42;i++)
        {
          printf(" %x",UART1_reviceBuf[i]);
          
        }
        
        printf("\n");
        for(int i=GPS_POS_LLH_P;i<GPS_POS_LLH_P+42;i++)
        {
          printf(" %x",UART2_reviceBuf[i]);
          
        }
        printf("\n");
        for(int i=GPS_POS_LLH_P;i<GPS_POS_LLH_P+42;i++)
        {
          printf(" %x",UART2_reviceBuf[GPS_DATA_LEN+i]);
          
        }
        printf("\n");
*/
       
  
      if(1 == checkFlag)
      {
         
        
        rtk_data_p = GPS_POS_LLH_P + 39; //定位标志地址
        
        if((UART1_reviceBuf[rtk_data_p] & 0x07) != 0)
        {
          rtk_flag = 0x01;
        }else
        {
          rtk_flag = 0x00;
        }
        
        if(0x01 == rtk_flag)
        {
            //检查RTK状态
            if((UART2_reviceBuf[rtk_data_p] & 0x07) != 0)
            {
              rtk_flag &= ~0x02;
              rtk_flag |= 0x02;
            }else
            {
              rtk_flag &= ~0x02;
            }   
        
            rtk_data_p = GPS_POS_LLH_P + GPS_DATA_LEN + 39; //定位标志地址
            
            if((UART2_reviceBuf[rtk_data_p] && 0x07) != 0)
            {
              rtk_flag &= ~0x02;
              rtk_flag |= 0x02;
            }else
            {
              rtk_flag &= ~0x02;
            }
            
          
        }
        
        //转化数据
        rtk_data_p = GPS_POS_LLH_P + 6; //时间地址
        
        gpsa_time_ms.data_byte[0] = UART1_reviceBuf[rtk_data_p];
        gpsa_time_ms.data_byte[1] = UART1_reviceBuf[rtk_data_p+1];
        gpsa_time_ms.data_byte[2] = UART1_reviceBuf[rtk_data_p+2];
        gpsa_time_ms.data_byte[3] = UART1_reviceBuf[rtk_data_p+3];
        
        rtk_data_p = GPS_POS_LLH_P + 10; //纬度
        for(uint8 i=0;i<8;i++)
        {
          gpsa_position[0].data_byte[i] = UART1_reviceBuf[rtk_data_p+i];
        }
        
        rtk_data_p = GPS_POS_LLH_P + 18; //经度
        for(uint8 i=0;i<8;i++)
        {
          gpsa_position[1].data_byte[i] = UART1_reviceBuf[rtk_data_p+i];
        }
        
        rtk_data_p = GPS_POS_LLH_P + 26; //海拔
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
          rtk_data_p = GPS_POS_LLH_P + 6;    //时间
          gpsbn_time_ms.data_byte[0] = UART2_reviceBuf[rtk_data_p];
          gpsbn_time_ms.data_byte[1] = UART2_reviceBuf[rtk_data_p+1];
          gpsbn_time_ms.data_byte[2] = UART2_reviceBuf[rtk_data_p+2];
          gpsbn_time_ms.data_byte[3] = UART2_reviceBuf[rtk_data_p+3];
          
          rtk_data_p = GPS_POS_LLH_P + 10; //纬度
          for(uint8 i=0;i<8;i++)
          {
            gpsbn_position[0].data_byte[i] = UART2_reviceBuf[rtk_data_p+i];
          }
          
          rtk_data_p = GPS_POS_LLH_P + 18; //经度
          for(uint8 i=0;i<8;i++)
          {
            gpsbn_position[1].data_byte[i] = UART2_reviceBuf[rtk_data_p+i];
          }
          
          rtk_data_p = GPS_POS_LLH_P + 26; //海拔
          for(uint8 i=0;i<8;i++)
          {
            gpsbn_position[2].data_byte[i] = UART2_reviceBuf[rtk_data_p+i];
          }
          
          //-----------------------旧组------------------------//
          rtk_data_p = GPS_POS_LLH_P + GPS_DATA_LEN  + 6;   //时间
          gpsbo_time_ms.data_byte[0] = UART2_reviceBuf[rtk_data_p];
          gpsbo_time_ms.data_byte[1] = UART2_reviceBuf[rtk_data_p+1];
          gpsbo_time_ms.data_byte[2] = UART2_reviceBuf[rtk_data_p+2];
          gpsbo_time_ms.data_byte[3] = UART2_reviceBuf[rtk_data_p+3];
          
          rtk_data_p = GPS_POS_LLH_P + GPS_DATA_LEN + 10; //纬度
          for(uint8 i=0;i<8;i++)
          {
            gpsbo_position[0].data_byte[i] = UART2_reviceBuf[rtk_data_p+i];
          }
          
          rtk_data_p = GPS_POS_LLH_P + GPS_DATA_LEN + 18; //经度
          for(uint8 i=0;i<8;i++)
          {
            gpsbo_position[1].data_byte[i] = UART2_reviceBuf[rtk_data_p+i];
          }
          
          rtk_data_p = GPS_POS_LLH_P + GPS_DATA_LEN + 26; //海拔
          for(uint8 i=0;i<8;i++)
          {
            gpsbo_position[2].data_byte[i] = UART2_reviceBuf[rtk_data_p+i];
          }
          
        }else
        {
          //--------------------------最新组--------------------------//
          rtk_data_p = GPS_POS_LLH_P + GPS_DATA_LEN + 6;   //时间    
          gpsbn_time_ms.data_byte[0] = UART2_reviceBuf[rtk_data_p];
          gpsbn_time_ms.data_byte[1] = UART2_reviceBuf[rtk_data_p+1];
          gpsbn_time_ms.data_byte[2] = UART2_reviceBuf[rtk_data_p+2];
          gpsbn_time_ms.data_byte[3] = UART2_reviceBuf[rtk_data_p+3];
          
          rtk_data_p = GPS_POS_LLH_P + GPS_DATA_LEN + 10; //纬度
          for(uint8 i=0;i<8;i++)
          {
            gpsbn_position[0].data_byte[i] = UART2_reviceBuf[rtk_data_p+i];
          }
          
          rtk_data_p = GPS_POS_LLH_P + GPS_DATA_LEN + 18; //经度
          for(uint8 i=0;i<8;i++)
          {
            gpsbn_position[1].data_byte[i] = UART2_reviceBuf[rtk_data_p+i];
          }
          
          rtk_data_p = GPS_POS_LLH_P + GPS_DATA_LEN + 26; //海拔
          for(uint8 i=0;i<8;i++)
          {
            gpsbn_position[2].data_byte[i] = UART2_reviceBuf[rtk_data_p+i];
          }
          
          //--------------------------旧组--------------------------//
          rtk_data_p = GPS_POS_LLH_P + 6;         //时间
          gpsbo_time_ms.data_byte[0] = UART2_reviceBuf[rtk_data_p];
          gpsbo_time_ms.data_byte[1] = UART2_reviceBuf[rtk_data_p+1];
          gpsbo_time_ms.data_byte[2] = UART2_reviceBuf[rtk_data_p+2];
          gpsbo_time_ms.data_byte[3] = UART2_reviceBuf[rtk_data_p+3];
          
          rtk_data_p = GPS_POS_LLH_P + 10; //纬度
          for(uint8 i=0;i<8;i++)
          {
            gpsbo_position[0].data_byte[i] = UART2_reviceBuf[rtk_data_p+i];
          }
          
          rtk_data_p = GPS_POS_LLH_P + 18; //经度
          for(uint8 i=0;i<8;i++)
          {
            gpsbo_position[1].data_byte[i] = UART2_reviceBuf[rtk_data_p+i];
          }
          
          rtk_data_p = GPS_POS_LLH_P + 26; //海拔
          for(uint8 i=0;i<8;i++)
          {
            gpsbo_position[2].data_byte[i] = UART2_reviceBuf[rtk_data_p+i];
          }
          
        }
        
        change_ms = gpsa_time_ms.data_uint32 - gpsbn_time_ms.data_uint32;
     //   gpsb_Tms  = gpsbn_time_ms.data_uint32 - gpsbo_time_ms.data_uint32;
        gpsb_Tms  = 100;
        
        if((gpsbn_position[0].data_double > 90 && gpsbn_position[0].data_double < 0)
           || (gpsbn_position[1].data_double > 180 && gpsbn_position[1].data_double < 0))
        {
          gpsbn_position[0].data_double = 0;
          gpsbn_position[1].data_double = 0;
          gpsbn_position[2].data_double = 0;
          gps_data_flag = 0;
          change_ms = 0;
          gpsb_Tms  = 0xefffffff;
        }
        
        if((gpsbo_position[0].data_double > 90 && gpsbo_position[0].data_double < 0)
           || (gpsbo_position[1].data_double > 180 && gpsbo_position[1].data_double < 0))
        {
          gpsbo_position[0].data_double = 0;
          gpsbo_position[1].data_double = 0;
          gpsbo_position[2].data_double = 0;
          gps_data_flag = 0;
          change_ms = 0;
          gpsb_Tms  = 0xefffffff;
        }
        
 
       
        if(1 == gps_data_flag)
        {
          //推算B当前经纬度海拔计算
          gpsb_position[0].data_double = gpsbn_position[0].data_double + ((gpsbn_position[0].data_double - gpsbo_position[0].data_double)/gpsb_Tms)*change_ms; //纬度
          gpsb_position[1].data_double = gpsbn_position[1].data_double + ((gpsbn_position[1].data_double*cos(gpsbn_position[0].data_double*PI/180) 
                                                                           - gpsbo_position[1].data_double*cos(gpsbo_position[0].data_double*PI/180))/gpsb_Tms)*change_ms;                    //经度    
          gpsb_position[2].data_double = gpsbn_position[2].data_double + ((gpsbn_position[2].data_double - gpsbo_position[2].data_double)/gpsb_Tms)*change_ms; //海拔
          
          //  
          //计算组合导航位置
          for(uint8 i=0;i<3;i++)
          {
            gpsu_position[i].data_double = (gpsb_position[i].data_double + gpsa_position[i].data_double)/2;
          }
                 
          //计算方位，默认A指向B，车身A在后，B在前。目前只考虑在北半球
          //使用atan2
          
          double x,y;
          y = gpsb_position[0].data_double - gpsa_position[0].data_double;
          x = gpsb_position[1].data_double*cos(gpsb_position[0].data_double*PI/180) - gpsa_position[1].data_double*cos(gpsa_position[0].data_double*PI/180);
          dir_radian.data_double = atan2(y,x)*180/PI;
        }else
        {
          dir_radian.data_double = 0;
          for(uint8 i=0;i<3;i++)
          {
            gpsu_position[i].data_double = 0;
          }
        }
        
             
                
   //     printf("GPS_1 时间：%d 纬度：%f 经度：%f 海拔：%f 标志：%d\n",gpsa_time_ms.data_uint32,gpsa_position[0].data_double,
  //            gpsa_position[1].data_double,gpsa_position[2].data_double,rtk_flag);
        
   //     printf("GPS_2 时间：%d 纬度：%f 经度：%f 海拔：%f 标志：%d\n",gpsbn_time_ms.data_uint32,gpsbn_position[0].data_double,
   //            gpsbn_position[1].data_double,gpsbn_position[2].data_double,rtk_flag);
        
  //      printf("GPS_3 时间：%d 纬度：%f 经度：%f 海拔：%f 标志：%d\n",gpsbo_time_ms.data_uint32,gpsbo_position[0].data_double,
  //             gpsbo_position[1].data_double,gpsbo_position[2].data_double,rtk_flag);
  //     printf("GPS_4 纬度：%f 经度：%f 海拔：%f 标志：%d\n",gpsb_position[0].data_double,
  //             gpsb_position[1].data_double,gpsb_position[2].data_double,rtk_flag);
        

        
        printf("GPS_5 角度：%f 纬度：%f 经度：%f 海拔：%f 标志：%d\n",dir_radian.data_double,gpsu_position[0].data_double,
               gpsu_position[1].data_double,gpsu_position[2].data_double,rtk_flag);
        printf("GPS_Time 时间差：%d 周期：%d\n",change_ms,gpsb_Tms);
        
        uint8 sendData[50];
        uint8 send_p = 0;
        sendData[send_p++] = 0x55;
        sendData[send_p++] = 0;       
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
        sendData[1] = send_p-1;   //有效数据长度,不含校验位和帧头
       
       
        uint16_t check=0;
        
        check = crc16_ccitt(&sendData[1], sendData[1]);
 
        //校验位
        sendData[send_p++] = check & 0xff;
        sendData[send_p]   = (check>>8) & 0xff;
        
        
    //    UART3_sendData(sendData[1]+3,sendData);

        uint8_t fra=(sendData[1]+3)/20;
        
        for(int i=0;i< fra;i++)
        {
          UART3_sendData(20,&sendData[20*i]);
          Delay_Ms(3);
        }
        Delay_Ms(3);
        UART3_sendData(sendData[1]+3-fra*20,&sendData[fra*20]);
   //     printf("帧长：%d\n",sendData[1]+3);
      //  Delay_Ms(100);

        
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

        UART3_sendData(41,err);
        
      }
      
    }
#endif
  }

  
}








