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

#define RTK_T_MS           100         //RTK发数据时间间隔ms



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

static uint8 rtk_flag;         //双定位有效标志0无效，bit0=1,RTK-A有效，bit1=1,RTK-B定位有效
static uint16 rtk_data_p;        //数据指针     
static uint8 gps_data_flag;            // 数据标志，1正常 0数据错误


static int change_ms;     //RTK的A组离最近一组数据的时间



static uint8 uart2_sp=0;

/*
@当 (x, y) 在第一象限, 0 < θ < PI/2.        东北方向
@当 (x, y) 在第二象限 PI/2 < θ≤PI.         西北方向
@当 (x, y) 在第三象限, -PI < θ < -PI/2.     西南方向
@当 (x, y) 在第四象限, -PI/2 < θ < 0.       东南方向
*/
byteToDouble dir_radian;   //方向，弧度表示

/*
********************************************************************************
函数 申明
********************************************************************************
*/
static uint8 CheckData2(void);
static uint8 CheckData1(void);
static void sendRTKData(void);
static void getRTK_GPSA(void);
static void getRTK_GPSB(void);

/*
********************************************************************************
mian函数
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
    
    //uart2接收校验，若数据出错侧延时，防止数据被截断
    if(1 == UART2_reviceFlag)
    {
      uart2_sp = UART2_dataNewFlag;  //当前接收数据最新组位置
      CheckData2();
      UART2_reviceFlag=0;
    }		
    
    //A组为参考，A接收到数据开始计算RTK组合导航位置 ,B组接收完两组数据后A组标志才置1

    if(1 == UART1_reviceFlag)
    {
      uint8_t checkFlag;
      uart2_sp = UART2_dataNewFlag;  //当前接收数据最新组位置
      checkFlag=1;
      //数据校验
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
        getRTK_GPSB();//先算A再算B ，顺序不能乱       
        
        
        //     printf("GPS_1 时间：%d 纬度：%f 经度：%f 海拔：%f 标志：%d\n",gpsa_time_ms.data_uint32,gpsa_position[0].data_double,
        //           gpsa_position[1].data_double,gpsa_position[2].data_double,rtk_flag);
        
        //     printf("GPS_2 时间：%d 纬度：%f 经度：%f 海拔：%f 标志：%d\n",gpsbn_time_ms.data_uint32,gpsbn_position[0].data_double,
        //             gpsbn_position[1].data_double,gpsbn_position[2].data_double,rtk_flag);
        
        //       printf("GPS_3 时间：%d 纬度：%f 经度：%f 海拔：%f 标志：%d\n",gpsbo_time_ms.data_uint32,gpsbo_position[0].data_double,
        //             gpsbo_position[1].data_double,gpsbo_position[2].data_double,rtk_flag);
        //    printf("GPS_4 纬度：%f 经度：%f 海拔：%f 标志：%d\n",gpsb_position[0].data_double,
        //           gpsb_position[1].data_double,gpsb_position[2].data_double,rtk_flag);
        
        printf("GPS_5 标志：%d 角度：%f 纬度：%f 经度：%f 海拔：%f \n",rtk_flag,dir_radian.data_double,gpsu_position[0].data_double,
               gpsu_position[1].data_double,gpsu_position[2].data_double);
        printf("GPS_Time 时间差：%d\n",change_ms);
        
        UART1_reviceFlag = 0;
        UART2_reviceFlag = 0;
        
        sendRTKData();
        
        
      }else //校验失败
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

描述：     获取RTK A的数据
参数：     无
返回值：    无
********************************************************************************
*/
static void getRTK_GPSA(void)
{
  uint8 i;
  
  //A组转化数据
  rtk_data_p = GPS_POS_LLH_P + 39; //定位标志地址
  
  if((UART1_reviceBuf[rtk_data_p] & 0x07) != 0)
  {
    rtk_flag = 0x01;
  }else
  {
    rtk_flag = 0x00;
  }
  
  rtk_data_p = GPS_POS_LLH_P + 6; //时间地址
  gpsa_time_ms.data_byte[0] = UART1_reviceBuf[rtk_data_p];
  gpsa_time_ms.data_byte[1] = UART1_reviceBuf[rtk_data_p+1];
  gpsa_time_ms.data_byte[2] = UART1_reviceBuf[rtk_data_p+2];
  gpsa_time_ms.data_byte[3] = UART1_reviceBuf[rtk_data_p+3];
  
  rtk_data_p = GPS_POS_LLH_P + 10; //纬度
  
  for(i=0;i<8;i++)
  {
    gpsa_position[0].data_byte[i] = UART1_reviceBuf[rtk_data_p+i];
  }
  
  rtk_data_p = GPS_POS_LLH_P + 18; //经度
  for(i=0;i<8;i++)
  {
    gpsa_position[1].data_byte[i] = UART1_reviceBuf[rtk_data_p+i];
  }
  
  rtk_data_p = GPS_POS_LLH_P + 26; //海拔
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

描述：     转化RTK B的数据
参数：     无
返回值：    无
********************************************************************************
*/

static void getRTK_GPSB(void)
{
  uint8 i;
  
  double x,y;
  
  //B组数据转化
  if(1 == uart2_sp)
  {
    //检查RTK状态
    if((UART2_reviceBuf[GPS_POS_LLH_P + 39] & 0x07) != 0 && (UART2_reviceBuf[GPS_POS_LLH_P + GPS_DATA_LEN*2 + 39] & 0x07) != 0)
    {
      rtk_flag &= ~0x02;
      rtk_flag |= 0x02;
    }else
    {
      rtk_flag &= ~0x02;
    }   
    
    //--------------------------最新组--------------------------//
    rtk_data_p = GPS_POS_LLH_P + 6;    //时间
    gpsbn_time_ms.data_byte[0] = UART2_reviceBuf[rtk_data_p];
    gpsbn_time_ms.data_byte[1] = UART2_reviceBuf[rtk_data_p+1];
    gpsbn_time_ms.data_byte[2] = UART2_reviceBuf[rtk_data_p+2];
    gpsbn_time_ms.data_byte[3] = UART2_reviceBuf[rtk_data_p+3];
    
    rtk_data_p = GPS_POS_LLH_P + 10; //纬度
    for(i=0;i<8;i++)
    {
      gpsbn_position[0].data_byte[i] = UART2_reviceBuf[rtk_data_p+i];
    }
    
    rtk_data_p = GPS_POS_LLH_P + 18; //经度
    for(i=0;i<8;i++)
    {
      gpsbn_position[1].data_byte[i] = UART2_reviceBuf[rtk_data_p+i];
    }
    
    rtk_data_p = GPS_POS_LLH_P + 26; //海拔
    for(i=0;i<8;i++)
    {
      gpsbn_position[2].data_byte[i] = UART2_reviceBuf[rtk_data_p+i];
    }
    
    //-----------------------旧组------------------------//
    rtk_data_p = GPS_POS_LLH_P + GPS_DATA_LEN*2  + 6;   //时间
    gpsbo_time_ms.data_byte[0] = UART2_reviceBuf[rtk_data_p];
    gpsbo_time_ms.data_byte[1] = UART2_reviceBuf[rtk_data_p+1];
    gpsbo_time_ms.data_byte[2] = UART2_reviceBuf[rtk_data_p+2];
    gpsbo_time_ms.data_byte[3] = UART2_reviceBuf[rtk_data_p+3];
    
    rtk_data_p = GPS_POS_LLH_P + GPS_DATA_LEN*2 + 10; //纬度
    for(i=0;i<8;i++)
    {
      gpsbo_position[0].data_byte[i] = UART2_reviceBuf[rtk_data_p+i];
    }
    
    rtk_data_p = GPS_POS_LLH_P + GPS_DATA_LEN*2 + 18; //经度
    for(i=0;i<8;i++)
    {
      gpsbo_position[1].data_byte[i] = UART2_reviceBuf[rtk_data_p+i];
    }
    
    rtk_data_p = GPS_POS_LLH_P + GPS_DATA_LEN*2 + 26; //海拔
    for(i=0;i<8;i++)
    {
      gpsbo_position[2].data_byte[i] = UART2_reviceBuf[rtk_data_p+i];
    }
    
    
    
  }else if(2 == uart2_sp)
  {
    //检查RTK状态
    if((UART2_reviceBuf[GPS_POS_LLH_P+39] & 0x07) != 0 && (UART2_reviceBuf[GPS_POS_LLH_P + GPS_DATA_LEN + 39] & 0x07) != 0)
    {
      rtk_flag &= ~0x02;
      rtk_flag |= 0x02;
    }else
    {
      rtk_flag &= ~0x02;
    }   
    
    
    //--------------------------最新组--------------------------//
    rtk_data_p = GPS_POS_LLH_P + GPS_DATA_LEN + 6;   //时间    
    gpsbn_time_ms.data_byte[0] = UART2_reviceBuf[rtk_data_p];
    gpsbn_time_ms.data_byte[1] = UART2_reviceBuf[rtk_data_p+1];
    gpsbn_time_ms.data_byte[2] = UART2_reviceBuf[rtk_data_p+2];
    gpsbn_time_ms.data_byte[3] = UART2_reviceBuf[rtk_data_p+3];
    
    rtk_data_p = GPS_POS_LLH_P + GPS_DATA_LEN + 10; //纬度
    for(i=0;i<8;i++)
    {
      gpsbn_position[0].data_byte[i] = UART2_reviceBuf[rtk_data_p+i];
    }
    
    rtk_data_p = GPS_POS_LLH_P + GPS_DATA_LEN + 18; //经度
    for(i=0;i<8;i++)
    {
      gpsbn_position[1].data_byte[i] = UART2_reviceBuf[rtk_data_p+i];
    }
    
    rtk_data_p = GPS_POS_LLH_P + GPS_DATA_LEN + 26; //海拔
    for(i=0;i<8;i++)
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
    for(i=0;i<8;i++)
    {
      gpsbo_position[0].data_byte[i] = UART2_reviceBuf[rtk_data_p+i];
    }
    
    rtk_data_p = GPS_POS_LLH_P + 18; //经度
    for(i=0;i<8;i++)
    {
      gpsbo_position[1].data_byte[i] = UART2_reviceBuf[rtk_data_p+i];
    }
    
    rtk_data_p = GPS_POS_LLH_P + 26; //海拔
    for(i=0;i<8;i++)
    {
      gpsbo_position[2].data_byte[i] = UART2_reviceBuf[rtk_data_p+i];
    }
    
  }else if(3 == uart2_sp)
  {
    //检查RTK状态
    if((UART2_reviceBuf[GPS_POS_LLH_P + GPS_DATA_LEN +39] & 0x07) != 0 && (UART2_reviceBuf[GPS_POS_LLH_P + GPS_DATA_LEN*2 + 39] & 0x07) != 0)
    {
      rtk_flag &= ~0x02;
      rtk_flag |= 0x02;
    }else
    {
      rtk_flag &= ~0x02;
    }   
    
    //--------------------------最新组--------------------------//
    rtk_data_p = GPS_POS_LLH_P + GPS_DATA_LEN*2 + 6;   //时间    
    
    gpsbn_time_ms.data_byte[0] = UART2_reviceBuf[rtk_data_p];
    gpsbn_time_ms.data_byte[1] = UART2_reviceBuf[rtk_data_p+1];
    gpsbn_time_ms.data_byte[2] = UART2_reviceBuf[rtk_data_p+2];
    gpsbn_time_ms.data_byte[3] = UART2_reviceBuf[rtk_data_p+3];
    
    rtk_data_p = GPS_POS_LLH_P + GPS_DATA_LEN*2 + 10; //纬度
    for(i=0;i<8;i++)
    {
      gpsbn_position[0].data_byte[i] = UART2_reviceBuf[rtk_data_p+i];
    }
    
    rtk_data_p = GPS_POS_LLH_P + GPS_DATA_LEN*2 + 18; //经度
    for(i=0;i<8;i++)
    {
      gpsbn_position[1].data_byte[i] = UART2_reviceBuf[rtk_data_p+i];
    }
    
    rtk_data_p = GPS_POS_LLH_P + GPS_DATA_LEN*2 + 26; //海拔
    for(i=0;i<8;i++)
    {
      gpsbn_position[2].data_byte[i] = UART2_reviceBuf[rtk_data_p+i];
    }
    
    //--------------------------旧组--------------------------//
    rtk_data_p = GPS_POS_LLH_P  + GPS_DATA_LEN + 6;         //时间
    gpsbo_time_ms.data_byte[0] = UART2_reviceBuf[rtk_data_p];
    gpsbo_time_ms.data_byte[1] = UART2_reviceBuf[rtk_data_p+1];
    gpsbo_time_ms.data_byte[2] = UART2_reviceBuf[rtk_data_p+2];
    gpsbo_time_ms.data_byte[3] = UART2_reviceBuf[rtk_data_p+3];
    
    rtk_data_p = GPS_POS_LLH_P  + GPS_DATA_LEN  + 10; //纬度
    for(i=0;i<8;i++)
    {
      gpsbo_position[0].data_byte[i] = UART2_reviceBuf[rtk_data_p+i];
    }
    
    rtk_data_p = GPS_POS_LLH_P  + GPS_DATA_LEN  + 18; //经度
    for(i=0;i<8;i++)
    {
      gpsbo_position[1].data_byte[i] = UART2_reviceBuf[rtk_data_p+i];
    }
    
    rtk_data_p = GPS_POS_LLH_P  + GPS_DATA_LEN  + 26; //海拔
    for(i=0;i<8;i++)
    {
      gpsbo_position[2].data_byte[i] = UART2_reviceBuf[rtk_data_p+i];
    }
    
  }
  
  //计算时间差
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
    //推算B当前经纬度海拔计算
    gpsb_position[0].data_double = gpsbn_position[0].data_double + ((gpsbn_position[0].data_double - gpsbo_position[0].data_double)/RTK_T_MS)*change_ms; //纬度
    gpsb_position[1].data_double = gpsbn_position[1].data_double + ((gpsbn_position[1].data_double*cos(gpsbn_position[0].data_double*PI/180) 
                                                                     - gpsbo_position[1].data_double*cos(gpsbo_position[0].data_double*PI/180))/RTK_T_MS)*change_ms;                    //经度    
    gpsb_position[2].data_double = gpsbn_position[2].data_double + ((gpsbn_position[2].data_double - gpsbo_position[2].data_double)/RTK_T_MS)*change_ms; //海拔
    
    //  
    //计算组合导航位置
    for(i=0;i<3;i++)
    {
      gpsu_position[i].data_double = (gpsb_position[i].data_double + gpsa_position[i].data_double)/2;
    }
    
    //计算方位，默认A指向B，车身A在后，B在前。目前只考虑在北半球
    //使用atan2
    
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

描述：     校验RTK数据,A组
参数：     无
返回值：    无
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
    //延时，处理数据被截断的情况  
    Delay_Ms(15);
 //   printf("crcA校验失败\n");
  }else
  {
     printf("crcA校验成功\n");
  }
  
  //开启接收中断
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);  
  
  return flag;
  
}	
/*
********************************************************************************
static uint8 CheckData2(void)

描述：     校验RTK数据 ，B组
参数：     无
返回值：    无
********************************************************************************
*/
static uint8 CheckData2(void)
{
  uint16 crc2;
 // int i;
  uint8 flag;
  flag=1;
  crc2=0;
  
  //数据校验
  if(1 ==  uart2_sp)
  {
    crc2 =  crc16_ccitt_table(&UART2_reviceBuf[GPS_POS_LLH_P+1], 39, crc2);
    if(crc2 != (UART2_reviceBuf[GPS_POS_LLH_P+40] |(UART2_reviceBuf[GPS_POS_LLH_P+41]<<8)))
    {
      //延时，处理数据被截断的情况
      Delay_Ms(15);
      flag=0;
      /*
      //打印接收到的数据
      for(i=GPS_POS_LLH_P;i<GPS_POS_LLH_P+42;i++)
      {
        printf(" %x",UART2_reviceBuf[GPS_DATA_LEN+i]);
        
      }
      printf("\n");
      printf("校验值1：%x %x\n",(uint8_t)crc2,(uint8_t)(crc2>>8));
      */
      
    }else
    {
    //   printf("crc1校验成功\n");
    }    
    
  }else if(2 ==  uart2_sp)
  {
    crc2 =  crc16_ccitt_table(&UART2_reviceBuf[GPS_POS_LLH_P+GPS_DATA_LEN+1], 39, crc2);
    if(crc2 != (UART2_reviceBuf[GPS_POS_LLH_P+GPS_DATA_LEN+40] |(UART2_reviceBuf[GPS_POS_LLH_P+GPS_DATA_LEN+41]<<8)))
    {
      //延时，处理数据被截断的情况
      Delay_Ms(15);   
      flag=0;
      /*
      //打印接收到的数据
      for(i=GPS_POS_LLH_P+GPS_DATA_LEN;i<GPS_POS_LLH_P+GPS_DATA_LEN+42;i++)
      {
        printf(" %x",UART2_reviceBuf[GPS_DATA_LEN+i]);
        
      }
      printf("\n");
      printf("校验值2：%x %x\n",(uint8_t)crc2,(uint8_t)(crc2>>8));
*/
    }else
    {
 //      printf("crc2校验成功\n");
    }
    
   
    
  }else if(3 ==  uart2_sp)
  {
    crc2 =  crc16_ccitt_table(&UART2_reviceBuf[GPS_POS_LLH_P+GPS_DATA_LEN*2+1], 39, crc2);
    
    if(crc2 != (UART2_reviceBuf[GPS_POS_LLH_P+GPS_DATA_LEN*2+40] |(UART2_reviceBuf[GPS_POS_LLH_P+GPS_DATA_LEN*2+41]<<8)))
    {
      //延时，处理数据被截断的情况
      Delay_Ms(15);     
      flag=0;
      
      /*
      //打印接收到的数据
      for(i=GPS_POS_LLH_P+GPS_DATA_LEN*2;i<GPS_POS_LLH_P+GPS_DATA_LEN*2+42;i++)
      {
        printf(" %x",UART2_reviceBuf[GPS_DATA_LEN+i]);
        
      }
      printf("\n");
      printf("校验值3：%x %x\n",(uint8_t)crc2,(uint8_t)(crc2>>8));
*/
    }else
    {
  //     printf("crc3校验成功\n");
    }
    
    
  }
  
  
  //开启接收中断
  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);  	
  
  return flag;
  
}	

/*
********************************************************************************
static void sendRTKData(void)

描述：     发送RTK数据到蓝牙
参数：     无
返回值：    无
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
  sendData[1] = send_p-1;   //有效数据长度,不含校验位和帧头
  
  check=0;
  check = crc16_ccitt(&sendData[1], sendData[1]);
  
  //校验位
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
 // printf("帧长：%d\n",sendData[1]+3);
}







