/**
******************************************************************************
* @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c 
* @author  MCD Application Team
* @version V3.5.0
* @date    08-April-2011
* @brief   Main Interrupt Service Routines.
*          This file provides template for all exceptions handler and 
*          peripherals interrupt service routine.
******************************************************************************
* @attention
*
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
* TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
* DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
* FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
* CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
* <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/

#include "stm32f10x_it.h"
#include "bsp_uart1.h"
#include "bsp_uart2.h"
#include "bsp_uart3.h"
#include "bsp_uart5.h"
#include "app_conf.h"


/*
****************************�û����뿪ʼ****************************************
*/   

#include <stdio.h>





static uint8    rtk_start=0;



static uint8   uart5_recevieStart = 0;
static uint8   uart5_dataCount = 0;   //�������ݼ���



/*
********************************************************************************
void USART1_IRQHandler(void)

������     USART1�����ж�
������     
����ֵ��    ��
˵����RTK��0xFFFF��Ϣ���Ϳ�ʼ��������ʼ�մ�����һ֡���ݿ�ʼ����
********************************************************************************
*/
void USART1_IRQHandler(void)
{
  static uint16   uart1_dataCount = 0; 
  static uint8   uart1_recevieStart = 0;
  static uint8  data=0;

  
  if(USART_GetITStatus(USART1,USART_IT_RXNE) != RESET)
  { 
    USART_ClearITPendingBit(USART1,USART_IT_RXNE); //����жϱ�־
    
    
    data= USART_ReceiveData(USART1); 
    
    
     if(1 == uart1_recevieStart)
     {
       UART1_reviceBuf[uart1_dataCount++] = data; 
     }
     //�ж�֡ͷ
     if(0x55 == data && 0 == uart1_recevieStart)
     {
        uart1_dataCount=0; 
        UART1_reviceBuf[uart1_dataCount++] = data;
        uart1_recevieStart =1;
     }
     
   
    //   printf(" %x",data);
    
    //�������
    if(GPS_DATA_LEN  == uart1_dataCount)
    { 
      if(rtk_start>3)  //RTK2�����������Ժ�ʼ��������
      {
        UART1_reviceFlag = 1;
        USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);     //���յ�125�ֽں�ֹͣ����
        
      } else{
        //		printf("uart2��û�н��յ�����\n");
      }
      uart1_recevieStart =0;
    }
    
  }
}

/*
********************************************************************************
void USART2_IRQHandler(void)

������     USART2�����ж�
������     
����ֵ��    ��
********************************************************************************
*/
void USART2_IRQHandler(void)
{
  static uint16   uart2_dataCount = 0;   //�������ݼ���	
  static uint8   uart2_recevieStart = 0;
  static uint8    data=0;
  if(USART_GetITStatus(USART2,USART_IT_RXNE) != RESET)
  { 
    
    USART_ClearITPendingBit(USART2,USART_IT_RXNE); //����жϱ�־
    
    
     data= USART_ReceiveData(USART2); 
     
     if(1 == uart2_recevieStart)
     {
       UART2_reviceBuf[uart2_dataCount++] = data; 
     }
     //�ж�֡ͷ
     if(0x55 == data && 0 == uart2_recevieStart)
     {
       
       //��λ���洢λ��
       if(0 == UART2_dataNewFlag)
       {
         uart2_dataCount = 0;//��һ��洢���
       }else if(1 == UART2_dataNewFlag)
       {
         uart2_dataCount = GPS_DATA_LEN;//�洢���
         
       }else if(2 == UART2_dataNewFlag)
       {
         uart2_dataCount = GPS_DATA_LEN*2;//�洢���
         
       } else if(3 == UART2_dataNewFlag)
       {
          uart2_dataCount = 0;//��һ��洢���
       }
       
       UART2_reviceBuf[uart2_dataCount++] = data; 
       uart2_recevieStart =1;
      
     }
     
 //   printf(" %x",UART2_reviceBuf[uart2_dataCount-1]);  
    
    //�������
    if(GPS_DATA_LEN  == uart2_dataCount || GPS_DATA_LEN *2 == uart2_dataCount || GPS_DATA_LEN *3 == uart2_dataCount)
    {
   //   printf("\n");
      
      //�����������־
      if(0 == UART2_dataNewFlag)
      {
        UART2_dataNewFlag =1;
        
      }else if(1 == UART2_dataNewFlag)
      {
        UART2_dataNewFlag = 2;   
        
      }else if(2 == UART2_dataNewFlag)
      {
        UART2_dataNewFlag = 3;
        
      } else if(3 == UART2_dataNewFlag)
      {
        UART2_dataNewFlag = 1;
      }
      
      
      rtk_start++; //���ռ���>3ʱuart1�ſ�ʼ��������
      uart2_recevieStart =0;
      UART2_reviceFlag=1;
      USART_ITConfig(USART2, USART_IT_RXNE, DISABLE); 
      
    }
    
  } 
  
}

/*
********************************************************************************
void USART3_IRQHandler(void)

������     USART3�����ж�
������     
����ֵ��    ��
********************************************************************************
*/
void USART3_IRQHandler(void)
{
  static u8 data;
  if(USART_GetITStatus(USART3,USART_IT_RXNE) != RESET)
  { 
    USART_ClearITPendingBit(USART3,USART_IT_RXNE); //����жϱ�־
    data = USART_ReceiveData(USART3);  
    
    
  }
}

/*
********************************************************************************
void UART5_IRQHandler(void)

������     UART5�����ж�
������     
����ֵ��    ��
********************************************************************************
*/
void UART5_IRQHandler(void)
{
  static u8 data;
  
  if(USART_GetITStatus(UART5,USART_IT_RXNE) != RESET)
  { 
    USART_ClearITPendingBit(UART5,USART_IT_RXNE); //����жϱ�־
    data = USART_ReceiveData(UART5);   
    
    
    if(2 == uart5_recevieStart)
    {
      UART5_reviceBuf[uart5_dataCount++] = data;   
    }
    
    if(0x55 == data && 0 == uart5_recevieStart && 0 == UART5_reviceFlag)   //��Ϣͷ
    {
      uart5_recevieStart = 1;
      
      
    }else if(0x53 == data && 1 == uart5_recevieStart)
    {
      UART5_reviceBuf[0] = 0x55;
      UART5_reviceBuf[1] = data;
      uart5_dataCount = 2; 
      uart5_recevieStart = 2;
      
    }
    //�������
    if(11 == uart5_dataCount)
    {
      UART5_reviceFlag = 1;
      uart5_dataCount = 0;    
    }
    
  }
}



/*
****************************�û��������****************************************
*/


/** @addtogroup STM32F10x_StdPeriph_Template
* @{
*/

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/


/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
* @brief  This function handles NMI exception.
* @param  None
* @retval None
*/
void NMI_Handler(void)
{
}

/**
* @brief  This function handles Hard Fault exception.
* @param  None
* @retval None
*/
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
* @brief  This function handles Memory Manage exception.
* @param  None
* @retval None
*/
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
* @brief  This function handles Bus Fault exception.
* @param  None
* @retval None
*/
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
* @brief  This function handles Usage Fault exception.
* @param  None
* @retval None
*/
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
* @brief  This function handles SVCall exception.
* @param  None
* @retval None
*/
void SVC_Handler(void)
{
}

/**
* @brief  This function handles Debug Monitor exception.
* @param  None
* @retval None
*/
void DebugMon_Handler(void)
{
}

/**
* @brief  This function handles PendSVC exception.
* @param  None
* @retval None
*/
void PendSV_Handler(void)
{
}

/**
* @brief  This function handles SysTick Handler.
* @param  None
* @retval None
*/
void SysTick_Handler(void)
{
}


/**
* @brief  EXTI1_IRQHandler
*         This function handles External line 1 interrupt request.
* @param  None
* @retval None
*/
void EXTI1_IRQHandler(void)
{
  if(EXTI_GetITStatus(EXTI_Line1) != RESET)
  {
    
    EXTI_ClearITPendingBit(EXTI_Line1);
  }
}
/**
* @brief  OTG_FS_IRQHandler
*          This function handles USB-On-The-Go FS global interrupt request.
*          requests.
* @param  None
* @retval None
*/
#ifdef USE_USB_OTG_FS
void OTG_FS_IRQHandler(void)
#else
void OTG_HS_IRQHandler(void)
#endif
{
  
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
* @brief  This function handles PPP interrupt request.
* @param  None
* @retval None
*/
/*void PPP_IRQHandler(void)
{
}*/



/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/

