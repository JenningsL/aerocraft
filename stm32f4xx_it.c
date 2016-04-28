//V1.0.0
#include "stm32f4xx_it.h"
#include "main.h"
#include "SysTick.h"
//NMI exception handler
void NMI_Handler(void)
{
}

//Hard Fault exception handler
void HardFault_Handler(void)
{
	while(1);
}

//Memory Manage exception handler
void MemManage_Handler(void)
{
  	while (1)
  	{
  	}
}

//Bus Fault exception handler
void BusFault_Handler(void)
{
  	while (1)
  	{
  	}
}

//Usage Fault exception handler
void UsageFault_Handler(void)
{
  	while (1)
  	{
  	}
}

//SVCall exception handler
void SVC_Handler(void)
{
}

//Debug Monitor exception handler
void DebugMon_Handler(void)
{
}

//PendSVC exception handler
void PendSV_Handler(void)
{
}

//SysTick handler
extern u32 ntime;
void SysTick_Handler(void)
{
	TimingDelay_Decrement();
}

void SDIO_IRQHandler(void)
{
  	
}

void SD_SDIO_DMA_IRQHANDLER(void)
{
  	
}

void DCMI_IRQHandler(void)
{

}
//TIM2定时器中断服务函数
void TIM2_IRQHandler(void)
{
  if(TIM_GetITStatus(TIM2,TIM_IT_Update)==SET )
  {
    //待写
    TIM_ClearITPendingBit (TIM2,TIM_IT_Update);
  }
  
}

//TIM4定时器中断服务函数
void TIM4_IRQHandler(void)
{
  if(TIM_GetITStatus(TIM4,TIM_IT_Update)==SET )
  {
    //待写
    TIM_ClearITPendingBit (TIM4,TIM_IT_Update);
  }
  
}

extern char USART2_RECV_BUF[300];
extern int ready;//判断一组数据传输是否完成
extern int length;

void USART2_IRQHandler(void)
{

  char temp;  
  if(USART_GetFlagStatus(USART2, USART_FLAG_RXNE)==SET)
  {

    temp=USART_ReceiveData(USART2);
    if(temp==0x00)//一组数据传输完成，可以进行处理
    {
      ready=1;
      USART2_RECV_BUF[length]=temp;
      length++;//增加的

      
    }

    else{
      USART2_RECV_BUF[length]=temp;
      length++;
      
    }
    USART_ClearITPendingBit (USART2,USART_IT_RXNE );//清除中断标志位
  }
  
  
  
  }//中断服务
void EXTI0_IRQHandler(void)
{
  if(EXTI_GetITStatus(EXTI_Line0) != RESET)
  {
    TIM_SetCompare1(TIM4, 0);
    TIM_SetCompare2(TIM4, 0);
    TIM_SetCompare3(TIM4, 0);
    TIM_SetCompare4(TIM4, 0);
    EXTI_ClearITPendingBit(EXTI_Line0);
  }
}