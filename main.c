#include "main.h"
#include "SysTick.h"	
#include "motor.h"
#include "PID.h"
#include "MPU6050.h"
#include "quaternion.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
/////////////////////////////
#define dataNum 4
char USART2_RECV_BUF[300];
int ready=0;
int length=0;
char mes[4][20];

unsigned int lastTime=0;
//////////////////////////////
float ACC_X,ACC_Y,ACC_Z,GYRO_X,GYRO_Y,GYRO_Z;//传入四元数函数计算的六个数据
void GetMessage();
void control(float pitch, float roll, float pitch_set, float roll_set);
float p,i,d;
int pitch,roll;

////////////////////////////////
EXTI_InitTypeDef   EXTI_InitStructure;
//userButton中断配置
void EXTILine0_Config(void)
{
  
  GPIO_InitTypeDef   GPIO_InitStructure;
  NVIC_InitTypeDef   NVIC_InitStructure;

  /* Enable GPIOA clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  /* Enable SYSCFG clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
  
  /* Configure PA0 pin as input floating */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Connect EXTI Line0 to PA0 pin */
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);

  /* Configure EXTI Line0 */
  EXTI_InitStructure.EXTI_Line = EXTI_Line0;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  /* Enable and set EXTI Line0 Interrupt to the lowest priority */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}
///////////////////////////
void main()
{
    EXTILine0_Config();
    SysTick_Init();
    COM2Init();//串口2
    
    TIM2_Config();
    TIM4_Config();
    IncPIDInit();
    I2C_Configuration();//I2C2初始化
    IIC_Stop();//停止I2C
    MPU6050_Inital();//MPU6050初始化
    adjustOffset();//校正偏移量
    fill_ACC_BUF();//填充滑动窗口数组
    
    

    
    PIDSetPp(4);
    PIDSetPi(0);
    PIDSetPd(400);
    PIDSetRp(4);
    PIDSetRi(0);
    PIDSetRd(400);//初始化了PID
    
    PIDSetIMAX();
    
    // auto unlock
    Delay_us(2000000);
    unLock();
    
      
    while(1)
    {
       if(ready==1){//数据准备好
        USART_ITConfig(USART2,USART_IT_RXNE,DISABLE);//暂时关闭接收中断
        GetMessage();
        
        
        if(mes[0][0]=='p')
        {
          p=atof(mes[1]);
          i=atof(mes[2]);
          d=atof(mes[3]);
          PIDSetPp(p);
          PIDSetPi(i);
          PIDSetPd(d);
        }//设置pid参数
        
         if(mes[0][0]=='r')
        {
          p=atof(mes[1]);
          i=atof(mes[2]);
          d=atof(mes[3]);
          PIDSetRp(p);
          PIDSetRi(i);
          PIDSetRd(d);
        }//设置pid参数
        
        if(mes[0][0]=='u')
        {
          unLock();
        }
        if(mes[0][0]=='b')
        {
          setThrottle(0);
          lock();
          setMotos(0, 0, 0, 0);
        }
        if(mes[0][0]=='t')
        {
          int t=atoi(mes[1]);//油门
          setThrottle(t);
          setMotos(t, t, t, t);
        }
        ready=0;
        length=0;
        USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);//打开接收中断
      }
      
    MPU6050_Read();//读取原始数据
    MPU6050_Dataanl();//减去偏移量
    Prepare_AccData();//平滑加速度数据
    IMU_getValues();//将四个参数转化为float
    
    IMUupdate(GYRO_X*Gyro_Gr,GYRO_Y*Gyro_Gr,GYRO_Z*Gyro_Gr,ACC_X,ACC_Y,ACC_Z);//计算姿态角

    if(isLocked()){
       control(Q_ANGLE.Pitch, Q_ANGLE.Roll, 0.0, 0.0);
    }

 

    Delay_us(2000);//2ms
    };

  

}
/////////////////////////////////////////////////////////////////
/*该函数将缓冲区内的数据转化几个单独的字符串，并存储在某个全局变量中*/
void GetMessage()
{
 
  
  int beginPos[dataNum],endPos[dataNum];//记录每个数据的开始和结束的位置
  beginPos[0]=0;
  int num=0;
  for(int x=0;x<length;x++)
  {
    if(USART2_RECV_BUF[x]==' '||USART2_RECV_BUF[x]=='\0')
    {
      endPos[num]=x-1;
      
      num++;
      if(num==dataNum ) break;//有dataNum个数据
      beginPos[num]=x+1;
    
    }
  }
  num=0;
  char temp[20];//临时存放分割好的字符串
 // int *Data=(int*)calloc(dataNum,sizeof(int));//用于存储转化后的数据
  for(int x=0;x<dataNum;x++)
  {

    for(int y=beginPos[x];y<=endPos[x];y++)
    {
      if((USART2_RECV_BUF[y]<48&&(USART2_RECV_BUF[y]!=45&&USART2_RECV_BUF[y]!=46))||(USART2_RECV_BUF[y]>57&&USART2_RECV_BUF[y]<97)||USART2_RECV_BUF[y]>122) continue;//跳过非法字符
      temp[num]=USART2_RECV_BUF[y];
      num++;
      if(y==endPos[x])
      {
        temp[num]='\0';// 最后一位结束符
        num=0;
      }
    }
    // (*(Data+x))=atoi(temp); //转化为整数
    strcpy(mes[x],temp);//更新单一字符串
    //mes[x]=temp;//更新单一字符串
 
  }
  //return Data;

}


