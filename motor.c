#include "motor.h"

const int THROTTLE_UNLOCK = 1700;
const int THROTTLE_MAX = 3500;
const int THROTTLE_MIN = 2000;

int throttle = 0;
u16 moto1=0,moto2=0,moto3=0,moto4=0;
int perr = 0,rerr = 0;
extern PID padjust,radjust;
int unLocked = 0;

void TIM4_GPIO_Config();
void TIM4_Config(void)
{
  TIM4_GPIO_Config();
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;
  uint16_t CCR1_Val = 0;//2000对应1ms
  uint16_t CCR2_Val = 0;
  uint16_t CCR3_Val = 0;
  uint16_t CCR4_Val = 0;
  uint16_t PrescalerValue = 0;

  TIM_TimeBaseStructure.TIM_Period = 39999;//一个周期为20ms
  TIM_TimeBaseStructure.TIM_Prescaler = 41;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

  /* PWM1 Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR1_Val;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  TIM_OC1Init(TIM4, &TIM_OCInitStructure);

  TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel2 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR2_Val;

  TIM_OC2Init(TIM4, &TIM_OCInitStructure);

  TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel3 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR3_Val;

  TIM_OC3Init(TIM4, &TIM_OCInitStructure);

  TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel4 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR4_Val;

  TIM_OC4Init(TIM4, &TIM_OCInitStructure);

  TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);

  TIM_ARRPreloadConfig(TIM4, ENABLE);

  /* TIM4 enable counter */
  TIM_Cmd(TIM4, ENABLE);
}
void TIM4_GPIO_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* TIM4 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

  /* GPIOC and GPIOB clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
  
  /* GPIOC Configuration: TIM4 CH1 (PD12) and TIM4 CH2 (PD13) */ /* GPIOB Configuration:  TIM4 CH3 (PD14) and TIM4 CH4 (PD15) */
 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOD, &GPIO_InitStructure); 
  


  /* Connect TIM4 pins to AF2 */  
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4); 
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_TIM4);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_TIM4); 
}
/*
 * 函数名：TIM2_Config
 * 描述  ：TIM2做定时中断
 * 输入  ：无
 * 输出  ：无
 * 调用  ：内部调用
 */
void TIM2_Config(void)
{
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  /* Time base configuration */    
  TIM_TimeBaseStructure.TIM_Period = 100;       //为一个定时周期0.01s
  TIM_TimeBaseStructure.TIM_Prescaler = 16799;     //设置预分频,10KHZ 
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1 ;  //设置时钟分频系数：不分频
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //向上计数模式
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
  TIM_ITConfig(TIM2,TIM_IT_Update|TIM_IT_Trigger,ENABLE);//使能溢出中断

  TIM_ARRPreloadConfig(TIM2, ENABLE);      // 使能TIM2重载寄存器ARR

  /* TIM2 enable counter */
  TIM_Cmd(TIM2, ENABLE);                   //使能定时器2
  TIM2_NVIC_Configuare();
}

//NVIC设置 系统中断管理
void TIM2_NVIC_Configuare(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x03;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;
    NVIC_InitStructure.NVIC_IRQChannel  = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        
    NVIC_Init(&NVIC_InitStructure);//TIM2 configuration 
}

void setMotos(int moto1, int moto2, int moto3, int moto4) {

  
    // limit max throttle
  moto1 = min(moto1, THROTTLE_MAX);
  moto2 = min(moto2, THROTTLE_MAX);
  moto3 = min(moto3, THROTTLE_MAX);
  moto4 = min(moto4, THROTTLE_MAX);
  //limit min throttle
  moto1 = max(moto1, 0);
  moto2 = max(moto2, 0);
  moto3 = max(moto3, 0);
  moto4 = max(moto4, 0);

  moto2 = moto2 + 40;
  moto4 = moto4 + 50;
  
  TIM_SetCompare1(TIM4, moto1);
  TIM_SetCompare2(TIM4, moto2);
  TIM_SetCompare3(TIM4, moto3);
  TIM_SetCompare4(TIM4, moto4);
}

void setThrottle(int t) {
  t = max(t, 0);
  t = min(t, THROTTLE_MAX);
  throttle = t;
}

int getThrottle() {
  return throttle;
}

void unLock(void) {
  for(int x=0;x<THROTTLE_UNLOCK;x+=100)
  {
    Delay_us(5000);
    setMotos(x, x, x ,x);
  }
  throttle = THROTTLE_UNLOCK;
  unLocked = 1;
  Delay_us(5000000); //5s
}

void lock(void) {
  unLocked = 0;
}

int isLocked() {
  return unLocked;
}

void control(float pitch, float roll, float pitch_set, float roll_set)
{
  
  PIDSetPoint(pitch_set, roll_set);
  
  //IncPIDCalc(pitch, roll);//增量式
  LocPIDCalc(pitch,roll);//位置式
  perr = padjust.iIncpid;
  rerr = radjust.iIncpid;
  if(perr>padjust.IMAX)
    perr=padjust.IMAX;
  if(perr<-padjust.IMAX)
    perr=-padjust.IMAX;
  if(rerr>radjust.IMAX)
    rerr=radjust.IMAX;
  if(rerr<-radjust.IMAX)
    rerr=-radjust.IMAX; 
  
  moto1 = throttle + perr + rerr;
  moto2 = throttle - perr - rerr ;
  moto3 = throttle - perr + rerr ;
  moto4 = throttle + perr - rerr;
  
  // limit max throttle
  moto1 = min(moto1, THROTTLE_MAX);
  moto2 = min(moto2, THROTTLE_MAX);
  moto3 = min(moto3, THROTTLE_MAX);
  moto4 = min(moto4, THROTTLE_MAX);
  //limit min throttle
  moto1 = max(moto1, THROTTLE_MIN);
  moto2 = max(moto2, THROTTLE_MIN);
  moto3 = max(moto3, THROTTLE_MIN);
  moto4 = max(moto4, THROTTLE_MIN);
  
  setMotos(moto1, moto2, moto3, moto4);
}