#include "PID.h"
//volatile PID   sPID;
//volatile PID   *sptr = &sPID;
PID padjust,radjust;
/*******************************************************************************
* 函数名称 : IncPIDInit
* 函数描述 : PID 参数初始化
* 函数输入 : 无
* 函数输出 : 无
* 函数返回 : 无
*******************************************************************************/

void IncPIDInit()
{
    padjust.SumError  = 0;
    padjust.LastError = 0;    //Error[-1]
    padjust.PrevError = 0;    //Error[-2]

    padjust.Proportion = 0;    //比例常数 Proportional Const
    padjust.Integral   = 0;    //积分常数 Integral Const
    padjust.Derivative = 0;    //微分常数 Derivative Const
    padjust.SetPoint   = 0;
    padjust.IMAX = 1000;
    
    radjust.SumError  = 0;
    radjust.LastError = 0;    //Error[-1]
    radjust.PrevError = 0;    //Error[-2]

    radjust.Proportion = 0;    //比例常数 Proportional Const
    radjust.Integral   = 0;    //积分常数 Integral Const
    radjust.Derivative = 0;    //微分常数 Derivative Const
    radjust.SetPoint   = 0;
    radjust.IMAX = 1000;
}
/*
void IncPIDInit()
{
	PID_ROL.P = 0;
	PID_ROL.I = 0;
	PID_ROL.D = 0;

	PID_ROL.IMAX = 1000;

	PID_ROL.pout = 0;
	PID_ROL.iout = 0;
	PID_ROL.dout = 0;
	PID_ROL.out = 0;

	PID_PIT.P = 0;
	PID_PIT.I = 0;
	PID_PIT.D = 0;

	PID_PIT.IMAX = 1000;

	PID_PIT.pout = 0;
	PID_PIT.iout = 0;
	PID_PIT.dout = 0;
	PID_PIT.out = 0;
}
*/
/*******************************************************************************
* 函数名称 : IncPIDCalc
* 函数描述 : 增量式 PID 控制计算
* 函数输入 : int 当前位置
* 函数输出 : 无
* 函数返回 : 增量式PID结果
*******************************************************************************/
float iError,dError;//dError是给位置式用的
void IncPIDCalc(float pNextPoint, float rNextPoint)
{
    //当前误差
    iError = padjust.SetPoint - pNextPoint;
    //增量计算
    padjust.iIncpid = (padjust.Proportion * iError)               //E[k]项
              - (padjust.Integral   * padjust.LastError)     //E[k－1]项
              + (padjust.Derivative * padjust.PrevError);   //E[k－2]项
    //存储误差，用于下次计算
    padjust.PrevError = padjust.LastError;
    padjust.LastError = iError;
    
    iError = radjust.SetPoint - rNextPoint;
    //增量计算
    radjust.iIncpid = (radjust.Proportion * iError)               //E[k]项
              - (radjust.Integral   * radjust.LastError)     //E[k－1]项
              + (radjust.Derivative * radjust.PrevError);   //E[k－2]项
    //存储误差，用于下次计算
    radjust.PrevError = radjust.LastError;
    radjust.LastError = iError;    
}
//位置式PID控制设计
void LocPIDCalc(float pNextPoint,float rNextPoint)
{
  //对roll角的计算
  iError = radjust.SetPoint - rNextPoint; //偏差
  radjust.SumError += iError; //积分
  dError = iError - radjust.LastError; //微分
  radjust.LastError = iError;
  radjust.iIncpid=(radjust.Proportion * iError //比例项
    + radjust.Integral * radjust.SumError //积分项
    + radjust.Derivative * dError); //微分项
  
  //对pitch角的计算
  iError = padjust.SetPoint - pNextPoint; //偏差
  padjust.SumError += iError; //积分
  dError = iError - padjust.LastError; //微分
  padjust.LastError = iError;
  padjust.iIncpid=(padjust.Proportion * iError //比例项
    + padjust.Integral * padjust.SumError //积分项
    + padjust.Derivative * dError); //微分项
}
void PIDSetPoint(float pSetpoint, float rSetpoint)
{
  padjust.SetPoint=pSetpoint;
  radjust.SetPoint=rSetpoint;
}

void PIDSetPp(float pKp)
{
  padjust.Proportion=pKp;
}

void PIDSetPi(float pKi)
{
  padjust.Integral=pKi;
}

void PIDSetPd(float pKd)
{
  padjust.Derivative=pKd;
}

void PIDSetRp(float rKp)
{
  radjust.Proportion=rKp;
}

void PIDSetRi(float rKi)
{
  radjust.Integral=rKi;
}

void PIDSetRd(float rKd)
{
  radjust.Derivative=rKd;
}

void PIDSetIMAX()
{
   padjust.IMAX = 200;
   radjust.IMAX = 200;
}
