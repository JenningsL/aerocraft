#ifndef __PID_H__
#define __PID_H__


typedef struct PID
{
    float IMAX;
  
    float  SetPoint;     //设定目标 Desired Value
    long SumError;                //误差累计

    float  Proportion;         //比例常数 Proportional Cons
    float  Integral;           //积分常数 Integral Const
    float  Derivative;         //微分常数 Derivative Const

    float LastError;               //Error[-1]
    float PrevError;               //Error[-2]
    
    float iIncpid;                   //增量值
    
} PID;
/*
typedef struct PID
{
	int IMAX;

	float P;
	float I;
	float D;

	float pout;
	float iout;
	float dout;

	int out;	
} PID;
*/
extern PID padjust,radjust;

void IncPIDInit();
void IncPIDCalc(float pNextPoint, float rNextPoint);
void LocPIDCalc(float pNextPoint,float rNextPoint);
void PIDSetIMAX();
void PIDSetPoint(float,float);
void PIDSetPp(float);
void PIDSetPi(float);
void PIDSetPd(float);
void PIDSetRp(float);
void PIDSetRi(float);
void PIDSetRd(float);

#endif