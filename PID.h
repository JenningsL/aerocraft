#ifndef __PID_H__
#define __PID_H__


typedef struct PID
{
    float IMAX;
  
    float  SetPoint;     //�趨Ŀ�� Desired Value
    long SumError;                //����ۼ�

    float  Proportion;         //�������� Proportional Cons
    float  Integral;           //���ֳ��� Integral Const
    float  Derivative;         //΢�ֳ��� Derivative Const

    float LastError;               //Error[-1]
    float PrevError;               //Error[-2]
    
    float iIncpid;                   //����ֵ
    
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