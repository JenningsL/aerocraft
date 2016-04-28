#include "PID.h"
//volatile PID   sPID;
//volatile PID   *sptr = &sPID;
PID padjust,radjust;
/*******************************************************************************
* �������� : IncPIDInit
* �������� : PID ������ʼ��
* �������� : ��
* ������� : ��
* �������� : ��
*******************************************************************************/

void IncPIDInit()
{
    padjust.SumError  = 0;
    padjust.LastError = 0;    //Error[-1]
    padjust.PrevError = 0;    //Error[-2]

    padjust.Proportion = 0;    //�������� Proportional Const
    padjust.Integral   = 0;    //���ֳ��� Integral Const
    padjust.Derivative = 0;    //΢�ֳ��� Derivative Const
    padjust.SetPoint   = 0;
    padjust.IMAX = 1000;
    
    radjust.SumError  = 0;
    radjust.LastError = 0;    //Error[-1]
    radjust.PrevError = 0;    //Error[-2]

    radjust.Proportion = 0;    //�������� Proportional Const
    radjust.Integral   = 0;    //���ֳ��� Integral Const
    radjust.Derivative = 0;    //΢�ֳ��� Derivative Const
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
* �������� : IncPIDCalc
* �������� : ����ʽ PID ���Ƽ���
* �������� : int ��ǰλ��
* ������� : ��
* �������� : ����ʽPID���
*******************************************************************************/
float iError,dError;//dError�Ǹ�λ��ʽ�õ�
void IncPIDCalc(float pNextPoint, float rNextPoint)
{
    //��ǰ���
    iError = padjust.SetPoint - pNextPoint;
    //��������
    padjust.iIncpid = (padjust.Proportion * iError)               //E[k]��
              - (padjust.Integral   * padjust.LastError)     //E[k��1]��
              + (padjust.Derivative * padjust.PrevError);   //E[k��2]��
    //�洢�������´μ���
    padjust.PrevError = padjust.LastError;
    padjust.LastError = iError;
    
    iError = radjust.SetPoint - rNextPoint;
    //��������
    radjust.iIncpid = (radjust.Proportion * iError)               //E[k]��
              - (radjust.Integral   * radjust.LastError)     //E[k��1]��
              + (radjust.Derivative * radjust.PrevError);   //E[k��2]��
    //�洢�������´μ���
    radjust.PrevError = radjust.LastError;
    radjust.LastError = iError;    
}
//λ��ʽPID�������
void LocPIDCalc(float pNextPoint,float rNextPoint)
{
  //��roll�ǵļ���
  iError = radjust.SetPoint - rNextPoint; //ƫ��
  radjust.SumError += iError; //����
  dError = iError - radjust.LastError; //΢��
  radjust.LastError = iError;
  radjust.iIncpid=(radjust.Proportion * iError //������
    + radjust.Integral * radjust.SumError //������
    + radjust.Derivative * dError); //΢����
  
  //��pitch�ǵļ���
  iError = padjust.SetPoint - pNextPoint; //ƫ��
  padjust.SumError += iError; //����
  dError = iError - padjust.LastError; //΢��
  padjust.LastError = iError;
  padjust.iIncpid=(padjust.Proportion * iError //������
    + padjust.Integral * padjust.SumError //������
    + padjust.Derivative * dError); //΢����
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
