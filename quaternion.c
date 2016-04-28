/******************************************************************************/
//quanternion.c
//利用四元数将陀螺仪的六个原始数据转化为欧拉角(Pitch,Roll)
//
/******************************************************************************/
#include "quaternion.h"
#include "math.h"
extern float ACC_X,ACC_Y,ACC_Z,GYRO_X,GYRO_Y,GYRO_Z;//传入四元数函数计算的六个数据,在MAIN.c中声明
extern float q[];
FLOAT_ANGLE  Q_ANGLE={0,0,0};//存放计算得出的欧拉角
/******************************************************************************/
//在调用Prepare_Data前先调用这个函数将ACC_X_BUF,ACC_Y_BUF,ACC_Z_BUF填上数据
/******************************************************************************/
void fill_ACC_BUF()
{
  for(int x=0;x<FILTER_NUM;x++)
  {
    MPU6050_Read();
    MPU6050_Dataanl();
    ACC_X_BUF[x] = MPU6050_LAST.ACCx;
    ACC_Y_BUF[x] = MPU6050_LAST.ACCy;
    ACC_Z_BUF[x] = MPU6050_LAST.ACCz;
  }
}
/******************************************************************************/
//将加速度数据求均值平滑
/******************************************************************************/
void Prepare_AccData(void)
{
	static uint8_t 	filter_cnt=0;
	
	int32_t temp1=0,temp2=0,temp3=0;
	uint8_t i;

        MPU6050_Read();
        MPU6050_Dataanl();
        
	ACC_X_BUF[filter_cnt] = MPU6050_LAST.ACCx;
	ACC_Y_BUF[filter_cnt] = MPU6050_LAST.ACCy;
	ACC_Z_BUF[filter_cnt] = MPU6050_LAST.ACCz;
	for(i=0;i<FILTER_NUM;i++)
	{
		temp1 += ACC_X_BUF[i];
		temp2 += ACC_Y_BUF[i];
		temp3 += ACC_Z_BUF[i];
	}
	ACC_X_INT = temp1 / FILTER_NUM;
	ACC_Y_INT = temp2 / FILTER_NUM;
	ACC_Z_INT = temp3 / FILTER_NUM;
	filter_cnt++;
	if(filter_cnt==FILTER_NUM)	filter_cnt=0;
}
void IMU_getValues(void) {  
	ACC_X=(float) ACC_X_INT;
        ACC_Y=(float) ACC_Y_INT;
        ACC_Z=(float) ACC_Z_INT;
        GYRO_X=(float)  MPU6050_LAST.GYROx;
        GYRO_Y=(float)  MPU6050_LAST.GYROy;
        GYRO_Z=(float)  MPU6050_LAST.GYROz;


}
////////////////////////////////////////////////////////////////////////////////
/*四元数计算姿态角*/
////////////////////////////////////////////////////////////////////////////////
#define Kp 10.0f                        
#define Ki 0.008f                         
#define halfT 0.005f   //更新时间的一半              

float q0 = 1.0f, q1 = 0.0, q2 = 0.0, q3 = 0.0;   
float exInt = 0.0, eyInt = 0.0, ezInt = 0.0;  
		
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az)
{
  float norm;
  float hx, hy, hz, bx, bz;
  float vx, vy, vz;// wx, wy, wz;
  float ex, ey, ez;

  float q0q0 = q0*q0;
  float q0q1 = q0*q1;
  float q0q2 = q0*q2;
  float q0q3 = q0*q3;
  float q1q1 = q1*q1;
  float q1q2 = q1*q2;
  float q1q3 = q1*q3;
  float q2q2 = q2*q2;
  float q2q3 = q2*q3;
  float q3q3 = q3*q3;

  norm = sqrt(ax*ax + ay*ay + az*az);      
  ax = ax /norm;
  ay = ay / norm;
  az = az / norm;//归一化，得到单位加速度
           
  vx = 2*(q1q3 - q0q2);												
  vy = 2*(q0q1 + q2q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3 ;

  ex = (ay*vz - az*vy) ;                           					
  ey = (az*vx - ax*vz) ;
  ez = (ax*vy - ay*vx) ;

  exInt = exInt + ex * Ki;								 
  eyInt = eyInt + ey * Ki;
  ezInt = ezInt + ez * Ki;

  gx = gx + Kp*ex + exInt;					   							
  gy = gy + Kp*ey + eyInt;
  gz = gz + Kp*ez + ezInt;				   							
					   
  q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
  q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
  q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
  q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;

  norm = sqrt(q0q0 + q1q1 + q2q2 + q3q3);
  q0 = q0 / norm;
  q1 = q1 / norm;
  q2 = q2 / norm;
  q3 = q3 / norm;
  

  Q_ANGLE.Pitch  = asin(-2 * q1q3 + 2 * q0q2)* 57.3; // pitch
  Q_ANGLE.Roll = atan2(2 * q2q3 + 2 * q0q1, -2 * q1q1 - 2 * q2q2 + 1)* 57.3; // roll

}