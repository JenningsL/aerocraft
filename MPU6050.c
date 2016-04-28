/******************************************************************************/
//MPU6050.c
//陀螺仪MPU6050的相关函数
//
/******************************************************************************/
#include <stm32f4_discovery.h>
#include "MPU6050.h"
MPU6050_RAW_DATA  MPU6050_LAST={0,0,0,0,0,0};
float ACC_OFFSET_X=0, ACC_OFFSET_Y=0, ACC_OFFSET_Z=0, GYRO_OFFSET_X=0, GYRO_OFFSET_Y=0, GYRO_OFFSET_Z=0;//偏移量
int GYRO_OFFSET_OK=1,ACC_OFFSET_OK=1;//默认不进行校正
/*********以下是MPU6050的相关函数**********************/
void MPU6050_Inital(void)
{
	Delay_us( 100000 );
	Single_Write_IIC1( SLAVEADRESS , PWR_MGMT_1 , 0x00 );
	Single_Write_IIC1( SLAVEADRESS , SMPLRT_DIV , 0x07 );
	Single_Write_IIC1( SLAVEADRESS , CONFIG , 0x07 );
	Single_Write_IIC1( SLAVEADRESS , GYRO_CONFIG , 0x18 );
	Single_Write_IIC1( SLAVEADRESS , ACCEL_CONFIG , 0x01 );
	Delay_us( 100000 );
}
short getAccX(void)
{
	short AccX = 0;
	char AccXH = 0 , AccXL = 0;

	AccXH = Single_Read_IIC1( SLAVEADRESS , ACCEL_XOUT_H );
	AccXL = Single_Read_IIC1( SLAVEADRESS , ACCEL_XOUT_L );

	AccX = (AccXH<<8)|AccXL;

	return AccX;
}

short getAccY(void)
{
	short AccY = 0;
	char AccYH = 0 , AccYL = 0;

	AccYH = Single_Read_IIC1( SLAVEADRESS , ACCEL_YOUT_H );
	AccYL = Single_Read_IIC1( SLAVEADRESS , ACCEL_YOUT_L );

	AccY = (AccYH<<8)|AccYL;

	return AccY;
}

short getAccZ(void)
{
	short AccZ = 0;
	char AccZH = 0 , AccZL = 0;

	AccZH = Single_Read_IIC1( SLAVEADRESS , ACCEL_ZOUT_H );
	AccZL = Single_Read_IIC1( SLAVEADRESS , ACCEL_ZOUT_L );

	AccZ = (AccZH<<8)|AccZL;

	return AccZ;
}

short getGyroX(void)
{
	short GyroX = 0;
	char GyroXH = 0 , GyroXL = 0; 
	
	GyroXH = Single_Read_IIC1( SLAVEADRESS , GYRO_XOUT_H );
	GyroXL = Single_Read_IIC1( SLAVEADRESS , GYRO_XOUT_H );
	
	GyroX = (GyroXH<<8)|GyroXL;
	
	return GyroX;	
}

short getGyroY(void)
{
   	short GyroY = 0;
	char GyroYH = 0 , GyroYL = 0; 
	
	GyroYH = Single_Read_IIC1( SLAVEADRESS , GYRO_YOUT_H );
	GyroYL = Single_Read_IIC1( SLAVEADRESS , GYRO_YOUT_H );
	
	GyroY = (GyroYH<<8)|GyroYL;
	
	return GyroY;	
}

short getGyroZ(void)
{
   	short GyroZ = 0;
	char GyroZH = 0 , GyroZL = 0; 
	
	GyroZH = Single_Read_IIC1( SLAVEADRESS , GYRO_ZOUT_H );
	GyroZL = Single_Read_IIC1( SLAVEADRESS , GYRO_ZOUT_H );
	
	GyroZ = (GyroZH<<8)|GyroZL;
	
	return GyroZ;	
}

short getTemperature(void)
{
 	short temperature = 0;
	char temperatureH = 0 , temperatureL = 0;

	temperatureH = Single_Read_IIC1( SLAVEADRESS , TEMP_OUT_H );
	temperatureL = Single_Read_IIC1( SLAVEADRESS , TEMP_OUT_L );

	temperature = (temperatureH<<8)|temperatureL;

	return temperature;
}
short getID(void)
{
  short id = 0;


	id = Single_Read_IIC1( SLAVEADRESS , WHO_AM_I );


	return id;
}
void MPU6050_Read(void)
{
  MPU6050_LAST.ACCx=getAccX();
  MPU6050_LAST.ACCy=getAccY();
  MPU6050_LAST.ACCz=getAccZ();
  MPU6050_LAST.GYROx=getGyroX();
  MPU6050_LAST.GYROy=getGyroY();
  MPU6050_LAST.GYROz=getGyroZ();
}
/***********************************************************/
//对MPU6050的数据进行简单处理,校正偏移
/***********************************************************/
void MPU6050_Dataanl(void)
{
	 MPU6050_LAST.ACCx-= ACC_OFFSET_X;
	 MPU6050_LAST.ACCy-= ACC_OFFSET_Y;
	 MPU6050_LAST.ACCz-= ACC_OFFSET_Z;
	//跳过温度ADC
	MPU6050_LAST.GYROx-= GYRO_OFFSET_X;
	MPU6050_LAST.GYROy-= GYRO_OFFSET_Y;
	MPU6050_LAST.GYROz-=GYRO_OFFSET_Z;//减去偏移量
	
	if(!GYRO_OFFSET_OK)
	{
		static int32_t	tempgx=0,tempgy=0,tempgz=0;
		static uint8_t cnt_g=0;
		
		if(cnt_g==0)
		{
			GYRO_OFFSET_X=0;
			GYRO_OFFSET_Y=0;
			GYRO_OFFSET_Z=0;
			tempgx = 0;
			tempgy = 0;
			tempgz = 0;
			cnt_g = 1;
			return;
		}
		tempgx+= MPU6050_LAST.GYROx;
		tempgy+= MPU6050_LAST.GYROy;
		tempgz+= MPU6050_LAST.GYROz;
		if(cnt_g==200)
		{
			GYRO_OFFSET_X=tempgx/cnt_g;
			GYRO_OFFSET_Y=tempgy/cnt_g;
			GYRO_OFFSET_Z=tempgz/cnt_g;
			cnt_g = 0;
			GYRO_OFFSET_OK = 1;
			return;
		}
		cnt_g++;
	}
	if(!ACC_OFFSET_OK)
	{
		static int32_t	tempax=0,tempay=0,tempaz=0;
		static uint8_t cnt_a=0;
		
		if(cnt_a==0)
		{
			ACC_OFFSET_X = 0;
			ACC_OFFSET_Y = 0;
			ACC_OFFSET_Z = 0;
			tempax = 0;
			tempay = 0;
			tempaz = 0;
			cnt_a = 1;
			return;
		}
		tempax+= MPU6050_LAST.ACCx;
		tempay+= MPU6050_LAST.ACCy;
		//tempaz+= MPU6050_ACC_LAST.Z;
		if(cnt_a==200)
		{
			ACC_OFFSET_X=tempax/cnt_a;
			ACC_OFFSET_Y=tempay/cnt_a;
			ACC_OFFSET_Z=tempaz/cnt_a;
			cnt_a = 0;
			ACC_OFFSET_OK = 1;
			return;
		}
		cnt_a++;		
	}//计算偏移量，在校正时调用一次
}
//校正偏移
void adjustOffset(void)
{
   GYRO_OFFSET_OK=0;
   ACC_OFFSET_OK=0;
   for(int x=0;x<200;x++)
   {
     MPU6050_Read();
     MPU6050_Dataanl();
   }
}