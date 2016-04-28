#ifndef _MPU6050_h_
#define _MPU6050_h_

#define	SMPLRT_DIV		0x19	//陀螺仪采样率，典型值：0x07(125Hz)
#define	CONFIG			0x1A	//低通滤波频率，典型值：0x06(5Hz)
#define	GYRO_CONFIG		0x1B	//陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
#define	ACCEL_CONFIG	0x1C	//加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz)
#define	ACCEL_XOUT_H	0x3B	
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40
#define	TEMP_OUT_H		0x41
#define	TEMP_OUT_L		0x42
#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44	
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48
#define	PWR_MGMT_1		0x6B	//电源管理，典型值：0x00(正常启用)
#define	WHO_AM_I		0x75	//IIC地址寄存器(默认数值0x68，只读)
#define	SLAVEADRESS		0xD0	//IIC写入时的地址字节数据，+1为读取

typedef struct MPU6050_RAW_DATA
{
  short ACCx;
  short ACCy;
  short ACCz;
  
  short GYROx;
  short GYROy;
  short GYROz;
} MPU6050_RAW_DATA ;//最后一次MPU6050读取的数据
extern MPU6050_RAW_DATA  MPU6050_LAST;

extern void MPU6050_Inital(void);


//获取加速度计的值
extern short getAccX(void);
extern short getAccY(void);
extern short getAccZ(void);

//获取陀螺仪的值
extern short getGyroX(void);
extern short getGyroY(void);
extern short getGyroZ(void);

//获取温度
extern short getTemperature(void);
extern short getID(void);

//读取MPU6050的数据
void MPU6050_Read(void);
void MPU6050_Dataanl(void);

void adjustOffset(void);
#endif

