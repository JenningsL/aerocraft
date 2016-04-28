#include <stm32f4_discovery.h>
#include "MPU6050.h"
#define RtA 		57.324841f			//???èμ????è	
#define AtR    		0.0174533f			//?èμ????è	
#define Acc_G 		0.0011963f				//?ó?ù?è±?3éG
#define Gyro_G 		0.0610351f		//???ù?è±?3é?è￡?′?2?êy??ó|íó?Y2000?è????	
#define Gyro_Gr		0.0010653f  //???ù?è±?3é???è￡?′?2?êy??ó|íó?Y2000?è????			
#define FILTER_NUM 	20

static short ACC_X_INT,ACC_Y_INT,ACC_Z_INT;

static short ACC_X_BUF[FILTER_NUM],ACC_Y_BUF[FILTER_NUM],ACC_Z_BUF[FILTER_NUM];//滑动窗口数组

typedef struct 
{
  float Pitch;
  float Roll;
  float Yaw;
}FLOAT_ANGLE;

extern FLOAT_ANGLE  Q_ANGLE;//经过四元数解算之后得到的姿态角
void fill_ACC_BUF();
void Prepare_AccData(void);
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az);
void IMU_getValues(void);
//extern struct MPU6050_RAW_DATA MPU6050_LAST;