#include <stm32f4_discovery.h>
#include "iic_analog.h"


/************************************************************/
/*模拟IIC引脚初始化函数*/
/************************************************************/
void I2C_Configuration( )
{
	GPIO_InitTypeDef GPIO_InitStructure;

	//使能时钟
         RCC_AHB1PeriphClockCmd (RCC_AHB1Periph_GPIOA, ENABLE);

	//配置引脚 PA6->SDA PA7->SCL
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6| GPIO_Pin_7;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOA, &GPIO_InitStructure);  

	//初始化ICC的模式
	SET_SDA;
	SET_SCL;  
}


/************************************************************/
/************************************************************/
void IIC_Delay(void)
{
	u32 i = 10;
	while( i-- );
}

/*******************************************************************
TWI_START
发送启动数据
*******************************************************************/
u8 IIC_Start(void)
{
	SET_SDA;
	IIC_DELAY;

	SET_SCL;
	IIC_DELAY;

	if( IIC_SDA_STATE == RESET )
	{
		return IIC_BUS_BUSY;
	}

	RESET_SDA;
	IIC_DELAY;

	RESET_SCL;
	IIC_DELAY;

	if( IIC_SDA_STATE == SET )
	{
		return IIC_BUS_ERROR;
	}

	return IIC_BUS_READY;
}

/*******************************************************************
TWI_STOP
发送停止数据
*******************************************************************/
void IIC_Stop(void)
{
	RESET_SDA;
	IIC_DELAY;

	SET_SCL;
	IIC_DELAY;

	SET_SDA;
	IIC_DELAY;
}

/*******************************************************************************
* 函数名称:TWI_SendNACK                                                                     
* 描    述:收到数据,发送NACK                                                                                                                                       
 *******************************************************************************/
void IIC_SendNACK(void)
{
	RESET_SDA;
	IIC_DELAY;
	SET_SCL;
	IIC_DELAY;
	RESET_SCL; 
	IIC_DELAY; 
}

/*******************************************************************************
* 函数名称:TWI_SendACK                                                                     
* 描    述:收到数据,发送ACK                                                                                                                                        
*******************************************************************************/
void IIC_SendACK(void)
{
	SET_SDA;
	IIC_DELAY;
	SET_SCL;
	IIC_DELAY;
	RESET_SCL; 
	IIC_DELAY;
}

/*******************************************************************************
 * 函数名称:TWI_SendByte                                                                     
 * 描    述:发送一个字节                                                                                                                                      
 *******************************************************************************/
u8 IIC_SendByte(u8 Data)
{
	 u8 i;
	 RESET_SCL;
	 for(i=0;i<8;i++)
	 {  
		//---------数据建立----------
		if(Data&0x80)
		{
			SET_SDA;
		}
		else
		{
			RESET_SDA;
		} 
		Data<<=1;
		IIC_DELAY;
		//---数据建立保持一定延时----
		//----产生一个上升沿[正脉冲] 
		SET_SCL;
		IIC_DELAY;
		RESET_SCL;
		IIC_DELAY;//延时,防止SCL还没变成低时改变SDA,从而产生START/STOP信号
		//---------------------------   
	 }
	 //接收从机的应答 
	 SET_SDA; 
	 IIC_DELAY;
	 SET_SCL;
	 IIC_DELAY;   
	 if(IIC_SDA_STATE)
	 {
		RESET_SCL;
		return IIC_NACK;
	 }
	 else
	 {
		RESET_SCL;
		return IIC_ACK;  
	 }    
}

/*******************************************************************************
 * 函数名称:TWI_ReceiveByte                                                                     
 * 描    述:接收一个字节                                                                                                                                       
 *******************************************************************************/
u8 IIC_RecvByte(void)
{
	 u8 i,Dat = 0;
	 SET_SDA;
	 RESET_SCL; 
	 Dat=0;
	 for(i=0;i<8;i++)
	 {
		SET_SCL;//产生时钟上升沿[正脉冲],让从机准备好数据 
		IIC_DELAY; 
		Dat<<=1;
		if(IIC_SDA_STATE) //读引脚状态
		{
			Dat|=0x01; 
		}   
		RESET_SCL;//准备好再次接收数据  
		IIC_DELAY;//等待数据准备好         
	 }
	 return Dat;
}

/******单字节写入*******************************************/
void Single_Write_IIC1(u8 SlaveAddress,u8 REG_Address,u8 REG_data)
{
    IIC_Start();                  //起始信号
    IIC_SendByte(SlaveAddress);   //发送设备地址+写信号
    IIC_SendByte(REG_Address);    //内部寄存器地址， //请参考中文pdf22页 
    IIC_SendByte(REG_data);       //内部寄存器数据， //请参考中文pdf22页 
    IIC_Stop();                   //发送停止信号
}

/********单字节读取*****************************************/
u8 Single_Read_IIC1(u8 SlaveAddress, u8 REG_Address)
{  
	u8 REG_data;
    IIC_Start();                          //起始信号
    IIC_SendByte(SlaveAddress);           //发送设备地址+写信号
    IIC_SendByte(REG_Address);            //发送存储单元地址，//从0开始	
    IIC_Start();                          //起始信号
    IIC_SendByte(SlaveAddress+1);         //发送设备地址+读信号
    REG_data = IIC_RecvByte();              //读出寄存器数据
	IIC_SendACK();   
	IIC_Stop();                           //停止信号
    return REG_data; 
}

