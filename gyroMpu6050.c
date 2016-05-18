#include "gyroMpu6050.h"

const int MPU_addr=0x68; 
int a=5;
uint8_t buffer[14],temp;
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
float gyroNow;
float gyroResol;
int32_t GxOffSet,GyOffSet,GzOffSet;
extern float angle;

void gyroTimerHandler(void)
{
	TimerIntClear(WTIMER0_BASE, TIMER_TIMB_TIMEOUT);
	
	i2cRecvBuf(MPU_addr,GYRO_XOUT_H,6,buffer);
	GyX=(((uint16_t)buffer[0]<<8)|buffer[1]);
	GyY=(((uint16_t)buffer[2]<<8)|buffer[3]);
	GyZ=(((uint16_t)buffer[4]<<8)|buffer[5])-GzOffSet;
	
	//UARTprintf("RAWX: %d RAWY: %d RAWZ: %d \n",GyX,GyY,GyZ);
	
	gyroNow=(GyZ)/131.0f;
	
	angle-=(gyroNow/200.0f);
	if(angle>=360)angle-=360.0f;
	else if(angle<0)angle+=360.0f;
}

void gyroTimerInit(void)
{
	SetupI2C();
	
	i2cSendByte(MPU_addr,0x6B,0x00);
	
	//Initialization of MPU6050
	SysCtlDelay((SysCtlClockGet()/30));
	
	//i2cSendByte(MPU_addr,0x6B,0x03);//Gyroscope Z is clock source
	i2cSendByte(MPU6050_ADDRESS, CONFIG, 0x03);//1khz sample rate
	i2cSendByte(MPU6050_ADDRESS, SMPLRT_DIV, 4);//divide sample rate with parameter+1=5
	i2cSendByte(MPU6050_ADDRESS, GYRO_CONFIG,0);//FSB=+-250dps
	i2cSendByte(MPU6050_ADDRESS, ACCEL_CONFIG,0);//FSB=+-2g*/
	gyroResol=250.0f/32768.0f;
	SysCtlDelay((SysCtlClockGet()/3));
	
	for(a=0;a<16;a++)
	{
		i2cRecvBuf(MPU_addr,GYRO_XOUT_H,6,buffer);
		GxOffSet=GxOffSet+(int16_t)(((uint16_t)buffer[0]<<8)|buffer[1]);
		GyOffSet=GyOffSet+(int16_t)(((uint16_t)buffer[2]<<8)|buffer[3]);
		GzOffSet=GzOffSet+(int16_t)(((uint16_t)buffer[4]<<8)|buffer[5]);
		
		//UARTprintf("GYRO OFFSET: %d %d %d\n",GxOffSet,GyOffSet,GzOffSet);
		SysCtlDelay((SysCtlClockGet()/30));
	}
	
	GxOffSet=(int)(GxOffSet/16.0f);
	GyOffSet=(int)(GyOffSet/16.0f);
	GzOffSet=(int)(GzOffSet/16.0f);
	//UARTprintf("GYRO OFFSET: %d %d %d\n",GxOffSet,GyOffSet,GzOffSet);
	SysCtlDelay((SysCtlClockGet()/30));
	
	
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER0);
	
	ROM_TimerConfigure(WTIMER0_BASE, TIMER_CFG_SPLIT_PAIR|TIMER_CFG_B_PERIODIC);
	ROM_TimerLoadSet(WTIMER0_BASE, TIMER_B, 399999);
	
	TimerIntRegister(WTIMER0_BASE,TIMER_B,gyroTimerHandler);
	
	ROM_TimerEnable(WTIMER0_BASE, TIMER_B);
	
	ROM_IntEnable(INT_WTIMER0B_TM4C123);
	IntPrioritySet(INT_WTIMER0B_TM4C123, 0x40);
	ROM_TimerIntEnable(WTIMER0_BASE, TIMER_TIMB_TIMEOUT);
}
