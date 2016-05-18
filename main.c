#include <stdbool.h>          // included to use boolean data type
#include <math.h>             // standard C math library
#include <stdint.h>           // C99 variable types
#include <stdio.h>            // standard C input output library
#include <string.h>

#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_uart.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_i2c.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/uart.h"
#include "driverlib/rom.h"
#include "driverlib/i2c.h"
#include "utils/uartstdio.h"

#include "sysTickDelay.h"
#include "MotorControl.h"
#include "PWM.h"
#include "timerInitialization.h"
#include "pwmForServo.h"
#include "gyroMpu6050.h"

#define absoluteVal(my_val) ((my_val) < 0) ? -(my_val) : (my_val)

#define MIN_TO_SEC 60         // 1 minute = 60 seconds
#define SEC_TO_MSEC 1000      // 1 second = 1000 miliseconds
#define SEC_TO_USEC 1000000   // 1 second = 10^6 microseconds
#define CM_TO_M 0.01f         // 1 cm = 0.01 meter 
#define RAD_TO_DEG 57.29578f    // 2*pi radians = 360 degrees
#define PI 3.14159265359f     // PI
#define CLOCK_FREQ  80000000  // 80 MHz clock frequency      
#define R 0.022f               // radius of the wheel: 2.2 cm
#define L 0.075f               // the distance from the center of mass to the omni-wheel: 7.5 cm



uint32_t systemClock;
int STOOOP;

typedef struct 
{
	float proportional;
	float last_proportional;
	float integral;
	float derivative;
	float error;
	volatile float set_point;
	unsigned long integral_windup;
	float Kp;
	float Ki;
	float Kd;
}pid_t;

pid_t pos_ang,ang;

volatile float xMe,yMe,xOp,yOp,angle, angleOp;
volatile float xMe_, yMe_, xOp_, yOp_, angle_;
float w[3] = {0,0,0};
bool dir[3] = {0,0,0};
void convertFromRobotVelToMotorVel(float Vx, float Vy, float Vang, float theta, float *ptr);
float angle_target;
bool flag = 0;
const int angle_offset = 0;
const int array_length = 11;
int route_x[array_length] = {60,40,60,40,60,40,55,40,60,40,50};
int route_y[array_length] = {60,42,60,41,45,57,50,52,38,52,50};
int pos_index = 0;
float pwm_offset = 250.0f;
float torque = 814.0f;
float pos_error = 0;
float xFire = 50.0f, yFire= 75.0f;
//bool mode = 0;
int mode = 0; // 0: Escape maneuvers
//				  // 1: Go to (xFire, yFire)
//				  // 2: Point forward
//				  // 3: Aim at target

//Error Variables
uint32_t errorPeriod=24000000;

//Variables of firing mechanism
char mssg[3];
char cmpStr[3];

int servoHold=2000;//2000;
int servoMin=300;
int servoFire=2750;

int servoPosition;

int direction=1;
int didIFire=0;
bool fireFlag;

float maxAngleError=15.0f;
uint32_t waitForFirePeriod=80000000*0.5;

void UART2Setup(uint32_t baudRate);
void UART4Setup(uint32_t baudRate);

uint32_t pidPeriod=80000;
void wTimer3BInit(uint32_t period); // Controller timer init
void timer1AInit(uint32_t period);
void timer1AHandler(void);

void ctrlHandler(void)
{
	ROM_TimerIntClear(WTIMER3_BASE, TIMER_TIMB_TIMEOUT);
	
	//GPIOPinWrite(GPIO_PORTD_BASE,GPIO_PIN_0,GPIO_PIN_0);
	if (mode == 0)
	{
		/*Motor1(0, dir[0]);
		Motor2(0, dir[1]);
		Motor3(0, dir[2]);*/
		
		pos_ang.error = 0;

		angle_target = atan2f(route_y[pos_index]-yMe,route_x[pos_index]-xMe) * RAD_TO_DEG;
		convertFromRobotVelToMotorVel(2800, 0,pos_ang.error, angle_target-angle+angle_offset, w);
		Motor1(w[0], dir[0]);
		Motor2(w[1], dir[1]);
		Motor3(w[2], dir[2]);	
		//sysTickDelay(1);
		
		pos_error = sqrt((route_y[pos_index]-yMe)*(route_y[pos_index]-yMe) + (route_x[pos_index]-xMe)*(route_x[pos_index]-xMe));

		if(pos_error < 10) 
		{
			pos_index++;
			// If last point, stop
			if(pos_index == array_length) pos_index=0;
			//pos_index %= array_length;
	//		pos.integral = 0;
	//		pos.last_proportional = 0;
		}
	}
	else if (mode == 1) // Go to (xFire, yFire) w/ constant speed
	{
		angle_target = atan2f(yFire-yMe, xFire-xMe) * RAD_TO_DEG;
		convertFromRobotVelToMotorVel(1500, 0, 0, angle_target-angle+angle_offset, w);
		Motor1(w[0], dir[0]);
		Motor2(w[1], dir[1]);
		Motor3(w[2], dir[2]);
		pos_error = sqrt((yFire-yMe)*(yFire-yMe) + (xFire-xMe)*(xFire-xMe));
		if(pos_error < 4) 
		{
			Motor1(0, dir[0]);
			Motor2(0, dir[1]);
			Motor3(0, dir[2]);
			mode = 2;
		}
	}
	else if (mode == 2) // Aim forward
	{
		ang.proportional = 90 - angle;
		if(ang.proportional > 180)
			ang.proportional -= 360;
		else if(ang.proportional <= -180)
			ang.proportional += 360;
		if (ang.proportional > 0)
			convertFromRobotVelToMotorVel(0, 0, 400, angle_offset, w);
		else
			convertFromRobotVelToMotorVel(0, 0, -400, angle_offset, w);
		Motor1(w[0], dir[0]);
		Motor2(w[1], dir[1]);
		Motor3(w[2], dir[2]);
		
		if (absoluteVal(ang.proportional) < 10)
		{
			mode = 3;
			ROM_TimerEnable(TIMER1_BASE, TIMER_A); //wait for small angle error timer
		}
	}
	else if (mode == 3){
		angleOp = atan2f(yOp-yMe,xOp-xMe) * RAD_TO_DEG;
		ang.proportional = angleOp - angle;

		if(ang.proportional > 180)
			ang.proportional -= 360;
		else if(ang.proportional <= -180)
			ang.proportional += 360;

		ang.integral += ang.proportional;
		ang.derivative = ang.proportional - ang.last_proportional;
		ang.last_proportional = ang.proportional;	
		ang.error = ang.Kp*ang.proportional + ang.Ki*ang.integral + ang.Kd*ang.derivative;			

		if(absoluteVal(ang.proportional)>maxAngleError)
		{
			ROM_TimerLoadSet(TIMER1_BASE, TIMER_A, waitForFirePeriod-1);
		}
		convertFromRobotVelToMotorVel(0, 0, ang.error, angle_offset, w);
		Motor1(w[0], dir[0]);
		Motor2(w[1], dir[1]);
		Motor3(w[2], dir[2]);
	}
	
	//GPIOPinWrite(GPIO_PORTD_BASE,GPIO_PIN_0,!(GPIO_PIN_0));
}

void fireHandler(void)
{
	UARTIntClear(UART2_BASE,UART_INT_RX);
	
	mssg[0]=UARTCharGetNonBlocking(UART2_BASE);
	mssg[1]=UARTCharGetNonBlocking(UART2_BASE);
	mssg[2]='\0';
	if(!strcmp(mssg,cmpStr))//speed=((speed+50)<=3500)?speed+50:3500;
	{
		//fireFlag=true;
		
		/*pwmB5Generate(2000);
		sysTickDelay(500);
		pwmB4Generate(servoFire);
		sysTickDelay(500);
		pwmB4Generate(servoHold);
		ROM_TimerEnable(TIMER2_BASE, TIMER_A);
		pwmB5Generate(1250);*/
		mode = 1;
		//Motor1(0,0);
		//Motor2(0,0);
		//Motor3(0,0);
		//while(1);
		
		
		//ROM_TimerEnable(TIMER1_BASE, TIMER_A);//wait for small angle error timer
	}
}
void UART4Handler()
{
  UARTIntClear(UART4_BASE,UART_INT_RX);
	if(!flag)
	{
		flag = 1;
		ROM_TimerEnable(WTIMER1_BASE, TIMER_B);//Enable datalost timer
		ROM_TimerEnable(WTIMER3_BASE, TIMER_B);//Enable PID timer
	}
  else ROM_TimerLoadSet(WTIMER1_BASE, TIMER_B, errorPeriod);
  
  
  if (UARTCharsAvail(UART4_BASE))
  {
    xMe_=((int16_t)((UARTCharGetNonBlocking(UART4_BASE)&0x000000FF)|((int16_t)((UARTCharGetNonBlocking(UART4_BASE)&0x000000FF)<<8))))*0.1f;
    yMe_=((int16_t)((UARTCharGetNonBlocking(UART4_BASE)&0x000000FF)|((int16_t)((UARTCharGetNonBlocking(UART4_BASE)&0x000000FF)<<8))))*0.1f;
    xOp_=((int16_t)((UARTCharGetNonBlocking(UART4_BASE)&0x000000FF)|((int16_t)((UARTCharGetNonBlocking(UART4_BASE)&0x000000FF)<<8))))*0.1f;
    yOp_=((int16_t)((UARTCharGetNonBlocking(UART4_BASE)&0x000000FF)|((int16_t)((UARTCharGetNonBlocking(UART4_BASE)&0x000000FF)<<8))))*0.1f;
    angle_=((int16_t)((UARTCharGetNonBlocking(UART4_BASE)&0x000000FF)|((int16_t)((UARTCharGetNonBlocking(UART4_BASE)&0x000000FF)<<8))))*0.1f;
    UARTCharGetNonBlocking(UART4_BASE);
    UARTCharGetNonBlocking(UART4_BASE);

		if(xMe_ != -1000)
		{			
			xMe = xMe_;
			yMe = yMe_;
		}
		if(xOp_ != -1000)
		{			
			xOp = xOp_;
			yOp = yOp_;
		}
		if(angle_ != -1000)
		{			
			angle = angle_;
		}
  }
  //UARTprintf("xop: %d yop: %d xMe: %d yMe: %d angle: %d\r\n",(int)xOp,(int)yOp,(int)xMe,(int)yMe,(int)angle);
}



int main(void)
{
	ROM_SysCtlClockSet(SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ|SYSCTL_SYSDIV_2_5);//80Mhz Clock
	ROM_FPUEnable();//Enable FPU
	ROM_FPULazyStackingEnable();//Enable FPU stacking while interrupt
	
	systemClock=SysCtlClockGet();
	sysTickSetup(0x00FFFFFF);
	ROM_IntMasterDisable();
	/*
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	
	ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
	ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
	
	UARTStdioConfig(0, 115200, SysCtlClockGet());*/
	
	
	UART4Setup((uint32_t)9600);
	
	PWM_Init();                       // Initialize PF1,PF2 and PF3 for PWM operation
	MotorInput_Init();
	wTimer1BInit(errorPeriod);
	
	
	UART2Setup(9600);
	cmpStr[0]='O';
	cmpStr[1]='K';
	cmpStr[2]='\0';
	servoPosition=servoHold;
	
	pwmModuleInit();
	pwmB5Generate(1250);
	pwmB4Generate(servoMin);
	sysTickDelay(1500);
	pwmB4Generate(servoHold);
	//timer2AInit((uint32_t)1600000);
	
	pos_ang.integral = 0;
	pos_ang.last_proportional = 0;
	pos_ang.Kp = 1;
	pos_ang.Ki = 0;
	pos_ang.Kd = 0;
	
	ang.integral = 0;
	ang.last_proportional = 0;
	ang.Kp = 18;
	ang.Ki = 0.0000075;
	ang.Kd = 1000;
	
	wTimer3BInit(pidPeriod);//PID Timer Init
	timer1AInit(waitForFirePeriod);
	
	
	//Gyro Initialization
	gyroTimerInit();
	
	//Debug led
	GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE,GPIO_PIN_0|GPIO_PIN_1);
	GPIOPadConfigSet(GPIO_PORTF_BASE,GPIO_PIN_0|GPIO_PIN_1,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD);
	
	/*GPIOPinWrite(GPIO_PORTD_BASE,GPIO_PIN_0,GPIO_PIN_0);
	SysCtlDelay((systemClock>>1)/3);
	GPIOPinWrite(GPIO_PORTD_BASE,GPIO_PIN_0,!(GPIO_PIN_0));
	SysCtlDelay((systemClock>>1)/3);*/
	
	ROM_IntMasterEnable();
	
	//didIFire=1;
	while(true)
	{
		if(didIFire)
		{
			GPIOPinWrite(GPIO_PORTD_BASE,GPIO_PIN_0,GPIO_PIN_0);
			if (direction)
			{
				servoPosition=servoPosition-25;
				pwmB4Generate(servoPosition);
				if (servoPosition==servoMin)direction=!direction;
			}
			else
			{
				servoPosition=servoPosition+25;
				pwmB4Generate(servoPosition);
				if(servoPosition==servoHold)
				{
					direction=!direction;
					didIFire=false;
				}
			}
			SysCtlDelay(systemClock/150);
		}
		GPIOPinWrite(GPIO_PORTD_BASE,GPIO_PIN_0,!GPIO_PIN_0);
	}
}

void UART2Setup(uint32_t baudRate)
{
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART2);
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
  
  GPIOPinConfigure(GPIO_PD6_U2RX);
  GPIOPinConfigure(GPIO_PD7_U2TX);

  ROM_GPIOPinTypeUART(GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_7);
  ROM_UARTIntDisable(UART2_BASE, 0xFFFFFFFF);//disable all the interrupts
	
  ROM_UARTConfigSetExpClk(UART2_BASE, SysCtlClockGet(), baudRate,(UART_CONFIG_PAR_NONE | UART_CONFIG_STOP_ONE |UART_CONFIG_WLEN_8));
  UARTFIFOEnable(UART2_BASE);
  ROM_UARTFIFOLevelSet(UART2_BASE, UART_FIFO_TX1_8,  UART_FIFO_RX1_8);//RX TX triggers when 1/8 of the buffer is available
    
   
  ROM_UARTIntEnable(UART2_BASE, UART_INT_RX);//enable only receive interrupt
  UARTIntRegister(UART2_BASE,fireHandler);//UART1Handler is default interrupt handler
	
  ROM_UARTIntEnable(UART2_BASE,UART_INT_RX);
	
	IntEnable(INT_UART2_TM4C123);
  IntPrioritySet(INT_UART2_TM4C123,0x40);
  ROM_UARTEnable(UART2_BASE);
  SysCtlDelay(100);
}

void UART4Setup(uint32_t baudRate)
{
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART4);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	ROM_GPIOPinConfigure(GPIO_PC4_U4RX);//C4 RX
	ROM_GPIOPinConfigure(GPIO_PC5_U4TX);//C5 TX

	ROM_GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5);
	ROM_UARTIntDisable(UART4_BASE, 0xFFFFFFFF);//disable all the interrupts

	ROM_UARTConfigSetExpClk(UART4_BASE, SysCtlClockGet(), baudRate,(UART_CONFIG_PAR_NONE | UART_CONFIG_STOP_ONE |UART_CONFIG_WLEN_8));
	UARTFIFOEnable(UART4_BASE);
	ROM_UARTFIFOLevelSet(UART4_BASE, UART_FIFO_TX1_8,  UART_FIFO_RX6_8);//RX TX triggers when 1/8 of the buffer is available
	 

	ROM_UARTIntEnable(UART4_BASE, UART_INT_RX);//enable only receive interrupt
	UARTIntRegister(UART4_BASE,UART4Handler);//UART1Handler is default interrupt handler

	ROM_UARTIntEnable(UART4_BASE,UART_INT_RX);
	IntEnable(INT_UART4_TM4C123);
	IntPrioritySet(INT_UART4_TM4C123,0x20);
	ROM_UARTEnable(UART4_BASE);
	SysCtlDelay(100);
}

void wTimer3BInit(uint32_t period)
{
	// Controller Timer
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER3);
	ROM_TimerConfigure(WTIMER3_BASE, TIMER_CFG_SPLIT_PAIR|TIMER_CFG_B_PERIODIC);
	
	ROM_TimerLoadSet(WTIMER3_BASE, TIMER_B, period-1);
	TimerIntRegister(WTIMER3_BASE,TIMER_B, ctrlHandler);
	
	ROM_IntEnable(INT_WTIMER3B_TM4C123); ////////////////////////////////////ROM_IntEnable(INT_TIMER1A_TM4C123);
	IntPrioritySet(INT_WTIMER3B_TM4C123, 0x40);
	ROM_TimerIntEnable(WTIMER3_BASE, TIMER_TIMB_TIMEOUT);
}

void timer1AHandler(void)
{
	ROM_TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
	ROM_TimerDisable(TIMER1_BASE, TIMER_A);
	//GPIOPinWrite(GPIO_PORTD_BASE,GPIO_PIN_0,GPIO_PIN_0);
	pwmB5Generate(2000);
	SysCtlDelay((SysCtlClockGet()/6));//sysTickDelay(500);
	pwmB4Generate(servoFire);
	SysCtlDelay((SysCtlClockGet()/6));//sysTickDelay(500);
	pwmB4Generate(servoHold);
	
	//ROM_TimerEnable(TIMER2_BASE, TIMER_A);
	pwmB5Generate(1250);
	mode=0;
	didIFire=1;
	//GPIOPinWrite(GPIO_PORTD_BASE,GPIO_PIN_0,!GPIO_PIN_0);
}

void timer1AInit(uint32_t period)
{
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
	ROM_TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);
	ROM_TimerLoadSet(TIMER1_BASE, TIMER_A, period-1);
	
	TimerIntRegister(TIMER1_BASE,TIMER_A,timer1AHandler);
	
	
	ROM_IntEnable(INT_TIMER1A_TM4C123);
	
	IntPrioritySet(INT_TIMER1A_TM4C123,0x60);
	ROM_TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
	
	//ROM_TimerEnable(TIMER1_BASE, TIMER_A);
}

/** @brief Convert translational and angular velocity of the robot to wheel angular velocities  
  * @input Vx: Velocity of the robot in x direction 
  *        Vy: Velocity of the robot in y direction 
  *        Vang: Angular velocity of the robot 	
	*        theta: Angle between robots frame and world frame
	*        *ptr: Pointer to wheel angular velocities
  * @output None
  */
void convertFromRobotVelToMotorVel(float Vx, float Vy, float Vang, float theta, float *ptr)
{
  int i;
  theta = theta / RAD_TO_DEG;
	
  ptr[1] = (-sinf(theta)*Vx+ cosf(theta)         * Vy +  Vang) ;  
  ptr[2] = (-sinf(PI/3 - theta)*Vx - cosf(PI/3 - theta)  * Vy +  Vang) ;
  ptr[0] = (sinf(PI/3 + theta)*Vx + -cosf(PI/3 + theta) * Vy +  Vang) ;
	
  for(i=0; i<3; i++)
  {
	if(ptr[i] <= 0)     // if pwm is negative, then change the direction
	{
		ptr[i] = -ptr[i];
		dir[i] = 1;
	}
	else
		dir[i] = 0;
	
	if((mode >= 2) && (ptr[i] >= 0.1f))     // if pwm is not close enough to zero, add the offset. (if pwm is zero, the robot will not move)
		ptr[i] += pwm_offset;	

	if(ptr[i] >= 3990)  // if pwm is greater than the max limit, saturate it to max value
		ptr[i] = 3990;
  }
}

