#include "timerInitialization.h"

uint32_t timerCounter;
bool state;
extern int STOOOP;


extern char mssg[3];
extern char cmpStr[3];
extern int servoHold;
extern int servoMin;
extern int servoFire;
extern int servoPosition;
extern int direction;

void wTimer1BHandler(void)
{
	TimerIntClear(WTIMER1_BASE, TIMER_TIMB_TIMEOUT);
	TimerDisable(WTIMER1_BASE, TIMER_B);
	Motor1(0,0);
	Motor2(0,0);
	Motor3(0,0);
	ROM_IntMasterDisable();
	while(1);
}

void wTimer1BInit(uint32_t period)//timer will be enabled with the first coming data
{		
	//Data Lost Timer
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER1);
	ROM_TimerConfigure(WTIMER1_BASE, TIMER_CFG_SPLIT_PAIR|TIMER_CFG_B_PERIODIC);
	
	ROM_TimerLoadSet(WTIMER1_BASE, TIMER_B, period-1);
	TimerIntRegister(WTIMER1_BASE,TIMER_B,wTimer1BHandler);
	
	ROM_IntEnable(INT_WTIMER1B);
	IntPrioritySet(INT_WTIMER1B,0x00);
	ROM_TimerIntEnable(WTIMER1_BASE, TIMER_TIMB_TIMEOUT);
	
}

void timer2AHandler(void)
{
	TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
	if(direction)
	{
		servoPosition=servoPosition-25;
		pwmB4Generate(servoPosition);
		if(servoPosition==servoMin)direction=!direction;
	}
	else
	{
		servoPosition=servoPosition+25;
		pwmB4Generate(servoPosition);
		if(servoPosition==servoHold)
		{
			ROM_TimerDisable(TIMER2_BASE, TIMER_A);
			direction=!direction;
		}
	}
}

void timer2AInit(uint32_t period)
{	
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
	ROM_TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC);
	ROM_TimerLoadSet(TIMER2_BASE, TIMER_A, period-1);
	
	TimerIntRegister(TIMER2_BASE,TIMER_A,timer2AHandler);
	ROM_TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
	
	//ROM_TimerEnable(TIMER1_BASE, TIMER_A);
	
	ROM_IntEnable(INT_TIMER2A_TM4C123);
	
	IntPrioritySet(INT_TIMER2A_TM4C123,0xE0);
}
