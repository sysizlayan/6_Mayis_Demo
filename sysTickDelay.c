#include <stdint.h>
#include "driverlib/systick.h"
#include "driverlib/sysctl.h"
#include "driverlib/rom.h"

void sysTickSetup(uint32_t period)
{
	ROM_SysTickPeriodSet(period&0x00FFFFFF);
	ROM_SysTickEnable();
}
void sysTickWait(uint32_t delay)
{
	volatile uint32_t elapsedTime;
	uint32_t startTime=SysTickValueGet();
	do
	{
		elapsedTime=(startTime-SysTickValueGet())&0x00FFFFFF;
	}while(elapsedTime<=delay);
}

void sysTickDelay(unsigned long delay)
{
  unsigned long i;
  for(i=0; i<delay; i++)sysTickWait(SysCtlClockGet()/1000);//get the clock, calculate 1 ms of delay
}


void sysTickDelayMicroSecond(unsigned long delay)
{
  unsigned long i;
  for(i=0; i<delay; i++)sysTickWait(SysCtlClockGet()/100000l);//get the clock, calculate 1 us of delay
}

