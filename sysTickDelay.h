#include <stdint.h>
#include "driverlib/systick.h"
#include "driverlib/sysctl.h"

void sysTickSetup(uint32_t period);
void sysTickDelay(unsigned long delay);
void sysTickWait(uint32_t delay);
void sysTickDelayMicroSecond(unsigned long delay);
