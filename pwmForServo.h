#include <stdint.h>
#include <stdbool.h>
#include "driverlib/sysctl.h"
#include "inc/hw_sysctl.h"

void pwmModuleInit(void);
void pwmB5Generate(uint16_t duty);
void pwmB4Generate(uint16_t duty);
