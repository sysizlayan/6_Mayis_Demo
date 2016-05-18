#include <stdbool.h>          // included to use boolean data type
#include <math.h>             // standard C math library
#include <stdint.h>           // C99 variable types
#include <stdio.h>            // standard C input output library
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
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/uart.h"
#include "driverlib/rom.h"
#include "driverlib/i2c.h"
#include "driverlib/pin_map.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"

#include "sysTickDelay.h"
#include "MotorControl.h"
#include "PWM.h"
#include "pwmForServo.h"

void wTimer1BHandler(void);
void wTimer1BInit(uint32_t period); // Data lost timer init

void timer2AHandler(void); // Servo handler
void timer2AInit(uint32_t period);
//void timer1AHandler(void);
