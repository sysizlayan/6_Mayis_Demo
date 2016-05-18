#include "pwmForServo.h"
#include "inc/tm4c123gh6pm.h"

void pwmModuleInit(void)
{
	volatile unsigned long delay;
	//SYSCTL_RCGC2_R|=SYSCTL_RCGC2_GPIOB;//port b clock
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	delay=SYSCTL_RCGC2_R;
	SYSCTL_RCGCPWM_R|=SYSCTL_RCGCPWM_R0;
	delay=SYSCTL_RCGC2_R;
	
	GPIO_PORTB_AFSEL_R|=0x30;
	//GPIO_PORTB_AFSEL_R&=~0x30;
	GPIO_PORTB_AMSEL_R&=~0x30;
	GPIO_PORTB_DEN_R|=0x30;
	GPIO_PORTB_DIR_R|=0x30;
	
	GPIO_PORTB_PCTL_R=(GPIO_PORTB_PCTL_R&0xFF00FFFF)+GPIO_PCTL_PB5_M0PWM3+GPIO_PCTL_PB4_M0PWM2;//port control
	
	SYSCTL_RCC_R|=SYSCTL_RCC_USEPWMDIV;//pwm clock division enable
	SYSCTL_RCC_R&=~SYSCTL_RCC_PWMDIV_M;//pwm division biti clear
	SYSCTL_RCC_R|=SYSCTL_RCC_PWMDIV_64;//pwm division /64
	
	PWM0_1_CTL_R=0;
	
	PWM0_1_GENA_R=0x0C8;
	
	PWM0_1_GENB_R=0xC08;
	
	PWM0_1_LOAD_R=24999;//50Hz
	PWM0_1_CTL_R|=0x00000001;
	
	PWM0_ENABLE_R|=0x0C;
}

void pwmB5Generate(uint16_t duty)
{
	PWM0_1_CMPB_R=duty-1;
}
void pwmB4Generate(uint16_t duty)
{
	PWM0_1_CMPA_R=duty-1;
}
