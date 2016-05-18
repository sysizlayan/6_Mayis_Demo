#include <stdint.h>

#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/i2c.h"
#include "driverlib/rom.h"

void SetupI2C(void);

// functions for writing/reading single bytes of data
uint8_t i2cRecvByte( uint8_t devId, uint8_t addr);

int i2cSendByte( uint8_t devId, uint8_t addr, uint8_t data);

// functions for writing/reading multiple bytes of data
int i2cRecvBuf(uint8_t devId, uint8_t addr, int32_t nBytes , uint8_t* pBuf );

int i2cSendBuf( uint8_t devId, uint8_t addr, int32_t nBytes , uint8_t* pBuf);

