#include "i2c.h"

uint32_t I2C_PORT=I2C1_BASE;

void SetupI2C(void)
{
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

	ROM_GPIOPinTypeI2CSCL(GPIO_PORTA_BASE, GPIO_PIN_6); //  special I2CSCL treatment for M4F devices
   ROM_GPIOPinTypeI2C(GPIO_PORTA_BASE, GPIO_PIN_7);

	ROM_GPIOPinConfigure(GPIO_PA6_I2C1SCL);
	ROM_GPIOPinConfigure(GPIO_PA7_I2C1SDA);

	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C1);

	ROM_I2CMasterInitExpClk(I2C_PORT, SysCtlClockGet(), 1); // 1 : 400Khz, 0 : 100Khz
}

uint8_t i2cRecvByte( uint8_t SlaveID, uint8_t addr)
{
	unsigned long ulRegValue = 0;
	
	// Wait until master module is done transferring.
	while(ROM_I2CMasterBusy(I2C_PORT));

    // Tell the master module what address it will place on the bus when
    // writing to the slave.
    ROM_I2CMasterSlaveAddrSet(I2C_PORT, SlaveID, 0);

    // Place the command to be sent in the data register.
    ROM_I2CMasterDataPut(I2C_PORT, addr);

    // Initiate send of data from the master.
    ROM_I2CMasterControl(I2C_PORT, I2C_MASTER_CMD_SINGLE_SEND);

    // Wait until master module is done transferring.
    while(ROM_I2CMasterBusy(I2C_PORT));

    // Check for errors.
    if(ROM_I2CMasterErr(I2C_PORT) != I2C_MASTER_ERR_NONE) return -1;

    // Tell the master module what address it will place on the bus when
    // reading from the slave.
    ROM_I2CMasterSlaveAddrSet(I2C_PORT, SlaveID, 1);

    // Tell the master to read data.
    ROM_I2CMasterControl(I2C_PORT, I2C_MASTER_CMD_SINGLE_RECEIVE);

    // Wait until master module is done receiving.
    while(ROM_I2CMasterBusy(I2C_PORT));

    // Check for errors.
    if(ROM_I2CMasterErr(I2C_PORT) != I2C_MASTER_ERR_NONE)return 0;

    // Read the data from the master.
    ulRegValue = ROM_I2CMasterDataGet(I2C_PORT);

    // Return the register value.
    return ulRegValue;
}

int i2cSendByte( uint8_t SlaveID, uint8_t addr, uint8_t data)
{
    // Wait until master module is done transferring.
    while(ROM_I2CMasterBusy(I2C_PORT));

    // Tell the master module what address it will place on the bus when
    // writing to the slave.
    ROM_I2CMasterSlaveAddrSet(I2C_PORT, SlaveID, 0);

    // Place the command to be sent in the data register.
    ROM_I2CMasterDataPut(I2C_PORT, addr);

    // Initiate send of data from the master.
    ROM_I2CMasterControl(I2C_PORT, I2C_MASTER_CMD_BURST_SEND_START);

    // Wait until master module is done transferring.
    while(ROM_I2CMasterBusy(I2C_PORT));

    // Check for errors.
    if(ROM_I2CMasterErr(I2C_PORT) != I2C_MASTER_ERR_NONE)return 0;

    // Place the value to be sent in the data register.
    ROM_I2CMasterDataPut(I2C_PORT, data);

    // Initiate send of data from the master.
    ROM_I2CMasterControl(I2C_PORT, I2C_MASTER_CMD_BURST_SEND_CONT);

    // Wait until master module is done transferring.
    while(ROM_I2CMasterBusy(I2C_PORT));

    // Check for errors.
    if(ROM_I2CMasterErr(I2C_PORT) != I2C_MASTER_ERR_NONE)return 0;

    // Initiate send of data from the master.
    ROM_I2CMasterControl(I2C_PORT, I2C_MASTER_CMD_BURST_SEND_FINISH);

    // Wait until master module is done transferring.
    while(ROM_I2CMasterBusy(I2C_PORT));

    // Check for errors.
    if(ROM_I2CMasterErr(I2C_PORT) != I2C_MASTER_ERR_NONE)return 0;

    // Return 1 if there is no error.
    return 1;
}

int i2cRecvBuf(uint8_t SlaveID, uint8_t addr, int32_t nBytes , uint8_t* pBuf )
{
    uint8_t nBytesCount;            // local variable used for byte counting/state determination
    uint16_t MasterOptionCommand;           // used to assign the commands for I2CMasterControl() function

    // Wait until master module is done transferring.
    while(ROM_I2CMasterBusy(I2C_PORT));

    // Tell the master module what address it will place on the bus when
    // writing to the slave.
    ROM_I2CMasterSlaveAddrSet(I2C_PORT, SlaveID, 0);

    // Place the command to be sent in the data register.
    ROM_I2CMasterDataPut(I2C_PORT, addr);
	
    // Initiate send of data from the master.
    ROM_I2CMasterControl(I2C_PORT, I2C_MASTER_CMD_SINGLE_SEND);

    // Wait until master module is done transferring.
    while(ROM_I2CMasterBusy(I2C_PORT));

    // Check for errors.
    if(ROM_I2CMasterErr(I2C_PORT) != I2C_MASTER_ERR_NONE)return 0;


	// Tell the master module what address it will place on the bus when
	// reading from the slave.
    ROM_I2CMasterSlaveAddrSet(I2C_PORT, SlaveID, true);

    // Start with BURST with more than one byte to write
    MasterOptionCommand = I2C_MASTER_CMD_BURST_RECEIVE_START;

	for(nBytesCount = 0; nBytesCount < nBytes; nBytesCount++)
    {
		// The second and intermittent byte has to be read with CONTINUE control word
		if(nBytesCount == 1)MasterOptionCommand = I2C_MASTER_CMD_BURST_RECEIVE_CONT;

		// The last byte has to be send with FINISH control word
		if(nBytesCount == nBytes - 1)MasterOptionCommand = I2C_MASTER_CMD_BURST_RECEIVE_FINISH;

		// Re-configure to SINGLE if there is only one byte to read
		if(nBytes == 1)MasterOptionCommand = I2C_MASTER_CMD_SINGLE_RECEIVE;

		// Initiate read of data from the slave.
		ROM_I2CMasterControl(I2C_PORT, MasterOptionCommand);

		// Wait until master module is done reading.
		while(ROM_I2CMasterBusy(I2C_PORT));

        // Check for errors.
        if(ROM_I2CMasterErr(I2C_PORT) != I2C_MASTER_ERR_NONE)return 0;

        // Move byte from register
                pBuf[nBytesCount] = ROM_I2CMasterDataGet(I2C_PORT);
    }

    // send number of received bytes
    return nBytesCount;
}


int i2cSendBuf( uint8_t SlaveID, uint8_t addr, int32_t nBytes , uint8_t* pBuf)
{
	uint8_t nBytesCount;            // local variable used for byte counting/state determination
	uint16_t MasterOptionCommand;           // used to assign the commands for I2CMasterControl() function

	// Wait until master module is done transferring.
	while(ROM_I2CMasterBusy(I2C_PORT));

    // Tell the master module what address it will place on the bus when
    // writing to the slave.
    ROM_I2CMasterSlaveAddrSet(I2C_PORT, SlaveID, false);

    // Place the value to be sent in the data register.
    ROM_I2CMasterDataPut(I2C_PORT, addr);

    // Initiate send of data from the master.
    ROM_I2CMasterControl(I2C_PORT, I2C_MASTER_CMD_BURST_SEND_START);

    // Wait until master module is done transferring.
    while(ROM_I2CMasterBusy(I2C_PORT));

    // Check for errors.
    if(ROM_I2CMasterErr(I2C_PORT) != I2C_MASTER_ERR_NONE)return 0;

	// Start with CONT for more than one byte to write
	MasterOptionCommand = I2C_MASTER_CMD_BURST_SEND_CONT;


	for(nBytesCount = 0; nBytesCount < nBytes; nBytesCount++)
	{
		// The second and intermittent byte has to be send with CONTINUE control word
		if(nBytesCount == 1)MasterOptionCommand = I2C_MASTER_CMD_BURST_SEND_CONT;

		// The last byte has to be send with FINISH control word
		if(nBytesCount == nBytes - 1)MasterOptionCommand = I2C_MASTER_CMD_BURST_SEND_FINISH;

		// Re-configure to SINGLE if there is only one byte to write
		if(nBytes == 1)MasterOptionCommand = I2C_MASTER_CMD_SINGLE_SEND;

		// Send data byte
		ROM_I2CMasterDataPut(I2C_PORT, pBuf[nBytesCount]);

		// Initiate send of data from the master.
		ROM_I2CMasterControl(I2C_PORT, MasterOptionCommand);

		// Wait until master module is done transferring.
		while(ROM_I2CMasterBusy(I2C_PORT));

		// Check for errors.
		if(ROM_I2CMasterErr(I2C_PORT) != I2C_MASTER_ERR_NONE)return 0;
    }

    // Return 1 if there is no error.
    return 1;
}

