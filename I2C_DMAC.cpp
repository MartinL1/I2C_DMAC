/*
  I2C_DMAC is a non-blocking I2C library that uses SERCOM3 in 
	conjunction with the Direct Memory Access Controller (DMAC).
	
	Copyright (C) Martin Lindupp 2018
	
	Initial release -- V1.0.0 

	The MIT License (MIT)

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in all
	copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
	SOFTWARE.
*/

#include <I2C_DMAC.h>

static I2C_DMAC* i2cDmacPtr = &I2C;			// Create an external object pointer to allow access to the I2C class
																				// "out of class context" in the external DMAC_Handler() function							

//
// Initialisation section: prep the the port pins, SERCOM3 and the DMAC
//
I2C_DMAC::I2C_DMAC() : dmacWriteChannel(0), dmacReadChannel(1), dmacPriority(0) {} 	// Constructor to initialise DMAC channel member variables

void I2C_DMAC::begin(uint32_t baudrate, uint8_t regAddrMode) 					// Set baud rate and the register address mode: 8-bit or 16-bit				
{  
	this->regAddrMode = regAddrMode;
	
	// Enable the SCL and SDA lines on SERCOM3 - Arduino Zero, M0 Pro or M0 compatible only
  PORT->Group[g_APinDescription[SCL].ulPort].PINCFG[g_APinDescription[SCL].ulPin].bit.PMUXEN = 1;  
  PORT->Group[g_APinDescription[SDA].ulPort].PINCFG[g_APinDescription[SDA].ulPin].bit.PMUXEN = 1;
  PORT->Group[g_APinDescription[SDA].ulPort].PMUX[g_APinDescription[SDA].ulPin >> 1].reg = PORT_PMUX_PMUXO_C | PORT_PMUX_PMUXE_C;

	if (!DMAC->CTRL.bit.DMAENABLE)			 // Enable the DMAC, if it hasn't already been enabled
	{
		NVIC_SetPriority(DMAC_IRQn, 0);    // Set the Nested Vector Interrupt Controller (NVIC) priority for the DMAC to 0 (highest) 
		NVIC_EnableIRQ(DMAC_IRQn);         // Connect the DMAC to the Nested Vector Interrupt Controller (NVIC)
		
		DMAC->CTRL.bit.SWRST = 1;											 												// Reset the DMAC
		while (DMAC->CTRL.bit.SWRST);																					// Wait for synchronization
		DMAC->BASEADDR.reg = (uint32_t)descriptor_section;                    // Set the DMAC descriptor base address
		DMAC->WRBADDR.reg = (uint32_t)wrb;                                    // Set the DMAC descriptor write-back address
		DMAC->CTRL.reg = DMAC_CTRL_DMAENABLE | DMAC_CTRL_LVLEN(0xf);          // Enable the DMAC peripheral and enable priority levels
	}
	
  // Enable GCLK0 on SERCOM3
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |              // Enable GCLK0 to SERCOM3
                     GCLK_CLKCTRL_GEN_GCLK0 |          // Select GCLK0 as source
                     GCLK_CLKCTRL_ID_SERCOM3_CORE;     // Select SERCOM3 as destination
  while (GCLK->STATUS.bit.SYNCBUSY);                   // Wait for synchronization

  NVIC_SetPriority(SERCOM3_IRQn, 0);    // Set the Nested Vector Interrupt Controller (NVIC) priority for SERCOM3 to 0 (highest) 
  NVIC_EnableIRQ(SERCOM3_IRQn);         // Connect SERCOM3 to the Nested Vector Interrupt Controller (NVIC)
	
	SERCOM3->I2CM.CTRLA.bit.SWRST = 1;                                          	// Reset SERCOM3
  while (SERCOM3->I2CM.CTRLA.bit.SWRST || SERCOM3->I2CM.SYNCBUSY.bit.SWRST);  	// Wait for synchronization
	
  SERCOM3->I2CM.CTRLA.reg = SERCOM_I2CM_CTRLA_MODE(I2C_MASTER_OPERATION);   		// Set I2C master mode                     
  SERCOM3->I2CM.CTRLB.reg = SERCOM_I2CM_CTRLB_SMEN;   										  		// Enable Smart Mode
  SERCOM3->I2CM.BAUD.bit.BAUD = SystemCoreClock / (2 * baudrate) - 7 ;      		// Set I2C master SCL baud rate
  
	SERCOM3->I2CM.CTRLA.bit.ENABLE = 1 ;           // Enable SERCOM3 in I2C master mode
  while (SERCOM3->I2CM.SYNCBUSY.bit.ENABLE);     // Wait for synchronization
  SERCOM3->I2CM.STATUS.bit.BUSSTATE = 0x01;      // Set the I2C bus to IDLE state    
  while (SERCOM3->I2CM.SYNCBUSY.bit.SYSOP);      // Wait for synchronization
	SERCOM3->I2CM.INTENSET.bit.ERROR = 1;					 // Enable SERCOM3 ERROR interrupts
}

void I2C_DMAC::begin(uint32_t baudrate)
{
	begin(baudrate, REG_ADDR_8BIT);			// Set baud rate, but default to 8-bit register address mode
}

void I2C_DMAC::begin()
{
	begin(100000);											// Set default I2C baud rate to 100kHz and default to 8-bit register address mode
}

//
// Tear down and tidy up resources
//
void I2C_DMAC::end() 
{  
	// Return the SCL and SDA lines on SERCOM3 to GPIO
  PORT->Group[g_APinDescription[SCL].ulPort].PINCFG[g_APinDescription[SCL].ulPin].bit.PMUXEN = 0;  
  PORT->Group[g_APinDescription[SDA].ulPort].PINCFG[g_APinDescription[SDA].ulPin].bit.PMUXEN = 0;
  PORT->Group[g_APinDescription[SDA].ulPort].PMUX[g_APinDescription[SDA].ulPin >> 1].reg = PORT_PMUX_PMUXO_A | PORT_PMUX_PMUXE_A;  
        
	SERCOM3->I2CM.CTRLA.bit.ENABLE = 0;            															// Disable the I2C master mode
  while (SERCOM3->I2CM.SYNCBUSY.bit.ENABLE);     															// Wait for synchronization
	SERCOM3->I2CM.CTRLA.bit.SWRST = 1;                                          // Reset SERCOM3
  while (SERCOM3->I2CM.CTRLA.bit.SWRST || SERCOM3->I2CM.SYNCBUSY.bit.SWRST);  // Wait for synchronization
	
	// Disable GCLK0 on SERCOM3
  REG_GCLK_CLKCTRL = /*GCLK_CLKCTRL_CLKEN |*/          // Disable GCLK0 to SERCOM3 - intentionally commented out
                     GCLK_CLKCTRL_GEN_GCLK0 |          // Select GCLK0 as source
                     GCLK_CLKCTRL_ID_SERCOM3_CORE;     // Select SERCOM3 as destination
  while (GCLK->STATUS.bit.SYNCBUSY);        					 // Wait for synchronization  
	
	NVIC_DisableIRQ(SERCOM3_IRQn);						// Disconnect SERCOM3 from the Nested Vector Interrupt Controller (NVIC)
  //NVIC_ClearPendingIRQ(SERCOM3_IRQn);
	
  DMAC->CTRL.bit.DMAENABLE = 0;          		// Disable the DMAC
	while(DMAC->CTRL.bit.DMAENABLE);			 		// Wait for synchronization			
	
	DMAC->CTRL.bit.SWRST = 1;									// Reset the DMAC
	while (DMAC->CTRL.bit.SWRST);							// Wait for synchronization
	NVIC_DisableIRQ(DMAC_IRQn);								// Disconnect the DMAC from the Nested Vector Interrupt Controller (NVIC)
  //NVIC_ClearPendingIRQ(DMAC_IRQn);
}

void I2C_DMAC::setClock(uint32_t baudrate)
{
	SERCOM3->I2CM.CTRLA.bit.ENABLE = 0;            													// Disable SERCOM3 in I2C master mode
  while (SERCOM3->I2CM.SYNCBUSY.bit.ENABLE);     													// Wait for synchronization
	SERCOM3->I2CM.BAUD.bit.BAUD = SystemCoreClock / (2 * baudrate) - 7 ;    // Set the I2C SCL frequency to the baud rate
	SERCOM3->I2CM.CTRLA.bit.ENABLE = 1 ;           													// Enable SERCOM3 in I2C master mode
  while (SERCOM3->I2CM.SYNCBUSY.bit.ENABLE);     													// Wait for synchronization
}

void I2C_DMAC::setWriteChannel(uint8_t channel)
{
	dmacWriteChannel = channel < 12 ? channel : dmacWriteChannel;			// Set the write DMAC channel, (default channel 0)
}

void I2C_DMAC::setReadChannel(uint8_t channel)
{
	dmacReadChannel = channel < 12 ? channel : dmacReadChannel;				// Set the read DMAC channel, (default channel 1)
}

void I2C_DMAC::setPriority(uint8_t priority)
{
	dmacPriority = priority < 4 ? priority : dmacPriority;						// Set the priority of both write and read channels (0 lowest, 3 highest)
}

//
// DMAC Section: Load the DMAC's transfer descriptors
//
void I2C_DMAC::setRegAddrMode(uint8_t regAddrMode)
{
	this->regAddrMode = regAddrMode;						// Set the register address mode: REG_ADDR_8BIT or REG_ADDR_16BIT
}

// Generic initialise write DMAC transfer function
void I2C_DMAC::initWriteBytes(uint8_t devAddress, uint16_t regAddress, uint8_t* data, uint8_t count, uint8_t regAddrLength)
{
	this->devAddress = devAddress;							// Copy the device address, plus transmission and register address length
	writeCount = count;
	this->regAddrLength = regAddrLength;
	
	DMAC->CHID.reg = DMAC_CHID_ID(dmacWriteChannel);                        // Activate specified DMAC write channel 
	// Set the DMAC level, trigger source and trigger action to beat (trigger for every byte transmitted)
	DMAC->CHCTRLB.reg = DMAC_CHCTRLB_LVL(dmacPriority) | DMAC_CHCTRLB_TRIGSRC(SERCOM3_DMAC_ID_TX) | DMAC_CHCTRLB_TRIGACT_BEAT; 
	DMAC->CHINTENSET.reg = DMAC_CHINTENSET_MASK;                          	// Enable all 3 interrupts: SUSP, TCMPL and TERR
	
	if (regAddrLength == 0)																									// Writing data only
	{
		descriptor.descaddr = 0;  																						// Set this to the last descriptor (no linked list)
		descriptor.srcaddr = (uint32_t)data + count;     											// Set the source address
		descriptor.dstaddr = (uint32_t)&SERCOM3->I2CM.DATA.reg;               // Set the destination address
		descriptor.btcnt = count;                                   					// Number of data bytes to transmit
		descriptor.btctrl = DMAC_BTCTRL_SRCINC | DMAC_BTCTRL_VALID; 					// Increment source address on BEAT transfer and validate descriptor	
		memcpy(&descriptor_section[dmacWriteChannel], &descriptor, sizeof(dmacdescriptor));  // Copy the descriptor into SRAM descriptor array
		return;																																// Data only initialisation complete
	}
	else if (regAddrLength == 1)																						// 8-bit write address mode
	{
		this->regAddress[0] = (uint8_t)regAddress;														// Copy the 8-bit register address
	}
	else																																		// 16-bit write address mode
	{
		this->regAddress[0] = (uint8_t)(regAddress >> 8);											// Copy the 16-bit register address MSB
		this->regAddress[1] = (uint8_t)(regAddress & 0xFF);										// Copy the 16-bit register address LSB
	}
	descriptor.descaddr = count > 0 ? (uint32_t)&linked_descriptor : 0;   	// Link to next descriptor if there's data 
	descriptor.srcaddr = (uint32_t)this->regAddress + regAddrLength;  			// Set the source address
	descriptor.dstaddr = (uint32_t)&SERCOM3->I2CM.DATA.reg;             	  // Set the destination address
	descriptor.btcnt = regAddrLength;                                   		// Size of the register address in bytes
	descriptor.btctrl = DMAC_BTCTRL_SRCINC | DMAC_BTCTRL_VALID;    					// Increment source address on BEAT transfer and validate descriptor
	memcpy(&descriptor_section[dmacWriteChannel], &descriptor, sizeof(dmacdescriptor));   // Copy the descriptor into SRAM descriptor array
	
	if (count > 0)																													// Append write data as linked descriptor
	{	
		linked_descriptor.descaddr = 0;     																	// Set linked_descriptor to last in the list
		linked_descriptor.srcaddr = (uint32_t)data + count;  			 						// Set the source address
		linked_descriptor.dstaddr = (uint32_t)&SERCOM3->I2CM.DATA.reg;        // Set the destination address
		linked_descriptor.btcnt = count;                                   		// Number of data bytes to transmit
		linked_descriptor.btctrl = DMAC_BTCTRL_SRCINC | DMAC_BTCTRL_VALID; 		// Increment source address on BEAT transfer and validate descriptor																																						 
	}
}

//
// Base (1st) layer functions - Independent DMAC initialisation with separate read/write
//
void I2C_DMAC::initWriteBytes(uint8_t devAddress, uint8_t* data, uint8_t count)
{
	initWriteBytes(devAddress, 0, data, count, 0);					// Initialise DMAC write transfer: data only, no register address
}

void I2C_DMAC::initWriteBytes(uint8_t devAddress, uint16_t regAddress, uint8_t* data, uint8_t count)
{
	initWriteBytes(devAddress, regAddress, data, count, regAddrMode);		// Initialise DMAC write transfer: register address + data
}

void I2C_DMAC::initWriteByte(uint8_t devAddress, uint16_t regAddress, uint8_t data)
{
	this->data = data;
	initWriteBytes(devAddress, regAddress, (uint8_t*)&this->data, 1, regAddrMode);	// Initialise DMAC write transfer: register address + 1 data byte
}

void I2C_DMAC::initWriteRegAddr(uint8_t devAddress, uint16_t regAddress)
{
	initWriteBytes(devAddress, regAddress, 0, 0, regAddrMode);			// Initialise DMAC write transfer: register address only, no data
}

uint8_t I2C_DMAC::getData()						
{
	return data;							// Return the received data byte
}

void I2C_DMAC::initReadBytes(uint8_t devAddress, uint8_t* data, uint8_t count)		 // Initialise DMAC read transfer: count bytes of data
{
	this->devAddress = devAddress;
	readCount = count;																					 // Copy the read byte count
	
	DMAC->CHID.reg = DMAC_CHID_ID(dmacReadChannel);                       // Activate the specified DMAC channel    
  // Set the DMAC level, trigger source and trigger action to beat (trigger for every byte received)                                              
  DMAC->CHCTRLB.reg = DMAC_CHCTRLB_LVL(dmacPriority) | DMAC_CHCTRLB_TRIGSRC(SERCOM3_DMAC_ID_RX) | DMAC_CHCTRLB_TRIGACT_BEAT; 
  DMAC->CHINTENSET.reg = DMAC_CHINTENSET_MASK;                          // Enable all 3 interrupts: SUSP, TCMPL and TERR
  descriptor.descaddr = 0;                                              // Single descriptor (no linked list)
  descriptor.srcaddr = (uint32_t)&SERCOM3->I2CM.DATA.reg;               // Set the source address
  descriptor.dstaddr = (uint32_t)data + count;  	                      // Set the destination address
  descriptor.btcnt = count;                                             // Number of data bytes to receive
  descriptor.btctrl = DMAC_BTCTRL_DSTINC | DMAC_BTCTRL_VALID;  					// Increment destination address on BEAT transfer and validate descriptor
	memcpy(&descriptor_section[dmacReadChannel], &descriptor, sizeof(dmacdescriptor)); // Copy the descriptor into SRAM descriptor array
}

void I2C_DMAC::initReadByte(uint8_t devAddress)
{
	this->devAddress = devAddress;
	initReadBytes(devAddress, &data, 1);				// Initialise DMAC read transfer: 1 byte of data
}

void I2C_DMAC::write()																					// Initiate DMAC write transmission on the I2C bus
{
	while (SERCOM3->I2CM.STATUS.bit.BUSSTATE == 0x2);    					// Wait while the I2C bus is BUSY
	if (SERCOM3->I2CM.STATUS.bit.BUSSTATE == 0x1)									// Check if the I2C bus state is at IDLE
	{
		writeBusy = true;
		DMAC->CHID.reg = DMAC_CHID_ID(dmacWriteChannel);            // Activate the DMAC write channel
		DMAC->CHCTRLA.reg |= DMAC_CHCTRLA_ENABLE;                   // Enable the DMAC write channel
		
		SERCOM3->I2CM.ADDR.reg = SERCOM_I2CM_ADDR_LEN(writeCount + regAddrLength) |    // Load the device address into the SERCOM3 ADDR register 
														 SERCOM_I2CM_ADDR_LENEN |
														 SERCOM_I2CM_ADDR_ADDR(devAddress << 1 | WRITE);
		while (SERCOM3->I2CM.SYNCBUSY.bit.SYSOP);										// Wait for synchronization	
	}
}

void I2C_DMAC::read()																						// Initiate DMAC read transmission on the I2C bus
{
	while (SERCOM3->I2CM.STATUS.bit.BUSSTATE == 0x2);   					// Wait while the I2C bus is BUSY
	if (SERCOM3->I2CM.STATUS.bit.BUSSTATE == 0x1)									// Check if the I2C bus state is at IDLE
	{
		readBusy = true;
		DMAC->CHID.reg = DMAC_CHID_ID(dmacReadChannel);							// Activate the DMAC read channel 
		DMAC->CHCTRLA.reg |= DMAC_CHCTRLA_ENABLE; 									// Enable the DMAC read channel

		SERCOM3->I2CM.ADDR.reg = SERCOM_I2CM_ADDR_LEN(readCount) |  		// Load the I2C slave device address into the SERCOM3 ADDR register 
														 SERCOM_I2CM_ADDR_LENEN |
														 SERCOM_I2CM_ADDR_ADDR(devAddress << 1 | READ);
		while (SERCOM3->I2CM.SYNCBUSY.bit.SYSOP);										// Wait for synchronization
	}
}

//
// 2nd layer functions - Combined DMAC initialisation with read or write
//
void I2C_DMAC::writeBytes(uint8_t devAddress, uint16_t regAddress, uint8_t* data, uint8_t count)
{
	initWriteBytes(devAddress, regAddress, data, count);			// Initialise DMAC write transfer: register address + data
	write();																									// Transmit the register address + data
}

void I2C_DMAC::writeByte(uint8_t devAddress, uint16_t regAddress, uint8_t data)
{
	initWriteByte(devAddress, regAddress, data);							// Initialise DMAC write transfer: register address + 1 byte data
	write();																									// Transmit the register address + data
}

void I2C_DMAC::writeRegAddr(uint8_t devAddress, uint16_t regAddress)
{
	initWriteRegAddr(devAddress, regAddress);									// Initialise DMAC write transfer: register address, no data
	write();																									// Transmit the register address
}

void I2C_DMAC::readBytes(uint8_t devAddress, uint8_t* data, uint8_t count)
{
	initReadBytes(devAddress, data, count);										// Initialise DMAC read transfer: data
	read();																										// Receive the data
}

void I2C_DMAC::readByte(uint8_t devAddress)																		
{
	initReadByte(devAddress);																	// Initialise DMAC read transfer: 1 byte data
	read(); 																									// Receive the data
}

//
// 3rd layer functions - Combined DMAC initialisation with read and write
//
void I2C_DMAC::readBytes(uint8_t devAddress, uint16_t regAddress, uint8_t* data, uint8_t count)
{
	writeRegAddr(devAddress, regAddress);				// Set the register address on the I2C slave device
	while(I2C.writeBusy);												// Wait for the write to complete
	readBytes(devAddress, data, count);					// Receive the returned data
}

void I2C_DMAC::readByte(uint8_t devAddress, uint16_t regAddress)
{
	writeRegAddr(devAddress, regAddress);				// Set the register address on the I2C slave device
	while(I2C.writeBusy);												// Wait for the write to complete
	readByte(devAddress);												// Receive the returned data byte
}

//
// DMAC Interrupt Handler: Busy Flag and Callback Section
//
void I2C_DMAC::attachWriteCallback(voidFuncPtr callback)	
{
	writeCallback = callback;						// Attach a write complete callback function
}

void I2C_DMAC::attachReadCallback(voidFuncPtr callback)	
{
	readCallback = callback;						// Attach a read complete callback function
}

void I2C_DMAC::attachDmacErrorCallback(voidFuncPtr callback)
{
	errorDmacCallback = callback;				// Attach a DMAC error callback function
}

void I2C_DMAC::attachSercom3ErrorCallback(voidFuncPtr callback)
{
	errorSercom3Callback = callback;		// Attach a SERCOM3 error callback function
}

void I2C_DMAC::detachWriteCallback()
{
	writeCallback = 0;									// Detach the write complete callback function
}

void I2C_DMAC::detachReadCallback()
{
	readCallback = 0;										// Detach the read complete callback function
}

void I2C_DMAC::detachDmacErrorCallback()
{
	errorDmacCallback = 0;							// Detach the DMAC error callback function
}

void I2C_DMAC::detachSercom3ErrorCallback()
{
	errorDmacCallback = 0;							// Detach the DMAC error callback function
}

void I2C_DMAC::DMAC_IrqHandler()
{
  //__disable_irq();																												// Disable interrupts
	uint8_t activeChannel = DMAC->INTPEND.reg & DMAC_INTPEND_ID_Msk;      // Get DMAC channel number
  DMAC->CHID.reg = DMAC_CHID_ID(activeChannel);                         // Switch to the active DMAC channel
  
	if (DMAC->CHINTFLAG.bit.TERR)																					// DMAC Transfer error (TERR)
	{
		if (i2cDmacPtr->errorDmacCallback)																	// Check if there's a DMAC error callback function
		{
			i2cDmacPtr->errorDmacCallback();																	// Call the callback function
		}
	}
	else if (DMAC->CHINTFLAG.bit.TCMPL)																		// DMAC transfer complete (TCMPL)
	{	
		if (activeChannel == i2cDmacPtr->dmacWriteChannel)									// Check write DMAC channel
		{		
			i2cDmacPtr->writeBusy = false;																		// Clear the write busy flag
			if (i2cDmacPtr->writeCallback)																		// Check if there's a write callback function
			{
				i2cDmacPtr->writeCallback();																		// Call the write callback function
			}	
		}
		else if (activeChannel == i2cDmacPtr->dmacReadChannel)							// Check read DMAC channel
		{
			i2cDmacPtr->readBusy = false;      																// Clear the read busy flag
			if (i2cDmacPtr->readCallback)																			// Check if there's a read callback function
			{
				i2cDmacPtr->readCallback();																			// Call the read callback function
			}		
		}	      
	}
	DMAC->CHINTFLAG.reg = DMAC_CHINTFLAG_MASK;															// Clear the DMAC channel interrupt flags
	//__enable_irq();																											  // Enable interrupts
}

void I2C_DMAC::SERCOM3_IrqHandler()
{
	if (SERCOM3->I2CM.INTFLAG.bit.ERROR && SERCOM3->I2CM.INTENSET.bit.ERROR)
	{
		if (i2cDmacPtr->errorSercom3Callback)																// Check if there's a SERCOM3 error callback function
		{
			i2cDmacPtr->errorSercom3Callback();																// Call the SERCOM3 error callback function
		}
		SERCOM3->I2CM.STATUS.reg |= SERCOM_I2CM_STATUS_LENERR | 						// Clear the status register flags - cleared by 
																SERCOM_I2CM_STATUS_BUSERR;							// writing to SERCOM3 ADDR.ADDR register anyway
		SERCOM3->I2CM.INTFLAG.bit.ERROR = 1;																// Clear the SERCOM3 error interrupt flag
	}
}
void DMAC_Handler() __attribute__((weak));			// Set as weakly declared linker symbol, so that the function can be overriden
void DMAC_Handler() 														// The DMAC_Handler() ISR
{
  I2C_DMAC::DMAC_IrqHandler();									// Call the I2C_DMAC's DMAC interrupt handler member function
}

void SERCOM3_Handler() __attribute__((weak));		// Set as weakly declared linker symbol, so that the function can be overriden
void SERCOM3_Handler() 													// The SERCOM3_Handler() ISR
{
	I2C_DMAC::SERCOM3_IrqHandler();								// Call the I2C_DMAC's SERCOM3 interrupt handler member function
}

I2C_DMAC I2C;																		// Instantiate the I2C object

