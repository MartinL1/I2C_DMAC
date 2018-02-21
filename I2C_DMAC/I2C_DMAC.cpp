/*
  I2C_DMAC is a non-blocking I2C library that uses a SERCOM in 
	conjunction with the Direct Memory Access Controller (DMAC).
	
	Copyright (C) Martin Lindupp 2018
	
	V1.0.0 -- Initial release 									
	V1.1.0 -- Add Arduino MKR and SAMD51 support, plus multiple I2C instances 
	V1.1.1 -- Replaced pinPeripheral() function with port register manipulation
	V1.1.2 -- Allow other classes to simultaneously use remaining DMAC channels

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

volatile I2C_DMAC* I2C_DMAC::i2cDmacPtrs[SERCOM_INST_NUM];		// Array of pointer to each instance (object) of this class
volatile uint8_t I2C_DMAC::instanceCounter;										// Number of instances (objects) of this class

volatile dmacdescriptor wrb[DMAC_CH_NUM] __attribute__ ((aligned (16)));						// DMAC write back descriptor array
dmacdescriptor descriptor_section[DMAC_CH_NUM] __attribute__ ((aligned (16)));			// DMAC channel descriptor array
dmacdescriptor descriptor __attribute__ ((aligned (16)));														// DMAC place holder descriptor

//
// Initialisation section: prep the the port pins, SERCOM and the DMAC
//
I2C_DMAC::I2C_DMAC(SERCOM* sercomClass, uint8_t pinSDA, uint8_t pinSCL) : // Constructor to initialise SERCOM, pins and DMAC channel member variables
	dmacWriteChannel(0), dmacReadChannel(1), dmacPriority(0) 
{																			
	this->pinSCL = pinSCL;																				// Copy the I2C pins
	this->pinSDA = pinSDA;
	
	if (sercomClass == &sercom0)																	// Use the sercom class to acquire the underlying SERCOMx peripheral																																																												
	{																															 
		sercom = SERCOM0;																						// This is used to allow the variant.h and variant.cpp files to
		dmacWriteTrigger = SERCOM0_DMAC_ID_TX;											// specify the sercom peripheral and the SCL/SDA pins used, just like
		dmacReadTrigger = SERCOM0_DMAC_ID_RX;												// the Wire library
#ifdef __SAMD51__
		genericClockId = SERCOM0_GCLK_ID_CORE;
		nvicId = SERCOM0_3_IRQn;
#else
		genericClockId = GCLK_CLKCTRL_ID_SERCOM0_CORE;  
		nvicId = SERCOM0_IRQn;
#endif
	}																															
	else if (sercomClass == &sercom1)															// The DMAC read and write trigger source as well as the generic clock id 
	{																															// and sercom's NVIC interrupt id are also copied 
		sercom = SERCOM1;
		dmacWriteTrigger = SERCOM1_DMAC_ID_TX;
		dmacReadTrigger = SERCOM1_DMAC_ID_RX;
#ifdef __SAMD51__
		genericClockId = SERCOM1_GCLK_ID_CORE;
		nvicId = SERCOM1_3_IRQn;
#else
		genericClockId = GCLK_CLKCTRL_ID_SERCOM1_CORE;  
		nvicId = SERCOM1_IRQn;
#endif
	}
	else if (sercomClass == &sercom2)
	{
		sercom = SERCOM2;
		dmacWriteTrigger = SERCOM2_DMAC_ID_TX;
		dmacReadTrigger = SERCOM2_DMAC_ID_RX;
#ifdef __SAMD51__
		genericClockId = SERCOM2_GCLK_ID_CORE;
		nvicId = SERCOM2_3_IRQn;
#else
		genericClockId = GCLK_CLKCTRL_ID_SERCOM2_CORE;
    nvicId = SERCOM2_IRQn;
#endif
	}
	else if (sercomClass == &sercom3)
	{
		sercom = SERCOM3;
		dmacWriteTrigger = SERCOM3_DMAC_ID_TX;
		dmacReadTrigger = SERCOM3_DMAC_ID_RX;		
#ifdef __SAMD51__
		genericClockId = SERCOM3_GCLK_ID_CORE;
		nvicId = SERCOM3_3_IRQn;
#else
		genericClockId = GCLK_CLKCTRL_ID_SERCOM3_CORE;
    nvicId = SERCOM3_IRQn;
#endif
	}
	else if (sercomClass == &sercom4)
	{
		sercom = SERCOM4;
		dmacWriteTrigger = SERCOM4_DMAC_ID_TX;
		dmacReadTrigger = SERCOM4_DMAC_ID_RX;
#ifdef __SAMD51__
		genericClockId = SERCOM4_GCLK_ID_CORE;
		nvicId = SERCOM4_3_IRQn;
#else
		genericClockId = GCLK_CLKCTRL_ID_SERCOM4_CORE;
    nvicId = SERCOM4_IRQn;
#endif
	}
	else if (sercomClass == &sercom5)
	{
		sercom = SERCOM5;
		dmacWriteTrigger = SERCOM5_DMAC_ID_TX;
		dmacReadTrigger = SERCOM5_DMAC_ID_RX;
#ifdef __SAMD51__
		genericClockId = SERCOM5_GCLK_ID_CORE;
		nvicId = SERCOM5_3_IRQn;
#else
		genericClockId = GCLK_CLKCTRL_ID_SERCOM5_CORE;
    nvicId = SERCOM5_IRQn;
#endif
	}
}																																		

void I2C_DMAC::begin(uint32_t baudrate, uint8_t regAddrMode) 				// Set baud rate and the register address mode: 8-bit or 16-bit				
{  
	this->regAddrMode = regAddrMode;
	
	// Enable the SCL and SDA lines on the sercom 
  PORT->Group[g_APinDescription[SCL].ulPort].PINCFG[g_APinDescription[SCL].ulPin].bit.PMUXEN = 1;  
  PORT->Group[g_APinDescription[SDA].ulPort].PINCFG[g_APinDescription[SDA].ulPin].bit.PMUXEN = 1;
  PORT->Group[g_APinDescription[SDA].ulPort].PMUX[g_APinDescription[SDA].ulPin >> 1].reg = 
		PORT_PMUX_PMUXO(PIO_SERCOM) | PORT_PMUX_PMUXE(PIO_SERCOM);
			
	if (!DMAC->CTRL.bit.DMAENABLE)			 // Enable the DMAC, if it hasn't already been enabled
	{
#ifdef __SAMD51__
		for (uint8_t i = 0; i < 5; i++)					 						// Iterate through the SAMD51's 5 DMAC interrupt channels
		{
			NVIC_ClearPendingIRQ((IRQn_Type)(DMAC_0_IRQn + i));	  // Clear any pending DMAC interrupts
			NVIC_SetPriority((IRQn_Type)(DMAC_0_IRQn + i), 0);  	// Set the Nested Vector Interrupt Controller (NVIC) priority for the DMAC to 0 (highest) 
			NVIC_EnableIRQ((IRQn_Type)(DMAC_0_IRQn + i));         // Connect the DMAC to the Nested Vector Interrupt Controller (NVIC)
		}
#else
		NVIC_ClearPendingIRQ(DMAC_IRQn);			 	 // Clear any pending DMAC interrupts
		NVIC_SetPriority(DMAC_IRQn, 0);    		 	 // Set the Nested Vector Interrupt Controller (NVIC) priority for the DMAC to 0 (highest) 
		NVIC_EnableIRQ(DMAC_IRQn);         		   // Connect the DMAC to the Nested Vector Interrupt Controller (NVIC)
#endif
		DMAC->CTRL.bit.SWRST = 1;											 											// Reset the DMAC
		while (DMAC->CTRL.bit.SWRST);																				// Wait for synchronization
		DMAC->BASEADDR.reg = (uint32_t)descriptor_section;                  // Set the DMAC descriptor base address
		DMAC->WRBADDR.reg = (uint32_t)wrb;                                  // Set the DMAC descriptor write-back address
		DMAC->CTRL.reg = DMAC_CTRL_DMAENABLE | DMAC_CTRL_LVLEN(0xf);        // Enable the DMAC peripheral and enable priority levels
	}
	
	NVIC_ClearPendingIRQ(nvicId);		// Clear any Nested Vector Interrupt Controller (NVIC) pending interrupts
	NVIC_SetPriority(nvicId, 0);    // Set the Nested Vector Interrupt Controller (NVIC) priority for the selected sercom to 0 (highest) 
	NVIC_EnableIRQ(nvicId); 				// Connect selected sercom to the Nested Vector Interrupt Controller (NVIC)
	
#ifdef __SAMD51__	
	// Enable GCLK1 (48MHz) on the selected sercom
	GCLK->PCHCTRL[genericClockId].reg = GCLK_PCHCTRL_CHEN | GCLK_PCHCTRL_GEN_GCLK1;
#else
	// Enable GCLK0 (48MHz) on the selected sercom
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN |              		// Enable GCLK0 to the selected sercom
											GCLK_CLKCTRL_GEN_GCLK0 |          		// Select GCLK0 as source
											GCLK_CLKCTRL_ID(genericClockId);      // Select the selected sercom as destination
	while (GCLK->STATUS.bit.SYNCBUSY); 												// Wait for synchronization
#endif

	sercom->I2CM.CTRLA.bit.SWRST = 1;                                          		// Reset the SERCOM
  while (sercom->I2CM.CTRLA.bit.SWRST || sercom->I2CM.SYNCBUSY.bit.SWRST);  		// Wait for synchronization
	
  sercom->I2CM.CTRLA.reg = SERCOM_I2CM_CTRLA_MODE(I2C_MASTER_OPERATION);   			// Set I2C master mode                     
  sercom->I2CM.CTRLB.reg = SERCOM_I2CM_CTRLB_SMEN;   										  			// Enable Smart Mode
#if __SAMD51__
  sercom->I2CM.BAUD.bit.BAUD = SERCOM_48M_REF / (2 * baudrate) - 7;      				// Set I2C master SCL baud rate
#else
	sercom->I2CM.BAUD.bit.BAUD = SystemCoreClock / (2 * baudrate) - 7;      			// Set I2C master SCL baud rate
#endif
	sercom->I2CM.CTRLA.bit.ENABLE = 1 ;            // Enable SERCOM in I2C master mode
  while (sercom->I2CM.SYNCBUSY.bit.ENABLE);      // Wait for synchronization
  sercom->I2CM.STATUS.bit.BUSSTATE = 0x01;       // Set the I2C bus to IDLE state    
  while (sercom->I2CM.SYNCBUSY.bit.SYSOP);       // Wait for synchronization
	sercom->I2CM.INTENSET.bit.ERROR = 1;					 // Enable SERCOM ERROR interrupts
	
	i2cDmacPtrs[instanceCounter++] = this;				 // Copy the pointer to "this" object to the I2C_DMAC pointer array
}																								 // and increment the instance counter

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
	for (uint8_t i = 0; i < instanceCounter; i++)						// Iterate through each I2C_DMAC instance
	{
		if (i2cDmacPtrs[i] == this)														// Find the location of "this" instance in the I2C_DMAC array pointer
		{
			i2cDmacPtrs[i] = 0;																	// Pull this instance pointer from the array
			if (i < instanceCounter - 1)												// If the current instance not the last instance
			{
				for (uint8_t j = i; j < instanceCounter - 1; j++)
				{
					i2cDmacPtrs[j] = i2cDmacPtrs[j + 1];						// Shift the I2C_DMAC pointer down the array
				}
				i2cDmacPtrs[instanceCounter - 1] = 0;							// Set the last instance pointer to NULL
			}
			instanceCounter--;																	// Decrement the instance counter
			break;																							// Escape from the (for) loop
		}
	}
	
	// Return the SCL and SDA lines on the sercom to GPIO
  PORT->Group[g_APinDescription[SCL].ulPort].PINCFG[g_APinDescription[SCL].ulPin].bit.PMUXEN = 0;  
  PORT->Group[g_APinDescription[SDA].ulPort].PINCFG[g_APinDescription[SDA].ulPin].bit.PMUXEN = 0;
  PORT->Group[g_APinDescription[SDA].ulPort].PMUX[g_APinDescription[SDA].ulPin >> 1].reg = 
		PORT_PMUX_PMUXO(0) | PORT_PMUX_PMUXE(0);
	
	sercom->I2CM.CTRLA.bit.ENABLE = 0;            															// Disable the I2C master mode
  while (sercom->I2CM.SYNCBUSY.bit.ENABLE);     															// Wait for synchronization
	sercom->I2CM.CTRLA.bit.SWRST = 1;                                           // Reset SERCOM3
  while (sercom->I2CM.CTRLA.bit.SWRST || sercom->I2CM.SYNCBUSY.bit.SWRST);  	// Wait for synchronization

#ifdef __SAMD51__
	// Disable GCLK1 (48Mhz) on the selected sercom
	GCLK->PCHCTRL[genericClockId].reg = /*GCLK_PCHCTRL_CHEN |*/ GCLK_PCHCTRL_GEN_GCLK1;
#else
	// Disable GCLK0 (48MHz) on the selected sercom
  GCLK->CLKCTRL.reg = /*GCLK_CLKCTRL_CLKEN |*/          	// Disable GCLK0 to the selected sercom - intentionally commented out
											GCLK_CLKCTRL_GEN_GCLK0 |          	// Select GCLK0 as source
											GCLK_CLKCTRL_ID(genericClockId);    // Select the selected sercom as destination
	while (GCLK->STATUS.bit.SYNCBUSY); 											// Wait for synchronization
#endif
	
	if (instanceCounter == 0)
	{
		NVIC_DisableIRQ(nvicId); 									// Disconnect selected sercom from the Nested Vector Interrupt Controller (NVIC)
		
		DMAC->CTRL.bit.DMAENABLE = 0;          		// Disable the DMAC
		while(DMAC->CTRL.bit.DMAENABLE);			 		// Wait for synchronization			
		DMAC->CTRL.bit.SWRST = 1;									// Reset the DMAC
		while (DMAC->CTRL.bit.SWRST);							// Wait for synchronization
#ifdef __SAMD51__
		for (uint8_t i = 0; i < 5; i++)					  // Iterate through the SAMD51's 5 DMAC interrupt channels
		{
			NVIC_DisableIRQ((IRQn_Type)(DMAC_0_IRQn + i));		// Disconnect the DMAC from the Nested Vector Interrupt Controller (NVIC)
		}
#else
		NVIC_DisableIRQ(DMAC_IRQn);								// Disconnect the DMAC from the Nested Vector Interrupt Controller (NVIC)
#endif
	}
}

void I2C_DMAC::setClock(uint32_t baudrate)
{
	sercom->I2CM.CTRLA.bit.ENABLE = 0;            													// Disable SERCOM3 in I2C master mode
  while (sercom->I2CM.SYNCBUSY.bit.ENABLE);     													// Wait for synchronization
#if __SAMD51__
  sercom->I2CM.BAUD.bit.BAUD = SERCOM_48M_REF / (2 * baudrate) - 7;      				// Set I2C master SCL baud rate
#else
	sercom->I2CM.BAUD.bit.BAUD = SystemCoreClock / (2 * baudrate) - 7;      			// Set I2C master SCL baud rate
#endif	
	sercom->I2CM.CTRLA.bit.ENABLE = 1 ;           													// Enable SERCOM3 in I2C master mode
  while (sercom->I2CM.SYNCBUSY.bit.ENABLE);     													// Wait for synchronization
}

void I2C_DMAC::setWriteChannel(uint8_t channel)
{
#ifdef __SAMD51__
	dmacWriteChannel = channel < 32 ? channel : dmacWriteChannel;			// Set the write DMAC channel, (default channel 0)
#else
	dmacWriteChannel = channel < 12 ? channel : dmacWriteChannel;			// Set the write DMAC channel, (default channel 0)
#endif
}

void I2C_DMAC::setReadChannel(uint8_t channel)
{
#ifdef __SAMD51__
	dmacReadChannel = channel < 32 ? channel : dmacReadChannel;				// Set the read DMAC channel, (default channel 1)
#else
	dmacReadChannel = channel < 12 ? channel : dmacReadChannel;				// Set the read DMAC channel, (default channel 1)
#endif
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
	this->devAddress = devAddress;							// Copy the device address, plus write byte count and register address length
	writeCount = count;
	this->regAddrLength = regAddrLength;
	
	while (sercom->I2CM.STATUS.bit.BUSSTATE == 0x2);    					// Wait while the I2C bus is BUSY
	if (sercom->I2CM.STATUS.bit.BUSSTATE == 0x1)									// Check if the I2C bus state is at IDLE
	{
#ifdef __SAMD51__
		// Set the DMAC level, trigger source and trigger action to burst (trigger for every byte transmitted)
		DMAC->Channel[dmacWriteChannel].CHCTRLA.reg = DMAC_CHCTRLA_TRIGSRC(dmacWriteTrigger) | DMAC_CHCTRLA_TRIGACT_BURST; 
		DMAC->Channel[dmacWriteChannel].CHPRILVL.reg = DMAC_CHPRILVL_PRILVL(dmacPriority);		// Set the channel priority level
		DMAC->Channel[dmacWriteChannel].CHINTENSET.reg = DMAC_CHINTENSET_MASK; 	// Enable all 3 interrupts: SUSP, TCMPL and TERR
#else	
		DMAC->CHID.reg = DMAC_CHID_ID(dmacWriteChannel);                        // Activate specified DMAC write channel 
		// Set the DMAC level, trigger source and trigger action to beat (trigger for every byte transmitted)
		DMAC->CHCTRLB.reg = DMAC_CHCTRLB_LVL(dmacPriority) | DMAC_CHCTRLB_TRIGSRC(dmacWriteTrigger) | DMAC_CHCTRLB_TRIGACT_BEAT; 
		DMAC->CHINTENSET.reg = DMAC_CHINTENSET_MASK;                          	// Enable all 3 interrupts: SUSP, TCMPL and TERR
#endif
		if (regAddrLength == 0)																									// Writing data only
		{
			descriptor.descaddr = 0;  																						// Set this to the last descriptor (no linked list)
			descriptor.srcaddr = (uint32_t)data + count;     											// Set the source address
			descriptor.dstaddr = (uint32_t)&sercom->I2CM.DATA.reg;                // Set the destination address
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
		descriptor.dstaddr = (uint32_t)&sercom->I2CM.DATA.reg;          	   	  // Set the destination address
		descriptor.btcnt = regAddrLength;                                   		// Size of the register address in bytes
		descriptor.btctrl = DMAC_BTCTRL_SRCINC | DMAC_BTCTRL_VALID;    					// Increment source address on BEAT transfer and validate descriptor
		memcpy(&descriptor_section[dmacWriteChannel], &descriptor, sizeof(dmacdescriptor));   // Copy the descriptor into SRAM descriptor array
		
		if (count > 0)																													// Append write data as linked descriptor
		{	
			linked_descriptor.descaddr = 0;     																	// Set linked_descriptor to last in the list
			linked_descriptor.srcaddr = (uint32_t)data + count;  			 						// Set the source address
			linked_descriptor.dstaddr = (uint32_t)&sercom->I2CM.DATA.reg;         // Set the destination address
			linked_descriptor.btcnt = count;                                   		// Number of data bytes to transmit
			linked_descriptor.btctrl = DMAC_BTCTRL_SRCINC | DMAC_BTCTRL_VALID; 		// Increment source address on BEAT transfer and validate descriptor																																						 
		}
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
	this->devAddress = devAddress;																// Copy device address and read byte count
	readCount = count;				

	while (sercom->I2CM.STATUS.bit.BUSSTATE == 0x2);    					// Wait while the I2C bus is BUSY
	if (sercom->I2CM.STATUS.bit.BUSSTATE == 0x1)									// Check if the I2C bus state is at IDLE
	{
#ifdef __SAMD51__
		// Set the DMAC level, trigger source and trigger action to burst (trigger for every byte received)
		DMAC->Channel[dmacReadChannel].CHCTRLA.reg = DMAC_CHCTRLA_TRIGSRC(dmacReadTrigger) | DMAC_CHCTRLA_TRIGACT_BURST; 
		DMAC->Channel[dmacReadChannel].CHPRILVL.reg = DMAC_CHPRILVL_PRILVL(dmacPriority);	// Set the channel priority level
		DMAC->Channel[dmacReadChannel].CHINTENSET.reg = DMAC_CHINTENSET_MASK; 	// Enable all 3 interrupts: SUSP, TCMPL and TERR
#else	
		DMAC->CHID.reg = DMAC_CHID_ID(dmacReadChannel);                       // Activate the specified DMAC channel    
		// Set the DMAC level, trigger source and trigger action to beat (trigger for every byte received)                                              
		DMAC->CHCTRLB.reg = DMAC_CHCTRLB_LVL(dmacPriority) | DMAC_CHCTRLB_TRIGSRC(dmacReadTrigger) | DMAC_CHCTRLB_TRIGACT_BEAT; 
		DMAC->CHINTENSET.reg = DMAC_CHINTENSET_MASK;                          // Enable all 3 interrupts: SUSP, TCMPL and TERR
#endif
		descriptor.descaddr = 0;                                              // Single descriptor (no linked list)
		descriptor.srcaddr = (uint32_t)&sercom->I2CM.DATA.reg;                // Set the source address
		descriptor.dstaddr = (uint32_t)data + count;  	                      // Set the destination address
		descriptor.btcnt = count;                                             // Number of data bytes to receive
		descriptor.btctrl = DMAC_BTCTRL_DSTINC | DMAC_BTCTRL_VALID;  					// Increment destination address on BEAT transfer and validate descriptor
		memcpy(&descriptor_section[dmacReadChannel], &descriptor, sizeof(dmacdescriptor)); // Copy the descriptor into SRAM descriptor array
	}
}

void I2C_DMAC::initReadByte(uint8_t devAddress)
{
	this->devAddress = devAddress;
	initReadBytes(devAddress, &data, 1);				// Initialise DMAC read transfer: 1 byte of data
}

void I2C_DMAC::write()																					// Initiate DMAC write transmission on the I2C bus
{
	while (sercom->I2CM.STATUS.bit.BUSSTATE == 0x2);    					// Wait while the I2C bus is BUSY
	if (sercom->I2CM.STATUS.bit.BUSSTATE == 0x1)									// Check if the I2C bus state is at IDLE
	{
		writeBusy = true;
#ifdef __SAMD51__
		DMAC->Channel[dmacWriteChannel].CHCTRLA.reg |= DMAC_CHCTRLA_ENABLE;		// Enable the DMAC write channel
#else
		DMAC->CHID.reg = DMAC_CHID_ID(dmacWriteChannel);            // Activate the DMAC write channel
		DMAC->CHCTRLA.reg |= DMAC_CHCTRLA_ENABLE;                   // Enable the DMAC write channel
#endif
		
		sercom->I2CM.ADDR.reg = SERCOM_I2CM_ADDR_LEN(writeCount + regAddrLength) |    // Load the device address into the SERCOM3 ADDR register 
														SERCOM_I2CM_ADDR_LENEN |
														SERCOM_I2CM_ADDR_ADDR(devAddress << 1 | WRITE);
		while (sercom->I2CM.SYNCBUSY.bit.SYSOP);										// Wait for synchronization	
	}
}

void I2C_DMAC::read()																						// Initiate DMAC read transmission on the I2C bus
{
	while (sercom->I2CM.STATUS.bit.BUSSTATE == 0x2);   						// Wait while the I2C bus is BUSY
	if (sercom->I2CM.STATUS.bit.BUSSTATE == 0x1)									// Check if the I2C bus state is at IDLE
	{
		readBusy = true;
#ifdef __SAMD51__
		DMAC->Channel[dmacReadChannel].CHCTRLA.reg |= DMAC_CHCTRLA_ENABLE;		// Enable the DMAC read channel
#else
		DMAC->CHID.reg = DMAC_CHID_ID(dmacReadChannel);							// Activate the DMAC read channel 
		DMAC->CHCTRLA.reg |= DMAC_CHCTRLA_ENABLE; 									// Enable the DMAC read channel
#endif
		
		sercom->I2CM.ADDR.reg = SERCOM_I2CM_ADDR_LEN(readCount) |  	// Load the I2C slave device address into the SERCOM3 ADDR register 
														SERCOM_I2CM_ADDR_LENEN |
														SERCOM_I2CM_ADDR_ADDR(devAddress << 1 | READ);
		while (sercom->I2CM.SYNCBUSY.bit.SYSOP);										// Wait for synchronization
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

void I2C_DMAC::attachSercomErrorCallback(voidFuncPtr callback)
{
	errorSercomCallback = callback;		 	// Attach a SERCOM error callback function
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

void I2C_DMAC::detachSercomErrorCallback()
{
	errorDmacCallback = 0;							// Detach the SERCOM error callback function
}

void I2C_DMAC::DMAC_IrqHandler()
{
  //__disable_irq();																										// Disable interrupts
	volatile I2C_DMAC* i2cDmacPtr = i2cDmacPtrs[0];												// Set I2C_DMAC pointer to the first I2C_DMAC instance
	uint8_t activeChannel = DMAC->INTPEND.reg & DMAC_INTPEND_ID_Msk;      // Get DMAC channel number
	
	if (instanceCounter > 1)																							// If there's only one instance of the I2C_DMAC object,
	{																																			// skip finding the active channel instance
		for (uint8_t i = 0; i < instanceCounter; i++)												// Find the I2C_DMAC instance from the active channel
		{
			if (activeChannel == i2cDmacPtrs[i]->dmacWriteChannel ||					// Compare the active channel against the object's
					activeChannel == i2cDmacPtrs[i]->dmacReadChannel)							//
			{
				i2cDmacPtr = i2cDmacPtrs[i];																		// Assign I2C_DMAC pointer to the object instance with 
			}																																	// the active channel
		}
	}

#ifdef __SAMD51__
	if (DMAC->Channel[activeChannel].CHINTFLAG.bit.TERR)									// DMAC Transfer error (TERR)
#else  
	DMAC->CHID.reg = DMAC_CHID_ID(activeChannel);                         // Switch to the active DMAC channel
	if (DMAC->CHINTFLAG.bit.TERR)																					// DMAC Transfer error (TERR)
#endif
	{
		if (i2cDmacPtr->errorDmacCallback)																	// Check if there's a DMAC error callback function
		{
			i2cDmacPtr->errorDmacCallback();																	// Call the callback function
		}
	}
#ifdef __SAMD51__
	else if (DMAC->Channel[activeChannel].CHINTFLAG.bit.TCMPL)						// DMAC transfer complete (TCMPL)
#else
	else if (DMAC->CHINTFLAG.bit.TCMPL)																		// DMAC transfer complete (TCMPL)
#endif
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
#ifdef __SAMD51__
	DMAC->Channel[activeChannel].CHINTFLAG.reg = DMAC_CHINTFLAG_MASK;			// Clear the DMAC channel interrupt flags
#else
	DMAC->CHINTFLAG.reg = DMAC_CHINTFLAG_MASK;														// Clear the DMAC channel interrupt flags
#endif
	//__enable_irq();																											// Enable interrupts
}

void I2C_DMAC::SERCOM_IrqHandler()
{
	if (sercom->I2CM.INTFLAG.bit.ERROR && sercom->I2CM.INTENSET.bit.ERROR)
	{
		if (errorSercomCallback)																			// Check if there's a SERCOM3 error callback function
		{
			errorSercomCallback();																			// Call the SERCOM3 error callback function
		}
		sercom->I2CM.STATUS.reg |= SERCOM_I2CM_STATUS_LENERR | 				// Clear the status register flags - cleared by 
															 SERCOM_I2CM_STATUS_BUSERR;					// writing to SERCOM3 ADDR.ADDR register anyway
		sercom->I2CM.INTFLAG.bit.ERROR = 1;														// Clear the SERCOM error interrupt flag
	}
}

#ifdef __SAMD51__
void DMAC_0_Handler() __attribute__((weak));			// Set as weakly declared linker symbol, so that the function can be overriden
void DMAC_0_Handler() 														// The DMAC_0_Handler() ISR
{
	I2C_DMAC::DMAC_IrqHandler();										// Call the I2C_DMAC's DMAC interrupt handler member function
}

void DMAC_1_Handler() __attribute__((weak));			// Set as weakly declared linker symbol, so that the function can be overriden
void DMAC_1_Handler() 														// The DMAC_1_Handler() ISR
{
  I2C_DMAC::DMAC_IrqHandler();										// Call the I2C_DMAC's DMAC interrupt handler member function											
}

void DMAC_2_Handler() __attribute__((weak));			// Set as weakly declared linker symbol, so that the function can be overriden
void DMAC_2_Handler() 														// The DMAC_2_Handler() ISR
{
  I2C_DMAC::DMAC_IrqHandler();										// Call the I2C_DMAC's DMAC interrupt handler member function
}

void DMAC_3_Handler() __attribute__((weak));			// Set as weakly declared linker symbol, so that the function can be overriden
void DMAC_3_Handler() 														// The DMAC_3_Handler() ISR
{
  I2C_DMAC::DMAC_IrqHandler();										// Call the I2C_DMAC's DMAC interrupt handler member function
}

void DMAC_4_Handler() __attribute__((weak));			// Set as weakly declared linker symbol, so that the function can be overriden
void DMAC_4_Handler() 														// The DMAC_4_Handler() ISR
{
	I2C_DMAC::DMAC_IrqHandler();										// Call the I2C_DMAC's DMAC interrupt handler member function
}
#else
void DMAC_Handler() __attribute__((weak));				// Set as weakly declared linker symbol, so that the function can be overriden
void DMAC_Handler() 															// The DMAC_Handler() ISR
{
	I2C_DMAC::DMAC_IrqHandler();										// Call the I2C_DMAC's DMAC interrupt handler member function
}
#endif

#if WIRE_INTERFACES_COUNT > 0										//
  /* In case new variant doesn't define these macros,
   * we put here the ones for Arduino Zero.
   *
   * These values should be different on some variants!
   */
  #ifndef PERIPH_WIRE
    #define PERIPH_WIRE        sercom3
		#ifdef __SAMD51__
			#define WIRE_IT_HANDLER  SERCOM3_3_Handler
		#else
			#define WIRE_IT_HANDLER  SERCOM3_Handler
		#endif
  #endif // PERIPH_WIRE
	I2C_DMAC I2C(&PERIPH_WIRE, PIN_WIRE_SDA, PIN_WIRE_SCL);		// Instantiate the I2C object
	
	void WIRE_IT_HANDLER() __attribute__((weak));
  void WIRE_IT_HANDLER() 																		// Call the I2C_DMAC's SERCOM interrupt handler member function
	{
		I2C.SERCOM_IrqHandler();
  }
	#ifdef __SAMD51__
	#else
	
	#endif
#endif

#if WIRE_INTERFACES_COUNT > 1
	I2C_DMAC I2C1(&PERIPH_WIRE1, PIN_WIRE1_SDA, PIN_WIRE1_SCL);

  void WIRE1_IT_HANDLER() __attribute__((weak));
  void WIRE1_IT_HANDLER() 																	// Call the I2C_DMAC's SERCOM interrupt handler member function
	{
    I2C1.SERCOM_IrqHandler();
  }
#endif

#if WIRE_INTERFACES_COUNT > 2
	I2C_DMAC I2C2(&PERIPH_WIRE2, PIN_WIRE2_SDA, PIN_WIRE2_SCL);

  void WIRE2_IT_HANDLER() __attribute__((weak));
  void WIRE2_IT_HANDLER() 																	// Call the I2C_DMAC's SERCOM interrupt handler member function
	{
    I2C2.SERCOM_IrqHandler();
  }
#endif

#if WIRE_INTERFACES_COUNT > 3
	I2C_DMAC I2C3(&PERIPH_WIRE3, PIN_WIRE3_SDA, PIN_WIRE3_SCL);

  void WIRE3_IT_HANDLER() __attribute__((weak));
  void WIRE3_IT_HANDLER() 																	// Call the I2C_DMAC's SERCOM interrupt handler member function
	{
    I2C3.SERCOM_IrqHandler();
  }
#endif

#if WIRE_INTERFACES_COUNT > 4
	I2C_DMAC I2C4(&PERIPH_WIRE4, PIN_WIRE4_SDA, PIN_WIRE4_SCL);
	
	void WIRE4_IT_HANDLER() __attribute__((weak));
  void WIRE4_IT_HANDLER() 																	// Call the I2C_DMAC's SERCOM interrupt handler member function
	{
    I2C4.SERCOM_IrqHandler();
  }
#endif

#if WIRE_INTERFACES_COUNT > 5
	I2C_DMAC I2C5(&PERIPH_WIRE5, PIN_WIRE5_SDA, PIN_WIRE5_SCL);
	
	void WIRE5_IT_HANDLER() __attribute__((weak));
  void WIRE5_IT_HANDLER() 																	// Call the I2C_DMAC's SERCOM interrupt handler member function
	{
    I2C5.SERCOM_IrqHandler();
  }
#endif
