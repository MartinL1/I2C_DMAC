/*
  I2C_DMAC is a non-blocking I2C library that uses SERCOM3 in 
	conjunction with the Direct Memory Access Controller (DMAC).
	
	Copyright (C) Martin Lindupp 2018
	
	V1.0.0 -- Initial release
	V1.1.0 -- Add Arduino MKR and SAMD51 support, plus multiple I2C instances 

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

#ifndef _I2C_DMAC_h
#define _I2C_DMAC_h

#include <Arduino.h>
#include <wiring_private.h>
#include <DMAC.h>

enum { REG_ADDR_8BIT = 1, REG_ADDR_16BIT };										// Register address mode definitions, 8-bit address or 16-bit address

class I2C_DMAC {
	public:
		I2C_DMAC(SERCOM* s, uint8_t pinSDA, uint8_t pinSCL);			// Class constructor to initialise, SERCOM class, pins and member variables
		void begin();																							// Begin with 100kHz I2C bus clock speed and 8-bit register address mode
		void begin(uint32_t baudrate);														// Begin with specified baud rate and 8-bit register address mode
		void begin(uint32_t baudrate, uint8_t regAddrMode);				// Begin with specified baud rate and register address mode
		void end();																								// Tear down and tidy up resources
		void setClock(uint32_t baudrate);													// Set the I2C bus clock speed to the specified baud rate
		void setWriteChannel(uint8_t channel);										// Set the DMAC write channel number (0 - 11), default 0
		void setReadChannel(uint8_t channel);											// Set the DMAC read channel number (0 - 11), default 1
		void setPriority(uint8_t priority);												// Set the priority level for both read and write DMAC channels (0 - 3), default 0
		void setRegAddrMode(uint8_t regAddrMode);									// Set the register address mode REG_ADDR_8BIT or REG_ADDR_16BIT
		void initWriteBytes(uint8_t devAddress, uint8_t* data, uint8_t count);			// Initialise DMAC to send data, (no register address)								
		void initWriteBytes(uint8_t devAddress, uint16_t regAddress, uint8_t* data, uint8_t count);  // Initialise DMAC to send register address + data
		void initWriteByte(uint8_t devAddress, uint16_t regAddress, uint8_t data);  // Initialise DMAC to send register address + 1 data byte 
		void initWriteRegAddr(uint8_t devAddress, uint16_t regAddress);							// Initialise DMAC to send register address, (no data)
		void initReadBytes(uint8_t devAddress, uint8_t* data, uint8_t count);				// Initialise DMAC to receive data
		void initReadByte(uint8_t devAddress);																			// Initialise DMAC to receive 1 data byte
		uint8_t getData();																				// Retrieve the received data byte
		void write();																							// Transmit on I2C bus
		void read();																							// Receive on I2C bus
		void writeBytes(uint8_t devAddress, uint16_t regAddress, uint8_t* data, uint8_t count);	// Transmit data to the I2C device's register address
		void writeByte(uint8_t devAddress, uint16_t regAddress, uint8_t data);		// Transmit 1 data byte to the I2C device's register address
		void writeRegAddr(uint8_t devAddress, uint16_t regAddress);								// Write the register address to the I2C device
		void readBytes(uint8_t devAddress, uint8_t* data, uint8_t count);					// Receive data from the I2C device
		void readByte(uint8_t devAddress);																				// Receive 1 data byte from the I2C device
		void readBytes(uint8_t devAddress, uint16_t regAddress, uint8_t* data, uint8_t count);	// Receive data from the I2C device's register address
		void readByte(uint8_t devAddress, uint16_t regAddress);		// Receive a byte from the I2C device's register address
		void attachWriteCallback(voidFuncPtr callback);						// Attach a DMAC write callback function
		void attachReadCallback(voidFuncPtr callback);						// Attach a DMAC read callback function
		void attachDmacErrorCallback(voidFuncPtr callback);				// Attach a DMAC error callback function
		void attachSercomErrorCallback(voidFuncPtr callback);			// Attach a SERCOM error callback function
		void detachWriteCallback();																// Detach the DMAC write callback function
		void detachReadCallback();																// Detach the DMAC read callback function
		void detachDmacErrorCallback();														// Detach the DAMC error callback function
		void detachSercomErrorCallback();													// Detach the SERCOM error callback function
		static void DMAC_IrqHandler(); 														// DMAC interrupt handler function
		void SERCOM_IrqHandler();																	// SERCOM interrupt handler function
		
		volatile boolean writeBusy;																// Write busy flag - indicates write transfer in progress
		volatile boolean readBusy;																// Read busy flag - indicates read transfer in progress
		static volatile I2C_DMAC* i2cDmacPtrs[SERCOM_INST_NUM];		// Array of pointer to each instance (object) of this class
		static volatile uint8_t instanceCounter;									// Number of instances (objects) of this class
	protected:
	// Generic initialise write DMAC transfer function
		void initWriteBytes(uint8_t devAddress, uint16_t regAddress, uint8_t* data, uint8_t count, uint8_t readAddrLength);
	private:	
		enum { WRITE, READ };                 // I2C read and write bits definitions, WRITE: 0, READ: 1
		dmacdescriptor linked_descriptor __attribute__ ((aligned (16)));			// DMAC linked descriptor		
		
		Sercom* sercom;										// Pointer to the selected SERCOMx peripheral
		uint8_t pinSDA;										// The selected SDA pin
		uint8_t pinSCL;										// The selected SCL pin
		uint8_t devAddress;								// The I2C device address
		uint8_t regAddress[2];						// The I2C device's register address sent MSB first, LSB last
		uint8_t data;											// Used for single byte data transfer
		uint8_t writeCount;								// The number of data bytes to transmit
		uint8_t readCount;								// The number of data bytes to receive
		uint8_t regAddrLength;						// The length of the register address, 0: data only, 1: 8-bit, 2: 16-bit
		uint8_t regAddrMode;							// The current register address mode: REG_ADDR_8BIT or REG_ADDR_16BIT
		uint8_t dmacPriority;							// The DMAC read and write priority level
		uint8_t dmacWriteTrigger;					// The DMAC write peripheral trigger source
		uint8_t dmacReadTrigger;					// The DMAC read peripheral trigger source
		uint8_t genericClockId;						// The generic clock ID used to select the sercom peripheral to be clocked
		IRQn_Type nvicId;									// The Nested Vector Interrupt (NVIC) interrupt type for the SERCOMx peripheral
		
		typedef void (*voidFuncPtr)(void);											// Alias for function pointer declarations
		volatile voidFuncPtr writeCallback;											// Write callback function pointer
		volatile voidFuncPtr readCallback;											// Read callback function pointer
		volatile voidFuncPtr errorDmacCallback;									// DMAC error callback function pointer
		volatile voidFuncPtr errorSercomCallback;								// SERCOM error callback function pointer
		volatile uint8_t dmacWriteChannel;											// DMAC write channel number (0 to 11), default 0
		volatile uint8_t dmacReadChannel;												// DMAC read channel number (0 to 11), default 1
};

#if WIRE_INTERFACES_COUNT > 0
  extern I2C_DMAC I2C;								// Indicate that the I2C object is externally instantiated
#endif
#if WIRE_INTERFACES_COUNT > 1
  extern I2C_DMAC I2C1;								// Indicate that the I2C1 object is externally instantiated
#endif
#if WIRE_INTERFACES_COUNT > 2
  extern I2C_DMAC I2C2;								// Indicate that the I2C2 object is externally instantiated
#endif
#if WIRE_INTERFACES_COUNT > 3
  extern I2C_DMAC I2C3;								// Indicate that the I2C3 object is externally instantiated
#endif
#if WIRE_INTERFACES_COUNT > 4
  extern I2C_DMAC I2C4;								// Indicate that the I2C4 object is externally instantiated
#endif
#if WIRE_INTERFACES_COUNT > 5
  extern I2C_DMAC I2C5;								// Indicate that the I2C5 object is externally instantiated
#endif

#endif

