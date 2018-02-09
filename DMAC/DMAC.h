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

// DMAC descriptor configuration file

#ifndef _DMAC_h
#define _DMAC_h

typedef struct {											// DMAC descriptor structure
	uint16_t btctrl;
	uint16_t btcnt;
	uint32_t srcaddr;
	uint32_t dstaddr;
	uint32_t descaddr;
} dmacdescriptor ;

static volatile dmacdescriptor wrb[12] __attribute__ ((aligned (16)));						// DMAC write back descriptor array
static dmacdescriptor descriptor_section[12] __attribute__ ((aligned (16)));			// DMAC channel descriptor array
static dmacdescriptor descriptor __attribute__ ((aligned (16)));									// DMAC place holder descriptor

#endif

