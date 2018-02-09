/*
  DMAC descriptor configuration file.
	
	Copyright (C) Martin Lindupp 2018. All rights reserved. 
	
	Initial release -- V1.0.0 

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
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

