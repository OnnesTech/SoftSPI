/*
 * Copyright (c) 2014, Majenko Technologies & 2023, VitanovG
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification, 
 * are permitted provided that the following conditions are met:
 * 
 *  1. Redistributions of source code must retain the above copyright notice, 
 *     this list of conditions and the following disclaimer.
 * 
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 * 
 *  3. Neither the name of Majenko Technologies nor the names of its contributors may be used
 *     to endorse or promote products derived from this software without 
 *     specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE 
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *
 *  SoftSPI.h - Software SPI (bit-banging) library that enables the use of all
 *              digital pins as SoftSPI pins.
 *
 *  Supports: 8 bit simultaneous read-write operation in any SPI_MODE(0-4)
 *            with LSBFIRST and MSBFRIST bit orders. 
 *
 *	Intended use: with a Teensy 4.1 microcontroller 
 *                (Possible to use with Arduino using wait() function)
 *
 *  Updated by: VitanovG on Sept. 22. 2023
 */
#ifndef _SOFTSPI_H
#define _SOFTSPI_H

#include <SPI.h>

struct SoftSPISettings
{
	// Variables (public)
	uint32_t clock;
	uint8_t bitOrder;
	uint8_t dataMode;
	
	// Default constructor
	SoftSPISettings()
	{
		clock = 4000000; // 4Mhz
		bitOrder = MSBFIRST;
		dataMode = SPI_MODE0;
	}
	
	// Parameterized constructor
	/* bitord - MSBFIRST/LSBFIRST; datamd - SPI_MODE0/1/2/3 */
	SoftSPISettings(uint32_t clk, uint8_t bitord, uint8_t datamd)
	{
		clock = clk;
		bitOrder = bitord;
		dataMode = datamd;
	}
};

//: public SPIClass 

class SoftSPI
{
	public:
		SoftSPI(uint8_t mosi, uint8_t miso, uint8_t sclk, uint8_t sync): SoftSPI(mosi, miso, sclk, sync, _SSPI_default_settings) {} //delegation
        SoftSPI(uint8_t mosi, uint8_t miso, uint8_t sclk, uint8_t sync, SoftSPISettings settings);
        void beginTransaction();
		void beginTransaction(SoftSPISettings settings);
        void endTransaction();
        void setBitOrder(uint8_t);
        void setDataMode(uint8_t);
        void setFreq(uint32_t);
        uint8_t transfer(uint8_t);
		void send(uint8_t);
		
    private:
		uint8_t _mosi;
        uint8_t _miso;
        uint8_t _sclk;
		uint8_t _sync;
		uint8_t _ckp;
		uint8_t _cke;
        uint8_t _order;
		uint32_t _delay;
		SoftSPISettings _SSPI_default_settings;
		SoftSPISettings _SSPIsettings;
		
		//void wait(uint_fast8_t del);
};
#endif
