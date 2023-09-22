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
 
#include <SoftSPI.h>

// ---------------------- Normal Functions -----------------------
/***************************************************************************//**
* @brief Reorders a Byte (8 bit) from MSB -> LSB or reverse
*
* @param val - 8 bit variable 
*******************************************************************************/
uint8_t swap_order(uint8_t val)
{
	return( ((val & 0x01) << 7) |
            ((val & 0x02) << 5) |
            ((val & 0x04) << 3) |
            ((val & 0x08) << 1) |
            ((val & 0x10) >> 1) |
            ((val & 0x20) >> 3) |
            ((val & 0x40) >> 5) |
            ((val & 0x80) >> 7) );
}

// ----------------------- Class Functions ------------------------
/***************************************************************************//**
* @brief Constructor for SoftSPI class
*
* @param inputs should all be valid digital pins
*******************************************************************************/
SoftSPI::SoftSPI(uint8_t mosi, uint8_t miso, uint8_t sclk, uint8_t sync) 
{
    _mosi = mosi;	// Master Out Slave In
    _miso = miso;	// Master In Slave Out
    _sclk = sclk;	// Serial Clock signal
	_sync = sync;	// Chip enable
    _delay = 250;	// 10^9/250 =~ 4Mhz SPI freq.	
	_ckp = 0;		// Clock Polarity (Clock LOW-0 or HIGH-1 when idle?)
    _cke = 0;		// Clock Phase (Data read on Falling-0 or Rising-1 edge?)
    _order = MSBFIRST; // Data bit ordering
}


/***************************************************************************//**
* @brief Uses default settings for beginTransaction()			
*******************************************************************************/
void SoftSPI::beginTransaction()
{
	SoftSPISettings default_settings;
	this->beginTransaction(default_settings);
}

/***************************************************************************//**
* @brief Sets the correct pin configuration for SPI transfer and the
*        frequency, bitOrder, and dataMode.			
*******************************************************************************/
void SoftSPI::beginTransaction(SoftSPISettings settings) 
{
    pinMode(_mosi, OUTPUT);
    pinMode(_miso, INPUT);
    pinMode(_sclk, OUTPUT);
	pinMode(_sync, OUTPUT);
	
	this->setFreq(settings.clock);
	this->setBitOrder(setting.bitOrder);
	this->setDataMode(settings.dataMode);
}

/***************************************************************************//**
* @brief Resets SPI pins to INPUT state
*******************************************************************************/
void SoftSPI::endTransaction() 
{
    pinMode(_mosi, INPUT);
    pinMode(_miso, INPUT);
    pinMode(_sclk, INPUT);
	pinMode(_sync, INPUT);
}

/***************************************************************************//**
* @brief Set the SoftSPI transfer bit order (MSBFIRST or LSBFIRST)
*******************************************************************************/
void SoftSPI::setBitOrder(uint8_t order) 
{
    _order = order & 1; // Only check last bit, 
}

/***************************************************************************//**
* @brief Set SoftSPI frequency (actual freq. maybe slightly lower)
*******************************************************************************/
void SoftSPI::setFreq(uint32_t f) 
{
	_delay = 1000000000ul/f; // 1s = 10^9 ns
}

/***************************************************************************//**
* @brief Set SoftSPI dataMode (SPI_MODE0/1/2/3)
*******************************************************************************/
void SoftSPI::setDataMode(uint8_t mode) 
{
    switch (mode) 
	{
        case SPI_MODE0:
            _ckp = 0;
            _cke = 0;
            break;
        case SPI_MODE1:
            _ckp = 0;
            _cke = 1;
            break;
        case SPI_MODE2:
            _ckp = 1;
            _cke = 0;
            break;
        case SPI_MODE3:
            _ckp = 1;
            _cke = 1;
            break;
    }
    digitalWriteFast(_sclk, _ckp ? HIGH : LOW); // Set sclk to correct idle signal
}

/*
void SoftSPI::wait(uint_fast8_t del) 
{
    for (uint_fast8_t i = 0; i < del; i++) {
        asm volatile("nop");
    }
}
*/
/***************************************************************************//**
* @brief Transfer 8 bits on SoftSPI by bit-banging, and return 8 bits read
*        from MISO
*******************************************************************************/
uint8_t SoftSPI::transfer(uint8_t cmd) 
{
    if (_order == LSBFIRST) { cmd = swap_order(cmd); }
    uint32_t del  = _delay >> 1; // time delay
	uint8_t out  = 0;			 // output var
    /*
     * CPOL := 0, CPHA := 0 => INIT = 0, PRE = Z|0, MID = 1, POST =  0
     * CPOL := 1, CPHA := 0 => INIT = 1, PRE = Z|1, MID = 0, POST =  1
     * CPOL := 0, CPHA := 1 => INIT = 0, PRE =  1 , MID = 0, POST = Z|0
     * CPOL := 1, CPHA := 1 => INIT = 1, PRE =  0 , MID = 1, POST = Z|1
     */
    int sclk = (_ckp) ? HIGH : LOW; // Clock polarity
    for (uint8_t bit = 0x80; bit; bit>>=1)
    {
		// 0-Falling 1-Rising edge data capture
        if (_cke) { sclk ^= 1; digitalWriteFast(_sclk, sclk); } // bitwise XOR (flips sclk)
        digitalWriteFast(_mosi, ((cmd & bit) ? HIGH : LOW)); 	// WRITE - Not Active edge
        delayNanoseconds(del);
        sclk ^= 1; digitalWriteFast(_sclk, sclk);			 	// Data read edge
        out |= digitalReadFast(_miso); out <<= 1;			 	// READ - Active Edge
        delayNanoseconds(del);
        if (!_cke) { sclk ^= 1; digitalWriteFast(_sclk, sclk); }
    }
	if (_order == LSBFIRST) { out = swap_order(out); }
    return out;
}
