/* AudioTiming Library for Teensy 3.X
/* Code is placed under the MIT license
 * Copyright (c) 2018 Frank Bösing
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "Arduino.h"
#if !defined(KINETISK)
#error This Library is not compatible to your CPU
#endif

#ifndef audio_timing_h_
#define audio_timing_h_


#include <Audio.h>
#include <DMAChannel.h>


class AudioTiming
{
	public:
	  void init(void);		
		void usePin(uint8_t pin);
		//void enableI2S_FSPin(uint8_t pin); // enable I2s Framesync Pin
		void begin(void);
		
	private:
	  void configurePDB(void);
		
	  DMAChannel pdbdma;
	  int8_t DMACh_pwm  = -1;
	  int8_t initialized = 0;
	  uint8_t pinnum = 255;	 
};


#endif