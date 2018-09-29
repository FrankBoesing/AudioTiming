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

#include "AudioTiming.h"


#define DEBUG
#define PDB_CONFIG (PDB_SC_TRGSEL(15) | PDB_SC_PDBEN | PDB_SC_PDBIE | PDB_SC_DMAEN)
static volatile const uint32_t DMAData[] = {0, PDB_CONFIG | PDB_SC_LDOK, PDB_CONFIG | PDB_SC_SWTRIG};

typedef struct __attribute__((packed, aligned(4))) {
  volatile const void * volatile SADDR;
  int16_t SOFF;
  union {
    uint16_t ATTR;
    struct {
      uint8_t ATTR_DST;
      uint8_t ATTR_SRC;
    };
  };
  union {
    uint32_t NBYTES;
    uint32_t NBYTES_MLNO;
    uint32_t NBYTES_MLOFFNO;
    uint32_t NBYTES_MLOFFYES;
  };
  int32_t SLAST;
  volatile void * volatile DADDR;
  int16_t DOFF;
  union {
    volatile uint16_t CITER;
    volatile uint16_t CITER_ELINKYES;
    volatile uint16_t CITER_ELINKNO;
  };
  int32_t DLASTSGA;
  volatile uint16_t CSR;
  union {
    volatile uint16_t BITER;
    volatile uint16_t BITER_ELINKYES;
    volatile uint16_t BITER_ELINKNO;
  };
} TCD_t;


static int searchDMAChannelSADDR(volatile void* value) {
  for (int ch = 0; ch < DMA_NUM_CHANNELS; ch++) {
    TCD_t *TCD = (TCD_t *)(0x40009000 + ch * 32);
    if (TCD->SADDR == value) return ch;
  }
  return -1;
}

static int searchDMAChannelDADDR(volatile void* value) {
  for (int ch = 0; ch < DMA_NUM_CHANNELS; ch++) {
    TCD_t *TCD = (TCD_t *)(0x40009000 + ch * 32);
    if (TCD->DADDR == value) return ch;
  }
  return -1;
}

void AudioTiming::init(void) {
  //is DMA enabled?
  if ( ((SIM_SCGC7 & SIM_SCGC7_DMA) != SIM_SCGC7_DMA) ||
       ((SIM_SCGC6 & SIM_SCGC6_DMAMUX) != SIM_SCGC6_DMAMUX)) return;

  //detect output_pwm
  if ((CORE_PIN3_CONFIG == (PORT_PCR_MUX(3) | PORT_PCR_DSE | PORT_PCR_SRE)) &&
      (CORE_PIN4_CONFIG = (PORT_PCR_MUX(3) | PORT_PCR_DSE | PORT_PCR_SRE)) &&
      (FTM1_C0SC == 0x69) && (FTM1_C1SC == 0x28) ) {
    int8_t x = searchDMAChannelDADDR(&FTM1_C0V);
    TCD_t *TCD = (TCD_t *)(0x40009000 + x * 32);
    if (TCD->SOFF == 4 && TCD->DOFF == 8) DMACh_pwm = x;
  }

  initialized = 1;

#if defined(DEBUG)
  if (DMACh_pwm >= 0) {
    Serial.print("DMA Channel PWM: ");
    Serial.println(DMACh_pwm);
  }
  Serial.println();
#endif
}

void AudioTiming::usePin(const uint8_t pin) {

  if (pin >= CORE_NUM_DIGITAL) return;

  volatile uint32_t *config;
  config = portConfigRegister(pin);
  //Configure PIN: Enable DMA request on rising edge (0x01) (falling: 0x02, both edges(0x03)).
  *config = (*config & (~PORT_PCR_IRQC_MASK)) | PORT_PCR_IRQC(0x01);
  pinnum = pin;
}

void AudioTiming::configurePDB(void) {

  if (!((SIM_SCGC6 & SIM_SCGC6_PDB) &&
        ((PDB0_SC & PDB_CONFIG) == PDB_CONFIG) &&
        (PDB0_IDLY == 1) &&
        (PDB0_CH0C1 == 0x0101) )) return;

  //Stop PDB
  PDB0_SC = 0;
  PDB0_MOD = 0;

  //setup DMA
  pdbdma.TCD->SADDR = &DMAData;
  pdbdma.TCD->SOFF = sizeof(uint32_t);
  pdbdma.TCD->ATTR = DMA_TCD_ATTR_SSIZE(2) | DMA_TCD_ATTR_DSIZE(2); //2 = 32 Bit
  pdbdma.TCD->NBYTES_MLNO = sizeof(DMAData);
  pdbdma.TCD->SLAST = -sizeof(DMAData);
  pdbdma.TCD->DADDR = &PDB0_SC;
  pdbdma.TCD->DOFF = 0;
  pdbdma.TCD->CITER_ELINKNO = 1;
  pdbdma.TCD->DLASTSGA = 0;
  pdbdma.TCD->BITER_ELINKNO = 1;

  //Trigger DMA with PIN:

  uint8_t mux = 0;
  uint32_t config = (uint32_t) portConfigRegister(pinnum) & 0xfffffff0;
  if (config == (uint32_t) &PORTA_PCR0) mux = DMAMUX_SOURCE_PORTA;
  else if (config == (uint32_t) &PORTB_PCR0) mux = DMAMUX_SOURCE_PORTB;
  else if (config == (uint32_t) &PORTC_PCR0) mux = DMAMUX_SOURCE_PORTC;
  else if (config == (uint32_t) &PORTD_PCR0) mux = DMAMUX_SOURCE_PORTD;
  else if (config == (uint32_t) &PORTE_PCR0) mux = DMAMUX_SOURCE_PORTE;

  pdbdma.triggerAtHardwareEvent(mux);
  //Restart, PDB now triggered by DMA
  pdbdma.enable();
}

void AudioTiming::begin(void)
{
  configurePDB();
}

