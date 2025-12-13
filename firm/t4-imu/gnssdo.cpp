/* -*- mode: C; -*- */
#define _GNSSDO_CPP_

//
//               struct  GPT  Pid  dac  pll  i2cbus dacaddr plladdr
// 24MHz           0      2    0    0    0    Wire    0x61   0x60
// OCXO            1      1    1    1         Wire1   0x61
// OCXO PLL        -      -              1    Wire1          0x60
//

#include        <Arduino.h>
#include        <Wire.h>
#include        <si5351.h>
#include        <strings.h>
#include        <math.h>
#include        <string.h>
#include        <InternalTemperature.h>

#include        "config.h"
#include        "system.h"

#include        "mcp4726.h"
#include        "eeprom.h"

#include        "gnssdo.h"

Si5351          si5351(SI5351_BUS_BASE_ADDR);
Si5351          si5351Ext(SI5351_BUS_BASE_ADDR, &Wire1);


enum gnssdoSeq {
  GNSSDO_SEQ_IDLE       = 0,
  GNSSDO_SEQ_PLL_INIT,
  GNSSDO_SEQ_PLL_WAIT,
  GNSSDO_SEQ_CHECK_FREQ_INIT,
  GNSSDO_SEQ_CHECK_FREQ_WAIT,
  GNSSDO_SEQ_SET_SOURCE_FREQ,
  GNSSDO_SEQ_WAIT_PLL,
  GNSSDO_SEQ_RUNNING,
};


struct _stGptReg {
  uint32_t         valGptIc1;
  uint32_t         valGptIc2;
  uint32_t         diffGptIc1;
  uint32_t         diffGptIc2;
  uint32_t         flagGptIc = 0;

  uint32_t              cntPps;
};


struct _stPid {
  int           unit;
  double         Kp, Ki, Kd;
  double         pid_P;
  double         pid_I;
  double         pid_D;
  double         diffPrev;
  double         diffStore;
  double         offset;
  double         thresholdLock;

  int8_t        diffRingBuf[CONFIG_GNSSDO_DIFF_HISTORY_LEN_MAX];
  int           diffRingEntry;
  int           diffRingDepth;
  int           diffRingLen;
  int           diffRingDepthChgUp;
  int           diffRingDepthChgDown;
  int           diffSum;

  int           dac;

  int           fLoopDis;
  int           debug;
#define GNSSDO_DEBUG_SHOW_FREQ_RESULT   (1<<0)
};


struct _stUnit {
  struct _stGptReg      gpt;
  struct _stPid         pid;            // 0: 24MHz/ocxo, 1: 24M
#define GNSSDO_STATISTICS_LIMIT         100
#define GNSSDO_STATISTICS_FS            32              // [-16 .. 15]  [-667 .. 625ns]@24MHz   ////, [-320ns .. 300ns]@50MHz
#define GNSSDO_STATISTICS_CENTER        ((GNSSDO_STATISTICS_FS)/2)
  int                   stat[GNSSDO_STATISTICS_FS];  // freq stat (ppsin - gnsspps) 16=0Hz, 0= <-15, 31= >15
};


struct _stGnssdo {
  uint32_t              t10msec;

  gnssdoSeq             seq;

  int                   seqTimer;
  int                   fExtClkPrev;
#define GNSSDO_EXTCLK_FLAG_INVALID      (-1)

  int                   refFreq;
  uint32_t              cntPpsPrev;

  struct _stUnit        sc[2];

  int                   tAlivePps;

  uint32_t              freqExtout;
  uint32_t              freqOcxoout0;
  uint32_t              freqOcxoout1;
};
static struct _stGnssdo gnssdo;


void
GnssdoInit(void)
{
  memset((void *)&gnssdo, 0, sizeof(gnssdo));

  gnssdo.seq = GNSSDO_SEQ_IDLE;
  gnssdo.fExtClkPrev = GNSSDO_EXTCLK_FLAG_INVALID;
  gnssdo.t10msec = SystemGetCounter();

  gnssdo.freqExtout   = EepromGet32(CONFIG_EEPROM_PLL_EXTOUT_FREQ_POS);
  gnssdo.freqOcxoout0 = EepromGet32(CONFIG_EEPROM_PLL_OCXOOUT0_FREQ_POS);
  gnssdo.freqOcxoout1 = EepromGet32(CONFIG_EEPROM_PLL_OCXOOUT1_FREQ_POS);

  GnssdoPidInit();

  GnssdoInitGpt(-1);
  GnssdoInitGpt(1);
  GnssdoInitGpt(2);

  GnssdoSetPllValue(0, CONFIG_GPT1_CLKIN_VALUE_24MHZ);

  return;
}


void
GnssdoLoop(void)
{
  int           fExtClk;

  if((gnssdo.t10msec - SystemGetCounter()) >= 10*SYSTEM_COUNTER_1M000S) {
    gnssdo.t10msec = SystemGetCounter();

    // check to connect the external clock input
    fExtClk = digitalRead(CONFIG_GPIO_EN_EXTOCXO);
    if(gnssdo.fExtClkPrev != fExtClk) {
      Serial.printf("# gnssdo: the clock source was changed [%s]\n", fExtClk?"int24MHz": "Extclk");
      digitalWrite(CONFIG_GPIO_SEL_24MHZ, fExtClk? 1: 0);

      gnssdo.seq = GNSSDO_SEQ_IDLE;
    }
    gnssdo.fExtClkPrev = fExtClk;


    // fix ref clock sequence
    if((gnssdo.tAlivePps - SystemGetCounter()) > 3000 * SYSTEM_COUNTER_1M000S) {
      gnssdo.sc[CONFIG_GNSSDO_IDX_24MHZ].gpt.cntPps = 0;
      gnssdo.seq = GNSSDO_SEQ_IDLE;
    }

    switch(gnssdo.seq) {
    case        GNSSDO_SEQ_IDLE:
      gnssdo.seq = GNSSDO_SEQ_PLL_INIT;
      break;
    case        GNSSDO_SEQ_PLL_INIT:
      GnssdoSetPllValue(0, CONFIG_GPT1_CLKIN_VALUE_24MHZ);
      gnssdo.seq = GNSSDO_SEQ_PLL_WAIT;
      break;
    case        GNSSDO_SEQ_PLL_WAIT:
      if(gnssdo.sc[CONFIG_GNSSDO_IDX_OCXO].gpt.cntPps >= 3) {
        gnssdo.seq = GNSSDO_SEQ_CHECK_FREQ_INIT;
      }
    case        GNSSDO_SEQ_CHECK_FREQ_INIT:
      gnssdo.seqTimer = SystemGetCounter();
      gnssdo.cntPpsPrev = GnssdoGetPpsCount(1);

      gnssdo.seq = GNSSDO_SEQ_CHECK_FREQ_WAIT;
      break;

    case        GNSSDO_SEQ_CHECK_FREQ_WAIT:
      if((gnssdo.seqTimer - SystemGetCounter()) > 4500 * SYSTEM_COUNTER_1M000S) {
        gnssdo.seq = GNSSDO_SEQ_SET_SOURCE_FREQ;
      }
      break;

    case        GNSSDO_SEQ_SET_SOURCE_FREQ:
      int               val;
      val  = GnssdoGetFrequency(CONFIG_GNSSDO_IDX_OCXO);
      val += 500;
      val /= 1000;
      val *= 1000;
      gnssdo.refFreq = val;
      GnssdoSetPllValue(0, gnssdo.refFreq);
      if(!digitalRead(CONFIG_GPIO_SEL_24MHZ)) {
        // set the ref clock, if the ocxo board is connected
        GnssdoSetPllValue(1, CONFIG_GPT1_CLKIN_VALUE_10MHZ);
      }
      Serial.printf("# gnssdo: refFreq: %d\n", gnssdo.refFreq);
      gnssdo.seqTimer = SystemGetCounter();

      gnssdo.seq = GNSSDO_SEQ_WAIT_PLL;
      break;

    case        GNSSDO_SEQ_WAIT_PLL:
      if((gnssdo.seqTimer - SystemGetCounter()) > 4000 * SYSTEM_COUNTER_1M000S) {
        GnssdoPidVcocxoInit();
        digitalWrite(CONFIG_GPIO_EN_EXTOUT, 1);
        gnssdo.seq = GNSSDO_SEQ_RUNNING;
      }
      break;
    case        GNSSDO_SEQ_RUNNING:
      GnssdoLockLed();

      break;
    }

  }

  return;
}


void
GnssdoInitGpt(int unit)
{
  if(unit == 1) {
    CCM_CCGR1 |= CCM_CCGR1_GPT1_SERIAL(CCM_CCGR_ON);    // clock enable for GPT1 module
    CCM_CCGR1 |= CCM_CCGR1_GPT1_BUS(CCM_CCGR_ON);       // clock enable for GPT1 module
    CCM_CMEOR |= CCM_CMEOR_MOD_EN_OV_GPT;

    GPT1_CR  =  0;
    GPT1_IR  =  0x3f;
    GPT1_CR |=  GPT_CR_CLKSRC(3);        // set clock source  1:ipg_clk, 3:extclk
    GPT1_CR |=  GPT_CR_SWR;              // sw reset
    GPT1_SR  =  0x3f;
    GPT1_CR |=  GPT_CR_ENMOD;           // reset the counter
    GPT1_CR &= ~GPT_CR_ENMOD;           // reset the counter

    GPT1_PR = GPT_PR_PRESCALER(0);      // set prescaler /1

    GPT1_CR |=  GPT_CR_IM2(1) | GPT_CR_IM1(1) | GPT_CR_CLKSRC(3) | GPT_CR_FRR | GPT_CR_DBGEN;     // IC2: rise, Src: peri clock, Freerun
    GPT1_CR |=  GPT_CR_EN;

    gnssdo.sc[CONFIG_GNSSDO_IDX_OCXO].gpt.valGptIc1 = GPT1_CNT;
    gnssdo.sc[CONFIG_GNSSDO_IDX_OCXO].gpt.valGptIc2 = GPT1_CNT;

    GPT1_IR |=  GPT_IR_IF1IE;           // enable CAPTURE1 interrupt (valid only ptp slave mode)
    GPT1_IR |=  GPT_IR_IF2IE;           // enable CAPTURE2 interrupt

    attachInterruptVector(IRQ_GPT1, GnssdoInterruptGpt1);
    NVIC_ENABLE_IRQ(IRQ_GPT1);

  } else if(unit == 2) {
    CCM_CCGR0 |= CCM_CCGR0_GPT2_SERIAL(CCM_CCGR_ON);    // clock enable for GPT2 module
    CCM_CCGR0 |= CCM_CCGR0_GPT2_BUS(CCM_CCGR_ON);       // clock enable for GPT2 module
    CCM_CMEOR |= CCM_CMEOR_MOD_EN_OV_GPT;

    GPT2_PR = GPT_PR_PRESCALER(0);      // set prescaler /1

    GPT2_CR  =  0;
    GPT2_IR  =  0x3f;
    GPT2_CR |=  GPT_CR_EN_24M;          // enable 24MHz clk
    GPT2_CR |=  GPT_CR_CLKSRC(5);       // set clock source  1:ipg_clk, 3:extclk, 3:ipg_clk_24M
    GPT2_CR |=  GPT_CR_SWR;             // sw reset
    GPT2_SR  =  0x3f;
    GPT2_CR |=  GPT_CR_ENMOD;           // reset the counter
    GPT2_CR &= ~GPT_CR_ENMOD;           // reset the counter

    GPT2_CR |=  GPT_CR_EN_24M | GPT_CR_IM2(1) | GPT_CR_IM1(1) | GPT_CR_CLKSRC(5) | GPT_CR_FRR;     // IC2: rise, Src: 1=peri clock/5=ipg_clk_24M, Freerun
    GPT2_CR |=  GPT_CR_EN;

    gnssdo.sc[CONFIG_GNSSDO_IDX_24MHZ].gpt.valGptIc1 = GPT2_CNT;
    gnssdo.sc[CONFIG_GNSSDO_IDX_24MHZ].gpt.valGptIc2 = GPT2_CNT;

    GPT2_IR |=  GPT_IR_IF1IE;           // enable CAPTURE1 interrupt (valid only ptp slave mode)
    GPT2_IR |=  GPT_IR_IF2IE;           // enable CAPTURE2 interrupt

    attachInterruptVector(IRQ_GPT2, GnssdoInterruptGpt2);
    NVIC_ENABLE_IRQ(IRQ_GPT2);
  }


  return;
}


//
// for 10MHz output
//
//                     +---+   +-- +               +---+  +-----+  +---+
// CLKO2_24MHz    -----|   |   |   |---(10MHz)-----|buf|--|balun|--|SMA|
// (GPIO_SD_B0_05)     |sel|---|pll|               +---+  +-----+  +---+
//                     |   |   |   |               +--------+
// EXTCLK ---(xxMHz)---|   |   |   |---(10/24MHz)--|CLK GPT1|
//                     +---+   +---+               |        |
//                                                 |        |
//                                                 |        |  +-------------+
// EXT_PPS-----------------------------------------|IC1     |--|ic1-ic2 diff |
//                                                 |        |  |  measurement|
//                                                 |        |  +-------------+
//                                                 |        |  (when the external vcxo is existed)
//                                                 |        |  +---+  +-------+  +--------+
// GNSS_PPS ---------------------------------------|IC2     |--|pid|--|ext_dac|--|ext_vcxo|
//                                                 +--------+  +---+  +-------+  +--------+
//
void
GnssdoInterruptGpt1(void)
{
  uint32_t      val, diff, psc;
  uint32_t      sr;
  struct _stGptReg *p;

  p = &gnssdo.sc[CONFIG_GNSSDO_IDX_OCXO].gpt;

  sr = GPT1_SR;
  GPT1_SR = sr;                 // clear interrupt

  // ext pps in
  if(sr & GPT_SR_IF1) {
    val = GPT1_ICR1;              // read the value
    diff = val - p->valGptIc1;
    p->valGptIc1 = val;

    if(/*diff <= CONFIG_GPT1_IC_VAL_MAX && diff >= CONFIG_GPT1_IC_VAL_MIN*/ 1) {
      psc = ((*(volatile uint16_t *)(0x401E4000 + 0x4c) >> 9) & 0xf) - 8;
      val = diff * 12 / 4 / (1<<psc) / CONFIG_CTRLOUT0_FREQ_DEFAULT;
      Serial.printf("gpt1 ic1 = %d\n", diff);
      p->diffGptIc1 = diff;

      p->flagGptIc |= 1;
    }
  }

  // pps in
  if(sr & GPT_SR_IF2) {
    val = GPT1_ICR2;              // read the value
    diff = val - p->valGptIc2;
    p->valGptIc2 = val;

    p->flagGptIc |= 2;

    if(/*(diff <= CONFIG_GPT1_IC_VAL_MAX && diff >= CONFIG_GPT1_IC_VAL_MIN)*/ 1) {
      if(!digitalRead(CONFIG_GPIO_SEL_24MHZ) && !gnssdo.sc[CONFIG_GNSSDO_IDX_OCXO].pid.fLoopDis) {
        GnssdoPidVcocxo(diff, gnssdo.refFreq);
      }

      psc = ((*(volatile uint16_t *)(0x401E4000 + 0x4c) >> 9) & 0xf) - 8;
      val = diff * 12 / 4 / (1<<psc) / CONFIG_CTRLOUT0_FREQ_DEFAULT;
      p->diffGptIc2 = diff;
      //Serial.printf("gpt1 ic2 = %d %d              %d %d\n", diff, 0,       psc, val);

      p->cntPps++;
    }
  }

  if(p->flagGptIc == 3) {
    p->flagGptIc = 0;
    val = p->valGptIc2 - p->valGptIc1;
    Serial.printf("gpt1: Slave PPS %d, GPS PPS %d, Delay %2d (%3dns)\n", p->diffGptIc2, p->diffGptIc1, val, val * 42);
    GnssdoStatistics(CONFIG_GNSSDO_IDX_OCXO, val);
  }

  return;
}
//
// for 10MHz output
//
//                                      +--------+
// vcxo 24MHz  ----------------(24MHz)--|clk GPT2|
//                                      |        |  +-------------+
// EXT_PPS------------------------------|IC1     |--|ic1-ic2 diff |
//                                      |        |  |  measurement|
//                                      |        |  +-------------+
//                                      |        |
//                                      |        |  +---+  +---+  +---+
// GNSS_PPS ----------------------------|IC2     |--|pid|--|dac|--|vcxo|
//                                      +--------+  +---+  +---+  +---+
//
//
void
GnssdoInterruptGpt2(void)
{
  uint32_t      val, diff, psc;
  uint32_t      sr;

  struct _stGptReg *p;

  p = &gnssdo.sc[CONFIG_GNSSDO_IDX_24MHZ].gpt;

  sr = GPT2_SR;
  GPT2_SR = sr;                 // clear interrupt

  if(sr & GPT_SR_IF1) {
    val = GPT2_ICR1;              // read the value
    diff = val - p->valGptIc1;
    p->valGptIc1 = val;

    //Serial.printf("gpt2ic1 %x %d\n", val, diff);

    if(diff <= CONFIG_GPT2_IC_VAL_MAX && diff >= CONFIG_GPT2_IC_VAL_MIN) {

      //psc = ((*(volatile uint16_t *)(0x401E4000 + 0x4c) >> 9) & 0xf) - 8;
      //val = diff * 12 / 4 / (1<<psc) / CONFIG_CTRLOUT0_FREQ_DEFAULT;
      //Serial.printf("gpt2 ic1 = %d\n", diff);
      p->diffGptIc1 = diff;

      p->flagGptIc |= 1;
    }
  }

  if(sr & GPT_SR_IF2) {
    val = GPT2_ICR2;              // read the value
    diff = val - p->valGptIc2;
    p->valGptIc2 = val;

    p->flagGptIc |= 2;

    if(diff <= CONFIG_GPT2_IC_VAL_MAX && diff >= CONFIG_GPT2_IC_VAL_MIN) {
      if(!gnssdo.sc[CONFIG_GNSSDO_IDX_24MHZ].pid.fLoopDis) {
        GnssdoPid24MHz(diff, CONFIG_GPT2_CLKIN_VALUE);
      }

      psc = ((*(volatile uint16_t *)(0x401E4000 + 0x4c) >> 9) & 0xf) - 8;
      val = diff * 12 / 4 / (1<<psc) / CONFIG_CTRLOUT0_FREQ_DEFAULT;
      p->diffGptIc2 = diff;
      //Serial.printf("gpt2 ic2 = %d %d              %d %d\n", diff, 0,       psc, val);

      p->cntPps++;
      gnssdo.tAlivePps = SystemGetCounter();
    }
  }

  if(p->flagGptIc == 3) {
    p->flagGptIc = 0;
    val = p->valGptIc2 - p->valGptIc1;
    Serial.printf("gpt2: Slave PPS %d, GPS PPS %d, Delay %2d (%3dns)\n", p->diffGptIc2, p->diffGptIc1, val, val * 20);
    GnssdoStatistics(CONFIG_GNSSDO_IDX_24MHZ, val);
  }

  return;
}


static uint32_t
GnssdoGetPpsCount(int unit)
{
  return gnssdo.sc[unit].gpt.cntPps;
}
static uint32_t
GnssdoGetFrequency(int unit)
{
  return gnssdo.sc[unit].gpt.diffGptIc2;
}




static void
GnssdoPidInit(void)
{
  GnssdoPid24MHzInit();
  GnssdoPidVcocxoInit();

  return;
}
static void
GnssdoPid24MHzInit(void)
{
  struct _stPid *p;

  p = &gnssdo.sc[CONFIG_GNSSDO_IDX_24MHZ].pid;

  memset(p, 0, sizeof(struct _stPid));
  p->unit = CONFIG_GNSSDO_IDX_24MHZ;

  p->Kp = CONfIG_GNSSDO_PID_24MHZ_KP;
  p->Ki = CONfIG_GNSSDO_PID_24MHZ_KI;
  p->Kd = CONfIG_GNSSDO_PID_24MHZ_KD;

  p->offset = CONfIG_GNSSDO_PID_24MHZ_OFFSET;
  p->dac    = p->offset;
  //p->diffRingDepth = CONFIG_GNSSDO_DIFF_HISTORY_LEN_MIN;
  p->diffRingDepth = 4;
  p->diffRingDepthChgUp   = 2;
  p->diffRingDepthChgDown = 4;
  p->thresholdLock = CONFIG_GNSSDO_LOCK_PPB_24MHZ;

  p->debug = 1;

  return;
}
static void
GnssdoPidVcocxoInit(void)
{
  struct _stPid *p;

  p = &gnssdo.sc[CONFIG_GNSSDO_IDX_OCXO].pid;

  memset(p, 0, sizeof(struct _stPid));
  p->unit = CONFIG_GNSSDO_IDX_OCXO;

  p->Kp = CONfIG_GNSSDO_PID_OCXO_KP;
  p->Ki = CONfIG_GNSSDO_PID_OCXO_KI;
  p->Kd = CONfIG_GNSSDO_PID_OCXO_KD;
  p->offset =  CONfIG_GNSSDO_PID_OCXO_OFFSET;
  p->dac    =  p->offset;
  p->diffRingDepth = CONFIG_GNSSDO_DIFF_HISTORY_LEN_MIN;
  p->diffRingDepthChgUp = 2;
  p->diffRingDepthChgDown = 4;
  p->thresholdLock = CONFIG_GNSSDO_LOCK_PPB_OCXO;

  //p->debug = 1;

  return;
}
static void
GnssdoPid24MHz(int current, int target)
{
  struct _stPid *p;
  int           dac;

  p = &gnssdo.sc[CONFIG_GNSSDO_IDX_24MHZ].pid;
  dac = GnssdoPid(p, current, target, 1);
  if(dac > 65535) dac = 65535;
  if(dac <     0) dac = 0;
  Mcp4726Set(0, dac);

  return;
}


static void
GnssdoPidVcocxo(int current, int target)
{
  struct _stPid *p;
  int           dac;

  p = &gnssdo.sc[CONFIG_GNSSDO_IDX_OCXO].pid;
  dac = GnssdoPid(p, current, target, 1);
  if(dac > 65535) dac = 65535;
  if(dac <     0) dac = 0;
  Mcp4726Set(1, dac);

  return;
}


static float
GnssdoPidGetFreqDifference(int unit)
{
  struct _stPid *p;

  p = &gnssdo.sc[unit].pid;

  return p->diffPrev;
}


static int
GnssdoPid(struct _stPid *p, int current, int target, int pol)
{
  int           diff, diff128;
  double        ds = 0.0;
  int           dac;
  int           update = 0;

  dac = p->dac;

  if(pol > 0) {
    diff = target - current;
  } else {
    diff = -target + current;
  }

  diff128 = diff;
  if(diff128 >  127) diff128 =  127;
  if(diff128 < -128) diff128 = -128;

  //p->diffSum -= p->diffRingBuf[p->diffRingEntry];
  p->diffSum += diff128;
  p->diffRingBuf[p->diffRingEntry] = diff128;

  p->diffRingEntry++;
  if(p->diffRingLen    < p->diffRingDepth) p->diffRingLen++;

  if(p->diffRingEntry >= p->diffRingDepth) {
    update = 1;
    ds = (double)p->diffSum/(double)p->diffRingLen;
    p->diffSum = 0;

    if(abs(p->diffSum) <= p->diffRingDepthChgUp && p->diffRingDepth < CONFIG_GNSSDO_DIFF_HISTORY_LEN_MAX) {
      bcopy(&p->diffRingBuf[0], &p->diffRingBuf[p->diffRingDepth], p->diffRingDepth);
      p->diffRingDepth <<= 1;
      p->diffRingLen   <<= 1;
    } else {
      p->diffRingEntry = 0;
    }

  } else {
    if(abs(p->diffSum) >= p->diffRingDepthChgDown && p->diffRingDepth > CONFIG_GNSSDO_DIFF_HISTORY_LEN_MIN) {
      int       posTail;

      update = 1;
      ds = (double)p->diffSum/(double)p->diffRingLen;
      p->diffSum = 0;

      posTail = p->diffRingDepth - p->diffRingEntry;
      p->diffRingDepth >>= 1;
      p->diffRingLen   >>= 1;
      if(p->diffRingEntry >= p->diffRingDepth) {
        bcopy(&p->diffRingBuf[p->diffRingEntry - p->diffRingDepth], &p->diffRingBuf[0], p->diffRingDepth);
      } else {
        bcopy(&p->diffRingBuf[0],       &p->diffRingBuf[p->diffRingDepth - p->diffRingEntry], p->diffRingEntry);                         // near Entry side
        bcopy(&p->diffRingBuf[posTail], &p->diffRingBuf[0],                                   p->diffRingDepth - p->diffRingEntry);      // older side
      }
      p->diffRingEntry = 0;

    } else {

    }

  }


  if(update) {
    p->pid_P  = ds;
    p->pid_I += ds; // * ((p->diffRingDepth >= 1024)? CONfIG_GNSSDO_PID_OCXO_KI_FINEFACTOR: 1.0);
    p->pid_D  = ds - p->diffPrev;
    p->diffPrev = ds;

    dac = (int)(p->Kp*p->pid_P + p->Ki*p->pid_I + p->Kd*p->pid_D + p->offset);
    p->dac = dac;

  } else {
#if 0
    if(abs(p->diffSum) >= 1 && p->diffRingDepth >= 256) {
      float val;
      //val = 1.5 * (float)p->diffSum / (1024.0 * CONFIG_GNSSDO_VCOCXO_FREQ_TICK);
      val = 1.5 * (float)p->diffSum * 0x10  +  (float) diff128 * 0x10;
      dac += (int)val;
    }
#endif
  }

  if(gnssdo.seq != GNSSDO_SEQ_RUNNING) dac = p->offset;

  if(p->debug & GNSSDO_DEBUG_SHOW_FREQ_RESULT) {
    Serial.printf("pid%d %d %4d/%4d freq:%d/diff:%2d/acc:%3d/a_l:%8.5f",
                  p->unit, gnssdo.seq, p->diffRingEntry, p->diffRingLen,
                  target, diff128, p->diffSum,
                  (float)p->diffSum/(float)p->diffRingLen);

    if(p->unit == 0) Serial.printf(", temp:%5.2f", InternalTemperature.readTemperatureC());
    if(p->unit == 1) Serial.printf(", temp:%5.2f", GnssdoTmp112GetTemperature());
    Serial.printf(", dac:%x", dac);
    Serial.printf("\n");
  }

  return dac;
}


void
GnssdoSetPllValue(int unit, int refFreq)
{
  if(unit == 0) {
    si5351.init(SI5351_CRYSTAL_LOAD_8PF, refFreq, 0);           // CLKIN input is set 10.000MHz(OCXO) / 24.000MHz(MCU)
    si5351.set_pll(SI5351_PLL_FIXED, SI5351_PLLA);              // PLLA fixed mode
    si5351.set_freq(gnssdo.freqExtout * 100ULL, SI5351_CLK0);   // CLKEXTOUT
    si5351.set_pll(SI5351_PLL_FIXED, SI5351_PLLB);              // PLLB fixed mode
    si5351.set_freq((uint64_t)refFreq * 100ULL, SI5351_CLK2);   // CLK2: 10MHz/24MHz (unit 0.01Hz) connect to GPT1CLKIN

  } else if(unit == 1) {
    si5351Ext.init(SI5351_CRYSTAL_LOAD_8PF, refFreq, 0);        // CLKIN input is set 10.000MHz(OCXO)   Rpi-VCOCXO board
    si5351Ext.set_pll(SI5351_PLL_FIXED, SI5351_PLLA);           // PLLA fixed mode
    si5351Ext.set_freq(gnssdo.freqOcxoout0 * 100ULL, SI5351_CLK0);      // CLKOUT0
    si5351Ext.set_pll(SI5351_PLL_FIXED, SI5351_PLLB);           // PLLB fixed mode
    si5351Ext.set_freq(gnssdo.freqOcxoout1 * 100ULL, SI5351_CLK2);      // CLKOUT1
  }

  return;
}


static void
GnssdoStatistics(int unit, int value)
{
  int           idx;

  if(abs(value) <= GNSSDO_STATISTICS_LIMIT) {
    idx = value + GNSSDO_STATISTICS_CENTER;
    if(idx < 0) idx = 0;
    if(idx > (GNSSDO_STATISTICS_FS-1)) idx = GNSSDO_STATISTICS_FS - 1;
    gnssdo.sc[unit].stat[idx]++;
  }

  return;
}


void
GnssdoCommand(int ac, char *av[])
{
  int           num = 0;
  int           val = 0;

  if(ac >= 3) {
    num = atoi(av[2]);
  }

  if(!strcmp(av[1], "piddis")) {
    gnssdo.sc[num].pid.fLoopDis = atoi(av[3]);

  } else if(!strcmp(av[1], "diagmeas")) {
    digitalWrite(CONFIG_GPIO_SEL_24MHZ, atoi(av[2])? 0: 1);
    GnssdoSetPllValue(0, atoi(av[3]));             // set freq
    digitalWrite(CONFIG_GPIO_EN_EXTOUT, 1);     // output enable

  } else if(!strcmp(av[1], "showstat")) {
    float       total, ave, sigma;
    int         cnt;
    int         diff;
    float       interval;

    interval = 1000000000 / (num? CONFIG_GPT2_CLKIN_VALUE: CONFIG_GPT1_CLKIN_VALUE_24MHZ);

    // show statistics
    for(int i = 0; i < GNSSDO_STATISTICS_FS; i++) {
      diff = i - GNSSDO_STATISTICS_CENTER;
      Serial.printf("%3d (%4dns): %5d\n", diff, (int)(diff * interval), gnssdo.sc[num].stat[i]);
    }
    // calc avarage
    total = 0.0;
    cnt = 0;
    for(int i = 1; i < GNSSDO_STATISTICS_FS-1; i++) {
      val = i - GNSSDO_STATISTICS_CENTER;
      total += (float)(val * gnssdo.sc[num].stat[i]);
      cnt += gnssdo.sc[num].stat[i];
    }
    ave = total / cnt;

    // calc stadard deviation
    total = 0.0;
    for(int i = 1; i < GNSSDO_STATISTICS_FS-1; i++) {
      val = i - GNSSDO_STATISTICS_CENTER;
      total += (powf((float)(val), 2.0) - powf(ave, 2.0)) * gnssdo.sc[num].stat[i];
    }
    sigma = sqrt(total / cnt);

    Serial.printf("total time: %5d(sec)\n", cnt);
    Serial.printf("ave: %.2f (%.1fns), sigma:%.2f (%.1fns)\n\n", ave, ave*interval, sigma, sigma*interval);

  } else if(!strcmp(av[1], "clearstat")) {
    memset(gnssdo.sc[num].stat,  0, sizeof(gnssdo.sc[num].stat));

  } else if(!strcmp(av[1], "setfreq")) {
    if(ac >= 4) {
      val = atoi(av[3]);
      if(num == 0) {
        EepromSet32(CONFIG_EEPROM_PLL_EXTOUT_FREQ_POS, val);
        EepromBurn();
        si5351.set_freq(val * 100ULL, SI5351_CLK0);
      } else if(num == 10) {
        EepromSet32(CONFIG_EEPROM_PLL_OCXOOUT0_FREQ_POS, val);
        EepromBurn();
        si5351Ext.set_freq(val * 100ULL, SI5351_CLK0);
      } else if(num == 11) {
        EepromSet32(CONFIG_EEPROM_PLL_OCXOOUT1_FREQ_POS, val);
        EepromBurn();
        si5351Ext.set_freq(val * 100ULL, SI5351_CLK2);
       }
    }

  } else if(!strcmp(av[1], "debug")) {
    if(ac >= 4) {
      gnssdo.sc[num].pid.debug = strtoul(av[3], NULL, 16);
    }
  }

  return;
}


// reutrn value: temperature (unit 0.0625/16)
static float
GnssdoTmp112GetTemperature(void)
{
  uint8_t       tx[8];
  uint8_t       rx[8];

  short         temp;
  float         tempf;

  memset(rx, 0, sizeof(rx));

  tx[0] = 0;
  tx[1] = 0;
  SystemI2cTransfer(1, CONFIG_I2C_ADDR_TMP112, tx, 1, rx, 2, NULL);

  temp  = ((rx[0] << 8) | rx[1]);
  tempf = (float)temp / 256.0;

  //Serial.printf("%d.%03d  %6.3f\n", rx[0], (rx[1]>>4)*62, tempf);

  return tempf;
}


static void
GnssdoLockLed(void)
{
  float         freq;
  int           sel;
  float         err;

  if(!digitalRead(CONFIG_GPIO_SEL_24MHZ)) {
    sel = CONFIG_GNSSDO_IDX_OCXO;
    freq = gnssdo.refFreq;
  } else {
    sel = CONFIG_GNSSDO_IDX_24MHZ;
    freq = CONFIG_GPT1_CLKIN_VALUE_24MHZ;
  }

  err = fabsf(GnssdoPidGetFreqDifference(sel) / freq);
  if(err <= gnssdo.sc[sel].pid.thresholdLock*0.8) {
    SystemSetLed0(1);
  } else if(err > gnssdo.sc[sel].pid.thresholdLock) {
    SystemSetLed0(0);
  }

  return;
}
