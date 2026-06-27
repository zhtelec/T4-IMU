#define _SYSTEM_CPP_

#include        <Arduino.h>
#include        <SPI.h>
#include        <Wire.h>

#include        <imxrt.h>

#include        "config.h"
#include        "mcp4726.h"

#include        "system.h"


struct _stSystemPid {
  int      pid_P;
  int      pid_I;
  int      pid_D;
  int      diffPrev;
  int      diffStore;
  int      cnt;
  int      interval;
};
struct _stSystemPid     systemPid50MHz;
struct _stSystemPid     systemPidVcocxo;

static int boardId = 0;
// if T4-PTPGM  0: v1.01,0: v1.00
static uint32_t boardIdSub = 0;

void
SystemPinSettings(void)
{
  // board id  GPIO2_IO[6:4]   GPIO2_DR
  IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_04    = 0xf000; // pull up 47k
  IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_05    = 0xf000; // pull up 47k
  IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_06    = 0xf000; // pull up 47k
  IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_09    = 0xf000; // pull up 47k
  IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_04    = 0x15;   // BOARD[0]
  IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_04    = 0x15;   // BOARD[1]
  IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_04    = 0x15;   // BOARD[2]
  IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_09    = 0x15;   // BOARD SUB[0]

  boardId = (GPIO2_DR >> 4) & 0x7;
  boardIdSub |= (GPIO2_DR >> 9) & 1;

  // the GPT clock is ipg clock
  CCM_CSCMR1 |=  CCM_CSCMR1_PERCLK_PODF(2);     // 100MHz / 2 = 50MHz
  CCM_CSCMR1 &= ~CCM_CSCMR1_PERCLK_CLK_SEL;     // sel ipg_clk
  // SS off
  //CCM_ANALOG_PLL_SYS_SS &= ~(1<<15);            // disable Spread spectrum

  // port settings
  pinMode(CONFIG_GPIO_IMU_DRDY10, INPUT_PULLDOWN);
  pinMode(CONFIG_GPIO_IMU_DRDY11, INPUT_PULLDOWN);

  pinMode(CONFIG_GPIO_IMU_CS0X, OUTPUT);
  pinMode(CONFIG_GPIO_IMU_CS1X, OUTPUT);
  //  pinMode(CONFIG_GPIO_IMU_CS2X_FLASH, OUTPUT);

  IOMUXC_SW_PAD_CTL_PAD_GPIO_B1_02    = 0x1;    // GPIO36  SPI1CS0
  IOMUXC_SW_PAD_CTL_PAD_GPIO_B1_03    = 0x1;    // GPIO37  SPI1CS1
  IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_12 = 0x1;    // GPIO39  SPI1CS2

  //digitalWrite(CONFIG_GPIO_IMU_CS2X_FLASH, 1);

  // switch and led
  pinMode(CONFIG_GPIO_SW0,  INPUT_PULLUP);
  pinMode(CONFIG_GPIO_SW1,  INPUT_PULLUP);

  pinMode(CONFIG_GPIO_IMU_POWER,  OUTPUT);
  digitalWrite(CONFIG_GPIO_IMU_POWER, 0);

  if(boardId == CONFIG_BOARDID_T4_IMU) {
    digitalWrite(CONFIG_GPIO_IMU_CS0X, 1);
    digitalWrite(CONFIG_GPIO_IMU_CS1X, 1);
    pinMode(CONFIG_GPIO_I2C_SDA, INPUT_PULLUP);
    pinMode(CONFIG_GPIO_I2C_SCL, INPUT_PULLUP);
    pinMode(CONFIG_GPIO_I2C1_SDA, INPUT_PULLUP);
    pinMode(CONFIG_GPIO_I2C1_SCL, INPUT_PULLUP);

    pinMode(CONFIG_GPIO_IMU_RESETX, OUTPUT);
    digitalWrite(CONFIG_GPIO_IMU_RESETX, 0);

    pinMode(CONFIG_GPIO_LED0_T4_IMU, OUTPUT);
    pinMode(CONFIG_GPIO_LED, OUTPUT);

    pinMode(CONFIG_GPIO_LED0_T4_IMU, OUTPUT);

  } else if(boardId == CONFIG_BOARDID_T4_PTPGM) {
    // SW
    digitalWrite(CONFIG_GPIO_IMU_CS0X, 1);
    digitalWrite(CONFIG_GPIO_IMU_CS1X, 1);
    // LED
    pinMode(CONFIG_GPIO_LED0_T4_IMU, OUTPUT);
    pinMode(CONFIG_GPIO_LED, OUTPUT);

    // ptp
    pinMode(CONFIG_GPIO_PTP_PPSOUT, OUTPUT);
    digitalWrite(CONFIG_GPIO_PTP_PPSOUT, 0);
    pinMode(CONFIG_GPIO_PTP_PPSMASK, OUTPUT);
    digitalWrite(CONFIG_GPIO_PTP_PPSMASK, 1);
    pinMode(CONFIG_GPIO_LED0_T4_PTPGM, OUTPUT);
    digitalWrite(CONFIG_GPIO_LED0_T4_PTPGM, 0);

    // PPS GPT1 IC2
    // GPT1 Clock
    IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B0_13 = 0x11;     // ALT1 (GPT1_CLK)
    IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B0_13 = 0x10000;  // hysteresis, pulldown
    IOMUXC_GPT1_IPP_IND_CLKIN_SELECT_INPUT = 0;     // GPT1CLK is  selected to GPIO_AD_B0_13_ALT1
    // GPT1 Input Capture1
    pinMode(48, INPUT_PULLDOWN);
    IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_24 = 0x14;       // ALT4 (GPT1_CAPTURE1)
    IOMUXC_SW_PAD_CTL_PAD_GPIO_EMC_24 = 0x10000;    // hysteresis, pulldown
    IOMUXC_GPT1_IPP_IND_CAPIN1_SELECT_INPUT = 0;    // GPT1IC1 is  selected to GPIO_EMC_24
    // GPT1 Input Capture2
    IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_23 = 0x14;       // ALT4 (GPT1_CAPTURE2)
    IOMUXC_SW_PAD_CTL_PAD_GPIO_EMC_23 = 0x10000;    // hysteresis, pulldown
    IOMUXC_GPT1_IPP_IND_CAPIN2_SELECT_INPUT = 0;    // GPT1IC2 is  selected to GPIO_EMC_23_ALT4

    // GPT2 Input Capture1
    //pinMode(CONFIG_GPIO_GPT2_IC1, INPUT_PULLDOWN);
    IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_41 = 0x11;     // ALT1 (GPT2_CAPTURE1)
    IOMUXC_SW_PAD_CTL_PAD_GPIO_EMC_41 = 0x10000;  // hysteresis, pulldown
    IOMUXC_GPT2_IPP_IND_CAPIN1_SELECT_INPUT = 0;  // GPT2IC1 is  selected to GPIO_EMC_41

    // GPT2 Input Capture2
    pinMode(CONFIG_GPIO_GPT2_IC2, INPUT_PULLDOWN);
    pinMode(40, INPUT_PULLDOWN);
    IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_04 = 0x18;     // ALT8 (GPT2_CAPTURE2)
    IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_04 = 0x10000;  // hysteresis, pulldown
    IOMUXC_GPT2_IPP_IND_CAPIN2_SELECT_INPUT = 1;    // GPT2IC2 is  selected to GPIO_AD_B1_04_ALT8

    // CLKO2 is out osc_clk (24MHz)
    IOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B0_05 = 0x6;       // ALT6 (CCM_CLKO2)
    CCM_CCOSR = 0x010e0000;

    pinMode(CONFIG_GPIO_EN_EXTOUT, OUTPUT);
    digitalWrite(CONFIG_GPIO_EN_EXTOUT, 0);
    //digitalWrite(CONFIG_GPIO_EN_EXTOUT, 1);

    pinMode(CONFIG_GPIO_EN_EXTOCXO, INPUT_PULLUP);
    pinMode(CONFIG_GPIO_SEL_24MHZ, OUTPUT);

  } else if(boardId == CONFIG_BOARDID_T4_SENSECAP) {
    pinMode(CONFIG_GPIO_IMU_CS2X,  OUTPUT);
    pinMode(CONFIG_GPIO_IMU_CS13X, OUTPUT);
    pinMode(CONFIG_GPIO_IMU_CS00X, OUTPUT);
    pinMode(CONFIG_GPIO_IMU_CS01X, OUTPUT);
    pinMode(CONFIG_GPIO_IMU_CS02X, OUTPUT);
    pinMode(CONFIG_GPIO_IMU_CS03X, OUTPUT);
    digitalWrite(CONFIG_GPIO_IMU_CS2X,  1);
    digitalWrite(CONFIG_GPIO_IMU_CS13X, 1);
    digitalWrite(CONFIG_GPIO_IMU_CS00X, 1);
    digitalWrite(CONFIG_GPIO_IMU_CS01X, 1);
    digitalWrite(CONFIG_GPIO_IMU_CS02X, 1);
    digitalWrite(CONFIG_GPIO_IMU_CS03X, 1);

    pinMode(CONFIG_GPIO_LED0_T4_IMU, OUTPUT);

    pinMode(CONFIG_GPIO_I2C_SDA,  INPUT_PULLUP);
    pinMode(CONFIG_GPIO_I2C_SCL,  INPUT_PULLUP);
    pinMode(CONFIG_GPIO_I2C1_SDA, INPUT_PULLUP);
    pinMode(CONFIG_GPIO_I2C1_SCL, INPUT_PULLUP);

    pinMode(CONFIG_GPIO_IMU_DRDY12, INPUT);
    pinMode(CONFIG_GPIO_IMU_DRDY13, INPUT);
    pinMode(CONFIG_GPIO_IMU_DRDY00, INPUT);
    pinMode(CONFIG_GPIO_IMU_DRDY01, INPUT);
    pinMode(CONFIG_GPIO_IMU_DRDY02, INPUT);
    pinMode(CONFIG_GPIO_IMU_DRDY03, INPUT);

    pinMode(CONFIG_B2_GPIO_LED_L, OUTPUT);

  }

  return;
}


int
SystemGetBoardId(void)
{
  return boardId;
}
uint32_t
SystemGetBoardIdSub(void)
{
  return boardIdSub;
}


void
SystemWaitCounter(uint32_t tout)
{
  uint32_t      t;

  t = SystemGetCounter();
  while((t - SystemGetCounter()) < tout);

  return;
}
void
SystemWaitUsCounter(uint32_t tout)
{
  uint32_t      t;

  t = SystemGetUsCounter();
  while((t - SystemGetUsCounter()) < tout);

  return;
}



SPISettings topSpi1Config(CONFIG_SPI_SPEED_DEFAULT, MSBFIRST, SPI_MODE3);
uint32_t        systemSpiRecoverTime[2];

int
SystemSpiInit(int unit)
{
  int           result = -1;
  int           port;

  if(unit < 0) {
    result = 0;
  } else if(unit == 0) {
    for(int i = 0; i < 4; i++) {
      port = SystemSpiGetCsxGpioNum(unit, i);
      if(port >= 0) {
        pinMode(port, OUTPUT);
        digitalWrite(port, 1);
      }
    }
    // spi setting
    SPI.setSCK (CONFIG_GPIO_IMU_SCK0);
    SPI.setMOSI(CONFIG_GPIO_IMU_MOSI0);
    SPI.setMISO(CONFIG_GPIO_IMU_MISO0);
    SPI.begin();

    SPI.beginTransaction(SPISettings(CONFIG_SPI_SPEED_DEFAULT, MSBFIRST, SPI_MODE3));
    SPI.transfer(0);           // dummy send
    SPI.endTransaction();

    result = 0;

  } else if(unit == 1) {
    for(int i = 0; i < 2; i++) {
      port = SystemSpiGetCsxGpioNum(unit, i);
      if(port >= 0) {
        pinMode(port, OUTPUT);
        digitalWrite(port, 1);
      }
    }
    if(SystemGetBoardId() == CONFIG_BOARDID_T4_SENSECAP) {
      for(int i = 2; i < 3; i++) {
        port = SystemSpiGetCsxGpioNum(unit, i);
        if(port >= 0) {
          pinMode(port, OUTPUT);
          digitalWrite(port, 1);
        }
      }
    }

    // spi setting
    SPI1.setSCK (CONFIG_GPIO_IMU_SCK);
    SPI1.setMOSI(CONFIG_GPIO_IMU_MOSI);
    SPI1.setMISO(CONFIG_GPIO_IMU_MISO);
    SPI1.begin();

    SPI1.beginTransaction(SPISettings(CONFIG_SPI_SPEED_DEFAULT, MSBFIRST, SPI_MODE3));
    SPI1.transfer(0);           // dummy send
    SPI1.endTransaction();


    result = 0;
  }

  return result;
}


#if 0
static int     systemSpiSpeed = CONFIG_SPI_SPEED;
void
SystemSpiSetSpeed(int unit, int speed)
{
  systemSpiSpeed = speed;
  return;
}
#endif


int
SystemSpiTransfer(int unit, int cs, const uint8_t *pTx, int lenTx, uint8_t *pRx, int lenRx, struct _stSystemSpiParam *pParam)
{
  int           result = -1;
  int           speed = CONFIG_SPI_SPEED_DEFAULT;
  uint32_t      t;

  if(SystemSpiGetCsxGpioNum(unit, cs) < 0) goto fail;

  t = systemSpiRecoverTime[unit] - SystemGetUsCounter();
  if(t < CONFIG_SPI_RECOVER_TIME) {
    SystemWaitUsCounter(CONFIG_SPI_RECOVER_TIME-t);
  }
  systemSpiRecoverTime[unit] = SystemGetUsCounter();

  SystemSpiSetCsx(unit, cs, 0);
  //if(unit == 1  &&  (cs == 2 || cs == 3)) SystemWaitUsCounter(100);      // adhoc: unit1,CS2|3 needs dly, GPIO15 is changed too slow

  if(pParam && pParam->tWaitCs) SystemWaitUsCounter(pParam->tWaitCs);

  if(pParam) speed = pParam->speed;

  if(unit == 1) {

    SPI1.beginTransaction(SPISettings(speed, MSBFIRST, SPI_MODE3));
    if(lenTx > 0) SPI1.transfer((void *)pTx, lenTx);
    if(pParam && pParam->tWait && lenRx) SystemWaitUsCounter(pParam->tWait);
    SPI1.transfer(pRx, lenRx);
    SPI1.endTransaction();

    result = lenRx;

  } else if(unit == 0) {

    SPI.beginTransaction(SPISettings(speed, MSBFIRST, SPI_MODE3));
    if(lenTx > 0) SPI.transfer((void *)pTx, lenTx);
    if(pParam && pParam->tWait && lenRx) SystemWaitUsCounter(pParam->tWait);
    SPI.transfer(pRx, lenRx);
    SPI.endTransaction();

    result = lenRx;

  }

  if(pParam && pParam->tWaitCs) SystemWaitUsCounter(pParam->tWaitCs);
  SystemSpiSetCsx(unit, cs, 1);

fail:
  return result;
}
int
SystemSpiGetCsxGpioNum(int unit, int numCs)
{
  int   port = -1;

  if(unit == 1) {
    switch(numCs) {
    case        0: port = CONFIG_GPIO_IMU_CS0X; break;
    case        1: port = CONFIG_GPIO_IMU_CS1X; break;
    case        2: port = CONFIG_GPIO_IMU_CS2X; break;
    }
    if(SystemGetBoardId() == CONFIG_BOARDID_T4_SENSECAP) {
      if(numCs == 3)  port = CONFIG_GPIO_IMU_CS13X;
    }
  } else if(unit == 0) {
    if(SystemGetBoardId() == CONFIG_BOARDID_T4_SENSECAP) {
      switch(numCs) {
      case        0: port = CONFIG_GPIO_IMU_CS00X; break;
      case        1: port = CONFIG_GPIO_IMU_CS01X; break;
      case        2: port = CONFIG_GPIO_IMU_CS02X; break;
      case        3: port = CONFIG_GPIO_IMU_CS03X; break;
      }
    }
  }

  return port;
}
int
SystemSpiSetCsx(int unit, int numCs, int val)
{
  int   port = -1;

  port = SystemSpiGetCsxGpioNum(unit, numCs);
  if(port >= 0) digitalWrite(port, val);

  return port;
}

int
SystemI2cInit(int unit)
{
  if(unit < 0) {
  } else if(unit == 0) {
    Wire.begin();
  } else if(unit == 1) {
    Wire1.begin();
  } else {

  }

  return 0;
}


int
SystemI2cTransfer(int unit, int addr, const uint8_t *pTx, int lenTx, uint8_t *pRx, int lenRx, struct _stSystemI2cParam *pParam)
{
  int           result = -1;
  int           re;
  TwoWire       *pWire = NULL;
  uint8_t       c;

  if(unit == 0) {
    pWire = &Wire;
  } else if(unit == 1) {
    pWire = &Wire1;
  }

  if(pWire) {
    pWire->beginTransmission(addr);
    pWire->write(pTx, lenTx);

    if(lenRx == 0) {
      re = pWire->endTransmission(true);
      if (re != 0) goto fail;

    } else {
      re = pWire->endTransmission(false);
      if (re != 0) goto fail;

      pWire->requestFrom(addr, lenRx);

      for(int i = 0; i < lenRx; i++) {
        c = pWire->read();
        *pRx++ = c;
      }
    }

    result = lenRx;
  }

fail:
  return result;
}


void
SystemI2cScan(int unit)
{
  TwoWire       *pWire;
  byte error, address;
  int nDevices = 0;

  if(unit == 0)      pWire = &Wire;
  else if(unit == 1) pWire = &Wire1;
  else               pWire = &Wire;

  Serial.printf("# I2C%d: Scanning...\n#  ", unit);
  for(address = 1; address < 127; address++ ) {
    pWire->beginTransmission(address);
    error = pWire->endTransmission();
    if (error == 0) {
      Serial.print(" 0x");
      if (address<16) Serial.print("0");
      Serial.print(address,HEX);
      nDevices++;
    }
  }
  Serial.println("");

  return;
}


//-------------------------------------------------------
// Systemtimegs()
//
#define EPOCH_YEAR              1970
#define TM_YEAR_BASE            1900

time_t
System_timegm1(struct tm *tm)
{
  int y, nleapdays;
  time_t t;
  /* days before the month */
  static const unsigned short moff[12] = {
    0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334
  };

  /*
   * XXX: This code assumes the given time to be normalized.
   * Normalizing here is impossible in case the given time is a leap
   * second but the local time library is ignorant of leap seconds.
   */

  /* minimal sanity checking not to access outside of the array */
  if ((unsigned) tm->tm_mon >= 12)                return (time_t) -1;
  if (tm->tm_year < EPOCH_YEAR - TM_YEAR_BASE)    return (time_t) -1;

  y = tm->tm_year + TM_YEAR_BASE - (tm->tm_mon < 2);
  nleapdays = y / 4 - y / 100 + y / 400 -
    ((EPOCH_YEAR-1) / 4 - (EPOCH_YEAR-1) / 100 + (EPOCH_YEAR-1) / 400);
  t = ((((time_t) (tm->tm_year - (EPOCH_YEAR - TM_YEAR_BASE)) * 365 +
         moff[tm->tm_mon] + tm->tm_mday - 1 + nleapdays) * 24 +
        tm->tm_hour) * 60 + tm->tm_min) * 60 + tm->tm_sec;

  return (t < 0 ? (time_t) -1 : t);
}


time_t
System_timegm2(struct tm *tm)
{
  // Unixエポックからの秒数を計算
  time_t year      = tm->tm_year + 1900LL; // tm_yearは1900年からのオフセット
  time_t month     = tm->tm_mon + 1LL;   // tm_monは0始まり（1月 = 0）
  time_t day       = tm->tm_mday;
  time_t hour      = tm->tm_hour;
  time_t minute    = tm->tm_min;
  time_t second    = tm->tm_sec;

  // 月ごとの日数（閏年を考慮しない場合）
  const int days_in_month[] = {31LL, 28LL, 31LL, 30LL, 31LL, 30LL, 31LL, 31LL, 30LL, 31LL, 30LL, 31LL};

  // 経過日数を計算
  time_t days = (year - 1970LL) * 365LL + ((year - 1969LL) / 4LL); // 閏年を考慮
  for (int i = 0; i < month - 1; i++) {
    days += days_in_month[i];
  }
  if (month > 2 && ((year % 4LL == 0 && year % 100LL != 0) || year % 400LL == 0)) {
    days++; // 閏年の場合、2月に1日追加
  }

  days += day - 1;

  // 秒数に変換
  return days * 86400LL + hour * 3600LL + minute * 60LL + second;
}


#if 0
static uint32_t         valGpt2Ic1;
static uint32_t         valGpt2Ic2;

static uint32_t         valGpt1Ic1;
static uint32_t         valGpt1Ic2;
void
SystemGptIcInit(int unit)
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

    valGpt1Ic1 = GPT1_CNT;
    valGpt1Ic2 = GPT1_CNT;

    GPT1_IR |=  GPT_IR_IF1IE;           // enable CAPTURE1 interrupt (valid only ptp slave mode)
    GPT1_IR |=  GPT_IR_IF2IE;           // enable CAPTURE2 interrupt

    attachInterruptVector(IRQ_GPT1, SystemGpt1Interrupt);
    NVIC_ENABLE_IRQ(IRQ_GPT1);

  } else if(unit == 2) {
    CCM_CCGR0 |= CCM_CCGR0_GPT2_SERIAL(CCM_CCGR_ON);    // clock enable for GPT2 module
    CCM_CCGR0 |= CCM_CCGR0_GPT2_BUS(CCM_CCGR_ON);       // clock enable for GPT2 module
    CCM_CMEOR |= CCM_CMEOR_MOD_EN_OV_GPT;

    GPT2_PR = GPT_PR_PRESCALER(0);      // set prescaler /1

    GPT2_CR  =  0;
    GPT2_IR  =  0x3f;
    GPT2_CR |=  GPT_CR_CLKSRC(1);        // set clock source  1:ipg_clk, 3:extclk
    GPT2_CR |=  GPT_CR_SWR;              // sw reset
    GPT2_SR  =  0x3f;
    GPT2_CR |=  GPT_CR_ENMOD;           // reset the counter
    GPT2_CR &= ~GPT_CR_ENMOD;           // reset the counter

    GPT2_CR |=  GPT_CR_IM2(1) | GPT_CR_IM1(1) | GPT_CR_CLKSRC(1) | GPT_CR_FRR;     // IC2: rise, Src: peri clock, Freerun
    GPT2_CR |=  GPT_CR_EN;

    valGpt2Ic1 = GPT2_CNT;
    valGpt2Ic2 = GPT2_CNT;

    GPT2_IR |=  GPT_IR_IF1IE;           // enable CAPTURE1 interrupt (valid only ptp slave mode)
    GPT2_IR |=  GPT_IR_IF2IE;           // enable CAPTURE2 interrupt

    attachInterruptVector(IRQ_GPT2, SystemGpt2Interrupt);
    NVIC_ENABLE_IRQ(IRQ_GPT2);
  }

  return;
}
#endif


//                                     analogWriteFrequency( freq * 10 )
//           +----------------+       +--------------------------+
//  ipg_clk  | TMR3.1         |       | TMR3.2                   |  ctrlout
//  ---------+ pres      1/10 +-------+ pres = C1Out   comp/load +---------
//           |                |       |                          |
//           +----------------+       +--------------------------+
//
//     1. init TMR3.1  1/10
//     2. analogWriteFrequency(freq*10), the TMR3.2 pres value is copied to TMR3.1 pres.
//     3. TMR3.2 pres is set C1Out
//
int             cycle    = 0;
int             cycleCnt = 0;
void
SystemCtrloutInit(int unit, float freq)
{
  IMXRT_TMR_CH_t        *p;
  uint32_t              pcs;
  float                 freqTmp;

  if(unit == 0) {
    freqTmp = freq - floor(freq);
    if(freqTmp == 0.0) {
      cycle = 1;
    } else {
      if(freqTmp > 0.5) freqTmp = 1.0 - freqTmp;
      cycle = (int)(1/(freqTmp - floor(freqTmp)));
    }

    //analogWriteResolution(CONFIG_GPIO_CTRLOUT0);
    analogWriteFrequency(CONFIG_GPIO_CTRLOUT0, freq * 10.0);


    analogWrite(CONFIG_GPIO_CTRLOUT0, 2);

    // get the PCS of TMR3.2
    p = &IMXRT_TMR3.CH[2];
    pcs = p->CTRL & TMR_CTRL_PCS(15);
    // cascade from counter1 output
    p->CTRL &= ~TMR_CTRL_PCS(15);
    p->CTRL |=  TMR_CTRL_PCS(5);

#if CONFIG_CTRLOUT0_POL_NEGATIVE
    p->SCTRL &= ~TMR_SCTRL_OPS;
#else
    p->SCTRL |= TMR_SCTRL_OPS;
#endif

    // TMR3.1
    p = &IMXRT_TMR3.CH[1];
    p->CNTR   = 0;
    p->LOAD   = 0xffff;
    p->COMP1  = 8;
    p->CMPLD1 = 8;
    p->CTRL   = TMR_CTRL_CM(1) | pcs | TMR_CTRL_LENGTH | TMR_CTRL_OUTMODE(6);
    p->SCTRL  = TMR_SCTRL_OEN;

  } else if(unit == 1) {
    analogWriteFrequency(CONFIG_GPIO_CTRLOUT1, 20);
    analogWrite(CONFIG_GPIO_CTRLOUT1, 2);

  }

  return;
}
void
SystemCtrloutClearCounter(int unit)
{
  IMXRT_TMR_CH_t        *p;

  if(unit == 0) {
    int         s;

    p = &IMXRT_TMR3.CH[2];
    s = *(int16_t *)&p->CNTR;

    if(cycleCnt == 0) {
      cycleCnt = cycle;

      if       (s < -2) {
        p->CNTR = 0xffff;
        p->CMPLD1--;
      } else if(s >  2) {
        p->CNTR = 1;
        p->CMPLD1++;
      }

    }
    cycleCnt--;
  } else if(unit == 1) {

  }

  return;
}


void
SystemSetLed0(int on)
{
  int           val;

  val = on? 1: 0;

  switch(SystemGetBoardId()) {
  case        CONFIG_BOARDID_T4_IMU:
  case        CONFIG_BOARDID_T4_SENSECAP:
    digitalWrite(CONFIG_GPIO_LED0_T4_IMU, val);
    break;
  default:
    digitalWrite(CONFIG_GPIO_LED0, val);
    break;
  }

  return;
}


void
SystemSetLedL(int on)
{
  int           val;

  val = on? 1: 0;

  switch(SystemGetBoardId()) {
  case        CONFIG_BOARDID_T4_SENSECAP:
    digitalWrite(CONFIG_B2_GPIO_LED_L, val);
    break;
  default:
    digitalWrite(CONFIG_GPIO_LED, val);
    break;
  }

  return;
}


void
SystemSetPowerEn(int on)
{
  int           val;

  val = on? 1: 0;
  digitalWrite(CONFIG_GPIO_IMU_POWER,  val);

  return;
}


void
SystemReset(void)
{
#ifndef __CORE_CM7_H_GENERIC
#define SCB_AIRCR_VECTKEY_Pos              16U                                            /*!< SCB AIRCR: VECTKEY Position */
#define SCB_AIRCR_PRIGROUP_Pos              8U                                            /*!< SCB AIRCR: PRIGROUP Position */
#define SCB_AIRCR_PRIGROUP_Msk             (7UL << SCB_AIRCR_PRIGROUP_Pos)                /*!< SCB AIRCR: PRIGROUP Mask */
#define SCB_AIRCR_SYSRESETREQ_Pos           2U                                            /*!< SCB AIRCR: SYSRESETREQ Position */
#define SCB_AIRCR_SYSRESETREQ_Msk          (1UL << SCB_AIRCR_SYSRESETREQ_Pos)             /*!< SCB AIRCR: SYSRESETREQ Mask */
#endif

    asm("dsb");                                                         // Ensure all outstanding memory accesses included
                                                                        // buffered write are completed before reset
    SCB_AIRCR  = (uint32_t)((0x5FAUL << SCB_AIRCR_VECTKEY_Pos)    |
                             (SCB_AIRCR & SCB_AIRCR_PRIGROUP_Msk) |
                             SCB_AIRCR_SYSRESETREQ_Msk    );            // Keep priority group unchanged
    asm("dsb");                                                         // Ensure completion of memory access
    while(1);
}
