#ifndef _SYSTEM_H_
#define _SYSTEM_H_


#define SystemGetCounter()      (-millis())

#define SYSTEM_COUNTER_1M000S   1
#define SYSTEM_COUNTER_10M00S   10
#define SYSTEM_COUNTER_100M0S   100
#define SYSTEM_COUNTER_1S000    1000
#define SYSTEM_COUNTER_10S00    10000
#define SYSTEM_COUNTER_100S0    100000
#define SYSTEM_COUNTER_1000S    1000000


#define SystemGetUsCounter()      (-micros())

#define SYSTEM_COUNTER_US1U000S   1
#define SYSTEM_COUNTER_US10U00S   10
#define SYSTEM_COUNTER_US100U0S   100
#define SYSTEM_COUNTER_US1M000S   1000
#define SYSTEM_COUNTER_US10M00S   10000
#define SYSTEM_COUNTER_US100M0S   100000
#define SYSTEM_COUNTER_US1S000    1000000
#define SYSTEM_COUNTER_US10S00    10000000
#define SYSTEM_COUNTER_US100S0    100000000

#define SystemGetCounter1MHz()  (micros())


struct _stSystemSpiParam {
  int           speed;
  uint32_t      tWait;          // wait after command
  uint32_t      tWaitCs;        // wait bwt cs and data
};
struct _stSystemI2cParam {
  int           speed;
};

void                    SystemPinSettings(void);
int                     SystemGetBoardId(void);

void                    SystemWaitCounter(uint32_t tout);
void                    SystemWaitUsCounter(uint32_t tout);

int                     SystemSpiInit(int unit);
void                    SystemSpiSetSpeed(int unit, int speed);
int                     SystemSpiTransfer(int unit, int cs, const uint8_t *pTx, int lenTx, uint8_t *pRx, int lenRx, struct _stSystemSpiParam *pParam);
int                     SystemSpiGetCsxGpioNum(int unit, int numCs);
int                     SystemSpiSetCsx(int unit, int numCs, int val);

int                     SystemI2cInit(int unit);
int                     SystemI2cTransfer(int unit, int addr, const uint8_t *pTx, int lenTx, uint8_t *pRx, int lenRx, struct _stSystemI2cParam *pParam);
void                    SystemI2cScan(int unit);

time_t                  System_timegm1(struct tm *tm);
time_t                  System_timegm2(struct tm *tm);

void                    SystemCtrloutInit(int unit, float freq);
void                    SystemCtrloutClearCounter(int unit);

void                    SystemGptIcInit(int unit);
void                    SystemGpt1Interrupt(void);
void                    SystemGpt2Interrupt(void);

void                    SystemPidCalcInit(void);
void                    SystemPidCalc50MHz(int current, int target);
void                    SystemPidCalcVcocxo(int current, int target);

void                    SystemSetLed0(int on);
void                    SystemSetLedL(int on);
void                    SystemReset(void);


#endif
