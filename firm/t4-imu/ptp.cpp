#define _PTP_CPP_

#include        <stdio.h>
#include        <stdlib.h>

#include        <Arduino.h>

#include        <t41-ptp.h>
#include        <QNEthernet.h>
#include        <TimeLib.h>

#include        "config.h"
#include        "system.h"
#include        "gnss.h"
#include        "ptp.h"


#define PTP_DEBUG_TIMESTAMP_BACKWARDS   0

namespace qn = qindesign::network;
IntervalTimer   ptpSyncTimer;
IntervalTimer   ptpAnnounceTimer;


struct _stPtp {
  int                   up;

  l3PTP                 *dev;

  bool                  modeMaster;
  bool                  modeSlave;
  bool                  p2p;

  uint32_t              ledTimer;

  bool                  fSync;
  bool                  fAnnounce;
  bool                  fPps;
  int                   noPPSCount;

  NanoTime              interrupt_s;
  NanoTime              interrupt_ns;
  NanoTime              pps_s;
  NanoTime              pps_ns;

  struct timespec       tsPps;

};

static struct _stPtp    ptp;


void
PtpInit(ptpMode_t mode)
{

  memset(&ptp, 0, sizeof(ptp));

  // state ptp
  qn::EthernetIEEE1588.begin();

  switch(mode) {
  case        ptpBoth:   ptp.modeSlave  = true;
  case        ptpMaster: ptp.modeMaster = true; break;
  case        ptpSlave:  ptp.modeSlave  = true; break;
  };
  static l3PTP ptpDev(ptp.modeMaster, ptp.modeSlave, ptp.p2p);
  ptp.dev = &ptpDev;
  ptp.dev->begin();


  // PPS Out
  // peripherial: ENET_1588_EVENT1_OUT
  // IOMUX: ALT6
  // teensy pin: 24 (GPIO_AD_B0_12)
  IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B0_12 = 6;
  //qn::EthernetIEEE1588.setChannelCompareValue(1, NS_PER_S);
  qn::EthernetIEEE1588.setChannelCompareValue(1, 0);
  qn::EthernetIEEE1588.setChannelMode(1, qn::EthernetIEEE1588.TimerChannelModes::kPulseHighOnCompare);
  qn::EthernetIEEE1588.setChannelOutputPulseWidth(1, 25);       // unit: 1588-clock 40ns  1-32 pulses
  if(ptp.modeSlave == true) {
    qn::EthernetIEEE1588.setChannelInterruptEnable(1, true);
  }

  if(ptp.modeMaster == true) {
    // PPS-IN
    // peripherial: ENET_1588_EVENT2_IN
    // IOMUX: ALT4
    // teensy pin: 15 (PTP signal & PPS_MASK(pin20))
    IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_03 = 4;
    //enable Channel2 rising edge trigger, and Configure Interrupt generation
    qindesign::network::EthernetIEEE1588.setChannelMode(2, qindesign::network::EthernetIEEE1588.TimerChannelModes::kCaptureOnRising);
    qindesign::network::EthernetIEEE1588.setChannelInterruptEnable(2, true);

    ptpSyncTimer.begin(PtpInterruptSync, 1000*1000);
    ptpAnnounceTimer.begin(PtpInterruptAnnounce, 1000*1000);

  }

  if(ptp.modeSlave == true || ptp.modeMaster == true) {
    attachInterruptVector(IRQ_ENET_TIMER, PtpInterruptTimer);
    NVIC_ENABLE_IRQ(IRQ_ENET_TIMER);
  }

  ptp.up = 1;

  return;
}


void
PtpLoop()
{
  ptp.dev->update();

  if(ptp.modeMaster == true) {
    // send packet
    if(ptp.fAnnounce) {
      ptp.fAnnounce = false;
      ptp.dev->announceMessage();
    }
    if(ptp.fSync) {
      ptp.fSync = false;
      ptp.dev->syncMessage();
    }
    if(ptp.fPps) {
      ptp.fPps = false;
      //Serial.printf("%lld.%09d  %lld.%09d\n", ptp.pps_s, ptp.pps_ns, ptp.interrupt_s, ptp.interrupt_ns);
      ptp.dev->ppsInterruptTriggered(((NanoTime)ptp.pps_s*NS_PER_S)+(NanoTime)ptp.pps_ns, ((NanoTime)ptp.interrupt_s*NS_PER_S)+(NanoTime)ptp.interrupt_ns);

      if(ptp.dev->getLockCount() > 5) {
        ptp.fSync = true;
      }
    }
  }

  if(ptp.ledTimer &&
     (ptp.ledTimer - SystemGetCounter()) > 100 * SYSTEM_COUNTER_1M000S) {
    ptp.ledTimer = 0;
    digitalWrite(CONFIG_GPIO_LED, 0);

  }

#if PTP_DEBUG_TIMESTAMP_BACKWARDS
  PtpCheckTimestampBackwards();
#endif

  return;
}


static void
PtpInterruptTimer(void)
{
  // counter interrupt for master
  if(qindesign::network::EthernetIEEE1588.getAndClearChannelStatus(2)) {
    if(ptp.modeMaster == true) {
      uint32_t            t;
      timespec            ts;
      struct timespec     tm;
      int                 re;

      GnssInterruptPps();

      qindesign::network::EthernetIEEE1588.getChannelCompareValue(2, t);
      t = ((NanoTime)t + NS_PER_S-30) % NS_PER_S;
      qindesign::network::EthernetIEEE1588.readTimer(ts);

      if(ts.tv_nsec < 100*1000*1000 && t > 900*1000*1000){
        ptp.pps_s       = ts.tv_sec;
        ptp.interrupt_s = ts.tv_sec - 1;
      } else {
        ptp.pps_s     = ts.tv_sec;
        if(ts.tv_nsec > 500*1000*1000) ptp.pps_s++;
        ptp.interrupt_s = ts.tv_sec;
      }

      re = GnssGetTime(&tm);
      if(re >= 0) {
        ptp.pps_s = tm.tv_sec;
      }

      ptp.interrupt_ns  = t;
      ptp.pps_ns        = 0;

      ptp.fPps          = true;
      ptp.noPPSCount    = 0;

    }
  }

  // counter interrupt for slave
  if(qindesign::network::EthernetIEEE1588.getAndClearChannelStatus(1)) {
    if(ptp.modeSlave == true) {
      qindesign::network::EthernetIEEE1588.readTimer(ptp.tsPps);

      SystemCtrloutClearCounter(0);
      SystemCtrloutClearCounter(1);
    }
  }

  digitalWrite(CONFIG_GPIO_LED, 1);
  ptp.ledTimer = SystemGetCounter() | 1;

  asm("dsb"); // allow write to complete so the interrupt doesn't fire twice

  return;
}
static void
PtpInterruptSync(void)
{
  if(ptp.noPPSCount > 5) {
    ptp.fSync = true;
  } else {
    ptp.noPPSCount++;
  }

  return;
}
static void
PtpInterruptAnnounce(void)
{
  ptp.fAnnounce = true;

  return;
}


uint32_t
PtpGetLockCount(void)
{
  uint32_t      val = 0;

  if(ptp.up) val = ptp.dev->getLockCount();

  return val;
}

void
PtpGetUnixTime(struct timespec *t)
{
  struct timespec tt = {0};

  qn::EthernetIEEE1588.readTimer(tt);
  *t = tt;

  return;
}


void
PtpGetUnixTimePps(struct timespec *t)
{
  *t = ptp.tsPps;

  return;
}


// buf needs to store 30 characters
int timespec2str(char *buf, uint len, struct timespec *ts)
{
  int ret;
  struct tm t = {0};

  tzset();
  if (localtime_r(&(ts->tv_sec), &t) == NULL) return 1;

  ret = strftime(buf, len, "%F %T", &t);
  if (ret == 0) return 2;
  len -= ret - 1;

  ret = snprintf(&buf[strlen(buf)], len, ".%09ld", ts->tv_nsec);
  if (ret >= (int)len) return 3;

  return 0;
}

void print_current_time(const char* header = nullptr)
{
  struct timespec t_timespec = {0};
  char timeStringBuff[64] = {0};

  qn::EthernetIEEE1588.readTimer(t_timespec);
  timespec2str(timeStringBuff, sizeof(timeStringBuff), &t_timespec);
  if (header != nullptr) {
    Serial.printf("%s : ", header);
  }
  Serial.println(timeStringBuff);

  return;
}

void get_timeval(struct timeval *tv)
{
  struct timespec t_timespec = {0};
  qn::EthernetIEEE1588.readTimer(t_timespec);
  TIMESPEC_TO_TIMEVAL(tv, &t_timespec);

  return;
}


static void
PtpCheckTimestampBackwards(void)
{
  struct timespec     tt;
  static uint64_t     tPrev;
  uint64_t            t;

  PtpGetUnixTime(&tt);

  t = tt.tv_sec * 1000000000 + tt.tv_nsec;
  //Serial.printf("now  %lld\r", t);
  if(t < tPrev) {
    Serial.printf("prev %lld\npres %lld    in ptp\n", tPrev, t);
  }
  tPrev = t;

  return;
}
