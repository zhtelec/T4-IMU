#define _GNSS_CPP_

#include        <stdio.h>
#include        <stdlib.h>
#include        <stdint.h>
#include        <time.h>
#include        <inttypes.h>

#include        <Arduino.h>
#include        <QNEthernet.h>
#include        <SparkFun_u-blox_GNSS_v3.h>
#include        <u-blox_config_keys.h>

#include        "config.h"
#include        "system.h"
#include        "network.h"
#include        "packet.h"
#include        "ptp.h"

#include        "gnss.h"

#define GNSS_DEBUG_SHOW_PACKET_ENABLE   0


static SFE_UBLOX_GNSS_SERIAL  GnssDev;

struct _stGnss {
  SFE_UBLOX_GNSS_SERIAL *dev;
  int64_t               tSecPrev;

  HardwareSerial        *devSerial;
  uint8_t               devSerialRxBuf[CONGIG_GNSS_SERIAL_RXBUF_SIZE];

  int                   week;
  int                   msecOfWeek;
  int                   leapSec;        // leap time from 1980 (GPS time)

  struct tm             tmGpsUtc;
  time_t                tGpsUtc;
  time_t                tai;
  int32_t               nano;

  uint8_t               seq;
  uint8_t               seqPps;

  int                   cntLock;        // +2 pps intr, -1 every 1sec

  struct _stPacketGnssPvt   pktLatest;


  enum debug_mode       debug;
  uint32_t              t1Sec;
};
static struct _stGnss   gnss;


void
GnssInit(void)
{
  int           re;
  uint32_t      val;

  memset(&gnss, 0, sizeof(gnss));

  gnss.devSerial = &CONFIG_GNSS_SERIAL_IF;
  CONFIG_GNSS_SERIAL_IF.addMemoryForRead(gnss.devSerialRxBuf, sizeof(gnss.devSerialRxBuf));
  gnss.devSerial->begin(CONGIG_GNSS_SERIAL_BAUD);
  gnss.dev = &GnssDev;

#if GNSS_DEBUG_SHOW_PACKET_ENABLE
  gnss.dev->enableDebugging(Serial);
#endif

  // begin the gnss library
  gnss.dev->begin(*gnss.devSerial, 10, true);
  // set the serial baud
  GnssInitSetBaud();

  // set basic settings
  val =  CONGIG_GNSS_SERIAL_BAUD;
  re = gnss.dev->setVal32(UBLOX_CFG_UART1_BAUDRATE, val, VAL_LAYER_RAM, 100);
  Serial.printf("# gnss: UART1_BAUDRATE = %d... done\n", val);
  gnss.devSerial->begin(CONGIG_GNSS_SERIAL_BAUD);

  val = 1;
  re = gnss.dev->setVal32(UBLOX_CFG_MSGOUT_UBX_NAV_TIMELS_UART1, val);
  Serial.printf("# gnss: MSGOUT_UBX_NAV_TIMELS_UART1 = %d... %s\n", val, re?"done": "ng");

  val = 1;
  re = gnss.dev->setVal32(UBLOX_CFG_NMEA_SVNUMBERING, val);
  Serial.printf("# gnss: NMEA_SVNUMBERING = %d... %s\n", val, re?"done": "ng");

  val = 1;
  re = gnss.dev->setVal8(UBLOX_CFG_NMEA_HIGHPREC, val);
  Serial.printf("# gnss: NMEA_HIGHPREC = %d... %s\n", val, re?"done": "ng");

  val = 1000;
  re = gnss.dev->setVal16(UBLOX_CFG_RATE_MEAS, val);
  Serial.printf("# gnss: RATE_MEAS = %d... %s\n", val, re?"done": "ng");

  val = 1000 * 1000;
  re = gnss.dev->setVal32(UBLOX_CFG_TP_PERIOD_LOCK_TP1, val);
  Serial.printf("# gnss: TP_PERIOD_LOCK_TP1 = %d... %s\n", val, re?"done": "ng");

  val =  100 * 1000;
  re = gnss.dev->setVal32(UBLOX_CFG_TP_LEN_LOCK_TP1, val);
  Serial.printf("# gnss: TP_LEN_LOCK_TP1 = %d... %s\n", val, re?"done": "ng");


  //gnss.dev->setAutoPVT(true);
  gnss.dev->setAutoPVTcallbackPtr(GnssCallbackPvt);

  return;
}


void
GnssLoop(void)
{
#if  0
  while(gnss.devSerial->available() > 0) {
    Serial.printf("%c", gnss.devSerial->read());
  }
#endif

  gnss.dev->checkUblox();
  gnss.dev->checkCallbacks();

  if((gnss.t1Sec - SystemGetCounter()) >= 1000) {
    gnss.t1Sec = SystemGetCounter();

    if(gnss.cntLock > 0) gnss.cntLock--;
  }

  return;
}


static int  gnssBaudScanList[] = {921600, 38400, 57600, 115200, 230400, 460800,
                                  1200, 2400, 4800, 9600, 19200, 1000000, 2000000};
static void
GnssInitSetBaud(void)
{
  for(int i = 0; i < sizeof(gnssBaudScanList)/sizeof(int); i++) {
    Serial.printf("# gnss: auto baud [%d]   \r", gnssBaudScanList[i]);
    gnss.devSerial->begin(gnssBaudScanList[i]);
    if (gnss.dev->getPVT()) {
      break;
    }
  }
  Serial.println("");

  return;
}


static void
GnssCallbackPvt(UBX_NAV_PVT_data_t *pPVT)
{
  int64_t                       adjTaiSec;
  int32_t                       adjTaiNano;

  struct _stPacketGnssPvt       *pPkt;

  sfe_ublox_ls_src_e            src;
  src = SFE_UBLOX_LS_SRC_GPS;
  gnss.leapSec = gnss.dev->getCurrentLeapSeconds(src, 10);

  gnss.tmGpsUtc.tm_year  = pPVT->year - 1900;
  gnss.tmGpsUtc.tm_mon   = pPVT->month-1;
  gnss.tmGpsUtc.tm_mday  = pPVT->day;
  gnss.tmGpsUtc.tm_hour  = pPVT->hour;
  gnss.tmGpsUtc.tm_min   = pPVT->min;
  gnss.tmGpsUtc.tm_sec   = pPVT->sec;

  gnss.tGpsUtc = timegm(&gnss.tmGpsUtc);
  gnss.tai     = gnss.tGpsUtc + gnss.leapSec + CONFIG_TIME_LEAPSEC_BWT_1970TO1980;
  gnss.nano    = pPVT->nano;

  adjTaiSec  = gnss.tai;
  adjTaiNano = pPVT->nano;
  if(pPVT->nano<0) {
    adjTaiSec--;
    adjTaiNano += CONFIG_GNSS_VAL10E9;
  }

  gnss.msecOfWeek = pPVT->iTOW;

  pPkt                  = &gnss.pktLatest;

  // header
  pPkt->hdr.type        = PACKET_TYPE_GNSS_INT64;
  pPkt->hdr.id          = 0;
  pPkt->hdr.seq         = gnss.seq;
  pPkt->hdr.tai_secH    = (adjTaiSec >> 32) & 0xff;
  pPkt->hdr.tai_sec     =  adjTaiSec & 0xffffffff;
  pPkt->hdr.tai_nsec    =  adjTaiNano;
  pPkt->hdr.ts_1MHz     = SystemGetCounter1MHz();

  // epoch time from UTC
  pPkt->epochtime       = (uint64_t)gnss.tGpsUtc;

  // gnss data
  pPkt->iTOW            = pPVT->iTOW;           // unit ms

  pPkt->year            = pPVT->year;           // UTC
  pPkt->month           = pPVT->month;
  pPkt->day             = pPVT->day;
  pPkt->hour            = pPVT->hour;
  pPkt->min             = pPVT->min;
  pPkt->sec             = pPVT->sec;
  pPkt->valid           = pPVT->valid.all;

  pPkt->tAcc            = pPVT->tAcc;           // unit ns    time accuracy
  pPkt->nano            = pPVT->nano;

  pPkt->fixType         = pPVT->fixType;       // 0:NoFix, 1:DeadReckoningOnly, 2:2dFix, 3:3dFix, 5:TimeOnly
  pPkt->flags           = pPVT->flags.all;
  pPkt->flags2          = pPVT->flags2.all;
  pPkt->numSV           = pPVT->numSV;          // pcs
  pPkt->height          = pPVT->height*10;      // unit 0.1mm (ublox output in mm)

  pPkt->lat             = pPVT->lat * 100LL;    // unit 10^-9 deg (ublox output is 10^-7)

  pPkt->lon             = pPVT->lon * 100LL;    // unit 10^-9 deg (ublox output is 10^-7)

  pPkt->hMSL            = pPVT->hMSL*10;        // unit 0.1mm  hMSL (ublox output in mm)
  pPkt->hAcc            = pPVT->hAcc*10;        // unit 0.1mm  horizontal accuracy (ublox output in mm)

  pPkt->vAcc            = pPVT->vAcc*10;        // unit mm    vertical accuracy
  pPkt->velN            = pPVT->velN;           // unit mm/s  to N is plus

  pPkt->velE            = pPVT->velE;           // unit mm/s  to E is plus
  pPkt->velD            = pPVT->velD;           // unit mm/s  to down is plus

  pPkt->gSpeed          = pPVT->gSpeed;         // unit mm/s
  pPkt->headMot         = pPVT->headMot;        // unit 10^-6 deg

  pPkt->sAcc            = pPVT->sAcc;           // unit mm/s  speed accuracy
  pPkt->headAcc         = pPVT->headAcc;        // unit 10^-6 deg   head accuracy

  pPkt->pDOP            = pPVT->pDOP;
  pPkt->flags3          = pPVT->flags3.all;

  pPkt->headVeh         = pPVT->headVeh;        // unit 10^-6 deg
  pPkt->magDec          = pPVT->magDec;

  pPkt->magAcc          = pPVT->magAcc;

  PacketCalcSumAndFillHeader((struct _stPacketGeneric *)pPkt, sizeof(struct _stPacketGnssPvt));

  NetworkSendUdp((uint8_t *)pPkt, sizeof(struct _stPacketGnssPvt));


  if(gnss.debug & GNSS_DEBUG_SHOW_TIME) {
    struct tm   *pUtc;
    Serial.printf("gps utc  %4d/%2d/%2d %2d:%2d:%2d (%lld.%09ld, ls:%d)\n",
                  gnss.tmGpsUtc.tm_year+1900, gnss.tmGpsUtc.tm_mon, gnss.tmGpsUtc.tm_mday,
                  gnss.tmGpsUtc.tm_hour, gnss.tmGpsUtc.tm_min, gnss.tmGpsUtc.tm_sec,
                  gnss.tGpsUtc,
                  gnss.nano,
                  gnss.leapSec + CONFIG_TIME_LEAPSEC_BWT_1970TO1980
                  );

    pUtc = gmtime(&gnss.tai);
    Serial.printf("tai      %4ld/%2ld/%2ld %2ld:%2ld:%2ld (%lld.%09ld)\n",
                  pUtc->tm_year+1900, pUtc->tm_mon, pUtc->tm_mday,
                  pUtc->tm_hour, pUtc->tm_min, pUtc->tm_sec,
                  adjTaiSec,
                  adjTaiNano
                  );
  }


  if(gnss.debug & GNSS_DEBUG_SHOW_PVT) {
    char        buf[256];
    int         len = 0;
    int         msec;
    if(pPVT->nano < 0) {
      msec = 0;
    } else {
      msec        = pPVT->nano >> 14;
      msec        = msec * ((16384 * 16384) / 15625);        // 1024*1024 / 1000,000
      msec       += 1<<19;
      msec      >>= 20;
    }
    int32_t     nano;
    nano = (pPVT->nano<0)? CONFIG_GNSS_VAL10E9+pPVT->nano: pPVT->nano;

    len  = sprintf(buf + len, "gps NAV PVT %lld.%09ld", gnss.tai, nano);
    len += sprintf(buf + len, " %04d%02d%02d%02d%02d%02d%03d %09ld", pPVT->year, pPVT->month, pPVT->day, pPVT->hour, pPVT->min, pPVT->sec, msec, pPVT->nano);
    len += sprintf(buf + len, " %ld: %ld %ld %ld (%4d %4d) ", pPVT->iTOW, pPVT->lat, pPVT->lon, pPVT->hMSL, (int)pPVT->hAcc*10, (int)pPVT->vAcc*10);
    len += sprintf(buf + len, " %ld %ld %ld\n", pPVT->gSpeed, pPVT->headMot, pPVT->headAcc);

    Serial.print(buf);
  }

  if(gnss.cntLock < CONFIG_GNSS_LOCK_COUNT * 2) gnss.cntLock += 2;

  gnss.seq++;


 return;
}


int
GnssGetTime(timespec *ptm)
{
  int result = -1;

  if(gnss.cntLock >= CONFIG_GNSS_LOCK_COUNT) {
    ptm->tv_sec   = gnss.tai;
    ptm->tv_nsec  = gnss.nano;

    result = 1;
  } else {
    ptm->tv_sec   = 0;
    ptm->tv_nsec  = 0;
  }

  return result;
}


void
GnssInterruptPps(void)
{
  struct _stPacketGnssPps       pkt;

  gnss.tai++;

  // header
  pkt.hdr.type        = PACKET_TYPE_GNSS_PPS;
  pkt.hdr.id          = 0;
  pkt.hdr.seq         = gnss.seqPps;
  pkt.hdr.tai_secH    = (gnss.tai >> 32) & 0xff;
  pkt.hdr.tai_sec     =  gnss.tai & 0xffffffff;
  pkt.hdr.tai_nsec    = 0;
  pkt.hdr.ts_1MHz     = SystemGetCounter1MHz();

  // epoch time from UTC
  pkt.epochtime       = (uint64_t)gnss.tGpsUtc + 1;
  pkt.tai             = (uint64_t)gnss.tai;

  PacketCalcSumAndFillHeader((struct _stPacketGeneric *)&pkt, sizeof(struct _stPacketGnssPps));
  NetworkSendUdp((uint8_t *)&pkt, sizeof(struct _stPacketGnssPps));

  gnss.seqPps++;

  return;
}


void
GnssSetDebugMode(uint32_t v, uint32_t mask)
{
  uint32_t      tmp;

  tmp = gnss.debug & ~mask;
  gnss.debug = (  enum debug_mode )(tmp | (v & mask));

  return;
}


void
GnssCommand(int ac, char *av[])
{
  if(       !strcmp(av[1], "showpvt")) {
    enum debug_mode     val = GNSS_DEBUG_SHOW_PVT;
    if(av[2][0] == '1') {
      GnssSetDebugMode(val, val);
    } else {
      GnssSetDebugMode(0, val);
    }

  } else if(!strcmp(av[1], "showtime")) {
    enum debug_mode     val = GNSS_DEBUG_SHOW_TIME;
    if(av[2][0] == '1') {
      GnssSetDebugMode(val, val);
    } else {
      GnssSetDebugMode(0, val);
    }
  }

  return;
}
