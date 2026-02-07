#define __IMUDRV_CPP__
// ========================================
//  Imudrv
// ========================================

//
// +----------------------------------------------------------------------+
// |   sensor_[init,start,stop,get,diagnosis]                             |
// |                                                  sensor.c            |
// +----------------------------------------------------------------------+
// |   imudrv_[init,deinit,start,stop,main,diagnosis, etc]                |
// |                                                  imudrv.c            |
// +----------------------------------------------------------------------+
// |   imudrv[Init,Probe,Start,Stop,Store]                                |
// |                                           +--------------------------+
// |                                           | imudrvSc_000List[]       |
// |                           imudrv.c     |      imudrv_000List.h       |
// +----------------------------------------+--+--------------------------+
// |   imudrvSpi[write,read,SetConfig]   |  | imudrvSc_LSM6DSR[]          |
// |                           imudrv.c  |  | imudrvSc_LSM6DSV[]          |
// +----------------------------------------|  |                          |
// |   IMU Device [LSM6DSR,LSM6DSV]         |  |                          |
// |                                        |  |       imudrv_*.c         |
// +----------------------------------------+  +--------------------------+
//
//  1. main()  main.c
//         call imudrvInit(-1)
//  2. main()  main.c
//         call sensor_init()
//  3. sensor_init()  sensor.c
//         sensor_driver_init()      setup spi module (speed=3125kHz)
//         sensor_driver_probe()     probe and attach IMU driver (check only IMU0)
//             search every driver in imudrv_000List[]
//             the driver is selected and spi speed is re-initiazlied, if matched
//         sensor_diagnosis()        all IMU are initialized
//             setup all IMU, if the chip id is matched
//


#include <stdio.h>
#include <string.h>

#include <Arduino.h>

#include "config.h"
#include "system.h"
#include "eeprom.h"

#include "ptp.h"
#include "imudrv.h"

#include "imudrv_list.h"

#define CONFIG_IMUSPI_RECV_BUFFER_DEPTH         32 // adhoc


typedef int16_t         imu_raw_data[7+2];      // ts, tmpr, accel[3], gryo[3]

typedef struct {
  int                 lock;
  int                 id;
  int                 phase;
} spi_exclusion;


extern int              imuFmtUart;
extern int              imuFmtIp;

struct _stImudrv             imudrv;

static const uint8_t    txtIMU[] = "IMU";
static const uint8_t    txtMAG[] = "MAG";
static const uint8_t    txtUKN[] = "UKN";


//*************************************************************************
//
//



//
// the first IMU is checked what imu is.
// the IMU code is uesd, if the register pattern is match to registrated code.
//
int
ImudrvProbe(void)
{
  int                           result = IMUDRV_ERRNO_NOMATCH;
  int         re;

  struct _stImudrvList          *p;
  int                           id = 0;
  struct _stSystemSpiParam      spiParam;
  //struct _stSystemI2cParam      i2cParam;

  // check the reg value and imu match list
  imudrv.pImudrvList = imudrvList;
  for(int k = 0;      ; k++) {
    result = IMUDRV_ERRNO_NOMATCH;
    p = imudrv.pImudrvList[k];
    if(p == NULL) break;

    if(p->probe) {
      // | 15--12 | 11-- 8 | 7 --------- 0|
      //  0 I2C     busNum   slave addr (0x00 -- 0x7f)
      //  1 SPI     busNUm   cs num     (CS0X -- CS15X)
      int               bus;
      for(bus = 0; bus < 0x2000; bus++) {
        if(bus < 0x0200) {       // if = I2C0, 1
          if((bus & 0xff) < 0x80) {
            result = ImudrvProbeStub(bus, p);

          } else {
            bus += 0x7f;
          }

        } else if(bus < 0x1000) {       // if = I2C2, 3
          result = IMUDRV_ERRNO_UNKNOWN;
          bus = 0xfff;   // skip

        } else if(bus < 0x2000) {       // if = SPI
          result = ImudrvProbeStub(bus, p);

          if((bus & 0x00ff) >= 15) bus |= 0xff;
        }

        if(result == IMUDRV_SUCCESS) {
          uint8_t       type;
          type          = ImudrvGetTypeDev(p->type);


          if(((bus >> 12) & 0xf) == 1) {
            Serial.printf("# id%d: %s%d (%x-%d) SPI%d CS%x device %s\n",
                          id, ImudrvGetSensorTypeTxt(type), imudrv.subIdInType[type],
                          type, imudrv.subIdInType[type],
                          (bus >> 8) & 0xf, bus & 0xff, p->name);
          } else if(((bus >> 12) & 0xf) == 0) {
            Serial.printf("# id%d: %s%d (%x-%d) I2C%d(0x%02x), device %s\n",
                          id, ImudrvGetSensorTypeTxt(type), imudrv.subIdInType[type],
                          type, imudrv.subIdInType[type],
                          (bus >> 8) & 0xf, bus & 0xff, p->name);
          }



          uint32_t      val;
          int           accfsr, gyrfsr, odr /*fmtUart, fmtIp*/;
          int           speed;
          val = EepromGet16(CONFIG_EEPROM_IMU_PARAM_POS);
          accfsr        = (val >> CONFIG_EEPROM_IMU_PARAM_ACC_SHIFT) & 0xf;
          gyrfsr        = (val >> CONFIG_EEPROM_IMU_PARAM_GYR_SHIFT) & 0xf;
          odr           = (val >> CONFIG_EEPROM_IMU_PARAM_ODR_SHIFT) & 0xf;

          spiParam = p->spiParam;
          val = EepromGet8(CONFIG_EEPROM_IMU_PARAM2_POS);
          if(val & CONFIG_EEPROM_IMU_PARAM2_SPEED_MASK) {
            speed       = ((val & CONFIG_EEPROM_IMU_PARAM2_SPEED_MASK) >> CONFIG_EEPROM_IMU_PARAM2_SPEED_SHIFT);
            spiParam.speed = speed * 1000 * 1000;
          }

          imudrv.sc[id].accfsr       = accfsr;
          imudrv.sc[id].gyrfsr       = gyrfsr;
          imudrv.sc[id].odr          = odr;
          imudrv.sc[id].spiParam     = spiParam;

          struct _stProbedSc          *pSc;
          int                         intr;
          pSc = &imudrv.sc[id];
          pSc->id      = id;
          pSc->subId   = imudrv.subIdInType[type]++;   // copy and increment
          pSc->bus     = bus;
          pSc->pDriver = p;
          re = ImudrvInit(id,
                          0,                 // power mode
                          pSc->accfsr,       // accfsr
                          pSc->gyrfsr,       // gyrfsr
                          pSc->odr           // odr
                          );
          if(re < 0) continue;

#if 0
          if(((bus >> 12) & 0xf) == 1) {
            Serial.printf("# id%d: %s%d (%x-%d) SPI%d CS%x device %s\n",
                          id, ImudrvGetSensorTypeTxt(type), imudrv.subIdInType[type],
                          type, imudrv.subIdInType[type],
                          (bus >> 8) & 0xf, bus & 0xff, p->name);
          } else if(((bus >> 12) & 0xf) == 0) {
            Serial.printf("# id%d: %s%d (%x-%d) I2C%d(0x%02x), device %s\n",
                          id, ImudrvGetSensorTypeTxt(type), imudrv.subIdInType[type],
                          type, imudrv.subIdInType[type],
                          (bus >> 8) & 0xf, bus & 0xff, p->name);
          }
#endif

          if((intr = ImudrvGetInterruptPin(bus)) >= 0) {
            Serial.printf("#       intr line: %d\n", intr);
            pSc->intrPort = intr;
            imudrv.idByIntr[intr] = id;

            ImudrvEnableInterrupt(intr);
            pSc->intrDet = 1;

          }

          id++;                 // inc for next device

        }
      }

    }
  }
  imudrv.cntProbedSc = id;

  return result;
}


static int
ImudrvProbeStub(int bus, struct _stImudrvList *p)
{
  int           result = IMUDRV_ERRNO_NOMATCH;
  int           re;
  struct _stSystemSpiParam spiParam;
  struct _stSystemI2cParam i2cParam;

  // spi bus
  const uint8_t *pPat;
  int           cntPat;
  int           cntTx;
  uint8_t       bufTx[32];
  int           cntRx;
  uint8_t       bufRx[32];
  int           match;
  //uint8_t       addr;

  re = p->probe(bus);
  if(re < 0) {
    result = IMUDRV_ERRNO_NOMATCH;
    goto fail;
  }

  pPat   = p->pPatternList;
  cntPat = p->cntPatternList;
  if(pPat != NULL) {

    match = 1;
    for(int i = 0; i < cntPat; i++) {
      cntTx = *pPat++;
      //addr = pPat[0];
      bufTx[0] = pPat[0];
      bufTx[1] = pPat[1];
      pPat += cntTx;
      cntRx = *pPat++;

      if(       ((bus >> 12) & 0xf) == 0) {          // I2C
        re = SystemI2cTransfer((bus >> 8) & 0xf, bus & 0xff, bufTx, cntTx, bufRx, cntRx, &i2cParam);

      } else if(((bus >> 12) & 0xf) == 1) {          // SPI
        bufTx[0] |= 0x80;
        bufTx[1] = 0;
        bufTx[2] = 0;
        spiParam = p->spiParam;
        spiParam.speed = CONFIG_SPI_SPEED_DEFAULT;        // low speed, when probe phase
        re = SystemSpiTransfer((bus >> 8) & 0xf, bus & 0x7f, bufTx, cntTx, bufRx, cntRx, &spiParam);
      }
      //Serial.printf("probe %x: addr:%x rx:%x %x %x\n", bus, 0, bufRx[0], bufRx[1], bufRx[2]);
      if(re < 1 || memcmp(pPat, bufRx, cntRx)) {
        match = 0;
        break;
      }
      pPat += cntRx;
      i += 2 + cntTx + cntRx;
    }
    if(match) {
      result = IMUDRV_SUCCESS;
    }
  }

fail:
  return result;

}


//
// initizlise
//
int
ImudrvInit(int id, int powermode, int accfsr, int gyrfsr, int odr)
{
  int           result = IMUDRV_ERRNO_NOTATTACHED;

  if(id < 0) {
    memset(&imudrv, 0, sizeof(imudrv));
    memset(imudrv.idByIntr, 0xff, sizeof(imudrv.idByIntr));

  } else {

    struct _stProbedSc          *p;
    p = &imudrv.sc[id];
    if(p->pDriver->init) {
      if(ImudrvGetTypeDev(p->pDriver->type) == IMUDRV_TYPE_DEV_IMU) {
        Serial.printf("#       init acc=%d, gry=%d, odr=%d\n", accfsr, gyrfsr, odr);
      }
      result = p->pDriver->init(id, p, powermode, accfsr, gyrfsr, odr);
    }

  }

  return result;
}


int
ImudrvLoop(void)
{
  struct _stProbedSc          *pSc;

  for(int id = 0; id < imudrv.cntProbedSc; id++) {
    pSc = &imudrv.sc[id];
    if(pSc->pDriver->loop) pSc->pDriver->loop(id, pSc);
  }

  return 0;
}


int
ImudrvStart(int id, int odr)
{
  int           result = IMUDRV_ERRNO_NOTATTACHED;
  struct _stProbedSc          *pSc;
  pSc = &imudrv.sc[id];

  if(pSc->pDriver->start) result = pSc->pDriver->start(id, pSc, odr);

  return result;
}


int
ImudrvStop(int id)
{
  int           result = IMUDRV_ERRNO_NOTATTACHED;
  struct _stProbedSc          *pSc;
  pSc = &imudrv.sc[id];

  if(pSc->pDriver->stop) result = pSc->pDriver->stop(id, pSc);

  return result;
}


float
ImudrvConvTmprFloat(int id, int32_t val)
{
  float         tmpr;
  struct _stProbedSc          *pSc;
  pSc = &imudrv.sc[id];

  tmpr = (float)val / pSc->pDriver->tmprDiv + pSc->pDriver->tmprOffset;

  return tmpr;
}


uint32_t
ImudrvGetDataRate(int id)
{
  struct _stProbedSc          *pSc;
  pSc = &imudrv.sc[id];

  return pSc->pDriver->dataRate;
}


void
ImudrvSetCb(void (*cb)(struct _stImuValue *p))
{
  imudrv.pCb = cb;

  return;
}
int
ImudrvGoCb(int id, struct _stProbedSc *pSc, struct _stImuValue *p)
{
  if(imudrv.pCb) {

    p->seq     = pSc->seq++;
    p->id      = pSc->id;
    p->subId   = pSc->subId;

    if(ImudrvGetInterruptPin(pSc->bus) >= 0) {
      p->ts      = imudrv.tIntrById[id];            // fill timestamp
      p->ts_1MHz = imudrv.tIntr1MHzById[id];        // fill internal 1MHz timestamp
    } else {
      PtpGetUnixTime(&p->ts);
      p->ts_1MHz = SystemGetCounter1MHz();
    }

    // fill float data, if it is not exist
    if(!(p->flag & IMUDRV_FLAG_FILLED_FLOAT) &&
       ((imuFmtUart == IMUDRV_FLAG_FILLED_FLOAT) || (imuFmtIp == IMUDRV_FLAG_FILLED_FLOAT))) {
      p->axf =  (float)p->ax * pSc->accelFactor;
      p->ayf =  (float)p->ay * pSc->accelFactor;
      p->azf =  (float)p->az * pSc->accelFactor;

      p->gxf =  (float)p->gx * pSc->gyroFactor;
      p->gyf =  (float)p->gy * pSc->gyroFactor;
      p->gzf =  (float)p->gz * pSc->gyroFactor;

      p->tempf = pSc->tempOffset + (float)p->temp * pSc->tempMul;

    }
    // fill S16 data, if it is not exist
    if(!(p->flag & IMUDRV_FLAG_FILLED_S16)) {
      p->ax =  0;
      p->ay =  0;
      p->az =  0;

      p->gx =  0;
      p->gy =  0;
      p->gz =  0;

      p->temp = 0;

    }

    imudrv.pCb(p);
  }

  return 0;
}


int
ImudrvRead(int id, int addr, uint8_t *ptr, int len)
{
  int                   re = -1;
  struct _stProbedSc    *p;

  p = &imudrv.sc[id];
  re = ImudrvReadBus(p->bus, addr, ptr, len, p);

  return re;
}
int
ImudrvReadSc(struct _stProbedSc *p, int addr, uint8_t *ptr, int len)
{
  int           re = -1;

  re = ImudrvReadBus(p->bus, addr, ptr, len, p);

  return re;
}
int
ImudrvReadBus(int bus, int addr, uint8_t *ptr, int len, struct _stProbedSc *p)
{
  int           re = -1;
  uint8_t       buf[4];

  if((bus & IMUDRV_BUS_IF_MASK) == (IMUDRV_BUS_IF_I2C)) {
    buf[0] = addr;
    re = SystemI2cTransfer((bus >> 8) & 0xf, bus & 0xff, buf, 1, ptr, len, &p->i2cParam);

  } else if((bus & IMUDRV_BUS_IF_MASK) == (IMUDRV_BUS_IF_SPI)) {
    if(addr >= 0) {
      buf[0] = addr | 0x80;
      re = SystemSpiTransfer((bus >> 8) & 0xf, bus & 0xff, buf, 1, ptr, len, &p->spiParam);
    } else {
      // recv without pos tx
      re = SystemSpiTransfer((bus >> 8) & 0xf, bus & 0xff, NULL, 0, ptr, len, &p->spiParam);
    }
  }

  return re;
}


int
ImudrvWrite(int id, int addr, uint8_t *ptr, int len)
{
  int                   re = -1;
  struct _stProbedSc    *p;

  p = &imudrv.sc[id];
  re = ImudrvWriteBus(p->bus, addr, ptr, len, p);

  return re;
}
int
ImudrvWriteSc(struct _stProbedSc *p, int addr, uint8_t *ptr, int len)
{
  int           re = -1;

  re = ImudrvWriteBus(p->bus, addr, ptr, len, p);

  return re;
}
int
ImudrvWriteBus(int bus, int addr, uint8_t *ptr, int len, struct _stProbedSc *p)
{
  int           re = -1;
  uint8_t       buf[16];
  int           cnt;

  cnt = len;
  if(cnt > sizeof(buf)) cnt = sizeof(buf);

  if((bus & IMUDRV_BUS_IF_MASK) == (IMUDRV_BUS_IF_I2C)) {
    re = SystemI2cTransfer((bus >> 8) & 0xf, addr & 0xff, buf, cnt, NULL, 0, &p->i2cParam);

  } else if((bus & IMUDRV_BUS_IF_MASK) == (IMUDRV_BUS_IF_SPI)) {
    for(int i = 0; i < cnt; i++) {
      buf[i] = ptr[i];
    }
    re = SystemSpiTransfer((bus >> 8) & 0xf, addr & 0x7f, buf, cnt, NULL, 0, &p->spiParam);

  }

  return re;
}


int
ImudrvSetConfig(int id, const uint8_t *ptr, int count)
{
  struct _stProbedSc    *p;

  p = &imudrv.sc[id];
  return ImudrvSetConfigBus(p->bus, ptr, count);
}
int
ImudrvSetConfigSc(struct _stProbedSc *p, const uint8_t *ptr, int count)
{
  return ImudrvSetConfigBus(p->bus, ptr, count);
}
int
ImudrvSetConfigBus(int bus, const uint8_t *ptr, int count)
{
  uint8_t       buf[2];

  for(int i = 0; i < count; i++) {
    if(*ptr == 0xff) {
      SystemWaitCounter(ptr[1]);
    } else {
      buf[0] = ptr[0];
      buf[1] = ptr[1];
      switch(bus & IMUDRV_BUS_IF_MASK) {
        //case IMUDRV_BUS_IF_I2C: SystemI2cTransfer((bus >> 8) & 0xf, bus & 0xff, buf, 2, NULL, 0, &p->i2cParam); break;
        //case IMUDRV_BUS_IF_SPI: SystemSpiTransfer((bus >> 8) & 0xf, bus & 0xff, buf, 2, NULL, 0, &p->spiParam); break;
      case IMUDRV_BUS_IF_I2C: SystemI2cTransfer((bus >> 8) & 0xf, bus & 0xff, buf, 2, NULL, 0, NULL); break;
      case IMUDRV_BUS_IF_SPI: SystemSpiTransfer((bus >> 8) & 0xf, bus & 0xff, buf, 2, NULL, 0, NULL); break;
      }
    }
    ptr += 2;
  }

  return count;
}
int
ImudrvSetConfig16(int id, const uint8_t *ptr, int count)
{
  struct _stProbedSc    *p;

  p = &imudrv.sc[id];
  return ImudrvSetConfig16Bus(p->bus, ptr, count, NULL);
}
int
ImudrvSetConfig16Sc(struct _stProbedSc *p, const uint8_t *ptr, int count)
{
  return ImudrvSetConfig16Bus(p->bus, ptr, count, NULL);
}
int
ImudrvSetConfig16Bus(int bus, const uint8_t *ptr, int count, void *pParam)
{
  uint8_t       buf[2];

  for(int i = 0; i < count; i++) {
    if(*ptr == 0xff) {
      SystemWaitCounter(ptr[1]);
    } else {
      buf[0] = ptr[0];
      buf[1] = ptr[1];
      buf[2] = ptr[2];
      switch(bus & IMUDRV_BUS_IF_MASK) {
        //case IMUDRV_BUS_IF_I2C: SystemI2cTransfer((bus >> 8) & 0xf, bus & 0xff, buf, 2, NULL, 0, &p->i2cParam); break;
        //case IMUDRV_BUS_IF_SPI: SystemSpiTransfer((bus >> 8) & 0xf, bus & 0xff, buf, 2, NULL, 0, &p->spiParam); break;
      case IMUDRV_BUS_IF_I2C: SystemI2cTransfer((bus >> 8) & 0xf, bus & 0xff, buf, 3, NULL, 0, pParam); break;
      case IMUDRV_BUS_IF_SPI: SystemSpiTransfer((bus >> 8) & 0xf, bus & 0xff, buf, 3, NULL, 0, pParam); break;
      }
    }
    ptr += 3;
  }

  return count;
}


static int
ImudrvGetInterruptPin(int bus)
{
  int           intr = -1;

  switch(bus) {
  case  0x0010:        // I2C0 0x10             // spresense imu adhoc
  case  0x1100:        // SPI1 CS0X
    intr = CONFIG_GPIO_IMU_DRDY10; break;
  case  0x1101:        // SPI1 CS1X
    intr = CONFIG_GPIO_IMU_DRDY11; break;
  case  0x1102:        // SPI1 CS2X
    intr = CONFIG_GPIO_IMU_DRDY12; break;
  case  0x1103:        // SPI1 CS3X
    intr = CONFIG_GPIO_IMU_DRDY13; break;
  case  0x1000:        // SPI0 CS0X
    intr = CONFIG_GPIO_IMU_DRDY00; break;
  case  0x1001:        // SPI0 CS1X
    intr = CONFIG_GPIO_IMU_DRDY01; break;
  case  0x1002:        // SPI0 CS2X
    intr = CONFIG_GPIO_IMU_DRDY02; break;
  case  0x1003:        // SPI0 CS3X
    intr = CONFIG_GPIO_IMU_DRDY03; break;
  }

  return intr;
}


int
ImudrvEnableInterrupt(int intr)
{
  void (*entry)() = NULL;

  switch(intr) {
  case  CONFIG_GPIO_IMU_DRDY10: entry = ImudrvInterruptDrdy10; break;
  case  CONFIG_GPIO_IMU_DRDY11: entry = ImudrvInterruptDrdy11; break;
  case  CONFIG_GPIO_IMU_DRDY12: entry = ImudrvInterruptDrdy12; break;
  case  CONFIG_GPIO_IMU_DRDY13: entry = ImudrvInterruptDrdy13; break;
  case  CONFIG_GPIO_IMU_DRDY00: entry = ImudrvInterruptDrdy00; break;
  case  CONFIG_GPIO_IMU_DRDY01: entry = ImudrvInterruptDrdy01; break;
  case  CONFIG_GPIO_IMU_DRDY02: entry = ImudrvInterruptDrdy02; break;
  case  CONFIG_GPIO_IMU_DRDY03: entry = ImudrvInterruptDrdy03; break;
  }

  if(entry != NULL) {
    attachInterrupt(intr, entry, RISING);
  }

  return -1;
}
int
ImudrvDisableInterrupt(int intr)
{
  detachInterrupt(intr);

  return 0;
}


FASTRUN static void
ImudrvInterruptDrdy10(void)
{
#if     CONFIG_IMU_CALC_TIME_PULSE
  if((imudrv.idByIntr[CONFIG_GPIO_IMU_DRDY10]) >= 0) {
    digitalWrite(CONFIG_IMU_CALC_TIME_PULSE, 1);
  }
#endif
  ImudrvInterruptDrdy(CONFIG_GPIO_IMU_DRDY10);

  return;
}
FASTRUN static void
ImudrvInterruptDrdy11(void)
{
  ImudrvInterruptDrdy(CONFIG_GPIO_IMU_DRDY11);
  return;
}
FASTRUN static void
ImudrvInterruptDrdy12(void)
{
  ImudrvInterruptDrdy(CONFIG_GPIO_IMU_DRDY12);
}
FASTRUN static void
ImudrvInterruptDrdy13(void)
{
  ImudrvInterruptDrdy(CONFIG_GPIO_IMU_DRDY13);
}
FASTRUN static void
ImudrvInterruptDrdy00(void)
{
  ImudrvInterruptDrdy(CONFIG_GPIO_IMU_DRDY00);
}
FASTRUN static void
ImudrvInterruptDrdy01(void)
{
  ImudrvInterruptDrdy(CONFIG_GPIO_IMU_DRDY01);
}
FASTRUN static void
ImudrvInterruptDrdy02(void)
{
  ImudrvInterruptDrdy(CONFIG_GPIO_IMU_DRDY02);
}
FASTRUN static void
ImudrvInterruptDrdy03(void)
{
  ImudrvInterruptDrdy(CONFIG_GPIO_IMU_DRDY03);
}
FASTRUN static void
ImudrvInterruptDrdy(int numDrdy)
{
  struct _stProbedSc    *p;
  int                   id;

  if((id = imudrv.idByIntr[numDrdy]) >= 0) {
    PtpGetUnixTime(&imudrv.tIntrById[id]);
    imudrv.tIntr1MHzById[id] = SystemGetCounter1MHz();
    p = &imudrv.sc[id];
    if(p->pDriver->intr) p->pDriver->intr(id, p);
  }

  return;
}


int
ImudrvGetNumOfSensors(void)
{
  return imudrv.cntProbedSc;
}


static const uint8_t *
ImudrvGetSensorTypeTxt(uint8_t type)
{
  const uint8_t       *p;

  switch(type) {
  case        IMUDRV_TYPE_DEV_IMU:        p = txtIMU; break;
  case        IMUDRV_TYPE_DEV_MAGNETICS:  p = txtMAG; break;
  default:                                p = txtUKN; break;
  }

  return p;
}
