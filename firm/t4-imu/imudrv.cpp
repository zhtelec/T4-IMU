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
        if(bus < 0x0100) {       // if = I2C0

          if((bus & 0xff) < 0x80) {
            result = ImudrvProbeStub(bus, p);
            if(result == IMUDRV_SUCCESS) break;

          } else {
            bus = 0xff;
          }

        } else if(bus < 0x1000) {       // if = I2C1, 2, 3
          result = IMUDRV_ERRNO_UNKNOWN;
          bus = 0xfff;   // skip

        } else if(bus < 0x2000) {       // if = SPI
          result = ImudrvProbeStub(bus, p);
          if(result == IMUDRV_SUCCESS) break;

          if((bus & 0x00ff) >= 15) bus |= 0xff;
        }
      }

      if(result == IMUDRV_SUCCESS) {
        if(((bus >> 12) & 0xf) == 1) {
          Serial.printf("# IMU%d: SPI%d CS%x device %s\n",
                        id, (bus >> 8) & 0xf, bus & 0xff, p->name);
        } else if(((bus >> 12) & 0xf) == 0) {
          Serial.printf("# IMU%d: I2C%d(0x%02x), device %s\n",
                        id, (bus >> 8) & 0xf, bus & 0xff, p->name);
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
        pSc->bus     = bus;
        pSc->pDriver = p;
        ImudrvInit(id,
                      0,                         // power mode
                      pSc->accfsr,       // accfsr
                      pSc->gyrfsr,       // gyrfsr
                      pSc->odr           // odr
                      );

        if((intr = ImudrvGetInterruptPin(bus)) >= 0) {
          Serial.printf("#       intr line: %d\n", intr);
          pSc->intrPort = intr;
          imudrv.idByIntr[intr] = id;
        }

        id++;
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
        spiParam = p->spiParam;
        spiParam.speed = CONFIG_SPI_SPEED_DEFAULT;        // low speed, when probe phase
        re = SystemSpiTransfer((bus >> 8) & 0xf, bus & 0x7f, bufTx, cntTx, bufRx, cntRx, &spiParam);
      }
      //Serial.printf("probe %x: addr:%x rx:%x %x\n", bus, 0, bufRx[0], bufRx[1]);
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
#if 0
    // fill the invalid value
    for(int i = 0; i < sizeof(imudrv.valueList)/sizeof(int16_t); i++) {
      imudrv.valueList[i] = IMUDRV_VALUE_INVALID;
    }
    imudrv.pImudrvList = imudrvList;
    imudrv.spiSpeed = CONFIG_IMUSPI_SPEED_DEFAULT;
#endif

    attachInterrupt(CONFIG_GPIO_IMU_DRDY10, ImudrvInterruptDrdy10, RISING);
    attachInterrupt(CONFIG_GPIO_IMU_DRDY11, ImudrvInterruptDrdy11, RISING);

  } else {

    struct _stProbedSc          *p;
    p = &imudrv.sc[id];
    if(p->pDriver->init) {
      Serial.printf("# IMU%d: init acc=%d, gry=%d, odr=%d\n", id, accfsr, gyrfsr, odr);
      result = p->pDriver->init(id, powermode, accfsr, gyrfsr, odr);
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

  if(pSc->pDriver->start) result = pSc->pDriver->start(id, odr);

  return result;
}


int
ImudrvStop(int id)
{
  int           result = IMUDRV_ERRNO_NOTATTACHED;
  struct _stProbedSc          *pSc;
  pSc = &imudrv.sc[id];

  if(pSc->pDriver->stop) result = pSc->pDriver->stop(id);

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

    p->ts      = imudrv.tIntrById[id];            // fill timestamp
    p->ts_1MHz = imudrv.tIntr1MHzById[id];        // fill internal 1MHz timestamp

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
ImudrvSpiReadBus(int bus, int addr, uint8_t *ptr, int len, struct _stSystemSpiParam *pParam)
{
  int           re;
  uint8_t       buf[4];

  if(addr >= 0) {
    buf[0] = addr | 0x80;
    re = SystemSpiTransfer((bus >> 8) & 0xf, bus & 0xff, buf, 1, ptr, len, pParam);
  } else {
    // recv without pos tx
    re = SystemSpiTransfer((bus >> 8) & 0xf, bus & 0xff, NULL, 0, ptr, len, pParam);
  }

  return re;
}
int
ImudrvSpiRead(int id, int addr, uint8_t *ptr, int len)
{
  int                   re;
  int                   bus;
  struct _stProbedSc    *pSc;
  uint8_t               buf[32];

  pSc = &imudrv.sc[id];
  bus = pSc->bus;

  buf[0] = addr | 0x80;


  re = SystemSpiTransfer((bus >> 8) & 0xf, bus & 0xff, buf, 1, ptr, len, &pSc->spiParam);

  return re;
}

int
ImudrvSpiWrite(int id, int addr, uint8_t *ptr, int len)
{
  int                   bus;
  struct _stProbedSc    *pSc;
  uint8_t               buf[32];

  pSc = &imudrv.sc[id];
  bus = pSc->bus;

  buf[0] = addr;
  if(len > (int)sizeof(buf)-1) len = sizeof(buf)-1;
  memcpy(buf+1, ptr, len);

  SystemSpiTransfer((bus >> 8) & 0xf, bus & 0xff, buf, len+1, NULL, 0, &pSc->spiParam);

  return len;
}


int
ImudrvSpiWriteNoAddr(int id, uint8_t *ptr, int len)
{
  SystemSpiTransfer(1, 0, ptr, len, NULL, 0, NULL);

  return len;
}

int
ImudrvSpiSetConfigBus(int bus, const uint8_t *ptr, int count)
{
  for(int i = 0; i < count; i++) {
    if(*ptr == 0xff) {
      SystemWaitCounter(ptr[1]);
    } else {
      SystemSpiTransfer((bus >> 8) & 0xf, bus & 0xff, ptr, 2, NULL, 0, NULL);
    }
    ptr += 2;
  }

  return count;
}
int
ImudrvSpiSetConfig(int id, const uint8_t *ptr, int count)
{
  int   bus;
  struct _stProbedSc          *pSc;

  pSc = &imudrv.sc[id];
  bus = pSc->bus;

  for(int i = 0; i < count; i++) {
    if(ptr[0] == 0xff) {
      SystemWaitCounter(ptr[1]);
    } else {
      SystemSpiTransfer((bus >> 8) & 0xf, bus & 0xff, ptr, 2, NULL, 0, &pSc->spiParam);
    }
    ptr += 2;
  }

  return count;
}


int
ImudrvI2cSetConfig(int id, const uint8_t *ptr, int count)
{
  int   bus;
  struct _stProbedSc          *pSc;

  pSc = &imudrv.sc[id];
  bus = pSc->bus;

  for(int i = 0; i < count; i++) {
    if(ptr[0] == 0xff) {
      SystemWaitCounter(ptr[1]);
    } else {
      SystemI2cTransfer((bus >> 8) & 0xf, bus & 0xff, ptr, 2, NULL, 0, &pSc->i2cParam);
    }
    ptr += 2;
  }

  return count;
}


static int
ImudrvGetInterruptPin(int bus)
{
  int           intr = -1;

  switch(bus) {
  case  0x0010:        // I2C0 0x10
  case  0x1100:        // SPI1 CS0X
    intr = CONFIG_GPIO_IMU_DRDY10; break;
  case  0x1101:        // SPI1 CS1X
    intr = CONFIG_GPIO_IMU_DRDY11; break;
  }

  return intr;
}


static void
ImudrvInterruptDrdy10(void)
{
  struct _stProbedSc    *p;
  int                   id;

  if((id = imudrv.idByIntr[CONFIG_GPIO_IMU_DRDY10]) >= 0) {
#if     CONFIG_IMU_CALC_TIME_PULSE
    digitalWrite(CONFIG_IMU_CALC_TIME_PULSE, 1);
#endif
    PtpGetUnixTime(&imudrv.tIntrById[id]);
    imudrv.tIntr1MHzById[id] = SystemGetCounter1MHz();
    p = &imudrv.sc[id];
    if(p->pDriver->intr) p->pDriver->intr(id, p);
  }

  return;
}
static void
ImudrvInterruptDrdy11(void)
{
  struct _stProbedSc    *p;
  int                   id;

  if((id = imudrv.idByIntr[CONFIG_GPIO_IMU_DRDY11]) >= 0) {
    PtpGetUnixTime(&imudrv.tIntrById[id]);
    imudrv.tIntr1MHzById[id] = SystemGetCounter1MHz();
    p = &imudrv.sc[id];
    if(p->pDriver->intr) p->pDriver->intr(id, p);
  }

  return;
}
