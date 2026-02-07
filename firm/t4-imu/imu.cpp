#define _IMU_CPP_

#include        <Arduino.h>

#include        <QNEthernet.h>

#include        "config.h"
#include        "system.h"


#include        "imudrv.h"
#include        "eeprom.h"
#include        "network.h"
#include        "packet.h"
#include        "sdlog.h"
#include        "fifo.h"

#include        "imu.h"

struct _stImu {
  int           numOfSensors;

  int           imuFmtUart;
  int           imuFmtIp;

  int           dFifo;
#if CONFIG_SENSOR_FIFO_USE_MALLOC
  uint8_t       bufFifo[2^CONFIG_SENSOR_FIFO_SIZE];
#endif

  int           debug = 0;
#define IMU_DEBUG_SHOW_ALL      (1<<0)
};
struct _stImu           imu;


int              imuFmtUart;
int              imuFmtIp;



void
ImuInit(void)
{
  uint32_t      val;

  memset((void *)&imu, 0, sizeof(imu));
  imu.dFifo             = -1;
  imu.imuFmtUart        = IMUDRV_FORMAT_DISABLE;
  imu.imuFmtIp          = IMUDRV_FORMAT_FLOAT;

  digitalWrite(CONFIG_GPIO_IMU_POWER,  1);
  SystemWaitUsCounter(1000 * SYSTEM_COUNTER_US1U000S);
  digitalWrite(CONFIG_GPIO_IMU_RESETX, 1);
  SystemWaitUsCounter(150 * SYSTEM_COUNTER_US1M000S);

#if CONFIG_SENSOR_FIFO_USE_MALLOC
  imu.dFifo = FifoCreate(CONFIG_SENSOR_FIFO_SIZE);
#else
  imu.dFifo = FifoCreateWithBuf(CONFIG_SENSOR_FIFO_SIZE, imu.bufFifo);
#endif

  ImudrvInit(-1, 0, 0, 0, 0);
  ImudrvProbe();
  ImudrvSetCb(ImuSendValue);

  val = EepromGet16(CONFIG_EEPROM_IMU_PARAM_POS);
  imu.imuFmtUart       = (val >> CONFIG_EEPROM_IMU_PARAM_UART_SHIFT)      & 0x3;
  imu.imuFmtIp         = (val >> CONFIG_EEPROM_IMU_PARAM_IP_SHIFT) & 0x3;
  imuFmtUart = imu.imuFmtUart;
  imuFmtIp   = imu.imuFmtIp;

  imu.numOfSensors = ImudrvGetNumOfSensors();

  return;
}


void
ImuLoop(void)
{
  uint8_t     buf[512];
  int         size = 0;

  ImudrvLoop();

#if CONFIG_SENSOR_USE_FIFO
  if(FifoGetDirtyLen(imu.dFifo) >= CONFIG_SENSOR_FIFO_THRESHOLD * imu.numOfSensors) {
    size = FifoReadOut(imu.dFifo, (char *)buf, sizeof(buf));
    if(size > 0)  {
      NetworkSendUdp(buf, size);
    }
  }
#endif

  return;
}


void
ImuStart(void)
{
  for(int i = 0; i < 1; i++) {
    ImudrvStart(i, 0);
  }

  return;
}


void
ImuCommand(int ac, char *av[])
{
  int           update = 0;
  uint32_t      val;
  uint8_t       val8;
  int           accfsr, gyrfsr, odr;

  val      = EepromGet16(CONFIG_EEPROM_IMU_PARAM_POS);
  accfsr   = val & CONFIG_EEPROM_IMU_PARAM_ACC_MASK;
  accfsr >>= CONFIG_EEPROM_IMU_PARAM_ACC_SHIFT;
  gyrfsr   = val & CONFIG_EEPROM_IMU_PARAM_GYR_MASK;
  gyrfsr >>= CONFIG_EEPROM_IMU_PARAM_GYR_SHIFT;
  odr      = val & CONFIG_EEPROM_IMU_PARAM_ODR_MASK;
  odr    >>= CONFIG_EEPROM_IMU_PARAM_ODR_SHIFT;

  if(       !strcmp(av[1], "param")) {
    if(ac == 2) {
      Serial.printf("accfsr 0x%x, gyrfsr 0x%x, odr 0x%x\n", accfsr, gyrfsr, odr);

    } if(ac >= 5) {
      accfsr  = strtoul(av[2], NULL, 0);
      gyrfsr  = strtoul(av[3], NULL, 0);
      odr     = strtoul(av[4], NULL, 0);
      val  = EepromGet16(CONFIG_EEPROM_IMU_PARAM_POS);
      val &= ~(CONFIG_EEPROM_IMU_PARAM_ODR_MASK | CONFIG_EEPROM_IMU_PARAM_GYR_MASK | CONFIG_EEPROM_IMU_PARAM_ACC_MASK);
      val |= (accfsr << CONFIG_EEPROM_IMU_PARAM_ACC_SHIFT) & CONFIG_EEPROM_IMU_PARAM_ACC_MASK;
      val |= (gyrfsr << CONFIG_EEPROM_IMU_PARAM_GYR_SHIFT) & CONFIG_EEPROM_IMU_PARAM_GYR_MASK;
      val |= (odr    << CONFIG_EEPROM_IMU_PARAM_ODR_SHIFT) & CONFIG_EEPROM_IMU_PARAM_ODR_MASK;
      EepromSet16(CONFIG_EEPROM_IMU_PARAM_POS, val);
      EepromBurn();

      update = 1;
    }

  } else if(       !strcmp(av[1], "afsr")) {
    if(ac == 2) {
      Serial.printf("0x%x\n", accfsr);

    } else if(ac >= 3) {
      accfsr  = strtoul(av[2], NULL, 0);
      val  = EepromGet16(CONFIG_EEPROM_IMU_PARAM_POS);
      val &= ~CONFIG_EEPROM_IMU_PARAM_ACC_MASK;
      val |= (accfsr << CONFIG_EEPROM_IMU_PARAM_ACC_SHIFT) & CONFIG_EEPROM_IMU_PARAM_ACC_MASK;
      EepromSet16(CONFIG_EEPROM_IMU_PARAM_POS, val);
      EepromBurn();

      update = 1;
    }

  } else if(!strcmp(av[1], "gfsr")) {
    if(ac == 2) {
      Serial.printf("0x%x\n", gyrfsr);

    } else if(ac >= 3) {
      gyrfsr  = strtoul(av[2], NULL, 0);
      val  = EepromGet16(CONFIG_EEPROM_IMU_PARAM_POS);
      val &= ~CONFIG_EEPROM_IMU_PARAM_GYR_MASK;
      val |= (gyrfsr << CONFIG_EEPROM_IMU_PARAM_GYR_SHIFT) & CONFIG_EEPROM_IMU_PARAM_GYR_MASK;
      EepromSet16(CONFIG_EEPROM_IMU_PARAM_POS, val);
      EepromBurn();

      update = 1;
    }

  } else if(!strcmp(av[1], "odr")) {
    if(ac == 2) {
      Serial.printf("0x%x\n", odr);

    } else if(ac >= 3) {
      odr  = strtoul(av[2], NULL, 0);
      val  = EepromGet16(CONFIG_EEPROM_IMU_PARAM_POS);
      val &= ~CONFIG_EEPROM_IMU_PARAM_ODR_MASK;
      val |= (odr << CONFIG_EEPROM_IMU_PARAM_ODR_SHIFT) & CONFIG_EEPROM_IMU_PARAM_ODR_MASK;
      EepromSet16(CONFIG_EEPROM_IMU_PARAM_POS, val);
      EepromBurn();

#if 0
      val = atoi(av[2]) * 256 / 100;
      set = 0;
      if(val) {
        for(set = 0; set < 16; set++) {
          if(val & 1) break;
          val >>= 1;
        }
      }
      if(set >= 16) set = 0;
#endif

      update = 1;
    }

  } else if(!strcmp(av[1], "speed")) {
    if(ac == 2) {
      val   = EepromGet8(CONFIG_EEPROM_IMU_PARAM2_POS);
      val  &= CONFIG_EEPROM_IMU_PARAM2_SPEED_MASK;
      val >>= CONFIG_EEPROM_IMU_PARAM2_SPEED_SHIFT;
      Serial.printf("0x%x\n", val);

    } else if(ac >= 3) {
      val    = atoi(av[2]) & CONFIG_EEPROM_IMU_PARAM2_SPEED_MASK;
      val  <<= CONFIG_EEPROM_IMU_PARAM2_SPEED_SHIFT;
      val8   = EepromGet8(CONFIG_EEPROM_IMU_PARAM2_POS);
      val8  &= ~CONFIG_EEPROM_IMU_PARAM2_SPEED_MASK;
      val8  |= val;
      EepromSet8(CONFIG_EEPROM_IMU_PARAM2_POS, val8);
      EepromBurn();
    }

  } else if(!strcmp(av[1], "show")) {
    if(av[2][0] == '1') {
      imu.debug |=  IMU_DEBUG_SHOW_ALL;
    } else {
      imu.debug &= ~IMU_DEBUG_SHOW_ALL;
    }

  } else if(!strcmp(av[1], "regget")) {
    uint8_t     buf[16];
    int         id;
    uint32_t    addr;
    int         cnt = 1;
    if(ac >= 5) cnt = atoi(av[4]);
    if(cnt > sizeof(buf)) cnt = sizeof(buf);
    if(ac >= 4) {
      id = atoi(av[2]);
      addr = strtoul(av[3], NULL, 16);
      ImudrvRead(id, addr, buf, cnt);
      Serial.printf("id%d.%02x: ", id, addr);
      for(int i = 0; i < cnt; i++) {
        if(i && !(i & 3)) Serial.printf(" ");
        Serial.printf(" %02x", buf[i]);
      }
      Serial.printf("\n");
    }

  } else if(!strcmp(av[1], "regset")) {
    uint8_t     buf[8];
    int         id;
    uint32_t    val;

    if(ac >= 5) {
      id = atoi(av[2]);
      buf[0] = strtoul(av[3], NULL, 16);
      val = strtoul(av[4], NULL, 16);
      buf[1] = val;
      ImudrvSetConfig(id, buf, 1);
    }

  } else if(!strcmp(av[1], "regset16")) {
    uint8_t     buf[8];
    int         id;
    uint32_t    val;

    if(ac >= 5) {
      id = atoi(av[2]);
      buf[0] = strtoul(av[3], NULL, 16);
      val = strtoul(av[4], NULL, 16);
      buf[1] =  val       & 0xff;
      buf[2] = (val >> 8) & 0xff;
      ImudrvSetConfig16(id, buf, 1);
    }

  }

  if(update) {
    int         id;
    id = 0;
    ImudrvInit(id, 0, accfsr, gyrfsr, odr);

  }

  return;
}


static void
ImuSendValue(struct _stImuValue *p)
{
  if(p->ts.tv_sec >= 5) {
    switch(ImudrvGetTypeDev(p->type)) {
    case      IMUDRV_TYPE_DEV_MAGNETICS: ImuSendValueMagnetics(p); break;
    case      IMUDRV_TYPE_DEV_IMU:       ImuSendValueImu(p);       break;
    //default:                             ImuSendValueImu(p);       break;
    }
  }

  return;
}


static void
ImuSendValueImu(struct _stImuValue *p)
{
  if(imu.imuFmtIp == IMUDRV_FORMAT_S16) {

  } else if(imu.imuFmtIp == IMUDRV_FORMAT_FLOAT) {
    struct _stPacketImuFloat  pkt;

    // header
    pkt.hdr.type     = PACKET_TYPE_IMU_FLOAT;
    pkt.hdr.id       = p->subId;
    pkt.hdr.seq      = p->seq;
    //pkt.hdr.len      = sizeof(pkt);
    pkt.hdr.tai_secH = (p->ts.tv_sec >> 32) & 0xff;
    pkt.hdr.tai_sec  =  p->ts.tv_sec;
    pkt.hdr.tai_nsec =  p->ts.tv_nsec;
    pkt.hdr.ts_1MHz  = SystemGetCounter1MHz();

    // data
    pkt.acc[0]  = p->axf;
    pkt.acc[1]  = p->ayf;
    pkt.acc[2]  = p->azf;
    pkt.gyr[0]  = p->gxf;
    pkt.gyr[1]  = p->gyf;
    pkt.gyr[2]  = p->gzf;
    pkt.temp    = p->tempf;
    pkt.tsChip  = p->tsChip;

    PacketCalcSumAndFillHeader((struct _stPacketGeneric *)&pkt, sizeof(pkt));

#if CONFIG_SENSOR_USE_FIFO
    FifoWriteIn(imu.dFifo, (char *)&pkt, sizeof(pkt));
#else
    NetworkSendUdp((uint8_t *)&pkt, sizeof(pkt));
#endif

    if(SystemGetBoardId() == CONFIG_BOARDID_T4_IMU) {
      char buf[256];
      int pos = 0;
      pos  = sprintf(buf,     "%09lld.%09ld", p->ts.tv_sec, p->ts.tv_nsec);
      pos += sprintf(buf+pos, " %08lx %08lx %d", p->tsChip, p->ts_1MHz, p->id);
      pos += sprintf(buf+pos, " %f %f %f %f %f %f %.1f\n",
                     p->axf, p->ayf, p->azf,
                     p->gxf, p->gyf, p->gzf, p->tempf);
      SdlogWrite(buf);
    }
  }

#if     CONFIG_IMU_CALC_TIME_PULSE
  digitalWrite(CONFIG_IMU_CALC_TIME_PULSE, 0);
#endif

  if(imu.debug & IMU_DEBUG_SHOW_ALL) {
    if(imu.imuFmtUart == IMUDRV_FORMAT_S16 || imu.imuFmtIp == IMUDRV_FORMAT_S16) {

      Serial.printf("%09lld.%09ld", p->ts.tv_sec, p->ts.tv_nsec);
      Serial.printf("  %8lx %8lx %2d", p->tsChip, p->ts_1MHz, p->id);
      Serial.printf("  %04x %04x %04x  %04x %04x %04x  %04x\n",
                    p->ax & 0xffff, p->ay & 0xffff, p->az & 0xffff,
                    p->gx & 0xffff, p->gy & 0xffff, p->gz & 0xffff,  p->temp & 0xffff);

    } else if(imu.imuFmtUart == IMUDRV_FORMAT_FLOAT || imu.imuFmtIp == IMUDRV_FORMAT_FLOAT) {
      float val;
      Serial.printf("%09lld.%09ld", p->ts.tv_sec, p->ts.tv_nsec);
      Serial.printf("  %8lx %8lx %2d", p->tsChip, p->ts_1MHz, p->id);
      Serial.printf("  %5d %5d %5d (mm/s^2)  %6d %6d %6d (mdeg/s) %d (mDegC)",
                    (int)(p->axf*1000), (int)(p->ayf*1000), (int)(p->azf*1000),
                    (int)(p->gxf*1000), (int)(p->gyf*1000), (int)(p->gzf*1000), (int)(p->tempf*1000));
      val = p->axf*p->axf + p->ayf*p->ayf+ p->azf*p->azf;
      Serial.printf(" %f (m/s^2)\n", sqrt(val));

    }
  }

  return;
}


static void
ImuSendValueMagnetics(struct _stImuValue *p)
{
  if(imu.imuFmtIp == IMUDRV_FORMAT_S16) {
  } else if(imu.imuFmtIp == IMUDRV_FORMAT_FLOAT) {
    struct _stPacketImuFloat  pkt;

    // header
    pkt.hdr.type     = PACKET_TYPE_MAGNETIC_FLOAT;
    pkt.hdr.id       = p->subId;
    pkt.hdr.seq      = p->seq;
    //pkt.hdr.len      = sizeof(pkt);
    pkt.hdr.tai_secH = (p->ts.tv_sec >> 32) & 0xff;
    pkt.hdr.tai_sec  =  p->ts.tv_sec;
    pkt.hdr.tai_nsec =  p->ts.tv_nsec;
    pkt.hdr.ts_1MHz  = SystemGetCounter1MHz();

    // data
    pkt.acc[0]  = p->axf;
    pkt.acc[1]  = p->ayf;
    pkt.acc[2]  = p->azf;
    pkt.gyr[0]  = p->gxf;
    pkt.gyr[1]  = p->gyf;
    pkt.gyr[2]  = p->gzf;
    pkt.temp    = p->tempf;
    pkt.tsChip  = p->tsChip;

    PacketCalcSumAndFillHeader((struct _stPacketGeneric *)&pkt, sizeof(pkt));

#if CONFIG_SENSOR_USE_FIFO
    FifoWriteIn(imu.dFifo, (char *)&pkt, sizeof(pkt));
#else
    NetworkSendUdp((uint8_t *)&pkt, sizeof(pkt));
#endif

  }

#if     CONFIG_IMU_CALC_TIME_PULSE
  digitalWrite(CONFIG_IMU_CALC_TIME_PULSE, 0);
#endif

  if(imu.debug & IMU_DEBUG_SHOW_ALL) {
    if(imu.imuFmtUart == IMUDRV_FORMAT_FLOAT || imu.imuFmtIp == IMUDRV_FORMAT_FLOAT) {
      Serial.printf("%09lld.%09ld", p->ts.tv_sec, p->ts.tv_nsec);
      Serial.printf("  %8lx %8lx %2d", p->tsChip, p->ts_1MHz, p->id);
      Serial.printf("  %5d %5d %5d (uT)  %d (mDegC)\n",
                    (int)(p->axf*1000000), (int)(p->ayf*1000000), (int)(p->azf*1000000),
                    (int)(p->tempf*1000));

    }
  }

  return;
}
