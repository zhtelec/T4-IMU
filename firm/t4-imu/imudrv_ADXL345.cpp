#define __IMUDRIVER_ADXL345_POLL_CPP__


#include        <stdint.h>
#include        <Arduino.h>

#include        "config.h"
#include        "system.h"
#include        "debug.h"

#include        "imudrv.h"

#define PARAM_CHIPNAME                  "ADXL345"
#define PARAM_TYPE                      ((IMUDRV_TYPE_DEV_IMU) | (IMUDRV_TYPE_POLLING_YES))
#define PARAM_CHIPID_ADDR               0x00
#define PARAM_CHIPID_VALUE              0xe5
#define PARAM_SPI_FREQ_IN_KHZ           10000
#define PARAM_DATA_POSITION             0x32
#define PARAM_DATA_LENGTH               6
#define PARAM_FIFO_POSITION             0x42
#define PARAM_FIFO_LENGTH               6
#define PARAM_TEMPERATURE_DIV           256.0
#define PARAM_TEMPERATURE_OFFSET        25.0


extern struct _stImudrv         imudrv;

const static uint8_t    probePatternList[] = {
  1, PARAM_CHIPID_ADDR, 1, PARAM_CHIPID_VALUE,  // WHO_AM_I
};

#if 0
const static uint8_t  bufEnable[] = {
};
#endif
const static uint8_t  settings[] = {
  0x2d, 0x08,
  //0x2c, 0x0c,   // bw_rate
  //0x31, 0x08,   // data format (+-4G)
};


static int
probe(int bus)
{
  int           result = IMUDRV_ERRNO_NEEDNEXTCHECK;

  //ImudrvSetConfigBus(bus, bufEnable, sizeof(bufEnable)/2);

  return result;
}


static int
init(int id, struct _stProbedSc *p, int powermode, int accfsr, int gyrfsr, int odr)
{
  int           result = IMUDRV_ERRNO_UNKNOWN;
  uint8_t       buff[4];
  uint8_t       c;
  int           t;

  ImudrvSetConfigSc(p, settings, sizeof(settings)/2);

  buff[0] = 0x2c;       // BW_RATE
  switch(odr){
  case IMUDRV_ODR_13HZ:   c = 0x07; t = 33;             break; //   30Hz
  case IMUDRV_ODR_25HZ:   c = 0x08; t = 40;             break; //   25Hz
  case IMUDRV_ODR_50HZ:   c = 0x09; t = 20;             break; //   50Hz
  default:
  case IMUDRV_ODR_100HZ:  c = 0x0a; t = 10; odr = 0x08; break; //  100Hz
  case IMUDRV_ODR_200HZ:  c = 0x0b; t = 5;              break; //  200Hz
  case IMUDRV_ODR_400HZ:  c = 0x0c; t = 2;              break; //  400Hz
  case IMUDRV_ODR_800HZ:  c = 0x0d; t = 1;              break; //  800Hz
  case IMUDRV_ODR_1600HZ: c = 0x0e; t = 1;              break; // 1600Hz
  case IMUDRV_ODR_3200HZ: c = 0x0f; t = 1;              break; // 3200Hz
  }
  buff[1] = c;

  accfsr = 10;          // force set +-16G
  buff[2] = 0x31;       // DATA_FORMAT
  switch(accfsr){
  case IMUDRV_ACCFSR_2G:  c = 0x00;                break; // +-2G
  default:
  case IMUDRV_ACCFSR_4G:  c = 0x01; accfsr = 0x08; break; // +-4G
  case IMUDRV_ACCFSR_8G:  c = 0x02;                break; // +-8G
  case IMUDRV_ACCFSR_16G: c = 0x03;                break; // +-16G
  }
  buff[3] = c | 0x08;

  //ImudrvI2cSetConfig(id, (uint8_t *)buff, 2);
  ImudrvSetConfigSc(p, buff, 2);

  p->accelFactor = IMUDRV_GRAVITY / 256.0;
  p->gyroFactor  = 0;
  p->tempDiv     = PARAM_TEMPERATURE_DIV;
  p->tempMul     = 1/PARAM_TEMPERATURE_DIV;
  p->tempOffset  = PARAM_TEMPERATURE_OFFSET;

  p->tPollingThresh = t * SYSTEM_COUNTER_US1M000S;
  p->tPolling = SystemGetUsCounter();

  result = IMUDRV_SUCCESS;

  return result;
}


static int
loop(int id, struct _stProbedSc *p)
{
  uint8_t               buf[32];
  struct _stImuValue    imu;

  if(PARAM_TYPE & IMUDRV_TYPE_POLLING_YES) {
    if((p->tPolling - SystemGetUsCounter()) >= p->tPollingThresh) {
      p->intrDet = 1;
    }
  }

  if(p->intrDet) {
    p->intrDet = 0;
    p->tPolling = SystemGetUsCounter();

    ImudrvReadSc(p, PARAM_DATA_POSITION, buf, PARAM_DATA_LENGTH);

    memset(&imu, 0, sizeof(imu));

    imu.id = id;

    strcpy((char *)imu.name, PARAM_CHIPNAME);
    imu.type   = PARAM_TYPE;

    imu.flag   = IMUDRV_FLAG_FILLED_S16;

    imu.ax     = ((buf[1] << 8) & 0xff00) | (buf[0] & 0xff);
    imu.ay     = ((buf[3] << 8) & 0xff00) | (buf[2] & 0xff);
    imu.az     = ((buf[5] << 8) & 0xff00) | (buf[4] & 0xff);

    //Serial.printf("IMU%d: %6d %6d %6d\n", id,  imu.ax, imu.ay, imu.az);

    ImudrvGoCb(id, p, &imu);
  }


#if 0
  static uint32_t       t  = 0;
  int   len = 32;
  int   bus = 0x100;
  if((t - SystemGetCounter()) >= 2*SYSTEM_COUNTER_1S000) {
    uint8_t     buf[256];
    t = SystemGetCounter();

    for(int j = 0; j < 4; j = j+ 1) {
      Serial.printf("IMU%d: ", j);
      ImudrvReadBus(p->bus & 0x0ff0 * j, 0x80 | 0x00, (unsigned char *)buf, len, p);
      for(int i = 0; i < len; i = i+ 1) {
        Serial.printf(" %02x", buf[i]);
      }
      Serial.printf("\n");
    }
  }
#endif

  return 0;
}


static int
start(int id, int odr)
{

    return 0;
}


static int
stop(int id)
{

    return 0;
}

// store
//
// [0]:     temperature
// [1,2,3]: ax, ay, az
// [4,5,6]: gx, gy, gz
static int
store(int id, int16_t *pD, uint8_t *pS)
{
  int           result = IMUDRV_ERRNO_UNKNOWN;

  return result;
}


static int
intr(int id, struct _stProbedSc *p)
{
  p->intrDet = 1;

  return 0;
}




//
// global functions and variables
//
struct _stImudrvList imudrvList_ADXL345Poll = {
  // device name
  .name =               PARAM_CHIPNAME,
  .type =               PARAM_TYPE,

  // register pattern table
  .cntPatternList =     sizeof(probePatternList),
  .pPatternList =       probePatternList,

  // data read position, length
  .dataAddr =           PARAM_DATA_POSITION,
  .dataLength =         PARAM_DATA_LENGTH,

  // spi speed
  .spiParam           = {
    .speed     = PARAM_SPI_FREQ_IN_KHZ * 1000,
    .tWait     = 0,
    .tWaitCs   = 0,
  },

  // temperature "to float" factor
  .tmprDiv =            PARAM_TEMPERATURE_DIV,        // divide
  .tmprOffset =         PARAM_TEMPERATURE_OFFSET,     // offset

  //  .chipIdAddr =         PARAM_CHIPID_ADDR,
  //  .chipIdValue =        PARAM_CHIPID_VALUE,

  // data rate
  .dataRate =           1920,                           // in Hz

  // functions
  .probe =              probe,
  .init =               init,
  .loop =               loop,
  .start =              start,
  .stop =               stop,
  .store =              store,
  .intr =               intr,
};


// end
