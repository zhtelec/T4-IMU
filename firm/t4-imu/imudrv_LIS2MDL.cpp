#define __IMUDRIVER_LIS2MDL_CPP__


#include        <stdint.h>
#include        <Arduino.h>

#include        "config.h"
#include        "system.h"

#include        "imudrv.h"


#define LIS2MDL_REG_CFG_REG_A           0x60
#define LIS2MDL_REG_CFG_REG_B           0x61
#define LIS2MDL_REG_CFG_REG_C           0x62
#define LIS2MDL_REG_INT_CTRL            0x63

#define LIS2MDL_REG_OUTX_L              0x68
#define LIS2MDL_REG_OUTX_H              0x69
#define LIS2MDL_REG_OUTY_L              0x6a
#define LIS2MDL_REG_OUTY_H              0x6b
#define LIS2MDL_REG_OUTZ_L              0x6c
#define LIS2MDL_REG_OUTZ_H              0x6d
#define LIS2MDL_REG_TEMP_OUT_L          0x6e
#define LIS2MDL_REG_TEMP_OUT_H          0x6f

#define PARAM_CHIPNAME                  "LIS2MDL"
#define PARAM_TYPE                      ((IMUDRV_TYPE_DEV_MAGNETICS))
#define PARAM_CHIPID_ADDR               0x4f
#define PARAM_CHIPID_VALUE              0x40
#define PARAM_SPI_FREQ_IN_KHZ           10000
#define PARAM_DATA_POSITION             (LIS2MDL_REG_OUTX_L)
#define PARAM_DATA_LENGTH               6
#define PARAM_FIFO_POSITION             0x78
#define PARAM_FIFO_LENGTH               7
#define PARAM_TEMPERATURE_DIV           8.0
#define PARAM_TEMPERATURE_OFFSET        25.0


extern struct _stImudrv         imudrv;


const static uint8_t    probePatternList[] = {
  1, PARAM_CHIPID_ADDR, 1, PARAM_CHIPID_VALUE,  // WHO_AM_I
  //1, 0x02,  1, 0x23,         //
};

const static uint8_t    bufSet4WSPI[] = {LIS2MDL_REG_CFG_REG_C, 0x24};
const static uint8_t    bufSetDRDY[]  = {LIS2MDL_REG_CFG_REG_C, 0x21};


static int
probe(int bus)
{
  int           result = IMUDRV_ERRNO_NEEDNEXTCHECK;

  ImudrvSetConfigBus(bus, &bufSet4WSPI[0], sizeof(bufSet4WSPI)/2);

  return result;
}


static int
init(int id, struct _stProbedSc *p, int powermode, int accfsr, int gyrfsr, int odr)
{
    int           result = IMUDRV_ERRNO_UNKNOWN;

    const uint8_t  settings[] = {
      //      0x02, 0x63,               // PIN_CTRL
      LIS2MDL_REG_INT_CTRL,  0xe5,      // xyz ien, high pulse, en intr
      LIS2MDL_REG_CFG_REG_A, 0x80,      // CFG_REG_A  en Temp, 10Hz (HighResolution)
      LIS2MDL_REG_CFG_REG_C, 0x21,      // CFG_REG_C  en mag, drdy intr
    };

    ImudrvSetConfigSc(p, settings, sizeof(settings)/2);
    ImudrvSetConfigSc(p, bufSetDRDY, sizeof(bufSetDRDY)/2);

    //struct _stProbedSc *p;
    //p = &imudrv.sc[id];

    p->accelFactor = 1.5 / (1000 * 10000);   // gauss * 1/10000 (T)
    p->gyroFactor  = 0.0;
    p->tempDiv     = PARAM_TEMPERATURE_DIV;
    p->tempMul     = 1/PARAM_TEMPERATURE_DIV;
    p->tempOffset  = PARAM_TEMPERATURE_OFFSET;

    result = IMUDRV_SUCCESS;

    return result;
}


static int
loop(int id, struct _stProbedSc *p)
{
  uint8_t               buf[32];
  struct _stImuValue    imu;

  if(p->intrDet) {
    ImudrvDisableInterrupt(p->intrPort);
    p->intrDet = 0;

    ImudrvSetConfigSc(p, bufSet4WSPI, sizeof(bufSet4WSPI)/2);

    imu.id = id;

    strcpy((char *)imu.name, PARAM_CHIPNAME);
    imu.type   = PARAM_TYPE;

    imu.flag   = IMUDRV_FLAG_FILLED_S16;

    ImudrvReadSc(p, LIS2MDL_REG_TEMP_OUT_L, buf, 2);
    imu.temp   = ((buf[ 1] << 8) & 0xff00) | (buf[ 0] & 0xff);

    ImudrvReadSc(p, PARAM_DATA_POSITION, buf, PARAM_DATA_LENGTH);
    imu.ax     = ((buf[ 1] << 8) & 0xff00) | (buf[ 0] & 0xff);
    imu.ay     = ((buf[ 3] << 8) & 0xff00) | (buf[ 2] & 0xff);
    imu.az     = ((buf[ 5] << 8) & 0xff00) | (buf[ 4] & 0xff);

    imu.gx     = 0.0;
    imu.gy     = 0.0;
    imu.gz     = 0.0;

    imu.tsChip = 0;

    //Serial.printf("%x %x %x %x   %x %x %x  %x\n", imu.tsChip, imu.ax, imu.ay, imu.az, imu.gx, imu.gy, imu.gz, imu.temp);

    ImudrvGoCb(id, p, &imu);

    ImudrvSetConfigSc(p, bufSetDRDY, sizeof(bufSetDRDY)/2);
    p->intrDet = 0;
    ImudrvEnableInterrupt(p->intrPort);
    p->intrDet = 0;
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
struct _stImudrvList imudrvList_LIS2MDL = {
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
  .dataRate =           10,                           // in Hz

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
