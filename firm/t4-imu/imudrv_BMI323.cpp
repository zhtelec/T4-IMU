#define __IMUDRV_BMI323_C__


#include        <stdint.h>
#include        <Arduino.h>

#include        "config.h"
#include        "system.h"

#include        "imudrv.h"

#define PARAM_CHIPNAME                  "BMI323"
#define PARAM_CHIPID_ADDR               0x00
#define PARAM_CHIPID_VALUE_H            0x00
#define PARAM_CHIPID_VALUE_L            0x43
#define PARAM_SPI_FREQ_IN_KHZ           10000
#define PARAM_DATA_POSITION             0x03
#define PARAM_DATA_LENGTH               18
#define PARAM_FIFO_POSITION             0x16
#define PARAM_FIFO_LENGTH               2
#define PARAM_TEMPERATURE_DIV           512.0
#define PARAM_TEMPERATURE_OFFSET        23.0


extern struct _stImudrv      imudrv;


const static uint8_t    probePatternList[] = {
  1, PARAM_CHIPID_ADDR,  2, PARAM_CHIPID_VALUE_H, PARAM_CHIPID_VALUE_L,         // WHO_AM_I
  //1, 0x20,  2, 0x00, 0x28,     // ALT_ACC_CONF
  //1, 0x21,  2, 0x00, 0x48,     // ALT_GYR_CONF
};


static int
probe(int bus)
{
  int           result = IMUDRV_ERRNO_NOMATCH;

  return result;
}


static int
init(int id, int powermode, int accfsr, int gyrfsr, int odr)
{
    int         result = IMUDRV_ERRNO_UNKNOWN;

    uint8_t     buff[16];
    uint32_t    c;

    const uint8_t  settings[] = {
#if 0
      // soft reset
      , 0x01, 0x2c,     // FEATURE_IO2
      , 0x00, 0x01,     // FEATURE_IO_STATUS
      , 0x00, 0x00,     // FEATURE_CTRL.engine_en = 1
#endif
      //addr, lsb, msb  // byte order
      0x38, 0x05, 0x00, // IO_INT_CTRL
      0x39, 0x00, 0x00, // INT_CONF
      0x3a, 0x00, 0x00, // INT_MAP1
      0x3b, 0x00, 0x05, // INT_MAP2

      //0x20, 0x28, 0x70, // ACC_CONF
      //0x21, 0x48, 0x70, // GYR_CONF
    };

    for(int i = 0; i < sizeof(settings); i+=3) {
      ImudrvSpiWrite(id, settings[i], &settings[i+1], 2);
    }

    uint32_t    accel, gyro;
    accel = 0x7000;
    gyro  = 0x7000;

    switch(odr){
        case IMUDRV_ODR_13HZ:   c = 0x06;             break; //   12.5Hz
        case IMUDRV_ODR_25HZ:   c = 0x06;             break; //   25Hz
        case IMUDRV_ODR_50HZ:   c = 0x07;             break; //   50Hz
        default:
        case IMUDRV_ODR_100HZ:  c = 0x08; odr = 0x08; break; //  100Hz
        case IMUDRV_ODR_200HZ:  c = 0x09;             break; //  200Hz
        case IMUDRV_ODR_400HZ:  c = 0x0a;             break; //  400Hz
        case IMUDRV_ODR_800HZ:  c = 0x0b;             break; //  800Hz
        case IMUDRV_ODR_1600HZ: c = 0x0c;             break; // 1600Hz
        case IMUDRV_ODR_3200HZ: c = 0x0d;             break; // 3200Hz
        case IMUDRV_ODR_6400HZ: c = 0x0e;             break; // 6400Hz
    }

    accel |= c;
    gyro  |= c;

    switch(gyrfsr){
        case IMUDRV_GYRFSR_125DPS:  c = 0x00 << 4;                break; // +- 125dps
        case IMUDRV_GYRFSR_250DPS:  c = 0x01 << 4;                break; // +- 250dps
        default:
        case IMUDRV_GYRFSR_500DPS:  c = 0x02 << 4; gyrfsr = 0x08; break; // +- 500dps
        case IMUDRV_GYRFSR_1000DPS: c = 0x03 << 4;                break; // +-1000dps
        case IMUDRV_GYRFSR_2000DPS: c = 0x04 << 4;                break; // +-2000dps
    }
    gyro |= c;

    switch(accfsr){
        case IMUDRV_ACCFSR_2G:  c = 0x00 << 4;                break; // +- 2G
        default:
        case IMUDRV_ACCFSR_4G:  c = 0x01 << 4; accfsr = 0x08; break; // +- 4G
        case IMUDRV_ACCFSR_8G:  c = 0x02 << 4;                break; // +- 8G
        case IMUDRV_ACCFSR_16G: c = 0x03 << 4;                break; // +-16G
    }
    accel |= c;

    struct _stProbedSc *p;
    p = &imudrv.sc[id];

    p->accelFactor = 4.0 * IMUDRV_GRAVITY / (32768.0*256.0) * (float)(1 << accfsr);
    p->gyroFactor  =       500.0   / (32768.0*256.0) * (float)(1 << gyrfsr);
    p->tempDiv     = PARAM_TEMPERATURE_DIV;
    p->tempMul     = 1/PARAM_TEMPERATURE_DIV;
    p->tempOffset  = PARAM_TEMPERATURE_OFFSET;

    buff[0] = accel  & 0xff;
    buff[1] = accel >> 8;
    buff[2] = gyro   & 0xff;
    buff[3] = gyro  >> 8;
    ImudrvSpiWrite(id, 0x20, buff, 4);

    result = IMUDRV_SUCCESS;

    return result;
}


static int
loop(int id, struct _stProbedSc *p)
{
  uint8_t               buf[32];
  struct _stImuValue    imu;

  if(p->intrDet) {
    p->intrDet = 0;

#if     CONFIG_IMU_CALC_TIME_PULSE
    digitalWrite(CONFIG_IMU_CALC_TIME_PULSE, 0);
#endif

    // the read transaction has one dummy byte bwt command and data/
    ImudrvSpiReadBus(p->bus & 0x0fff, PARAM_DATA_POSITION | 0x80, buf, PARAM_DATA_LENGTH+1, &p->spiParam);

#if     CONFIG_IMU_CALC_TIME_PULSE
    digitalWrite(CONFIG_IMU_CALC_TIME_PULSE, 1);
#endif

    imu.id = id;

    strcpy(imu.name, PARAM_CHIPNAME);

    imu.flag   = IMUDRV_FLAG_FILLED_S16;

    imu.ax     = ((buf[ 2] << 8) & 0xff00) | (buf[ 1] & 0xff);
    imu.ay     = ((buf[ 4] << 8) & 0xff00) | (buf[ 3] & 0xff);
    imu.az     = ((buf[ 6] << 8) & 0xff00) | (buf[ 5] & 0xff);

    imu.gx     = ((buf[ 8] << 8) & 0xff00) | (buf[ 7] & 0xff);
    imu.gy     = ((buf[10] << 8) & 0xff00) | (buf[ 9] & 0xff);
    imu.gz     = ((buf[12] << 8) & 0xff00) | (buf[11] & 0xff);

    imu.temp   = ((buf[14] << 8) & 0xff00) | (buf[13] & 0xff);

    imu.tsChip = ((buf[18] << 24) & 0xff000000) | ((buf[17] << 16) & 0xff0000) | ((buf[16] << 8) & 0xff00) | (buf[15] & 0xff);

    //    Serial.printf("%x %x %x   %x %x %x  %x\n", ax, ay, az, gx, gy, gz, temp);

    ImudrvGoCb(id, p, &imu);
  }

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
struct _stImudrvList imudrvList_BMI323 = {
  // device name
  .name =               PARAM_CHIPNAME,

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
