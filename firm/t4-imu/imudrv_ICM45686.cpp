#define __IMUDRV_ICM45686_CPP__

#include        <stdint.h>

#include        <Arduino.h>

#include        "config.h"
#include        "system.h"

#include        "imudrv.h"

#define PARAM_CHIPNAME                  "ICM45686"
#define PARAM_CHIPID_ADDR               0x72
#define PARAM_CHIPID_VALUE              0xe9
#define PARAM_SPI_FREQ_IN_KHZ           24000
#define PARAM_DATA_POSITION             0x00
#define PARAM_DATA_LENGTH               16
#define PARAM_FIFO_POSITION             0x12
#define PARAM_FIFO_LENGTH               13
#define PARAM_TEMPERATURE_DIV           128.0
#define PARAM_TEMPERATURE_OFFSET        25.0


extern struct _stImudrv      imudrv;


const static uint8_t    probePatternList[] = {
  1, PARAM_CHIPID_ADDR,  1, PARAM_CHIPID_VALUE,         // WHO_AM_I
  1, 0x20,  1, 0x20,            // FIFO CONFIG2
  1, 0x34,  1, 0x02,            // DRIVE_CONFIG1
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
    uint8_t     c;

    const uint8_t  settings[] = {
      0x10, 0x00,       // PWR_MGMT0 all off

      //0x1b, 0x35,       // ACCEL_CONFIG0 [6:4]FS=3(+-4G),    [3:0]ODR=5(1.6kHz)
      //0x1c, 0x35,       // GYRO_CONFIG0  [7:4]FS=3(+-500dps) [3:0]ODR=5(1.6kHz)

      0x16, 0x04,       // INT1_CONFIG0 [2] DRDY=1
      0x17, 0x00,       // INT1_CONFIG1 all zero
      0x18, 0x01,       // INT1_CONFIG2 [2]DRIVE=0(push-pull), [1]MODE=0(pulse), [0]POL=1(high)

      0x58, 0x63,       // SMC_CONTROL_0 timestamp en

      0x10, 0x0f,       // PWR_MGMT0 [3:2]GYRO_MODE=3(low noise), [1:0]ACCEL_MODE=3(low noise)
    };

    // buff[0,1] aacel|odr, buff[2,3] gyro|odr
    buff[0] = 0x1b; // CTRL1  accel odr
    buff[2] = 0x1c; // CTRL2  gyro  odr
    switch(odr){
        case IMUDRV_ODR_13HZ:   c = 0x0c;             break; //   12.5Hz
        case IMUDRV_ODR_25HZ:   c = 0x0b;             break; //   25Hz
        case IMUDRV_ODR_50HZ:   c = 0x0a;             break; //   50Hz
        default:
        case IMUDRV_ODR_100HZ:  c = 0x09; odr = 0x08; break; //  100Hz
        case IMUDRV_ODR_200HZ:  c = 0x08;             break; //  200Hz
        case IMUDRV_ODR_400HZ:  c = 0x07;             break; //  400Hz
        case IMUDRV_ODR_800HZ:  c = 0x06;             break; //  800Hz
        case IMUDRV_ODR_1600HZ: c = 0x05;             break; // 1600Hz
        case IMUDRV_ODR_3200HZ: c = 0x04;             break; // 3200Hz
        case IMUDRV_ODR_6400HZ: c = 0x03;             break; // 6400Hz
    }
    buff[1] = c;
    buff[3] = c;

    switch(gyrfsr){
        case IMUDRV_GYRFSR_125DPS:  c = 0x05 << 4;                break; // +- 125dps
        case IMUDRV_GYRFSR_250DPS:  c = 0x04 << 4;                break; // +- 250dps
        default:
        case IMUDRV_GYRFSR_500DPS:  c = 0x03 << 4; gyrfsr = 0x08; break; // +- 500dps
        case IMUDRV_GYRFSR_1000DPS: c = 0x02 << 4;                break; // +-1000dps
        case IMUDRV_GYRFSR_2000DPS: c = 0x01 << 4;                break; // +-2000dps
        case IMUDRV_GYRFSR_4000DPS: c = 0x00 << 4;                break; // +-4000dps
    }
    buff[3] |= c;

    switch(accfsr){
        case IMUDRV_ACCFSR_2G:  c = 0x04 << 4;                break; // +- 2G
        default:
        case IMUDRV_ACCFSR_4G:  c = 0x03 << 4; accfsr = 0x08; break; // +- 4G
        case IMUDRV_ACCFSR_8G:  c = 0x02 << 4;                break; // +- 8G
        case IMUDRV_ACCFSR_16G: c = 0x01 << 4;                break; // +-16G
        case IMUDRV_ACCFSR_32G: c = 0x00 << 4;                break; // +-32G
    }
    buff[1] |= c;

    struct _stProbedSc *p;
    p = &imudrv.sc[id];

    p->accelFactor = 4.0 * IMUDRV_GRAVITY / (32768.0*256.0) * (float)(1 << accfsr);
    p->gyroFactor  =       500.0   / (32768.0*256.0) * (float)(1 << gyrfsr);
    p->tempDiv     = PARAM_TEMPERATURE_DIV;
    p->tempMul     = 1/PARAM_TEMPERATURE_DIV;
    p->tempOffset  = PARAM_TEMPERATURE_OFFSET;

    ImudrvSpiSetConfig(id, (uint8_t *)buff, 2);
    ImudrvSpiSetConfig(id, (uint8_t *)settings, sizeof(settings)/2);

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

    // the read transaction has one dummy byte bwt command and data/
    ImudrvSpiReadBus(p->bus & 0x0fff, PARAM_DATA_POSITION | 0x80, buf, PARAM_DATA_LENGTH+1, &p->spiParam);

    imu.id = id;

    strcpy((char *)imu.name, PARAM_CHIPNAME);

    imu.flag  = IMUDRV_FLAG_FILLED_S16;

    imu.ax    = ((buf[ 1] << 8) & 0xff00) | (buf[ 0] & 0xff);
    imu.ay    = ((buf[ 3] << 8) & 0xff00) | (buf[ 2] & 0xff);
    imu.az    = ((buf[ 5] << 8) & 0xff00) | (buf[ 4] & 0xff);

    imu.gx     = ((buf[ 7] << 8) & 0xff00) | (buf[ 6] & 0xff);
    imu.gy     = ((buf[ 9] << 8) & 0xff00) | (buf[ 8] & 0xff);
    imu.gz     = ((buf[11] << 8) & 0xff00) | (buf[10] & 0xff);

    imu.temp   = ((buf[13] << 8) & 0xff00) | (buf[12] & 0xff);

    imu.tsChip = ((buf[15] << 8) & 0xff00) | (buf[14] & 0xff);

    //    Serial.printf("%x %x %x   %x %x %x  %x\n", ax, ay, az, gx, gy, gz, temp);

    ImudrvGoCb(id, p, &imu);
  }

  return 0;
}


static int
start(int id, int odr)
{
    uint8_t     buff[2];

#ifdef CONFIG_IMUSPI_USE_FIFO
#endif

    buff[0] = 0x10; //PWR_MGMT0
    buff[1] = 0x0f; //low noise
    ImudrvSpiSetConfig(id, buff, 1);

    return 0;
}


static int
stop(int id)
{
    uint8_t     buff[2];

    buff[0] = 0x10; //PWR_MGMT0
    buff[1] = 0x00; //off

    ImudrvSpiSetConfig(id, buff, 1);

    return 0;
}

// store
// buffer structure       imu addr
// [0]:     temperature   [12]
// [1,2,3]: gx, gy, gz    [ 6, 8,10] gyro
// [4,5,6]: ax, ay, az    [ 0, 2, 4] accel
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
struct _stImudrvList imudrvList_ICM45686 = {
  // device name
  .name =               "ICM45686",

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
  //.chipIdValue =        PARAM_CHIPID_VALUE,

  // data rate
  .dataRate =           1600,                           // in Hz

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
