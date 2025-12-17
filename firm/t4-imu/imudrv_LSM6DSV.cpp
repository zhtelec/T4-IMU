#define __IMUDRIVER_LSM6DSV_CPP__


#include        <stdint.h>
#include        <Arduino.h>

#include        "config.h"
#include        "system.h"

#include        "imudrv.h"

#define PARAM_CHIPNAME                  "LSM6DSV"
#define PARAM_CHIPID_ADDR               0x0f
#define PARAM_CHIPID_VALUE              0x70
#define PARAM_SPI_FREQ_IN_KHZ           10000
#define PARAM_DATA_POSITION             0x20
#define PARAM_DATA_LENGTH               14
#define PARAM_FIFO_POSITION             0x78
#define PARAM_FIFO_LENGTH               7
#define PARAM_TEMPERATURE_DIV           256.0
#define PARAM_TEMPERATURE_OFFSET        25.0


extern struct _stImudrv         imudrv;


const static uint8_t    probePatternList[] = {
  1, PARAM_CHIPID_ADDR, 1, PARAM_CHIPID_VALUE,  // WHO_AM_I
  1, 0x02,  1, 0x23,         // PIN_CTRL
  1, 0x12,  1, 0x44,         // CTRL3
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
    int           result = IMUDRV_ERRNO_UNKNOWN;

    uint8_t buff[16];
    uint8_t c;

    const uint8_t  settings[] = {
      //      0x02, 0x63,               // PIN_CTRL
      0x03, 0x01,               // IF_CFG  disable I2C, I3C
      0x0d, 0x01,               // INT1_CTRL DRDY_ACCEL
      0x0e, 0x00,               // INT2_CTRL
      0x12, 0x44,               // CTRL3: block data update, auto increment
      0x13, 0x0a,               // CTRL4: DRDY masked, DRDY pulsed 65us
      0x50, 0x40,               // FUNC_EN: ts enable
    };

    ImudrvSpiSetConfig(id, settings, sizeof(settings)/2);


    buff[0] = 0x10; // CTRL1  accel odr
    buff[2] = 0x11; // CTRL2  gyro  odr
    switch(odr){
        case IMUDRV_ODR_13HZ:   c = 0x03;             break; //   15Hz
        case IMUDRV_ODR_25HZ:   c = 0x04;             break; //   30Hz
        case IMUDRV_ODR_50HZ:   c = 0x05;             break; //   60Hz
        default:
        case IMUDRV_ODR_100HZ:  c = 0x06; odr = 0x08; break; //  120Hz
        case IMUDRV_ODR_200HZ:  c = 0x07;             break; //  240Hz
        case IMUDRV_ODR_400HZ:  c = 0x08;             break; //  480Hz
        case IMUDRV_ODR_800HZ:  c = 0x09;             break; //  960Hz
        case IMUDRV_ODR_1600HZ: c = 0x0a;             break; // 1920Hz
        case IMUDRV_ODR_3200HZ: c = 0x0b;             break; // 3840Hz
        case IMUDRV_ODR_6400HZ: c = 0x0c;             break; // 7680Hz
    }

    buff[1] = c;
    buff[3] = c;

    buff[4] = 0x15;     // CTRL6  gyro fs
    switch(gyrfsr){
        case IMUDRV_GYRFSR_125DPS:  buff[5] = 0x00;                break; // +-125dps
        case IMUDRV_GYRFSR_250DPS:  buff[5] = 0x01;                break; // +-250dps
        default:
        case IMUDRV_GYRFSR_500DPS:  buff[5] = 0x02; gyrfsr = 0x08; break; // +-500dps
        case IMUDRV_GYRFSR_1000DPS: buff[5] = 0x03;                break; // +-1000dps
        case IMUDRV_GYRFSR_2000DPS: buff[5] = 0x04;                break; // +-2000dps
        case IMUDRV_GYRFSR_4000DPS: buff[5] = 0x0c;                break; // +-4000dps
    }

    buff[6] = 0x17;     // CTRL8  accel fs
    switch(accfsr){
        case IMUDRV_ACCFSR_2G:  buff[7] = 0x00;                break; // +-2G
        default:
        case IMUDRV_ACCFSR_4G:  buff[7] = 0x01; accfsr = 0x08; break; // +-4G
        case IMUDRV_ACCFSR_8G:  buff[7] = 0x02;                break; // +-8G
        case IMUDRV_ACCFSR_16G: buff[7] = 0x03;                break; // +-16G
    }

    struct _stProbedSc *p;
    p = &imudrv.sc[id];

    p->accelFactor = 4.0 * IMUDRV_GRAVITY / (32768.0*256.0) * (float)(1 << accfsr);
    p->gyroFactor  =       500.0   / (32768.0*256.0) * (float)(1 << gyrfsr);
    p->tempDiv     = PARAM_TEMPERATURE_DIV;
    p->tempMul     = 1/PARAM_TEMPERATURE_DIV;
    p->tempOffset  = PARAM_TEMPERATURE_OFFSET;

    ImudrvSpiSetConfig(id, (uint8_t *)buff, 4);

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

    ImudrvSpiReadBus(p->bus & 0x0fff, PARAM_DATA_POSITION | 0x80, buf, PARAM_DATA_LENGTH, &p->spiParam);

    imu.id = id;

    strcpy((char *)imu.name, PARAM_CHIPNAME);

    imu.flag   = IMUDRV_FLAG_FILLED_S16;

    imu.ax     = ((buf[ 9] << 8) & 0xff00) | (buf[ 8] & 0xff);
    imu.ay     = ((buf[11] << 8) & 0xff00) | (buf[10] & 0xff);
    imu.az     = ((buf[13] << 8) & 0xff00) | (buf[12] & 0xff);

    imu.gx     = ((buf[ 3] << 8) & 0xff00) | (buf[ 2] & 0xff);
    imu.gy     = ((buf[ 5] << 8) & 0xff00) | (buf[ 4] & 0xff);
    imu.gz     = ((buf[ 7] << 8) & 0xff00) | (buf[ 6] & 0xff);

    imu.temp   = ((buf[ 1] << 8) & 0xff00) | (buf[ 0] & 0xff);

    imu.tsChip = 0;
    ImudrvSpiReadBus(p->bus & 0x0fff, 0x40 | 0x80, buf, 4, &p->spiParam);
    imu.tsChip = ((buf[2] << 16) & 0xff0000) | ((buf[1] << 8) & 0xff00) | (buf[0] & 0xff);

    //Serial.printf("%x %x %x %x   %x %x %x  %x\n", imu.tsChip, imu.ax, imu.ay, imu.az, imu.gx, imu.gy, imu.gz, imu.temp);

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
      ImudrvSpiReadBus(p->bus & 0x0ff0 * j, 0x80 | 0x00, (unsigned char *)buf, len, &p->spiParam);
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
struct _stImudrvList imudrvList_LSM6DSV = {
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
