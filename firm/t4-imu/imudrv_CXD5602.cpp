#define _IMUDRV_CXD5602_CPP_


#include        <stdint.h>
#include        <Arduino.h>

#include        "config.h"
#include        "system.h"

#include        "imudrv.h"

static int      start(int id, int odr);

#define PARAM_CHIPNAME                  "CXD5602"
#define PARAM_CHIPID_ADDR0              0x15
#define PARAM_CHIPID_VALUE0_0           1
#define PARAM_CHIPID_ADDR1              0x17
#define PARAM_CHIPID_VALUE1_0           1
#define PARAM_SPI_FREQ_IN_KHZ           6000
#define PARAM_DATA_POSITION             0x00
#define PARAM_DATA_LENGTH               32
#define PARAM_FIFO_POSITION             0x16
#define PARAM_FIFO_LENGTH               2
#define PARAM_TEMPERATURE_DIV           512.0
#define PARAM_TEMPERATURE_OFFSET        23.0

struct cxd5602pwbimu_data_s {
  uint32_t timestamp;       /* timestamp */
  float temp;               /* temperature */
  float gx;                 /* gyro x */
  float gy;                 /* gyro y */
  float gz;                 /* gyro z */
  float ax;                 /* accel x */
  float ay;                 /* accel y */
  float az;                 /* accel z */
};


// spresense imu registers
#define CXD5602PWBIMU_INJECTION_READY           (0x01)
#define CXD5602PWBIMU_UPDATE_RESULT             (0x02)
#define CXD5602PWBIMU_INJECT_BINARY             (0x03)
#define CXD5602PWBIMU_VERIFY_BINARY             (0x04)
#define CXD5602PWBIMU_CHANGE_TO_UPDATEMODE      (0x05)

#define CXD5602PWBIMU_FW_VER                    (0x10) /* Firmware version */
#define CXD5602PWBIMU_HW_REVISION               (0x11) /* HW Revision */
#define CXD5602PWBIMU_HW_UNIQUE_ID              (0x12) /* HW UID */
#define CXD5602PWBIMU_FSR                       (0x13) /* Full Scale */
#define CXD5602PWBIMU_ODR                       (0x14) /* Output Data Rate */
#define CXD5602PWBIMU_INTR_ENABLE               (0x15) /* Interrupt enable */
#define CXD5602PWBIMU_FIFO_THRESH               (0x17) /* FIFO threshold */
#define CXD5602PWBIMU_OUTPUT_ENABLE             (0x18) /* Output Enable */
#define CXD5602PWBIMU_USER_CALIB_COEF           (0x19) /* User calibration */
#define CXD5602PWBIMU_USER_CALIB_FLASH          (0x1a) /* Flash user calib value */

#define CXD5602PWBIMU_MODE                      (0xff) /* Output Mode */

// FSR (accel, gyro)
#define FSR_ACCEL_2_G                           (0x00 << 4) /* Set ACCEL FullScale +/-2G */
#define FSR_ACCEL_4_G                           (0x01 << 4) /* Set ACCEL FullScale +/-4G */
#define FSR_ACCEL_8_G                           (0x02 << 4) /* Set ACCEL FullScale +/-8G */
#define FSR_ACCEL_16_G                          (0x03 << 4) /* Set ACCEL FullScale +/-16G */

#define FSR_GYRO_125_DPS                        (0x00) /* Set GYRO FullScale +/-125dps */
#define FSR_GYRO_250_DPS                        (0x01) /* Set GYRO FullScale +/-250dps */
#define FSR_GYRO_500_DPS                        (0x02) /* Set GYRO FullScale +/-500dps */
#define FSR_GYRO_1000_DPS                       (0x03) /* Set GYRO FullScale +/-1000dps */
#define FSR_GYRO_2000_DPS                       (0x04) /* Set GYRO FullScale +/-2000dps */
#define FSR_GYRO_4000_DPS                       (0x05) /* Set GYRO FullScale +/-4000dps */

// ODR configuration
#define ODR_15HZ                                (0x00) /* Set output data rate to 15Hz */
#define ODR_30HZ                                (0x01) /* Set output data rate to 30Hz */
#define ODR_60HZ                                (0x02) /* Set output data rate to 60Hz */
#define ODR_120HZ                               (0x03) /* Set output data rate to 120Hz */
#define ODR_240HZ                               (0x04) /* Set output data rate to 240Hz */
#define ODR_480HZ                               (0x05) /* Set output data rate to 480Hz */
#define ODR_960HZ                               (0x06) /* Set output data rate to 960Hz */
#define ODR_1920HZ                              (0x07) /* Set output data rate to 1920Hz */

// Output ENABLE
#define OUTPUT_DISABLE                          (0x00) /* Disable 6axis data output */
#define OUTPUT_ENABLE                           (0x01) /* Enable 6axis data output */

// Update result status
#define RESULT_NOEXEC                           (0x00)
#define RESULT_SUCCESS                          (0x01)
#define RESULT_FAILURE                          (0x02)


extern struct _stImudrv      imudrv;


const static uint8_t    probePatternList[] = {
  1, PARAM_CHIPID_ADDR0,  1, PARAM_CHIPID_VALUE0_0,
  1, PARAM_CHIPID_ADDR1,  1, PARAM_CHIPID_VALUE1_0,
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
    //uint8_t gyr_enable = register_get_value(REG_IMU_CHOICE);

    const uint8_t  settings[] = {
      CXD5602PWBIMU_FIFO_THRESH,        1,
      CXD5602PWBIMU_INTR_ENABLE,        1,
      CXD5602PWBIMU_OUTPUT_ENABLE,      OUTPUT_ENABLE,
    };

    uint8_t    fsr = 0, odrval;
    switch(odr){
        default:
        case IMUDRV_ODR_100HZ:  c = ODR_120HZ; odr = IMUDRV_ODR_100HZ; break; //  125Hz
        case IMUDRV_ODR_200HZ:  c = ODR_240HZ;             break; //  240Hz
        case IMUDRV_ODR_400HZ:  c = ODR_480HZ;             break; //  480Hz
        case IMUDRV_ODR_800HZ:  c = ODR_960HZ;             break; //  960Hz
        case IMUDRV_ODR_1600HZ: c = ODR_1920HZ;            break; // 1920Hz
    }
    odrval = c;

    switch(gyrfsr) {
        case IMUDRV_GYRFSR_250DPS:  c = FSR_GYRO_250_DPS;                break; // +- 250dps
        default:
        case IMUDRV_GYRFSR_500DPS:  c = FSR_GYRO_500_DPS; gyrfsr = IMUDRV_GYRFSR_500DPS; break; // +- 500dps
        case IMUDRV_GYRFSR_1000DPS: c = FSR_GYRO_1000_DPS;               break; // +-1000dps
        case IMUDRV_GYRFSR_2000DPS: c = FSR_GYRO_2000_DPS;               break; // +-2000dps
        //case IMUDRV_GYRFSR_4000DPS: c = FSR_GYRO_4000_DPS;               break; // +-4000dps
    }
    fsr |= c;

    switch(accfsr) {
        case IMUDRV_ACCFSR_2G:  c = FSR_ACCEL_2_G;                break; // +- 2G
        default:
        case IMUDRV_ACCFSR_4G:  c = FSR_ACCEL_4_G; accfsr = IMUDRV_ACCFSR_4G; break; // +- 4G
        case IMUDRV_ACCFSR_8G:  c = FSR_ACCEL_8_G;                break; // +- 8G
        case IMUDRV_ACCFSR_16G: c = FSR_ACCEL_16_G;               break; // +-16G
    }
    fsr |= c;

    struct _stProbedSc *p;
    p = &imudrv.sc[id];

    p->accelFactor = 1.0;
    p->gyroFactor  = 1.0;

#if 0
    p->accelFactor = 4.0 * IMUDRV_GRAVITY / (32768.0*256.0) * (float)(1 << accfsr);
    p->gyroFactor  =       500.0   / (32768.0*256.0) * (float)(1 << gyrfsr);

    p->tempDiv     = PARAM_TEMPERATURE_DIV;
    p->tempMul     = 1/PARAM_TEMPERATURE_DIV;
    p->tempOffset  = PARAM_TEMPERATURE_OFFSET;
#endif

    buff[0] = CXD5602PWBIMU_FSR;
    buff[1] = fsr;
    buff[2] = CXD5602PWBIMU_ODR;
    buff[3] = odrval;

    ImudrvI2cSetConfig(id, (uint8_t *)&buff[0], 1);  // set fsr
    ImudrvI2cSetConfig(id, (uint8_t *)&buff[2], 1);  // set odr
    for(int i = 0; i < (int)sizeof(settings); i+=2) {
      ImudrvI2cSetConfig(id, (uint8_t *)&settings[i], 1);
    }

    result = IMUDRV_SUCCESS;

    return result;
}


static int
loop(int id, struct _stProbedSc *p)
{
  uint8_t               buf[64];
  struct _stImuValue    imu;

  if(p->intrDet || digitalRead(p->intrPort)) {
    p->intrDet = 0;

#if     CONFIG_IMU_CALC_TIME_PULSE
    digitalWrite(CONFIG_IMU_CALC_TIME_PULSE, 0);
#endif

    // the read transaction has one dummy byte bwt command and data/
    int bus = 0x1100;
    memset(buf, 0, PARAM_DATA_LENGTH);
    buf[0] = 0x80;
    ImudrvSpiReadBus(bus & 0x0fff, -1, buf, PARAM_DATA_LENGTH, &p->spiParam);

#if     CONFIG_IMU_CALC_TIME_PULSE
    digitalWrite(CONFIG_IMU_CALC_TIME_PULSE, 1);
#endif

    imu.id = id;

    strcpy((char *)imu.name, PARAM_CHIPNAME);

    imu.flag = IMUDRV_FLAG_FILLED_FLOAT;

    imu.axf = *(float *) &buf[20];
    imu.ayf = *(float *) &buf[24];
    imu.azf = *(float *) &buf[28];

    imu.gxf = *(float *) &buf[ 8] * 180.0 / PI;
    imu.gyf = *(float *) &buf[12] * 180.0 / PI;
    imu.gzf = *(float *) &buf[16] * 180.0 / PI;

    imu.tempf = *(float *) &buf[4];

    imu.tsChip = ((buf[3] << 24) & 0xff000000) | ((buf[2] << 16) & 0xff0000) | ((buf[1] << 8) & 0xff00) | (buf[0] & 0xff);

#if 0
    Serial.printf("%2x %2x %2x %2x  %2x %2x %2x %2x\n", buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);
    Serial.printf("%8x  %7d %7d %7d   %7d %7d %7d  %d\n",
                  imu.tsChip,
                  (int)(imu.axf * 1000000), (int)(imu.ayf * 1000000), (int)(imu.azf * 1000000),
                  (int)(imu.gxf * 1000000), (int)(imu.gyf * 1000000), (int)(imu.gzf * 1000000),
                  (int)(imu.tempf * 1000));
#endif

    ImudrvGoCb(id, p, &imu);
  }

  return 0;
}


static int
start(int id, int odr)
{
  const uint8_t  settings[] = {
    CXD5602PWBIMU_OUTPUT_ENABLE,      OUTPUT_ENABLE,
  };

  ImudrvI2cSetConfig(id, (uint8_t *)settings, 1);

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
struct _stImudrvList imudrvList_CXD5602 = {
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
    .tWait     = (20*SYSTEM_COUNTER_US1U000S),
    .tWaitCs   = (10*SYSTEM_COUNTER_US1U000S),
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
