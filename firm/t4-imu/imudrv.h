/* ========================================
*/

#ifndef __IMUDRV_H__
#define __IMUDRV_H__
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#define IMUDRV_GRAVITY       (9.80665)


#define IMUDRV_ODR_13HZ      0x5
#define IMUDRV_ODR_25HZ      0x6
#define IMUDRV_ODR_50HZ      0x7
#define IMUDRV_ODR_100HZ     0x8
#define IMUDRV_ODR_200HZ     0x9
#define IMUDRV_ODR_400HZ     0xa
#define IMUDRV_ODR_800HZ     0xb
#define IMUDRV_ODR_1600HZ    0xc
#define IMUDRV_ODR_3200HZ    0xd
#define IMUDRV_ODR_6400HZ    0xe

#define IMUDRV_ACCFSR_2G     0x7
#define IMUDRV_ACCFSR_4G     0x8
#define IMUDRV_ACCFSR_8G     0x9
#define IMUDRV_ACCFSR_16G    0xa
#define IMUDRV_ACCFSR_32G    0xb

#define IMUDRV_GYRFSR_125DPS         0x6
#define IMUDRV_GYRFSR_250DPS         0x7
#define IMUDRV_GYRFSR_500DPS         0x8
#define IMUDRV_GYRFSR_1000DPS        0x9
#define IMUDRV_GYRFSR_2000DPS        0xa
#define IMUDRV_GYRFSR_4000DPS        0xb

#define IMUDRV_FORMAT_DISABLE  (0)
#define IMUDRV_FORMAT_S16      (1)
#define IMUDRV_FORMAT_FLOAT    (2)

typedef struct {
  uint8_t       size;
  uint8_t       dummy[2];   // for 2 bytes align padding
  uint8_t       id;
  uint32_t      ts;
  int32_t       tmpr;
  int32_t       gyr[3];     // x, y, z
  int32_t       acc[3];     // x, y, z
} imu_data_type;


struct _stProbedSc {
  int                           id;
  uint8_t                       subId;
  uint16_t                      bus;            // [15:12]: 0=I2C, 1=SPI
                                                // [11:8]: bus number
                                                // [6:0] i2c slave 7bit addr, if I2C
                                                // [3:0] spi cs line number,  if SPI
#define IMUDRV_BUS_IF_POS       12
#define IMUDRV_BUS_IF_MASK      (0xf << (IMUDRV_BUS_IF_POS))
#define IMUDRV_BUS_IF_I2C       (0   << (IMUDRV_BUS_IF_POS))
#define IMUDRV_BUS_IF_SPI       (1   << (IMUDRV_BUS_IF_POS))
#define IMUDRV_BUS_NUM_POS      8
#define IMUDRV_BUS_NUM_MASK     (0xf << (IMUDRV_BUS_NUM_POS))
#define IMUDRV_BUS_SLAVE_POS    0
#define IMUDRV_BUS_SLAVE_MASK   (0x7f << (IMUDRV_BUS_SLAVE_POS))

  int                           seq;
  int                           intrPort;
  int                           intrDet;
  struct _stSystemSpiParam      spiParam;
  struct _stSystemI2cParam      i2cParam;

  uint8_t                       accfsr;
  uint8_t                       gyrfsr;
  uint8_t                       odr;
  uint8_t                       fmtUart;
  uint8_t                       fmtIp;

  float                         accelFactor;
  float                         gyroFactor;
  float                         tempOffset;
  float                         tempDiv;
  float                         tempMul;

  struct _stImudrvList          *pDriver;

  uint32_t                      tPolling;
  uint32_t                      tPollingThresh;
};


struct _stImuValue {
  uint8_t               header;
  uint8_t               sum;
  struct timespec       ts;
  uint32_t              ts_1MHz;
  uint8_t               id;
  uint8_t               seq;
  uint32_t              tsChip;

  uint32_t              flag;
#define IMUDRV_FLAG_FILLED_S16       (1   << 0)
#define IMUDRV_FLAG_FILLED_FLOAT     (2   << 0)
#define IMUDRV_FLAG_FILLED_MASK      (0xf << 0)

  uint8_t               name[8];
  int                   type;
  uint8_t               subId;

  short                 ax;
  short                 ay;
  short                 az;
  short                 gx;
  short                 gy;
  short                 gz;
  short                 temp;

  float                 axf;
  float                 ayf;
  float                 azf;
  float                 gxf;
  float                 gyf;
  float                 gzf;
  float                 tempf;
};


struct _stImudrv {
  int16_t                       valueList[0x100];
#define IMUDRV_VALUE_INVALID         (-1)

  uint8_t                       subIdInType[0x100];    // sudID in each Type  IMU0,1,2, MAG0,1,2, etc

  struct _stImudrvList       **pImudrvList;
  //int                           numChoice[CONFIG_SNESOR_NUM_MAX];
#define IMUDRV_NUMCHOICE_INVALID     (IMUDRV_ERRNO_NOMATCH)
  struct _stProbedSc            sc[CONFIG_SENSOR_NUM_MAX];
  int                           cntProbedSc;

  // temperature factor
  float                         tmprDiv;
  float                         tmprOffset;


  void                          (*pIntr[64])(int id);
  int                           idByIntr[64];
  struct timespec               tIntrById[64];                  // ptp sync time
  uint32_t                      tIntr1MHzById[64];              // internal 1MHz clock
  void                          (*pCb)(struct _stImuValue *p);

};



// the definitions of the return value
#define IMUDRV_ERRNO_NEEDNEXTCHECK   1
#define IMUDRV_SUCCESS               0
#define IMUDRV_ERRNO_UNKNOWN         (-1)
#define IMUDRV_ERRNO_NOMATCH         (-2)
#define IMUDRV_ERRNO_NOTATTACHED     (-3)
#define IMUDRV_ERRNO_DEVICENOTREADY  (-4)


#define IMUDRV_IOCTL_SET_ACCEL_ODR   0x0100
#define IMUDRV_IOCTL_SET_GYRO_ODR    0x0200
#define         SET_ODR_DIV16                   5
#define         SET_ODR_DIV8                    5
#define         SET_ODR_DIV4                    6
#define         SET_ODR_DIV2                    7
#define         SET_ODR_STANDARD                8       // 1.6kHz or 1.9kHz
#define         SET_ODR_MUL2                    9

#define IMUDRV_IOCTL_SET_ACCEL_FS    0x0800
#define         SET_ACCEL_FS_PM2G               7
#define         SET_ACCEL_FS_PM4G               8       // and otherwise
#define         SET_ACCEL_FS_PM8G               9
#define         SET_ACCEL_FS_PM16G              10

#define IMUDRV_IOCTL_SET_GYRO_FS     0x0900
#define         SET_GYRO_FS_PM250DPS            7
#define         SET_GYRO_FS_PM500DPS            8       // and otherwise
#define         SET_GYRO_FS_PM100DPS            9
#define         SET_GYRO_FS_PM2000DPS           10


struct _stImudrvList {
  // device name
  uint8_t                               name[16];
  int                                   type;           // device type
#define IMUDRV_TYPE_DEV_POS             (0)
#define IMUDRV_TYPE_DEV_MASK            (0xff << (IMUDRV_TYPE_DEV_POS))
#define IMUDRV_TYPE_DEV_IMU             ((CONFIG_SENSOR_TYPE_IMU_FLOAT)         << (IMUDRV_TYPE_DEV_POS))
#define IMUDRV_TYPE_DEV_MAGNETICS       ((CONFIG_SENSOR_TYPE_MAGNETIC_FLOAT)    << (IMUDRV_TYPE_DEV_POS))
#define ImudrvGetTypeDev(x)             ((x) & (IMUDRV_TYPE_DEV_MASK))

#define IMUDRV_TYPE_IFCONF_POS          (8)
#define IMUDRV_TYPE_IFCONF_MASK         (3 << (IMUDRV_TYPE_IFCONF_POS))
#define IMUDRV_TYPE_IFCONF_SPI          (0 << (IMUDRV_TYPE_IFCONF_POS))
#define IMUDRV_TYPE_IFCONF_I2C          (1 << (IMUDRV_TYPE_IFCONF_POS))
#define ImudrvGetTypeIfconf(x)          ((x) & (IMUDRV_TYPE_IFCONF_MASK))

#define IMUDRV_TYPE_IFDATA_POS          (10)
#define IMUDRV_TYPE_IFDATA_MASK         (3 << (IMUDRV_TYPE_IFDATA_POS))
#define IMUDRV_TYPE_IFDATA_SPI          (0 << (IMUDRV_TYPE_IFDATA_POS))
#define IMUDRV_TYPE_IFDATA_I2C          (1 << (IMUDRV_TYPE_IFDATA_POS))
#define ImudrvGetTypeIfdata(x)          ((x) & (IMUDRV_TYPE_IFDATA_MASK))

#define IMUDRV_TYPE_POLLING_POS         (12)
#define IMUDRV_TYPE_POLLING_MASK        (1 << (IMUDRV_TYPE_POLLING_POS))
#define IMUDRV_TYPE_POLLING_NO          (0 << (IMUDRV_TYPE_POLLING_POS))
#define IMUDRV_TYPE_POLLING_YES         (1 << (IMUDRV_TYPE_POLLING_POS))
#define ImudrvGetTypePolling(x)         ((x) & (IMUDRV_TYPE_POLLING_MASK))

  // register pattern list
  int                                   cntPatternList;
  //struct _stImudrvRegPattern         *pPatternList;
  const uint8_t                         *pPatternList;

  // read position, length
  uint8_t                               dataAddr;
  uint8_t                               dataLength;
  // read position, length (fifo)
  uint8_t                               fifoAddr;
  uint8_t                               fifoLength;

  // chip id for diagnosis
  uint8_t                               chipIdAddr;
  uint8_t                               chipIdValue;

  // spi speed
  struct _stSystemSpiParam              spiParam;
  struct _stSystemI2cParam              i2cParam;

  // temperature "to float" factor
  float                                 tmprDiv;
  float                                 tmprOffset;

  // data rate (default)
  uint32_t                              dataRate;       // 1600Hz, 1920Hz


  // funciton
  int                                   (*probe)(int bus);
  int                                   (*init )(int id, struct _stProbedSc *p, int powermode, int accfsr, int gyrfsr, int odr);
  int                                   (*loop )(int id, struct _stProbedSc *p);
  int                                   (*start)(int id, struct _stProbedSc *p, int odr);
  int                                   (*stop )(int id, struct _stProbedSc *p);
  int                                   (*store)(int id, int16_t *pD, uint8_t *pS);
  int                                   (*intr )(int id, struct _stProbedSc *p);
};



int             ImudrvProbe(void);
int             ImudrvInit(int id, int powermode, int accfsr, int gyrfsr, int odr);
int             ImudrvLoop(void);
int             ImudrvStart(int id, int odr);
int             ImudrvStop(int id);
void            ImudrvSetCb(void (*cb)(struct _stImuValue *p));
int             ImudrvGoCb(int id, struct _stProbedSc *pSc, struct _stImuValue *p);

int             ImudrvSetConfigSc(struct _stProbedSc *p, const uint8_t *ptr, int count);
int             ImudrvSetConfigBus(int bus, const uint8_t *ptr, int count);
int             ImudrvSetConfig16Sc(struct _stProbedSc *p, const uint8_t *ptr, int count);
int             ImudrvSetConfig16Bus(int bus, const uint8_t *ptr, int count, void *pParam);
#define       IMUDRV_SETCONFIG_WAIT     0xff
int             ImudrvReadSc(struct _stProbedSc *p, int addr, uint8_t *ptr, int len);
int             ImudrvReadBus(int bus, int addr, uint8_t *ptr, int len, struct _stProbedSc *p);

  //int             ImudrvSpiWrite(int id, int addr, uint8_t *ptr, int len);

float           ImudrvConvTmprFloat(int id, int32_t val);
uint32_t        ImudrvGetDataRate(int id);

int             ImudrvGetNumOfSensors(void);

int             ImudrvEnableInterrupt(int intr);
int             ImudrvDisableInterrupt(int intr);

#ifdef  __IMUDRV_CPP__
static int      ImudrvProbeStub(int bus, struct _stImudrvList *p);
static int      ImudrvGetInterruptPin(int bus);
static void     ImudrvInterruptDrdy10(void);
static void     ImudrvInterruptDrdy11(void);
static void     ImudrvInterruptDrdy12(void);
static void     ImudrvInterruptDrdy13(void);
static void     ImudrvInterruptDrdy00(void);
static void     ImudrvInterruptDrdy01(void);
static void     ImudrvInterruptDrdy02(void);
static void     ImudrvInterruptDrdy03(void);
static void     ImudrvInterruptDrdy(int numDrdy);

static const uint8_t *ImudrvGetSensorTypeTxt(uint8_t type);

#endif


#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif  // __IMUDRV_H__
