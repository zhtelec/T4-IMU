#ifndef _CONFIG_H_
#define _CONFIG_H_


#define CONFIG_VERSION_TEXT                     "0.00.00.202512170"


// GPIO
#define CONFIG_GPIO_IMU_DRDY10                  (0)     // SPI1 CS0X IMU0
#define CONFIG_GPIO_IMU_DRDY11                  (1)     // SPI1 CS1X IMU1
#define CONFIG_GPIO_SW0                         (2)
#define CONFIG_GPIO_SW1                         (3)
#define CONFIG_GPIO_EN_EXTOUT                   (9)
#define CONFIG_GPIO_CTRLOUT1                    (11)    // ctrl out1
#define CONFIG_GPIO_LED                         (13)
#define CONFIG_GPIO_CTRLOUT0                    (14)    // ctrl out0
#define CONFIG_GPIO_PTP_PPSIN                   (15)    // pin15 -- ENET_TIMER_IC
#define CONFIG_GPIO_PTP_PPSMASK                 (20)    // 
#define CONFIG_GPIO_PTP_PPSOUT                  (24)    // ptp slave  ppsout
#define CONFIG_GPIO_IMU_SCK                     (27)
#define CONFIG_GPIO_IMU_MOSI                    (26)
#define CONFIG_GPIO_IMU_MISO                    (39)
#define CONFIG_GPIO_IMU_CS0X                    (36)
#define CONFIG_GPIO_IMU_CS1X                    (37)
#define CONFIG_GPIO_IMU_CS2X                    (38)
#define CONFIG_GPIO_IMU_CS13X                   (15)    // T4-SENSECAP
#define CONFIG_GPIO_IMU_CS2X_FLASH              (CONFIG_GPIO_IMU_CS2X)
#define CONFIG_GPIO_IMU_POWER                   (32)    // spresense POWER
#define CONFIG_GPIO_IMU_RESETX                  (33)    // spresense XRST
#define CONFIG_GPIO_IMU_SCK0                    (13)
#define CONFIG_GPIO_IMU_MOSI0                   (11)
#define CONFIG_GPIO_IMU_MISO0                   (12)
#define CONFIG_GPIO_IMU_CS00X                   (14)      // T4-SENSECAP
#define CONFIG_GPIO_IMU_CS01X                   (25)      // T4-SENSECAP
#define CONFIG_GPIO_IMU_CS02X                   (23)      // T4-SENSECAP
#define CONFIG_GPIO_IMU_CS03X                   (22)      // T4-SENSECAP

#define CONFIG_GPIO_LED0_T4_IMU                 (40)
#define CONFIG_GPIO_LED0_T4_PTPGM               (23)
#define CONFIG_GPIO_LED0                        (23)

#define CONFIG_GPIO_I2C_SDA                     (18)
#define CONFIG_GPIO_I2C_SCL                     (19)
#define CONFIG_GPIO_I2C1_SDA                    (17)
#define CONFIG_GPIO_I2C1_SCL                    (16)

#define CONFIG_GPIO_GPT2_IC1                    (15)    // for gps pps ic
#define CONFIG_GPIO_GPT2_IC2                    (41)    // for slave pps ic

// GNSS
#define CONFIG_GPIO_EN_EXTOCXO                  (42)    // EN_EXTOCXO  0: exist, 1: no
#define CONFIG_GPIO_SEL_24MHZ                   (43)    // SEL 24MHz/OCXO  0: EXT OCXO, 1: CPU CLOCK 24MHZ

#define CONFIG_GPIO_IMU_DRDY12                  (4)     // SPI1 CS2X IMU2
#define CONFIG_GPIO_IMU_DRDY13                  (5)     // SPI1 CS3X IMU3
#define CONFIG_GPIO_IMU_DRDY00                  (6)     // SPI0 CS0X IMU4
#define CONFIG_GPIO_IMU_DRDY01                  (8)     // SPI0 CS1X IMU5
#define CONFIG_GPIO_IMU_DRDY02                  (9)     // SPI0 CS2X IMU6
#define CONFIG_GPIO_IMU_DRDY03                  (10)    // SPI0 CS3X IMU7
#define CONFIG_B2_GPIO_LED_L                    (41)    // SENSECAP LED L

//#define CONFIG_GPIO_SERIAL1_RXD                 (52)    // orignal is pin 0
//#define CONFIG_GPIO_SERIAL1_TXD                 (53)    // orignal is pin 1
#define CONFIG_GPIO_SERIAL2_RXD                 (7)
#define CONFIG_GPIO_SERIAL2_TXD                 (8)


// board id
#define CONFIG_BOARDID_T4_IMU                   0
#define CONFIG_BOARDID_T4_PTPGM                 1
#define CONFIG_BOARDID_T4_SENSECAP              2
#define CONFIG_BOARDID_T4_ID3                   3
#define CONFIG_BOARDID_T4_ID4                   4
#define CONFIG_BOARDID_T4_ID5                   5
#define CONFIG_BOARDID_T4_ID6                   6
#define CONFIG_BOARDID_T4_ID7                   7




// SPI
#define CONFIG_SPI_SPEED_DEFAULT                (1*1000*1000)
#define CONFIG_SPI_RECOVER_TIME                 (10*SYSTEM_COUNTER_US1U000S)


// I2C
#define CONFIG_I2C_ADDR_TMP112                  0x48    // bus1


// EEPROM
#define CONFIG_EEPROM_SIZE                      256

#define CONFIG_EEPROM_SUM_POS                   0x000           //  1bytes
#define CONFIG_EEPROM_SUM_LEN                   1
#define CONFIG_EEPROM_MAGIC_POS                 0x002           //  2bytes
#define CONFIG_EEPROM_MAGIC_LEN                 2
#define CONFIG_EEPROM_MAGIC_VALUE               0x55aa
#define CONFIG_EEPROM_SYS_SLEEP_POS             0x004           //  2bytes
#define CONFIG_EEPROM_SYS_SLEEP_LEN             2
#define CONFIG_EEPROM_SYS_SLEEP_DISABLE         0xffff          // 0xffff: disable, otherwise: sec to sleep

//#define CONFIG_EEPROM_SYS_ROOMID_POS            0x00f           //  1bytes
#define CONFIG_EEPROM_SYS_ROOMNAME_POS          0x010           // 16bytes
#define CONFIG_EEPROM_SYS_ROOMNAME_LEN          16
#define CONFIG_EEPROM_SSID_POS                  0x020           // 32bytes
#define CONFIG_EEPROM_SSID_LEN                  32
#define CONFIG_EEPROM_PASSWORD_POS              0x040           // 32bytes
#define CONFIG_EEPROM_PASSWORD_LEN              32
#define CONFIG_EEPROM_MYIP_POS                  0x060           //  4bytes
#define CONFIG_EEPROM_MYIP_LEN                  4
#define CONFIG_EEPROM_NETMASK_POS               0x064           //  4bytes
#define CONFIG_EEPROM_NETMASK_LEN               4
#define CONFIG_EEPROM_GATEWAY_POS               0x068           //  4bytes
#define CONFIG_EEPROM_GATEWAY_LEN               4
#define CONFIG_EEPROM_DNS1_POS                  0x06c           //  4bytes
#define CONFIG_EEPROM_DNS1_LEN                  4
#define CONFIG_EEPROM_DNS2_POS                  0x070           //  4bytes
#define CONFIG_EEPROM_DNS2_LEN                  4
#define CONFIG_EEPROM_MYPORT_POS                0x074           //  2bytes
#define CONFIG_EEPROM_MYPORT_LEN                2
#define CONFIG_EEPROM_IPFLAG_POS                0x076           //  2bytes
#define CONFIG_EEPROM_IPFLAG_LEN                2
#define         CONFIG_EEPROM_IPFLAG_STATIC_SHIFT       0
#define         CONFIG_EEPROM_IPFLAG_STATIC_MASK        (1 << (CONFIG_EEPROM_IPFLAG_STATIC_SHIFT))
#define         CONFIG_EEPROM_IPFLAG_STATIC_NO          (0 << (CONFIG_EEPROM_IPFLAG_STATIC_SHIFT))
#define         CONFIG_EEPROM_IPFLAG_STATIC_YES         (1 << (CONFIG_EEPROM_IPFLAG_STATIC_SHIFT))

#define CONFIG_EEPROM_MULTICASTIP_POS           0x078           //  4bytes
#define CONFIG_EEPROM_MULTICASTIP_LEN           4
#define CONFIG_EEPROM_MULTICASTPORT_POS         0x07c           //  2bytes
#define CONFIG_EEPROM_MULTICASTPORT_LEN         2

#define CONFIG_EEPROM_PTP_POS                   0x80
#define CONFIG_EEPROM_PTP_LEN                   1
#define         CONFIG_EEPROM_PTP_MODE_SHIFT            0
#define         CONFIG_EEPROM_PTP_MODE_MASK             (1 << (CONFIG_EEPROM_PTP_MODE_SHIFT))           // 
#define         CONFIG_EEPROM_PTP_MODE_ONLYSLAVE        (0 << (CONFIG_EEPROM_PTP_MODE_SHIFT))           // slave
#define         CONFIG_EEPROM_PTP_MODE_MASTER           (1 << (CONFIG_EEPROM_PTP_MODE_SHIFT))           // master and slvae

#define CONFIG_EEPROM_IMU_PARAM_POS             0xa0    // [7:4] gyrfsr, [3:0] accfsr
#define CONFIG_EEPROM_IMU_PARAM_LEN             2
#define CONFIG_EEPROM_IMU_PARAM_ACC_SHIFT       0
#define CONFIG_EEPROM_IMU_PARAM_ACC_MASK        (0xf << (CONFIG_EEPROM_IMU_PARAM_ACC_SHIFT))
#define CONFIG_EEPROM_IMU_PARAM_GYR_SHIFT       4
#define CONFIG_EEPROM_IMU_PARAM_GYR_MASK        (0xf << (CONFIG_EEPROM_IMU_PARAM_GYR_SHIFT))
#define CONFIG_EEPROM_IMU_PARAM_ODR_SHIFT       8
#define CONFIG_EEPROM_IMU_PARAM_ODR_MASK        (0xf << (CONFIG_EEPROM_IMU_PARAM_ODR_SHIFT))
#define CONFIG_EEPROM_IMU_PARAM_UART_SHIFT      12
#define CONFIG_EEPROM_IMU_PARAM_UART_MASK       (0x3 << (CONFIG_EEPROM_IMU_PARAM_UART_SHIFT))
#define CONFIG_EEPROM_IMU_PARAM_IP_SHIFT        14
#define CONFIG_EEPROM_IMU_PARAM_IP_MASK         (3 << (CONFIG_EEPROM_IMU_PARAM_IP_SHIFT))
#define CONFIG_EEPROM_IMU_PARAM2_POS            0xa2
#define CONFIG_EEPROM_IMU_PARAM2_LEN            1
#define CONFIG_EEPROM_IMU_PARAM2_SPEED_SHIFT    0       // spi speed in MHz  0: disable, 1--15: 1--15MHz
#define CONFIG_EEPROM_IMU_PARAM2_SPEED_MASK     (0xf << (CONFIG_EEPROM_IMU_PARAM2_SPEED_SHIFT))
#define CONFIG_EEPROM_IMU_PARAM2_RESERVED_SHIFT 4
#define CONFIG_EEPROM_IMU_PARAM2_RESERVED_MASK  (0xf << (CONFIG_EEPROM_IMU_PARAM_RESERVED_SHIFT))


#define CONFIG_EEPROM_IMU_RESERVED_POS          0xa3
#define CONFIG_EEPROM_IMU_RESERVED_LEN          1

#define CONFIG_EEPROM_PLL_EXTOUT_FREQ_POS       0xc0    // "gnssdo setfreq 0 xxxx"
#define CONFIG_EEPROM_PLL_EXTOUT_FREQ_LEN       4
#define CONFIG_EEPROM_PLL_OCXOOUT0_FREQ_POS     0xc4    // "gnssdo setfreq 10 xxxx"
#define CONFIG_EEPROM_PLL_OCXOOUT0_FREQ_LEN     4
#define CONFIG_EEPROM_PLL_OCXOOUT1_FREQ_POS     0xc8    // "gnssdo setfreq 11 xxxx"
#define CONFIG_EEPROM_PLL_OCXOOUT1_FREQ_LEN     4


// IP
#define CONFIG_STATIC_IP_IMU                    NetworkIpStrToIpAddress((char *)"192.168.1.11")
#define CONFIG_STATIC_IP_PTPGM                  NetworkIpStrToIpAddress((char *)"192.168.1.10")
#define CONFIG_STATIC_IP_ETC                    NetworkIpStrToIpAddress((char *)"192.168.1.30")
#define CONFIG_MASK_IP                          NetworkIpStrToIpAddress((char *)"255.255.255.0")
#define CONFIG_MYPORT                           10000
#define CONFIG_MULTICAST_IP                     NetworkIpStrToIpAddress((char *)"239.0.0.1")
#define CONFIG_MULTICAST_PORT                   23901
#define CONFIG_NETWORK_HOSTNAME_T4_IMU          "t4-imu"
#define CONFIG_NETWORK_HOSTNAME_T4_PTPGM        "t4-ptpgm"
#define CONFIG_NETWORK_HOSTNAME_T4_SENSECAP     "t4-sensecap"
#define CONFIG_NETWORK_HOSTNAME_T4_ID3          "id3"
#define CONFIG_NETWORK_HOSTNAME_T4_ID4          "id4"
#define CONFIG_NETWORK_HOSTNAME_T4_ID5          "id5"
#define CONFIG_NETWORK_HOSTNAME_T4_ID6          "id6"
#define CONFIG_NETWORK_HOSTNAME_T4_ID7          "id7"


// IMU
#define CONFIG_SENSOR_NUM_MAX                   8
#define CONFIG_IMU_CALC_TIME_PULSE              4       // 0: disable otherwise: enable and port
#define CONFIG_SENSOR_USE_FIFO                  1
#define CONFIG_SENSOR_FIFO_USE_MALLOC           1
#define CONFIG_SENSOR_FIFO_SIZE                 9       // 2^x
#define CONFIG_SENSOR_FIFO_THRESHOLD            20      //


// GNSS
#define CONFIG_GNSS_SERIAL_IF                   Serial2
#define CONGIG_GNSS_SERIAL_BAUD                 921600
#define CONGIG_GNSS_SERIAL_RXBUF_SIZE           2048
#define CONGIG_GNSS_UPDATE_INTERVAL             50000  // in us
#define CONFIG_GNSS_VAL10E9                     (1000*1000*1000)
#define CONFIG_GNSS_LOCK_COUNT                  (5)


// GNSSDO
#define CONFIG_GNSSDO_IDX_24MHZ                 0
#define CONFIG_GNSSDO_IDX_OCXO                  1

#define CONFIG_GNSSDO_FREQ_EXTOUT               (10*1000*1000)
#define CONFIG_GNSSDO_FREQ_OCXOOUT0             (25000*1000)
#define CONFIG_GNSSDO_FREQ_OCXOOUT1             (24576*1000)

#define CONFIG_GNSSDO_DIFF_HISTORY_LEN_MAX      2048
#define CONFIG_GNSSDO_DIFF_HISTORY_LEN_MIN      1

#define CONFIG_GNSSDO_LOCK_PPB_OCXO             (  1.0/(1000.0*1000000.0))        //   1ppb
#define CONFIG_GNSSDO_LOCK_PPB_24MHZ            (500.0/(1000.0*1000000.0))        // 0.5ppm

#define CONFIG_GNSSDO_VCOCXO_FREQ_TICK          (24.0/1000000.0)

// T4-PTPGM 1.00
#if 1
#define CONfIG_GNSSDO_PID_24MHZ_KP              (10.0)
#define CONfIG_GNSSDO_PID_24MHZ_KI              (2.0)
#define CONfIG_GNSSDO_PID_24MHZ_KD              (0.02)
#define CONfIG_GNSSDO_PID_24MHZ_OFFSET          34300.0
#endif
#if 0
#define CONfIG_GNSSDO_PID_24MHZ_KP              (-10.0)
#define CONfIG_GNSSDO_PID_24MHZ_KI              (-2.0)
#define CONfIG_GNSSDO_PID_24MHZ_KD              (-0.02)
#define CONfIG_GNSSDO_PID_24MHZ_OFFSET          38300.0
#endif

#define CONfIG_GNSSDO_PID_OCXO_KP               (-300.0)
#define CONfIG_GNSSDO_PID_OCXO_KI               (-48000.0)
#define CONfIG_GNSSDO_PID_OCXO_KD               (-10.0)
#define CONfIG_GNSSDO_PID_OCXO_KI_FINEFACTOR    (1/4)
#define CONfIG_GNSSDO_PID_OCXO_OFFSET           36000.0

#define CONFIG_GNSSDO_DAC_TICK                  0x10

// TIME
#define timegm(tm)                              System_timegm1(tm)
#define CONFIG_TIME_LEAPSEC_BWT_1970TO1980      19              // TAI -19


// VCOCXO --> PLL -> 24MHz/10MHz
// 24MHz VCXO -- PLL -- 50MHz PeriClock
#define CONFIG_GPT1_CLKIN_VALUE_10MHZ           (10*1000*1000)
#define CONFIG_GPT1_CLKIN_VALUE_24MHZ           (24*1000*1000)
#define CONFIG_GPT1_IC_VAL_MIN                  ((CONFIG_GPT1_CLKIN_VALUE) / 1000 *  999)       // bottom limit
#define CONFIG_GPT1_IC_VAL_MAX                  ((CONFIG_GPT1_CLKIN_VALUE) / 1000 * 1001)       // top limit

#define CONFIG_GPT2_CLKIN_VALUE                 (24000000)
#define CONFIG_GPT2_IC_VAL_MIN                  ((CONFIG_GPT2_CLKIN_VALUE) / 1000 *  999)      // bottom limit
#define CONFIG_GPT2_IC_VAL_MAX                  ((CONFIG_GPT2_CLKIN_VALUE) / 1000 * 1001)      // top limit


// CTRLOUT
//#define CONFIG_CAM_GPT_IC_VAL_MIN               (10000000 /1000 *  999)       // bottom limit
//#define CONFIG_CAM_GPT_IC_VAL_MAX               (10000000 /1000 * 1001)       // top limit
#define CONFIG_CTRLOUT0_FREQ_DEFAULT             60
#define CONFIG_CTRLOUT0_PULSE_US                 100
#define CONFIG_CTRLOUT0_POL_NEGATIVE             1
//#define CONFIG_CTRLOUT0_PWM_FREQ                 3611
#define CONFIG_CTRLOUT0_PWM_RESO                 16
//#define CONFIG_CTRLOUT0_PWM_PRESCALER            4


#endif
