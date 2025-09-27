/* -*- mode: C; -*- */

#define _EEPROM_CPP_

#include        <Arduino.h>
#include        <EEPROM.h>
#include        <QNEthernet.h>

#include        "config.h"
#include        "system.h"

#include        "network.h"
#include        "imudrv.h"

#include        "eeprom.h"

// -1: no init
//  0: init
//  1: check sum ok
static int      eepromUp = -1;
int
EepromInit(void)
{
  int           result = -1;
  uint16_t      magic;
  uint8_t       sum;

  if(/*EEPROM.begin(CONFIG_EEPROM_SIZE)*/ 1)  {
    eepromUp = 0;
  }
  Serial.print("# eeprom: magic and sum check\n");
  if(eepromUp >= 0) {
    magic = EepromGet16(CONFIG_EEPROM_MAGIC_POS);
    sum   = EepromCalcSum();

    Serial.printf("# eeprom:   magic %x          -> %s\n", magic, (magic == CONFIG_EEPROM_MAGIC_VALUE)? "ok": "ng");
    Serial.printf("# eeprom:   sum calc:%x data:%x -> %s\n", sum, EepromGet8(CONFIG_EEPROM_SUM_POS),
                  (sum == EepromGet8(CONFIG_EEPROM_SUM_POS))?  "ok": "ng");

    if(magic == CONFIG_EEPROM_MAGIC_VALUE && sum == EepromGet8(CONFIG_EEPROM_SUM_POS)) {

    } else {
      uint8_t   val8;
      //uint16_t  val16;
      uint32_t  val32;

      // init eeprom data structure
      Serial.print("# eeprom: magic init\n");
      for(int i = 0; i < CONFIG_EEPROM_SIZE/4; i++) EepromSet32(i, 0);

      EepromSet16(CONFIG_EEPROM_MAGIC_POS,          CONFIG_EEPROM_MAGIC_VALUE);
      EepromSet8 (CONFIG_EEPROM_SSID_POS,           '\0');
      EepromSet8 (CONFIG_EEPROM_PASSWORD_POS,       '\0');
      switch(SystemGetBoardId()) {
      case CONFIG_BOARDID_T4_IMU:     val32 = CONFIG_STATIC_IP_IMU;   break;
      case CONFIG_BOARDID_T4_PTPGM:   val32 = CONFIG_STATIC_IP_PTPGM; break;
      default:                        val32 = CONFIG_STATIC_IP_ETC;   break;
      }
      EepromSet32(CONFIG_EEPROM_MYIP_POS,           val32);
      EepromSet32(CONFIG_EEPROM_NETMASK_POS,        CONFIG_MASK_IP);
      EepromSet16(CONFIG_EEPROM_MYPORT_POS,         CONFIG_MYPORT);
      EepromSet32(CONFIG_EEPROM_MULTICASTIP_POS,    CONFIG_MULTICAST_IP);
      EepromSet16(CONFIG_EEPROM_MULTICASTPORT_POS,  CONFIG_MULTICAST_PORT);
      EepromSet16(CONFIG_EEPROM_IPFLAG_POS,         0);         // DHCP
      EepromSet16(CONFIG_EEPROM_IMU_PARAM_POS,      ((IMUDRV_FORMAT_FLOAT    << CONFIG_EEPROM_IMU_PARAM_IP_SHIFT) |
                                                     (IMUDRV_FORMAT_DISABLE  << CONFIG_EEPROM_IMU_PARAM_UART_SHIFT) |
                                                     (IMUDRV_ODR_25HZ        << CONFIG_EEPROM_IMU_PARAM_ODR_SHIFT) |
                                                     (IMUDRV_GYRFSR_250DPS   << CONFIG_EEPROM_IMU_PARAM_GYR_SHIFT) |
                                                     (IMUDRV_ACCFSR_16G      << CONFIG_EEPROM_IMU_PARAM_ACC_SHIFT)));
      EepromSet16(CONFIG_EEPROM_IMU_PARAM2_POS,     0);
      val8 = 0;
      if(SystemGetBoardId() == CONFIG_BOARDID_T4_PTPGM) {
        val8 = CONFIG_EEPROM_PTP_MODE_MASTER;
      }
      EepromSet8(CONFIG_EEPROM_PTP_POS, val8);

      EepromSet32(CONFIG_EEPROM_PLL_EXTOUT_FREQ_POS,   CONFIG_GNSSDO_FREQ_EXTOUT);
      EepromSet32(CONFIG_EEPROM_PLL_OCXOOUT0_FREQ_POS, CONFIG_GNSSDO_FREQ_OCXOOUT0);
      EepromSet32(CONFIG_EEPROM_PLL_OCXOOUT1_FREQ_POS, CONFIG_GNSSDO_FREQ_OCXOOUT1);

      EepromBurn();
    }
    eepromUp = 1;
  }

  return result;
}
uint8_t
EepromCalcSum(void)
{
  uint8_t       sum = 0;

  for(int i = 1; i < CONFIG_EEPROM_SIZE; i++) {
    sum += EEPROM.read(i);
  }
  return sum;
}

uint8_t
EepromGet8(int addr)
{
  uint8_t      val;
  val  = EEPROM.read(addr);

  return val;
}
uint16_t
EepromGet16(int addr)
{
  uint16_t      val;
  val  = EEPROM.read(addr);
  val |= EEPROM.read(addr+1) <<  8;

  return val;
}
uint32_t
EepromGet32(int addr)
{
  uint32_t      val;
  val  = EEPROM.read(addr);
  val |= EEPROM.read(addr+1) <<  8;
  val |= EEPROM.read(addr+2) << 16;
  val |= EEPROM.read(addr+3) << 24;

  return val;
}
void
EepromSet8(int addr, uint8_t val)
{
  EEPROM.write(addr,    val);

  return;
}
void
EepromSet16(int addr, uint16_t val)
{
  EEPROM.write(addr,   (val      ) & 0xff);
  EEPROM.write(addr+1, (val >>  8) & 0xff);

  return;
}
void
EepromSet32(int addr, uint32_t val)
{
  EEPROM.write(addr,   (val      ) & 0xff);
  EEPROM.write(addr+1, (val >>  8) & 0xff);
  EEPROM.write(addr+2, (val >> 16) & 0xff);
  EEPROM.write(addr+3, (val >> 24) & 0xff);

  return;
}
void
EepromRead(int addr, uint8_t *ptr, int len)
{
  for(int i = 0; i < len; i++) {
    *ptr++ = EEPROM.read(addr++);
  }

  //return eepromU;
  return;
}
void
EepromWrite(int addr, uint8_t *ptr, int len)
{
  for(int i = 0; i < len; i++) {
    EEPROM.write(addr++, *ptr++);
  }

  return;
}
void
EepromBurn(void)
{
  uint8_t       sum;
  sum = EepromCalcSum();
  EepromSet8(CONFIG_EEPROM_SUM_POS, sum);
  //EEPROM.commit();
  return;
}


void
EepromCommand(int ac, char *av[])
{
  if(!strcmp(av[1], "erase")) {
    if(ac >= 3 && strtoul(av[2], NULL, 16) == 0xdeadbeef) {
      for(int i = 0; i < CONFIG_EEPROM_SIZE; i++) {
        EepromSet8(i, 0xff);
      }
    }
    //EEPROM.commit();
    Serial.print("erased\n");

  } else if(!strcmp(av[1], "showall")) {
    uint32_t    val;
    IPAddress   ip;
    Serial.printf("checksum:           %02x\n", EepromGet8(CONFIG_EEPROM_SUM_POS));
    Serial.printf("magic number:     %04x\n", EepromGet16(CONFIG_EEPROM_MAGIC_POS));
    Serial.printf("system sleep:     %04x\n", EepromGet16(CONFIG_EEPROM_SYS_SLEEP_POS));
    val = EepromGet16(CONFIG_EEPROM_IPFLAG_POS);
    Serial.printf("i/f flag:         %04x (%s)\n", val, (val&1)?"staticIP":"DHCP");

    ip = EepromGet32(CONFIG_EEPROM_MYIP_POS);
    Serial.printf("my ip:        %3d.%3d.%3d.%3d\n", ip[0], ip[1], ip[2], ip[3]);
    ip = EepromGet32(CONFIG_EEPROM_NETMASK_POS);
    Serial.printf("netmask:      %3d.%3d.%3d.%3d\n", ip[0], ip[1], ip[2], ip[3]);
    ip = EepromGet32(CONFIG_EEPROM_MULTICASTIP_POS);
    Serial.printf("multicast ip:    %3d.%3d.%3d.%3d\n", ip[0], ip[1], ip[2], ip[3]);

    Serial.printf("multicast port:    %6d\n", EepromGet16(CONFIG_EEPROM_MULTICASTPORT_POS));
    val = EepromGet16(CONFIG_EEPROM_IMU_PARAM_POS);
    Serial.printf("imu param:        %04x [15:14]ip, [13:12]uart, [11:8]odr, [7:4]gyr, [3:0]acc\n", val);
    val = EepromGet8(CONFIG_EEPROM_IMU_PARAM2_POS);
    Serial.printf("imu param2:         %02x                                           [7:4]rsv, [3:0]speed\n", val);

    val = EepromGet8(CONFIG_EEPROM_PTP_POS);
    Serial.printf("ptp mode:           %02x                                           [7:1]rsv, [0]master en\n", val);

    val = EepromGet32(CONFIG_EEPROM_PLL_EXTOUT_FREQ_POS);
    Serial.printf("clock out(T4-PTPGM):   %9d\n", val);
    val = EepromGet32(CONFIG_EEPROM_PLL_OCXOOUT0_FREQ_POS);
    Serial.printf("clock out0(T4-VCOCXO): %9d\n", val);
    val = EepromGet32(CONFIG_EEPROM_PLL_OCXOOUT1_FREQ_POS);
    Serial.printf("clock out1(T4-VCOCXO): %9d\n", val);

  }

  return;
}
