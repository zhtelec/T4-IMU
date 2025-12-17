/* -*- mode: C; -*- */
#define _TOP_CPP_

#include        <stdint.h>
#include        <stdio.h>

#include        <Arduino.h>
#include        <QNEthernet.h>
#include        <InternalTemperature.h>

#include        "config.h"
#include        "system.h"

#include        "imu.h"
#include        "imudrv.h"
#include        "command.h"
#include        "network.h"
#include        "ptp.h"
#include        "eeprom.h"
#include        "gnss.h"
#include        "sdlog.h"
#include        "mcp4726.h"
#include        "gnssdo.h"
#include        "fifo.h"


#include        "top.h"

extern int      imuFmtUart;
extern int      imuFmtMulticast;


static const char *boardName[] = {
  "T4-IMU",
  "T4-PTPGM",
  "T4-SENSECAP",
  "N/A",
  "N/A",
  "N/A",
  "N/A",
  "N/A",
};

void putchar_(char character)
{
  Serial.print(character);
  Serial.print(' ');
}

void
TopInit(void)
{
  int           boardId;
  uint8_t       mac[6];

  // port settings
  SystemPinSettings();
  Serial.begin(115200);
  SystemWaitCounter(500);
  FifoInit();

  Serial.printf("--------\n# version: %s\n", CONFIG_VERSION_TEXT);

  boardId = SystemGetBoardId();
  Serial.printf("# board id: %d (%s)\n", boardId, boardName[boardId & 7]);
  Serial.printf("# cpu clock: %dHz\n", F_CPU_ACTUAL);
  Serial.printf("# bus clock: %dHz\n", F_BUS_ACTUAL);

  qindesign::network::Ethernet.macAddress(mac);
  Serial.printf("# mac addr:  %02x:%02x:%02x:%02x:%02x:%02x\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  SystemSpiInit(-1);            // init spi
  SystemSpiInit(1);             // init spi bus1

  SystemI2cInit(-1);
  SystemI2cInit(0);
  SystemI2cInit(1);

  EepromInit();
  CommandInit();

  NetworkInit();                // network settings
  NetworkInitUdp();

  int           id;
  id = SystemGetBoardId();
  if(id == CONFIG_BOARDID_T4_PTPGM) {
    GnssInit();
    GnssdoInit();
  } else if(id == CONFIG_BOARDID_T4_IMU) {
    SystemCtrloutInit(0, 20);           // for test
    SystemCtrloutInit(1, 100);          // for test

    ImuInit();                          // imu settings
    SdlogInit();

  } else if(id == CONFIG_BOARDID_T4_SENSECAP) {
    SystemSpiInit(0);                   // init spi bus0
    ImuInit();                          // imu settings
  }

  return;
}


void
TopLoop(void)
{
  int           id;
  CommandLoop();
  NetworkLoop();
  id = SystemGetBoardId();
  if(id == CONFIG_BOARDID_T4_PTPGM) {
    GnssLoop();
    GnssdoLoop();
  } else if(id == CONFIG_BOARDID_T4_IMU) {
    ImuLoop();
    SdlogLoop();
  } else if(id == CONFIG_BOARDID_T4_SENSECAP) {
    ImuLoop();
  }


#if 0
  static uint32_t       t;
  if((t - SystemGetCounter()) >= SYSTEM_COUNTER_1S000) {
    t = SystemGetCounter();
    Serial.printf("%f\n", InternalTemperature.readTemperatureC());
  }
#endif

  return;
}
