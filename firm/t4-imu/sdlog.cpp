/* -*- mode: C; -*- */
#define _SDLOG_CPP_


#include        <stdint.h>

#include        <Arduino.h>
#include        <SdFat.h>
#include        <RingBuf.h>

#include        "system.h"
#include        "config.h"
#include        "ptp.h"
#include        "sdlog.h"

struct _stSdlog {
  int                   up;
  int                   fSdActive;
  uint32_t              tSwCheck;
  int                   fSw0Prev;
  int                   tSync;
  char                  fname[256];
};

struct _stSdlog         sdlog;


// Use FIFO SDIO.
#define SD_CONFIG SdioConfig(FIFO_SDIO)

// Interval between points for 25 ksps.
#define LOG_INTERVAL_USEC       40

// Size to log 10 byte lines at 25 kHz for more than ten minutes.
#define LOG_FILE_SIZE           (100*1000*1000)

#define RING_BUF_CAPACITY       (16 * 512)

SdFs            sd;
FsFile          file;

// RingBuf for File type FsFile.
RingBuf<FsFile, RING_BUF_CAPACITY> rb;


void
SdlogInit(void)
{
  memset(&sdlog, 0, sizeof(sdlog));

  sdlog.fSw0Prev = digitalRead(CONFIG_GPIO_SW0);

  if(!sd.begin(SD_CONFIG)) {
    Serial.print("# sd init error\n");
    goto fail;
  }


  sdlog.up = 1;

  //file.close();

fail:
  return;
}


int
SdlogOpen(void)
{
  int                   result = -1;
  struct timespec       ts;
  struct tm             t = {0};

  if(!sdlog.up) goto fail;

  PtpGetUnixTime(&ts);
  tzset();
  if (localtime_r(&(ts.tv_sec), &t) == NULL) goto fail;
  strftime(sdlog.fname, sizeof(sdlog.fname), "imu_%Y%m%d_%H%M.txt", &t);
  //Serial.println(sdlog.fname);

  // Open or create file - truncate existing file.
  if(!file.open(sdlog.fname, O_RDWR | O_CREAT | O_APPEND)) {
    Serial.println("# open failed\n");
    goto fail;
  }

#if 0
  // File must be pre-allocated to avoid huge
  // delays searching for free clusters.
  int err;
  if(!(err = file.preAllocate(LOG_FILE_SIZE))) {
    Serial.print("preAllocate failed ");
    Serial.println(err);
    file.close();
    goto fail;
  }
#endif

  // initialize the RingBuf.
  rb.begin(&file);

  result = 0;

fail:
  return result;
}


void
SdlogClose(void)
{
  rb.sync();
  //file.truncate();
  //file.rewind();
  file.close();

  return;
}


void
SdlogLoop(void)
{
  struct timespec       ts;
  int                   re;

  if((sdlog.tSwCheck - SystemGetUsCounter()) >= 50*SYSTEM_COUNTER_US1M000S) {
    sdlog.tSwCheck = SystemGetUsCounter();

    if(sdlog.fSw0Prev == 1 && !digitalRead(CONFIG_GPIO_SW0)) {
      if(sdlog.fSdActive == 0) {

        re = SdlogOpen();
        if(re == 0) {
          Serial.printf("# start sd record  at %lld.%09ld\n", ts.tv_sec, ts.tv_nsec);
          Serial.printf("# file name: %s\n", sdlog.fname);
          sdlog.fSdActive = 1;
          SystemSetLed0(1);

          PtpGetUnixTime(&ts);
          rb.printf("# started at %lld.%09ld\n", ts.tv_sec, ts.tv_nsec);
        } else {
          Serial.printf("# no sd or file error\n");
        }


      } else {
        sdlog.fSdActive = 0;
        SystemSetLed0(0);
        Serial.println("# stop  sd record");

        rb.sync();
        SdlogClose();

      }
    }
    sdlog.fSw0Prev = digitalRead(CONFIG_GPIO_SW0);

  }

  return;
}


void
SdlogWrite(char *str)
{
  int   cnt;

  if(sdlog.fSdActive) {
    rb.write(str);
    if (int err = rb.getWriteError()) {
      // Error caused by too few free bytes in RingBuf.
      Serial.print("WriteError ");
      Serial.println(err);
    }

    cnt = rb.bytesUsed();
    if(cnt >= (RING_BUF_CAPACITY * 7 / 8)) {
      sdlog.tSync = SystemGetUsCounter();
      rb.sync();
      sdlog.tSync -= SystemGetUsCounter();
      if(sdlog.tSync > 500) {
        Serial.printf("# rb.sync() %dB %dus\n", cnt, sdlog.tSync);
      }
    }
  }

  return;
}
