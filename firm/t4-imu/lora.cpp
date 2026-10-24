#define         _LORA_CPP_

#include        <stdint.h>
#include        <stdio.h>

#include        <Arduino.h>

#include        "config.h"
#include        "system.h"

#include        "lora.h"

#define LORA_DATA_BUF_SIZE      (CONFIG_LORA_DATA_BUF_SIZE)
#define LORA_SERIAL_BUF_SIZE    (CONFIG_LORA_SERIAL_BUF_SIZE)

static const char       NIBBLE2HEX[] = {'0', '1', '2', '3',  '4', '5', '6', '7',  '8', '9', 'A', 'B',  'C', 'D', 'E', 'F'};


struct _stLora {
  int           up;
  int           seq;
  int           txFlag;
  uint32_t      txTout;

  char          txDataBuf[LORA_DATA_BUF_SIZE];
  int           txDataPos;
  int           txDataLen;

  char          txSerialBuf[LORA_SERIAL_BUF_SIZE];
  int           txSerialLen;

  char          serialTmp[LORA_SERIAL_BUF_SIZE];
  int           serialTmpLen;
  char          serialBuf[LORA_SERIAL_BUF_SIZE];
  int           serialBufLen;

};
static struct _stLora   lora;

void
LoraInit(void)
{
  int           re;
  char          buf[LORA_SERIAL_BUF_SIZE];

  memset(&lora, 0, sizeof(lora));

  re = LoraSetBaudrate();
  if(re < 0) goto fail;

  CONFIG_LORA_SERIAL_IF.begin(CONGIG_LORA_SERIAL_BAUD);
  LoraSerialFlush();
  SystemWaitCounter(5);

  LoraSerialPuts("\nAT\n");
  LoraSerialGetsPoll(buf, sizeof(buf), 50);

  LoraSerialPuts("\nAT+VER\n");
  LoraSerialGetsPoll(buf, sizeof(buf), 50);

  LoraSerialPuts("\nAT+MODE=TEST\n");
  LoraSerialGetsPoll(buf, sizeof(buf), 300);

  LoraSerialPuts("\nAT+TEST=" CONFIG_LORA_RFCFG "\n");
  LoraSerialGetsPoll(buf, sizeof(buf), 100);

  lora.up = 1;

fail:
  return;
}


void
LoraLoop(void)
{
  char      buf[LORA_SERIAL_BUF_SIZE];
  int       len;

  if(!lora.up) return;

  LoraSerialFetch();

  len = LoraSerialGets(buf, sizeof(buf), 100);
  if(len > 0) {
    //Serial.printf("xxxx +txlrpkt -> %3d [%s]\n", len, buf);
  }

  if(lora.txFlag) {
    if((len > 0 && !strcmp(buf, "+TEST: TX DONE")) || (lora.txTout - SystemGetCounter()) >= 1500) {
      lora.txFlag = 0;
    }

  } else {
    if(lora.txDataLen) {
      const char        *p;
      int               pos = 0;
      int               len;
      uint8_t           id, sum = 0;

      len = lora.txDataLen;
      if(len > CONFIG_LORA_MAX_BYTE_LEN) len = CONFIG_LORA_MAX_BYTE_LEN;
      strncpy(lora.txSerialBuf, "AT+TEST=TXLRPKT,\"", LORA_SERIAL_BUF_SIZE);
      pos = 17;

      // header
      uint8_t           header[4];
      header[0] = 0x50 | (lora.seq & 0xf);     // seq
      header[1] = len + 4;        // length: header + data
      header[2] = 0x01;           // id
      header[3] = 0;              // sum

      for(int i = 0; i < sizeof(header)-1; i++) {
        sum += header[i];
        lora.txSerialBuf[pos++] = NIBBLE2HEX[(header[i] >> 4) & 0xf];
        lora.txSerialBuf[pos++] = NIBBLE2HEX[ header[i]       & 0xf];
      }

      p = &lora.txDataBuf[lora.txDataPos];
      for(int i = 0; i < len; i++) {
        sum += *p++;
      }
      header[3] = sum;
      lora.txSerialBuf[pos++] = NIBBLE2HEX[(header[3] >> 4) & 0xf];
      lora.txSerialBuf[pos++] = NIBBLE2HEX[ header[3]       & 0xf];

      // data
      p = &lora.txDataBuf[lora.txDataPos];
      for(int i = 0; i < len; i++) {
        lora.txSerialBuf[pos++] = NIBBLE2HEX[(*p >> 4) & 0xf];
        lora.txSerialBuf[pos++] = NIBBLE2HEX[ *p       & 0xf];
        p++;
      }
      lora.txSerialBuf[pos++] = '\"';
      lora.txSerialBuf[pos++] = '\n';
      lora.txSerialBuf[pos]   = '\0';
      lora.txSerialLen = pos;
      LoraSerialPuts(lora.txSerialBuf);
      lora.txTout = SystemGetCounter();
      lora.txFlag = 1;

      lora.seq++;

      lora.txDataLen -= len;
      lora.txDataPos += len;
      if(!lora.txDataLen) {
        lora.txDataPos = 0;
      }
    }

  }

  return;
}


int
LoraSend(const char *ptr, int len)
{
  int   result = -1;

  if(!lora.up) return result;

  // exit, if the buffer is not empty.
  if(lora.txDataLen != 0) goto fail;

  if(len > LORA_DATA_BUF_SIZE) len = LORA_DATA_BUF_SIZE;
  memcpy(lora.txDataBuf, ptr, len);
  lora.txDataLen = len;

  result = len;

fail:
  return result;
}


//
// return value: -1: no device, 0: the baud is already set, 1: the baud was changed
//
static const int        baud[] = {
  CONGIG_LORA_SERIAL_BAUD, 9600, 14400, 19200, 38400, 57600, 76800, 115200, 230400,
};
static int
LoraSetBaudrate(void)
{
  int           result = -1;            // no device
  char          buf[256];
  int           len;

  // set baud 230400bps
  for(int i = 0; i < (int)(sizeof(baud)/sizeof(int)); i++) {
    Serial.printf("# lora: change baud %d       \r", baud[i]);
    Serial.flush();

    CONFIG_LORA_SERIAL_IF.begin(baud[i]);
    SystemWaitCounter(5);
    LoraSerialFlush();

    LoraSerialPuts("AT\n");
    buf[0] = '\0';
    len = LoraSerialGetsPoll(buf, sizeof(buf), 50);

    LoraSerialPuts("AT\n");
    buf[0] = '\0';
    len = LoraSerialGetsPoll(buf, sizeof(buf), 50);

    if(len > 0 && !strcmp(buf, "+AT: OK")) {
      result = 0;       // already selected
      if(baud[i] != CONGIG_LORA_SERIAL_BAUD) {
        // set baud
        snprintf(buf, sizeof(buf), "AT+UART=BR,%d\n", CONGIG_LORA_SERIAL_BAUD);
        LoraSerialPuts(buf);
        buf[0] = '\0';
        len = LoraSerialGetsPoll(buf, sizeof(buf), 500);
        //Serial.printf("xxxx +at -> [%s] %d\n", buf, len);
        // reset
        LoraSerialPuts("AT+RESET\n");
        len = LoraSerialGetsPoll(buf, sizeof(buf), 50);
        //Serial.printf("xxxx +at -> [%s] %d\n", buf, len);
        SystemWaitCounter(100);
        result = 1;     // selected
      }
      Serial.printf("# lora: device detected  baud:%d\n", CONGIG_LORA_SERIAL_BAUD);

      CONFIG_LORA_SERIAL_IF.begin(CONGIG_LORA_SERIAL_BAUD);
      LoraSerialFlush();

      break;
    }
  }

  return result;
}


static void
LoraSerialFlush(void)
{
  while(CONFIG_LORA_SERIAL_IF.available() > 0) {
    CONFIG_LORA_SERIAL_IF.read();
  }

  return;
}


static int
LoraSerialPuts(const char *str)
{
  CONFIG_LORA_SERIAL_IF.print(str);

  return strlen(str);
}


static int
LoraSerialGets(char *str, int len, uint32_t timeout_ms)
{
  int           sz = 0;

  if(lora.serialBufLen > 0) {
    if(len > lora.serialBufLen) len = lora.serialBufLen;
    memcpy(str, lora.serialBuf, len);
    str[len] = '\0';
    lora.serialBufLen = 0;
    sz = len;
  }

  return sz;
}


static int
LoraSerialGetsPoll(char *str, int len, uint32_t timeout_ms)
{
  uint32_t      t;
  int           sz = 0;

  t = SystemGetCounter();
  while((t - SystemGetCounter()) <= timeout_ms) {
    if(lora.serialBufLen > 0) {
      if(len > lora.serialBufLen) len = lora.serialBufLen;
      memcpy(str, lora.serialBuf, len);
      str[len] = '\0';
      lora.serialBufLen = 0;
      sz = len;
      break;
    }

    LoraSerialFetch();
  }

  return sz;
}


static void
LoraSerialFetch(void)
{
  char  c;

  if(CONFIG_LORA_SERIAL_IF.available() > 0) {
    c = CONFIG_LORA_SERIAL_IF.read();
    //Serial.printf("%02x ", c);
    if(c == '\r') {

    } else if(c == '\n') {
      //Serial.printf("\n");

      memcpy(lora.serialBuf, lora.serialTmp, lora.serialTmpLen);
      lora.serialBufLen = lora.serialTmpLen;
      lora.serialBuf[lora.serialBufLen] = '\0';
      lora.serialTmpLen = 0;

    } else {
      lora.serialTmp[lora.serialTmpLen++] = c;
    }
  }

  return;
}


static void
LoraSerialResponseAndPrint(void)
{
  char          buf[LORA_SERIAL_BUF_SIZE];
  int           len;
  uint32_t      t;

  t = SystemGetCounter();
  while((t - SystemGetCounter()) < 500) {
    LoraSerialFetch();
    len = LoraSerialGets(buf, sizeof(buf), 10);

    if(len < 0) break;

    if(len > 0) {
      Serial.printf("xxxx AT -> [%s] %d\n", buf, len);
      break;
    }
  }

  return;
}
