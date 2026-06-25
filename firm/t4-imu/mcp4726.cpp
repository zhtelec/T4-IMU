#define _MCP4726_C_

#include        <string.h>
#include        <stdint.h>
#include        <stdlib.h>
#include        <stdio.h>
#include        <string.h>
#include        <ctype.h>

#include        <Arduino.h>

#include        "config.h"
#include        "system.h"

#include        "mcp4726.h"

void
Mcp4726Init(int unit)
{
  return;
}

/*
 *  MCP4726 setting
 *    value range: 0 -- 0xffff
 *    set   range: 0 -- 0xfff (12bit dac)  the value is divide by 16
 */
void
Mcp4726Set(int unit, int value)
{
  uint8_t       buf[4];
  int           len;
  int           addr;

  value += 8;   // round to top 12bit
  if(value > 65536) value = 65535;

#if 0
  // update DAC value and store to EEPROM
  addr = 0x61;
  buf[0] = 0x40;
  buf[1] = (value >> 8) & 0xff;
  buf[2] = (value     ) & 0xff;
  len = 3;
  SystemI2cTransfer(0, addr, buf, len, NULL, 0, NULL);
#endif

  // update only DAC value
  addr = 0x61;
  buf[0] = (value >> 12) & 0x0f;
  buf[1] = (value >>  4) & 0xff;
  len = 2;
  SystemI2cTransfer(unit, addr, buf, len, NULL, 0, NULL);

  return;
}


void
Mcp4726SetWithEeprom(int unit, int value, int valueEeprom)
{
  uint8_t       buf[5];
  int           len;
  int           addr;

  value += 7;   // round to top 12bit
  if(value > 65536) value = 65535;

  // update DAC value and store to EEPROM
  addr = 0x61;
  buf[0] = 0x60;
  buf[1] = (value >> 8) & 0xff;
  buf[2] = (value     ) & 0xf0;
  buf[3] = (valueEeprom >> 8) & 0xff;
  buf[4] = (valueEeprom     ) & 0xf0;

  len = 5;
  SystemI2cTransfer(unit, addr, buf, len, NULL, 0, NULL);

  return;
}


void
Mcp4726Get(int unit, uint16_t *pVal, uint16_t *pValEeprom)
{
  uint8_t       bufRx[6];
  int           addr;

  addr = 0x61;
  SystemI2cTransfer(unit, addr, NULL, 0, bufRx, 5, NULL);

  //  Serial.printf("xx %x %x %x %x %x\n", bufRx[0], bufRx[1], bufRx[2], bufRx[3], bufRx[4]);
  if(pVal)       *pVal       = ((bufRx[1] <<  8) |  bufRx[2])       & 0xfff0;
  if(pValEeprom) *pValEeprom = ((bufRx[3] << 12) | (bufRx[4] << 4)) & 0xfff0;


  return;
}
