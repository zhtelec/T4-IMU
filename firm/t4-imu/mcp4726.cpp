#define _MCP4726_C_

#include        <string.h>
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
