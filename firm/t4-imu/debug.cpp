/* -*- mode: C; -*- */
#define _DEBUG_CPP_


#include        <stdint.h>
#include        <stdio.h>
#include        <Arduino.h>

#include        "config.h"
#include        "system.h"

#include        "debug.h"

struct _stDebug {
  uint32_t      t1sec;
  uint32_t      tUartAs6668;
};
static struct _stDebug         debug;


void
DebugInit(void)
{
  memset(&debug, 0, sizeof(debug));

#if CONFIG_DEBUG_UART_AS6668
  DebugUartAs6668Init();
#endif

  return;
}


void
DebugLoop(void)
{

#if CONFIG_DEBUG_UART_AS6668
  DebugUartAs6668Loop();
#endif

  if((debug.t1sec - SystemGetCounter()) > SYSTEM_COUNTER_1S000) {
    debug.t1sec = SystemGetCounter();

  }

  return;
}

static void
DebugUartAs6668Init(void)
{
  // the refresh rate is change 5Hz
  //Serial5.print("$PCAS02,200*2A\r\n");
  //Serial7.print("$PCAS02,200*2A\r\n");

  // the refresh rate is change 1Hz
  //Serial5.print("$PCAS02,1000*0F\r\n");
  //Serial7.print("$PCAS02,1000*0F\r\n");

  return;
}
static void
DebugUartAs6668Loop(void)
{
  while(Serial5.available() > 0) {
    Serial.write(Serial5.read());
  }
  while(Serial7.available() > 0) {
    Serial.write(Serial7.read());
  }

  if((debug.tUartAs6668 - SystemGetCounter()) > SYSTEM_COUNTER_1S000) {
    debug.tUartAs6668 = SystemGetCounter();

    //Serial5.print("$PCAS02,200*2A\r\n");
    //Serial7.print("$PCAS02,200*2A\r\n");
    Serial5.print("$PCAS02,1000*0F\r\n");
    Serial7.print("$PCAS02,1000*0F\r\n");
  }

  return;
}
