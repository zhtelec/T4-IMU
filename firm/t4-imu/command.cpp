/* -*- mode: C; -*- */

#define	_COMMAND_C_

#include        <string.h>
#include        <stdlib.h>
#include        <stdio.h>
#include        <string.h>
#include        <ctype.h>

#include        <Arduino.h>
#include        <QNEthernet.h>

#include        "config.h"
#include        "eeprom.h"
#include        "network.h"
#include        "system.h"
#include        "imu.h"
#include        "mcp4726.h"
#include        "gnssdo.h"
#include        "gnss.h"

#include        "command.h"
#include        "help.h"

extern char     mainVersionFirm[];

//extern struct _stMain           sys;
static struct _stCommand        command;

static int                      cmdac;
static char                     *cmdav[16];
uint32_t                        debug;
int                             debugId;

void
CommandInit(void)
{
  memset(&command, 0, sizeof(command));
  CommandClearParse(0);

  return;
}


void
CommandLoop(void)
{
  int                   unit;

  unit = 0;

#if 0
  unsigned char         c;

  while(Serial.available()) {
    c = Serial.read();

    if(c == '\b') {
      if(command.posRecv > 0) {
        command.posRecv--;
        Serial.print(" \b");
      }

    } else if(c == '\r' || c == '\n') {
      Serial.print("\n");
      command.bufRecv[command.posRecv] = '\0';
      CommandParser(unit, (char *)command.bufRecv, command.posRecv);
      command.posRecv = 0;

    } else {
      if(command.posRecv < COMMAND_RECV_BUFFER_SIZE - 1) {
        Serial.printf("%c", c);
        command.bufRecv[command.posRecv] = c;
        command.posRecv++;
      }
    }
  }
#endif

#if 0
  re = CommandLoopRead(unit);
  if(re > 0) {
    command.bufResponse[0] = '\0';
    CommandExec(unit, cmdac, cmdav);
    CommandClearParse(unit);

    if(command.bufResponse[0] != '\0') {
      printf("%s\n", command.bufResponse);
    }
  }
#endif


  if(cmdac > 0) {
    command.bufResponse[0] = '\0';
    CommandExec(unit, cmdac, cmdav);
    CommandClearParse(unit);

    if(command.bufResponse[0] != '\0') {
      printf("%s\n", command.bufResponse);
    }
  }
  CommandLoopRead(unit);

  return;
}


int
CommandLoopRead(int unit)
{
  int                   result = 0;

  unsigned char         c;

  unit = 0;

  while(Serial.available()) {
    c = Serial.read();

    if(c == '\b') {
      if(command.posRecv > 0) {
        command.posRecv--;
        Serial.print("\b \b");
      }

    } else if(c == 0x03) {
      Serial.print("^C\n");
      command.posRecv = 0;

    } else if(c == '\r' || c == '\n') {
      Serial.print("\n");
      if(command.posRecv > 1) {
        command.bufRecv[command.posRecv] = '\0';
        result = CommandParser(unit, (char *)command.bufRecv, command.posRecv);
        //command.posRecv = 0;
      } else {
        cmdac = 0;
      }

    } else {
      if(command.posRecv < COMMAND_RECV_BUFFER_SIZE - 1) {
        Serial.printf("%c", c);
        command.bufRecv[command.posRecv] = c;
        command.posRecv++;
      }
    }
  }

  return result;
}
void
CommandGetParse(int unit, int *pac, char ***pav)
{
  *pac = cmdac;
  *pav = (char **)&cmdav;

  return;
}
void
CommandClearParse(int unit)
{
  command.posRecv = 0;
  cmdac = -1;
  return;
}
int
CommandGetch(int unit)
{
  int           c = -1;

  if(command.posRecv > 0) {
    c = command.bufRecv[0];
    command.posRecv = 0;
  }

  return c;
}


static int
CommandDelim(char *av[], char *p, int len)
{
  char          *tp;
  int           cnt = 0;

  tp = strtok(p, " \t");
  while(tp != NULL) {
    av[cnt] = tp;
    tp = strtok(NULL, " \t");
    cnt++;
  }

  return cnt;
}


static int
CommandExecVersion(int unit, int ac, char *av[])
{
  Serial.print(CONFIG_VERSION_TEXT);
  if(ac >= 2 && av[1][1] == 'v') {
    Serial.printf("  (compiled at %s on %s)", __TIME__, __DATE__);
  }
  Serial.println("");

  return 0;
}


static void
CommandExec(int unit, int ac, char *av[])
{

  if(0) {

    //  } else if(!strcmp(av[0], "wifi")) {
    //WifiCommand(unit, ac, av);

  } else if(!strcmp(av[0], "imu")) {
    ImuCommand(ac, av);

  } else if(!strcmp(av[0], "gnssdo")) {
    GnssdoCommand(ac, av);

  } else if(!strcmp(av[0], "gnss")) {
    GnssCommand(ac, av);

  } else if(!strcmp(av[0], "dac")) {
    int num, val;
    num = atoi(av[1]);
    val = strtoul(av[2], NULL, 16);
    Mcp4726Set(num, val);

  } else if(!strcmp(av[0], "network")) {
    NetworkCommand(ac, av);

  } else if(!strcmp(av[0], "eeprom")) {
    EepromCommand(ac, av);

  } else if(!strcmp(av[0], "i2cscan")) {
    uint32_t    val;
    if(ac >= 2) {
      val = strtoul(av[1], NULL, 16);
      SystemI2cScan(val);
    }

  } else if(!strcmp(av[0], "debug")) {
    CommandDebug(unit, ac, av);

  } else if(!strcmp(av[0], "help")) {
    Serial.print(helpString);

  } else if(!strcmp(av[0], "info")) {
  } else if(!strcmp(av[0], "reset")) {
    SystemReset();

  } else if(!strcmp(av[0], "firmupdate")) {
    uint32_t    val;
    if(ac >= 2) {
      val = strtoul(av[1], NULL, 16);
      if(val == 0xdeadbeef) {
        _reboot_Teensyduino_();
        while(1);
      }
    }

  } else if(!strcmp(av[0], "version")) {
    CommandExecVersion(unit, ac, av);

  }

  return;
}


int
CommandParser(int unit, char *p, int len)
{
  int                   i;

  int                   sum, sumCalc = 0;

  {
    /* check comment line */
    if(p[0] == '#') {
      /*   "#!cxx [data]"
       *    #!c82 led 1 4 4 4
       *    #!C09 led 1 10 10 10
       *    #!c22 led 1 10 10 10          ### ng
       */
      if(p[1] == '!' &&
         (p[2] == 'c' || p[2] == 'C')) {
        p[5] = '\0';
        sum = strtoul((const char *)&p[3], NULL, 16);
        for(i = 6; i < len; i++) {
          sumCalc += p[i];
        }
        sumCalc &= 0xff;
#if 0
        sc.print(sum, HEX);
        sc.print(" ");
        sc.println(sumCalc, HEX);
#endif
        if(sumCalc != sum || len < 9) goto end;
        p += 6;
        len -= 6;
      } else {
        goto end;
      }
    }

    cmdac = CommandDelim(cmdav, p, len);
 #if 0
    if(cmdac < 1) goto end;

    command.bufResponse[0] = '\0';
    CommandExec(unit, cmdac, cmdav);

    if(command.bufResponse[0] != '\0') {
      printf("%s\n", command.bufResponse);
    }
#endif
  }

end:
  return cmdac;
}


static int
CommandDebug(int unit, int ac, char *av[])
{
  return 0;
}
