/* -*- mode: C; -*- */

#define	_NETWORK_CPP_

#include        <stdint.h>
#include        <stdlib.h>
#include        <string.h>

#include        <Arduino.h>
#include        <QNEthernet.h>

#include        "config.h"
#include        "system.h"
#include        "eeprom.h"
#include        "top.h"
#include        "ptp.h"

#include        "network.h"

namespace qn = qindesign::network;

// Teensy network setting.
static uint16_t         ipflag;
static IPAddress        staticIP;
static IPAddress        subnetMask;
static IPAddress        gateway;
IPAddress               multicastIP;
uint16_t                multicastPort;

static int              linkStateUpPrev = 0;
static int              etherIpEnable = 0;
static uint32_t         t1Sec;

static byte             mac[6];

static qn::EthernetUDP  udp;

void
NetworkInit(void)
{

  ipflag        = EepromGet32(CONFIG_EEPROM_IPFLAG_POS);
  staticIP      = EepromGet32(CONFIG_EEPROM_MYIP_POS);
  subnetMask    = EepromGet32(CONFIG_EEPROM_NETMASK_POS);
  gateway       = EepromGet32(CONFIG_EEPROM_GATEWAY_POS);

  multicastIP   = EepromGet32(CONFIG_EEPROM_MULTICASTIP_POS);
  multicastPort = EepromGet16(CONFIG_EEPROM_MULTICASTPORT_POS);

  // Setup networking
  qn::Ethernet.macAddress(mac);
  switch(SystemGetBoardId()) {
  case  CONFIG_BOARDID_T4_IMU:       qn::Ethernet.setHostname(CONFIG_NETWORK_HOSTNAME_T4_IMU);   break;
  case  CONFIG_BOARDID_T4_PTPGM:     qn::Ethernet.setHostname(CONFIG_NETWORK_HOSTNAME_T4_PTPGM); break;
  case  CONFIG_BOARDID_T4_SENSECAP:  qn::Ethernet.setHostname(CONFIG_NETWORK_HOSTNAME_T4_SENSECAP); break;
  case  CONFIG_BOARDID_T4_ID3:       qn::Ethernet.setHostname(CONFIG_NETWORK_HOSTNAME_T4_ID3);   break;
  case  CONFIG_BOARDID_T4_ID4:       qn::Ethernet.setHostname(CONFIG_NETWORK_HOSTNAME_T4_ID4);   break;
  case  CONFIG_BOARDID_T4_ID5:       qn::Ethernet.setHostname(CONFIG_NETWORK_HOSTNAME_T4_ID5);   break;
  case  CONFIG_BOARDID_T4_ID6:       qn::Ethernet.setHostname(CONFIG_NETWORK_HOSTNAME_T4_ID6);   break;
  case  CONFIG_BOARDID_T4_ID7:       qn::Ethernet.setHostname(CONFIG_NETWORK_HOSTNAME_T4_ID7);   break;
  }

  if(ipflag & CONFIG_EEPROM_IPFLAG_STATIC_MASK) {
    // enable with the static ip
    qn::Ethernet.begin(staticIP, subnetMask, gateway);
    //qn::Ethernet.setDNSServerIP(gateway);
  } else {
    // enable and start dhcpc
    qn::Ethernet.begin();
  }

  return;
}


void
NetworkLoop(void)
{
  int   linkStateUp;

  linkStateUp = qn::Ethernet.linkState();

  if(linkStateUp && linkStateUpPrev && etherIpEnable) {
    PtpLoop();

  }

  if((t1Sec - SystemGetCounter()) >= 1*SYSTEM_COUNTER_1S000) {
    t1Sec = SystemGetCounter();

    etherIpEnable = qn::Ethernet.localIP();

    if(linkStateUp) {
      etherIpEnable = qn::Ethernet.localIP();
      if(etherIpEnable) {
        if(linkStateUpPrev != linkStateUp) {
          IPAddress     ip;
          ptpMode_t     mode;

          ip = qn::Ethernet.localIP();
          Serial.printf("eth link (%02x:%02x:%02x:%02x:%02x:%02x)", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
          Serial.printf(" up and got ip %d.%d.%d.%d\n", ip[0], ip[1], ip[2], ip[3]);

          if(NetworkIsMulticast(multicastIP)) qn::Ethernet.joinGroup(multicastIP);
          Serial.printf("multicastIp: %d.%d.%d.%d\n", multicastIP[0], multicastIP[1], multicastIP[2], multicastIP[3]);

          mode = ptpSlave;
          if(EepromGet8(CONFIG_EEPROM_PTP_POS) & CONFIG_EEPROM_PTP_MODE_MASK) {
            mode = ptpBoth;
          }
          Serial.printf("ptp: running %s mode\n", (mode == ptpSlave)? "SLAVE": "MASTER");
          PtpInit(mode);
          linkStateUpPrev = 1;
        }
      }
    } else {
      linkStateUpPrev = 0;
    }
  }

  return;
}


void
NetworkCommand(int ac, char *av[])
{
  if(       !strcmp(av[1], "ip")) {
    IPAddress   ip;
    if(ac >= 3) {
      ip   = NetworkIpStrToIpAddress(av[2]);
      EepromSet32(CONFIG_EEPROM_MYIP_POS, ip);
      if(ac >= 4) {
        ip   = NetworkIpStrToIpAddress(av[3]);
        EepromSet32(CONFIG_EEPROM_NETMASK_POS, ip);
      }
      if(ac >= 5) {
        ip   = NetworkIpStrToIpAddress(av[4]);
        EepromSet32(CONFIG_EEPROM_GATEWAY_POS, ip);
      }
      EepromBurn();
    }

#if 0
    if(       !strcmp(av[1], "myip")) {
    IPAddress   ip;
    ip   = NetworkIpStrToIpAddress(av[2]);
    EepromSet32(CONFIG_EEPROM_MYIP_POS, ip);
    EepromBurn();

  } else if(!strcmp(av[1], "myport")) {
    uint32_t    port;
    port  = strtoul(av[2], NULL, 0);
    EepromSet16(CONFIG_EEPROM_MYPORT_POS, port);
    EepromBurn();
#endif

    } else if(!strcmp(av[1], "multiip")) {
      IPAddress   ip;
      ip   = NetworkIpStrToIpAddress(av[2]);
      EepromSet32(CONFIG_EEPROM_MULTICASTIP_POS, ip);
      EepromBurn();

    } else if(!strcmp(av[1], "multiport")) {
      uint16_t    port;
      port = strtoul(av[2], NULL, 0);
      EepromSet16(CONFIG_EEPROM_MULTICASTPORT_POS, port);
      EepromBurn();

    } else if(!strcmp(av[1], "ipflag")) {
    uint16_t    val;

    if(!strcmp(av[2], "static")) {
      val = EepromGet16(CONFIG_EEPROM_IPFLAG_POS);
      val |= CONFIG_EEPROM_IPFLAG_STATIC_YES;
      EepromSet16(CONFIG_EEPROM_IPFLAG_POS, val);
      EepromBurn();
    } else if(!strcmp(av[2], "dhcp")) {
      val = EepromGet16(CONFIG_EEPROM_IPFLAG_POS);
      val &= ~CONFIG_EEPROM_IPFLAG_STATIC_MASK;
      EepromSet16(CONFIG_EEPROM_IPFLAG_POS, val);
      EepromBurn();
    }
  } else if(!strcmp(av[1], "ptp")) {
    uint8_t     val;
    val  = EepromGet8(CONFIG_EEPROM_PTP_POS);
    if(!strcmp(av[2], "master")) {
      val |= CONFIG_EEPROM_PTP_MODE_MASTER;
      EepromSet8(CONFIG_EEPROM_PTP_POS, val);
      EepromBurn();
    } else if(!strcmp(av[2], "slave")) {
      val &= ~CONFIG_EEPROM_PTP_MODE_MASK;
      EepromSet8(CONFIG_EEPROM_PTP_POS, val);
      EepromBurn();
    }
  }

  return;
}


uint32_t
NetworkIpStrTo32(char *str)
{
  char          buf[64], *p;
  uint32_t      val = 0;

  strncpy(buf, str, sizeof(buf)-1);
  buf[sizeof(buf)-1] = '\0';

  p = buf;
  while(*p) {
    if(*p == '.') *p = '\0';
    p++;
  }

  p = buf;
  for(int i = 0; i < 4; i++) {
    val  |= (atoi(p) & 0xff) << (i*8);
    while(*p++);
  }

  return val;
}
#if 1
// the code was not modified
IPAddress
NetworkIpStrToIpAddress(char *str)
{
  char          buf[64], *p;
  //uint32_t      val;
  IPAddress     ip;

  strncpy(buf, str, sizeof(buf)-1);
  buf[sizeof(buf)-1] = '\0';

  p = buf;
  while(*p) {
    if(*p == '.') *p = '\0';
    p++;
  }

  p = buf;
  for(int i = 0; i < 4; i++) {
    ip[i] = atoi(p);
    while(*p++);
  }

  return ip;
}
#endif
IPAddress
NetworkIp32ToIpAddress(uint32_t ip32)
{
  IPAddress     ip((ip32 >> 24) & 0xff,
                   (ip32 >> 16) & 0xff,
                   (ip32 >>  8) & 0xff,
                   (ip32      ) & 0xff );

  return ip;
}
uint32_t
Network_ntohl(uint32_t netlong)
{
  uint32_t      host;
  host  = (netlong >> 24) &       0xff;
  host |= (netlong >>  8) &     0xff00;
  host |= (netlong <<  8) &   0xff0000;
  host |= (netlong << 24) & 0xff000000;
  return host;
}
uint16_t
Network_ntohs(uint16_t netshort)
{
  uint16_t      host;
  host  = (netshort >>  8) &       0xff;
  host |= (netshort <<  8) &     0xff00;
  return host;
}
uint32_t
Network_htonl(uint32_t hostlong)
{
  uint32_t      net;
  net  = (hostlong >> 24) &       0xff;
  net |= (hostlong >>  8) &     0xff00;
  net |= (hostlong <<  8) &   0xff0000;
  net |= (hostlong << 24) & 0xff000000;
  return net;
}
uint16_t
Network_htons(uint16_t hostshort)
{
  uint16_t      net;
  net  = (hostshort >>  8) &       0xff;
  net |= (hostshort <<  8) &     0xff00;
  return net;
}


int
NetworkIsMulticast(IPAddress ip)
{
  return (ip[0] & 0xf0) == 0xe0;
}


void
NetworkInitUdp(void)
{
  udp.begin(CONFIG_MYPORT);

  return;
}
int
NetworkSendUdp(uint8_t *ptr, int len)
{
  int                   re;

  re = udp.send(multicastIP, multicastPort, ptr, len);

  return re;
}
