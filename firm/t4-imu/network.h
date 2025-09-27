#ifndef _NETWORK_H_
#define _NETWORK_H_

void            NetworkInit(void);
void            NetworkLoop(void);

void            NetworkCommand(int ac, char *av[]);
uint32_t        NetworkIpStrTo32(char *str);
IPAddress       NetworkIpStrToIpAddress(char *str);
IPAddress       NetworkIp32ToIpAddress(uint32_t ip32);

uint32_t        Network_ntohl(uint32_t netlong);
uint16_t        Network_ntohs(uint16_t netshort);
uint32_t        Network_htonl(uint32_t hostlong);
uint16_t        Network_htons(uint16_t hostshort);

int             NetworkIsMulticast(IPAddress ip);

void            NetworkInitUdp(void);
int             NetworkSendUdp(uint8_t *ptr, int len);

#endif
