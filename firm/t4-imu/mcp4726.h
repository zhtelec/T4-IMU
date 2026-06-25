#ifndef _MCP4726_H_
#define _MCP4726_H_


void            Mcp4726Init(int unit);
void            Mcp4726Set(int unit, int value);
void            Mcp4726SetWithEeprom(int unit, int value, int valueEeprom);
void            Mcp4726Get(int unit, uint16_t *pVal, uint16_t *pValEeprom);

#endif
