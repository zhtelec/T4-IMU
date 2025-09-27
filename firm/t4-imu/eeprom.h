#ifndef _EEPROM_H_
#define _EEPROM_H_


int             EepromInit(void);
uint8_t         EepromCalcSum(void);
uint8_t         EepromGet8(int addr);
uint16_t        EepromGet16(int addr);
uint32_t        EepromGet32(int addr);
void            EepromSet8(int addr, uint8_t val);
void            EepromSet16(int addr, uint16_t val);
void            EepromSet32(int addr, uint32_t val);
void            EepromRead(int addr, uint8_t *ptr, int len);
void            EepromWrite(int addr, uint8_t *ptr, int len);
void            EepromBurn(void);

void            EepromCommand(int ac, char *av[]);

#endif
