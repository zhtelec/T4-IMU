#ifndef _SDLOG_H_
#define _SDLOG_H_

void            SdlogInit(void);
void            SdlogLoop(void);
int             SdlogOpen(void);
void            SdlogClose(void);
void            SdlogWrite(char *str);
void            SdlogLoopWrite(char *str);


#endif
