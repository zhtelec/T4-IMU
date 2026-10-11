#ifndef _LORA_H_
#define _LORA_H_


void            LoraInit(void);
void            LoraLoop(void);
int             LoraSend(const char *ptr, int len);

#ifdef  _LORA_CPP_
static int      LoraSetBaudrate(void);
static void     LoraSerialFlush(void);
static int      LoraSerialPuts(const char *str);
static int      LoraSerialGets(char *str, int len, uint32_t timeout_ms);
static int      LoraSerialGetsPoll(char *str, int len, uint32_t timeout_ms);
static void     LoraSerialFetch(void);
static void     LoraSerialResponseAndPrint(void);
#endif

#endif
