#ifndef _PTP_H_
#define _PTP_H_

typedef enum {
  ptpMaster = 0,
  ptpSlave,
  ptpBoth,
} ptpMode_t;

void            PtpInit(ptpMode_t mode);
void            PtpLoop(void);
uint32_t        PtpGetLockCount(void);
void            PtpGetUnixTime(struct timespec *t);
void            PtpGetUnixTimePps(struct timespec *t);
int             timespec2str(char *buf, uint len, struct timespec *ts);


#ifdef  _PTP_CPP_
static void             PtpInterruptTimer(void);
static void             PtpInterruptSync(void);
static void             PtpInterruptAnnounce(void);
static void             PtpCheckTimestampBackwards(void);
#endif

#endif
