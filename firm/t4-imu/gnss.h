#ifndef _GNSS_H_
#define _GNSS_H_


enum debug_mode {
  GNSS_DEBUG_SHOW_PVT           = (1 << 0),
  GNSS_DEBUG_SHOW_TIME          = (1 << 1),
  GNSS_DEBUG_SHOW_HPPOSLLH      = (1 << 2),
  GNSS_DEBUG_SHOW_RTCM1005      = (1 << 3),
};


void            GnssInit(void);
void            GnssLoop(void);
int             GnssGetTime(timespec *ptm);
void            GnssInterruptPps(void);
void            GnssSetDebugMode(uint32_t v, uint32_t mask);
void            GnssCommand(int ac, char *av[]);


#ifdef  _GNSS_CPP_
static void     GnssInitSetBaud(void);
static void     GnssCallbackPvt(UBX_NAV_PVT_data_t *pData);
static void     GnssCallbackHpPosLLH(UBX_NAV_HPPOSLLH_data_t *pHpPos);
static void     GnssCallbackRtcm1005(RTCM_1005_data_t *);
#endif


#endif
