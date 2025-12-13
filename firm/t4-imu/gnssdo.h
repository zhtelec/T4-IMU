#ifndef _GNSSDO_H_
#define _GNSSDO_H_


void            GnssdoInit(void);
void            GnssdoLoop(void);
void            GnssdoSetPllValue(int unit, int refFreq);
void            GnssdoCommand(int ac, char *av[]);


#ifdef  _GNSSDO_CPP_


static void             GnssdoInitGpt(int unit);
static void             GnssdoInterruptGpt1(void);
static void             GnssdoInterruptGpt2(void);
static uint32_t         GnssdoGetPpsCount(int unit);
static uint32_t         GnssdoGetFrequency(int unit);

static void             GnssdoPidInit(void);
static void             GnssdoPid24MHzInit(void);
static void             GnssdoPidVcocxoInit(void);
static void             GnssdoPid24MHz(int current, int target);
static void             GnssdoPidVcocxo(int current, int target);
static float            GnssdoPidGetFreqDifference(int unit);
static int              GnssdoPid(struct _stPid *p, int current, int target, int pol);
static void             GnssdoStatistics(int unit, int value);
static float            GnssdoTmp112GetTemperature(void);
static void             GnssdoLockLed(void);


#endif

#endif
