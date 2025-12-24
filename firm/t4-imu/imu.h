#ifndef _IMU_H_
#define _IMU_H_

void            lower_trigger(void);


void            ImuInit(void);
void            ImuLoop(void);
void            ImuStart(void);
void            ImuCommand(int ac, char *av[]);


#ifdef _IMU_CPP_

static void     ImuSendValue(struct _stImuValue *p);
static void     ImuSendValueImu(struct _stImuValue *p);
static void     ImuSendValueMagnetics(struct _stImuValue *p);

#endif

#endif
