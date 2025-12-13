#ifndef __IMUDRVR_LIST_H__
#define __IMUDRVR_LIST_H__

#include        <stddef.h>

#include        "imudrv.h"

#ifdef  __IMUDRV_CPP__


extern struct _stImudrvList    imudrvList_CXD5602;
extern struct _stImudrvList    imudrvList_LSM6DSV;
extern struct _stImudrvList    imudrvList_ICM45686;
extern struct _stImudrvList    imudrvList_BMI323;
extern struct _stImudrvList    imudrvList_BMI330;


struct _stImudrvList *imudrvList[] = {
  &imudrvList_CXD5602,          // Spresense imu
  &imudrvList_LSM6DSV,
  &imudrvList_ICM45686,
  &imudrvList_BMI323,
  &imudrvList_BMI330,
  NULL,
};

#endif

#endif
// end
