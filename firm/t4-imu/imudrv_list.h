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
extern struct _stImudrvList    imudrvList_LIS2MDL;
extern struct _stImudrvList    imudrvList_BMM150Poll;
extern struct _stImudrvList    imudrvList_ADXL345Poll;


struct _stImudrvList *imudrvList[] = {
  &imudrvList_CXD5602,          // Spresense imu
  &imudrvList_LSM6DSV,
  &imudrvList_ICM45686,
  &imudrvList_BMI323,
  &imudrvList_BMI330,
  &imudrvList_LIS2MDL,
  &imudrvList_ADXL345Poll,
  &imudrvList_BMM150Poll,
  NULL,
};

#endif

#endif
// end
