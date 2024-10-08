#ifndef __SBSP_RTC_H__
#define __SBSP_RTC_H__
#ifdef __cplusplus
extern "C"{
#endif


#include "stm32f4xx_hal.h"


void sBSP_RTC_Init();
void sBSP_RTC_SetTime(uint8_t hour,uint8_t min,uint8_t sec);
void sBSP_RTC_SetDate(uint16_t year,uint8_t month,uint8_t day,uint8_t weekday);
void sBSP_RTC_GetTime(RTC_TimeTypeDef* param_time);
void sBSP_RTC_GetDate(RTC_DateTypeDef* param_date);

void HAL_RTC_MspInit(RTC_HandleTypeDef* rtcHandle);




#ifdef __cplusplus
}
#endif
#endif