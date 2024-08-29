#ifndef __MAIN_H__
#define __MAIN_H__
#ifdef __cplusplus
extern "C"{
#endif


#include "stm32f4xx_hal.h"
#include <math.h>
#include "arm_math.h"
#include <stdbool.h>
#include <ctype.h>

//BSP
#include "sBSP_SYS.h"
#include "sBSP_RCC.h"
#include "sBSP_GPIO.h"
#include "sBSP_UART.h"
#include "sBSP_TIM.h"
#include "sBSP_RTC.h"
#include "sBSP_DWT.h"
#include "sBSP_DMA.h"
//HMI
#include "sHMI_Debug.h"
#include "sHMI_LED.h"
//DRV
#include "sDRV_ICM42688.h"
#include "sDRV_LIS3MDLTR.h"
#include "sDRV_LPS22HBTR.h"
#include "sDRV_ST7789V.h"
//APP

#include "sLib.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "FreeRTOS_CLI.h"
#include "task.h"

#include "lv_conf.h"
#include "lvgl.h"

    
void Error_Handler();
void vApplicationMallocFailedHook();
void vApplicationIdleHook();
void vApplicationTickHook();
void vApplicationStackOverflowHook( TaskHandle_t xTask,char * pcTaskName );

#ifdef __cplusplus
}
#endif
#endif

