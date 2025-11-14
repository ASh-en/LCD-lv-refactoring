#ifndef __RTC_H
#define __RTC_H	 
 
/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"
#include "stm32f7xx_eval.h"
#include "stdarg.h"
#include "string.h"
#include "stm32f7xx_eval_lcd.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define RTC_CLOCK_SOURCE_LSE
/*#define RTC_CLOCK_SOURCE_LSI*/

#ifdef RTC_CLOCK_SOURCE_LSI
#define RTC_ASYNCH_PREDIV    0x7F
#define RTC_SYNCH_PREDIV     0x0130
#endif

#ifdef RTC_CLOCK_SOURCE_LSE
#define RTC_ASYNCH_PREDIV  0x7F
#define RTC_SYNCH_PREDIV   0x00FF
#endif
void RTC_Init(void);
void RTC_TimeShow(uint8_t* showtime);
extern RTC_HandleTypeDef RtcHandle;
void Error_Handler(void);
#endif  
















