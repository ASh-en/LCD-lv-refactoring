#ifndef __TIM7_H
#define __TIM7_H	 
 
/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"
#include "stm32f7xx_eval.h"
#include "stdarg.h"
#include "string.h"
#include "stm32f7xx_hal_tim.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* User can use this section to tailor USARTx/UARTx instance used and associated
   resources */
#define TIMx                           TIM7
#define TIMx_CLK_ENABLE()              __HAL_RCC_TIM7_CLK_ENABLE()


/* Definition for TIMx's NVIC */
#define TIMx_IRQn                      TIM7_IRQn
#define TIMx_IRQHandler                TIM7_IRQHandler
	 


extern TIM_HandleTypeDef TIM7_Handler;      //¶¨Ê±Æ÷7¾ä±ú

void TIM7_Init(uint16_t arr,uint16_t psc); 

#endif  
















