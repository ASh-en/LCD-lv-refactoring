#ifndef __ADC_H
#define __ADC_H	 
 
/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"
#include "stm32f7xx_eval.h"
#include "stdarg.h"
#include "string.h"
#include "stm32f7xx_eval_lcd.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Definition for ADCx clock resources */
#define ADCx_CLK_ENABLE()                 __HAL_RCC_ADC3_CLK_ENABLE()
#define ADCx_CHANNEL_GPIO_CLOCK_ENABLE()  __HAL_RCC_GPIOA_CLK_ENABLE()

#define ADCx_FORCE_RESET()                __HAL_RCC_ADC_FORCE_RESET()
#define ADCx_RELEASE_RESET()              __HAL_RCC_ADC_RELEASE_RESET()

/* Definition for ADCx Channel Pin */
#define ADCx_CHANNEL_PIN                  GPIO_PIN_3
#define ADCx_CHANNEL_GPIO_PORT            GPIOA

/* Definition for ADCx's Channel */
#define ADCx_CHANNEL                      ADC_CHANNEL_3
/* ADC handler declaration */
 extern ADC_HandleTypeDef    AdcHandle;
 void MX_ADC1_Init(void);
 void Error_Handler(void);
 uint16_t Get_ADC(uint32_t ch);
#endif  
















