#ifndef __HC05_H
#define __HC05_H	 
 
/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"
#include "stm32f7xx_eval.h"
#include "stdarg.h"
#include "string.h"
#include "uart6.h"
#include "stm32f7xx_hal_tim.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* User can use this section to tailor USARTx/UARTx instance used and associated
   resources */

////////////////////////////////////////////////////////////////////////////
#define HC05_KEY(n) (n?HAL_GPIO_WritePin(GPIOG,GPIO_PIN_9,GPIO_PIN_SET):HAL_GPIO_WritePin(GPIOG,GPIO_PIN_9,GPIO_PIN_RESET))	//À¶ÑÀ¿ØÖÆKEYÐÅºÅ
#define HC05_LED 	HAL_GPIO_ReadPin(GPIOG,GPIO_PIN_6)  //À¶ÑÀÁ¬½Ó×´Ì¬ÐÅºÅ
 

uint8_t HC05_Init(void);
//void HC05_CFG_CMD(uint8_t *str);
uint8_t HC05_Get_Role(void);
uint8_t HC05_Set_Cmd(uint8_t* atstr);	
uint8_t HC05_SetName_Cmd(uint8_t* atstr);

#endif  
















