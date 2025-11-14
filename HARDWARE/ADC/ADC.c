#include "stm32f7xx_hal.h"
#include "stm32f7xx_eval.h"			 
#include "string.h"	
#include "ADC.h"	 
#include "math.h"

 void MX_ADC1_Init(void)
{
  /*##-1- Configure the ADC peripheral #######################################*/
  AdcHandle.Instance          = ADC3;
  
  AdcHandle.Init.ClockPrescaler        = ADC_CLOCKPRESCALER_PCLK_DIV4;
  AdcHandle.Init.Resolution            = ADC_RESOLUTION_12B;
  AdcHandle.Init.ScanConvMode          = DISABLE;                       /* Sequencer disabled (ADC conversion on only 1 channel: channel set on rank 1) */
  AdcHandle.Init.ContinuousConvMode    = DISABLE;                       /* Continuous mode disabled to have only 1 conversion at each conversion trig */
  AdcHandle.Init.DiscontinuousConvMode = DISABLE;                       /* Parameter discarded because sequencer is disabled */
  AdcHandle.Init.NbrOfDiscConversion   = 0;
  AdcHandle.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;        /* Conversion start trigged at each external event */
  AdcHandle.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_T1_CC1;
  AdcHandle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
  AdcHandle.Init.NbrOfConversion       = 1;
  AdcHandle.Init.DMAContinuousRequests = DISABLE;
  AdcHandle.Init.EOCSelection          = DISABLE;
  
  if (HAL_ADC_Init(&AdcHandle) != HAL_OK)
  {
    /* ADC initialization Error */
		BSP_LCD_DisplayStringAtLine(4, (uint8_t*)"    ADC ERROR");
    Error_Handler();
  }
}

uint16_t Get_ADC(uint32_t ch)
{
	  ADC_ChannelConfTypeDef sConfig;
	
	  sConfig.Channel = ch;
		sConfig.Rank = 1;
		sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
		if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK)
		{
				Error_Handler();
		}
		
		if (HAL_ADC_Start(&AdcHandle) != HAL_OK)
		{
    /* Start Conversation Error */
			Error_Handler();
		}
		HAL_ADC_PollForConversion(&AdcHandle, 10);

		return(uint16_t)HAL_ADC_GetValue(&AdcHandle);	
}
//void Error_Handler(void)
//{
//  /* Turn LED3 on */
//  BSP_LED_On(LED3);
//  while(1)
//  {
//  }
//}
//////////////////////////////////////////////////////////////////