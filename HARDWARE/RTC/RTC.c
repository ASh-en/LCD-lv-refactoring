#include "stm32f7xx_hal.h"
#include "stm32f7xx_eval.h"			 
#include "string.h"	
#include "RTC.h"	 
#include "math.h"

//////////////////////////////////////////////////////////////////
void RTC_Init(void)
{

	RtcHandle.Instance = RTC;
  RTC_DateTypeDef  sdatestructure;
  RTC_TimeTypeDef  stimestructure;
  /* Configure RTC prescaler and RTC data registers */
  /* RTC configured as follows:
      - Hour Format    = Format 24
      - Asynch Prediv  = Value according to source clock
      - Synch Prediv   = Value according to source clock
      - OutPut         = Output Disable
      - OutPutPolarity = High Polarity
      - OutPutType     = Open Drain */
  RtcHandle.Init.HourFormat     = RTC_HOURFORMAT_24;
  RtcHandle.Init.AsynchPrediv   = RTC_ASYNCH_PREDIV;
  RtcHandle.Init.SynchPrediv    = RTC_SYNCH_PREDIV;
  RtcHandle.Init.OutPut         = RTC_OUTPUT_DISABLE;
  RtcHandle.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  RtcHandle.Init.OutPutType     = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&RtcHandle) != HAL_OK)
  {
    /* Initialization Error */
    BSP_LCD_DisplayStringAtLine(4, (uint8_t*)"    rtc OFF");
    Error_Handler();
    
  }
  /*##-1- Configure the Date #################################################*/
  /* Set Date: Tuesday February 18th 2015 */
  sdatestructure.Year = 0x19;
  sdatestructure.Month = RTC_MONTH_OCTOBER;
  sdatestructure.Date = 0x26;
  sdatestructure.WeekDay = RTC_WEEKDAY_SATURDAY;
  
  if(HAL_RTC_SetDate(&RtcHandle,&sdatestructure, RTC_FORMAT_BCD) != HAL_OK)//执行一次即可设定好
  {
    /* Initialization Error */
    Error_Handler(); 
  } 
  
  /*##-2- Configure the Time #################################################*/
  /* Set Time: 02:20:00 */
  stimestructure.Hours = 0x16;
  stimestructure.Minutes = 0x48;
  stimestructure.Seconds = 0x50;
  stimestructure.TimeFormat = RTC_HOURFORMAT12_AM;
  stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE ;
  stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;
  
  if(HAL_RTC_SetTime(&RtcHandle,&stimestructure, RTC_FORMAT_BCD) != HAL_OK) //执行一次即可设定好
  {
    /* Initialization Error */
    Error_Handler(); 
  }   
  
}
//static void RTC_AlarmConfig(void)
//{
//  RTC_DateTypeDef  sdatestructure;
//  RTC_TimeTypeDef  stimestructure;
//  RTC_AlarmTypeDef salarmstructure;
// 
//  /*##-1- Configure the Date #################################################*/
//  /* Set Date: Tuesday February 18th 2015 */
//  sdatestructure.Year = 0x15;
//  sdatestructure.Month = RTC_MONTH_FEBRUARY;
//  sdatestructure.Date = 0x18;
//  sdatestructure.WeekDay = RTC_WEEKDAY_TUESDAY;
//  
//  if(HAL_RTC_SetDate(&RtcHandle,&sdatestructure, RTC_FORMAT_BCD) != HAL_OK)
//  {
//    /* Initialization Error */
//    Error_Handler(); 
//  } 
//  
//  /*##-2- Configure the Time #################################################*/
//  /* Set Time: 02:20:00 */
//  stimestructure.Hours = 0x22;
//  stimestructure.Minutes = 0x14;
//  stimestructure.Seconds = 0x50;
//  stimestructure.TimeFormat = RTC_HOURFORMAT12_AM;
//  stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE ;
//  stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;
//  
//  if(HAL_RTC_SetTime(&RtcHandle,&stimestructure, RTC_FORMAT_BCD) != HAL_OK)
//  {
//    /* Initialization Error */
//    Error_Handler(); 
//  }  
//
//  /*##-3- Configure the RTC Alarm peripheral #################################*/
//  /* Set Alarm to 02:20:30 
//     RTC Alarm Generation: Alarm on Hours, Minutes and Seconds */
//  salarmstructure.Alarm = RTC_ALARM_A;
//  salarmstructure.AlarmDateWeekDay = RTC_WEEKDAY_MONDAY;
//  salarmstructure.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
//  salarmstructure.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY;
//  salarmstructure.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_NONE;
//  salarmstructure.AlarmTime.TimeFormat = RTC_HOURFORMAT12_AM;
// salarmstructure.AlarmTime.Hours = 0x13;
//  salarmstructure.AlarmTime.Minutes = 0x20;
//  salarmstructure.AlarmTime.Seconds = 0x30;
//  salarmstructure.AlarmTime.SubSeconds = 0x56;
//  
//  if(HAL_RTC_SetAlarm_IT(&RtcHandle,&salarmstructure, RTC_FORMAT_BCD) != HAL_OK)
//  {
//    /* Initialization Error */
//    Error_Handler(); 
//  }
//}
void Error_Handler(void)
{
  /* Turn LED3 on */
  BSP_LED_On(LED3);
  while(1)
  {
  }
}

//////////////////////////////////////////////////////////////////