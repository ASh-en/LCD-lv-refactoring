/**
  ******************************************************************************
  * @file    ov5640.h
  * @author  MCD Application Team
  * @version V1.0.2
  * @date    02-December-2014
  * @brief   This file contains all the functions prototypes for the ov5640.c
  *          driver.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __OV5640_H
#define __OV5640_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include "../Common/camera.h"
   
/** @addtogroup BSP
  * @{
  */ 

/** @addtogroup Components
  * @{
  */ 
  
/** @addtogroup ov5640
  * @{
  */

/** @defgroup OV5640_Exported_Types
  * @{
  */
     
/**
  * @}
  */ 

/** @defgroup OV5640_Exported_Constants
  * @{
  */
/** 
  * @brief  OV5640 ID  
  */  
#define  OV5640_ID    0x5640
/** 
  * @brief  OV5640 Registers  
  */
#define OV5640_SENSOR_PIDH       0x300A
#define OV5640_SENSOR_PIDL       0x300B
#define OV5640_FIRMWARE_ADDR     0x8000
/** 
 * @brief  OV5640 Features Parameters  
 */
#define OV5640_BRIGHTNESS_LEVEL0        0x40   /* Brightness level -2         */
#define OV5640_BRIGHTNESS_LEVEL1        0x30   /* Brightness level -1         */
#define OV5640_BRIGHTNESS_LEVEL2        0x20   /* Brightness level 0          */
#define OV5640_BRIGHTNESS_LEVEL3        0x10   /* Brightness level +1         */
#define OV5640_BRIGHTNESS_LEVEL4        0x00   /* Brightness level +2         */

#define OV5640_BLACK_WHITE_BW           0x18   /* Black and white effect      */
#define OV5640_BLACK_WHITE_NEGATIVE     0x40   /* Negative effect             */
#define OV5640_BLACK_WHITE_BW_NEGATIVE  0x58   /* BW and Negative effect      */
#define OV5640_BLACK_WHITE_NORMAL       0x00   /* Normal effect               */

#define OV5640_CONTRAST_LEVEL0          0x3418 /* Contrast level -2           */
#define OV5640_CONTRAST_LEVEL1          0x2A1C /* Contrast level -2           */
#define OV5640_CONTRAST_LEVEL2          0x2020 /* Contrast level -2           */
#define OV5640_CONTRAST_LEVEL3          0x1624 /* Contrast level -2           */
#define OV5640_CONTRAST_LEVEL4          0x0C28 /* Contrast level -2           */

#define OV5640_COLOR_EFFECT_ANTIQUE     0xA640 /* Antique effect              */
#define OV5640_COLOR_EFFECT_BLUE        0x40A0 /* Blue effect                 */
#define OV5640_COLOR_EFFECT_GREEN       0x4040 /* Green effect                */
#define OV5640_COLOR_EFFECT_RED         0xC040 /* Red effect                  */

/**
  * @}
  */
  
/** @defgroup OV5640_Exported_Functions
  * @{
  */ 
void     ov5640_Init(uint16_t DeviceAddr, uint32_t resolution);
void     ov5640_Config(uint16_t DeviceAddr, uint32_t feature, uint32_t value, uint32_t BR_value);
uint16_t ov5640_ReadID(uint16_t DeviceAddr);

void     CAMERA_IO_Init(void);
void     CAMERA_IO_Write(uint8_t addr, uint8_t reg, uint8_t value);
uint8_t  CAMERA_IO_Read(uint8_t addr, uint8_t reg);
void     CAMERA_Delay(uint32_t delay);

void OV5640_Set_Brightness(uint8_t bright);
void OV5640_Set_Contrast(uint8_t contrast);
void OV5640_Set_Color_Effect(uint8_t value);
void OV5640_Set_BandW(uint8_t BlackWhite);
void OV5640_LED_Ctrl(uint8_t led);

/* CAMERA driver structure */
extern CAMERA_DrvTypeDef   ov5640_drv;
/**
  * @}
  */    
#ifdef __cplusplus
}
#endif

#endif /* __OV5640_H */
/**
  * @}
  */ 

/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
