/**
  ******************************************************************************
  * @file    g104sn03.h
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    18-February-2014
  * @brief   This file contains all the constants parameters for the ampire640480
  *          LCD component.
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
#ifndef __G104SN03_H
#define __G104SN03_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/  

/** @addtogroup BSP
  * @{
  */ 

/** @addtogroup Components
  * @{
  */ 
  
/** @addtogroup G104SN03
  * @{
  */

/** @defgroup G104SN03_Exported_Types
  * @{
  */
   
/**
  * @}
  */ 

/** @defgroup G104SN03_Exported_Constants
  * @{
  */
  
/** 
  * @brief  G104SN03 Size  
  */    
#define  G104SN03_WIDTH    ((uint16_t)800)             /* LCD PIXEL WIDTH            */
#define  G104SN03_HEIGHT   ((uint16_t)600)              /* LCD PIXEL HEIGHT           */

/** 
  * @brief  G104SN03 Timing  
  */ 
//#define  G104SN03_HSYNC            ((uint16_t)2)      /* Horizontal synchronization */
//#define  G104SN03_HBP              ((uint16_t)46)     /* Horizontal back porch      */
//#define  G104SN03_HFP              ((uint16_t)40)    /* Horizontal front porch     */
//#define  G104SN03_VSYNC            ((uint16_t)1)      /* Vertical synchronization   */
//#define  G104SN03_VBP              ((uint16_t)3)     /* Vertical back porch        */
////#define  G104SN03_VBP              ((uint16_t)1)     /* Vertical back porch        */
//#define  G104SN03_VFP              ((uint16_t)1)     /* Vertical front porch       */

//Use AT080TN52
#define  G104SN03_HSYNC            ((uint16_t)2)      /* Horizontal synchronization */
#define  G104SN03_HBP              ((uint16_t)46)     /* Horizontal back porch      */
#define  G104SN03_HFP              ((uint16_t)40)    /* Horizontal front porch     */
#define  G104SN03_VSYNC            ((uint16_t)1)      /* Vertical synchronization   */
#define  G104SN03_VBP              ((uint16_t)23)     /* Vertical back porch        */
#define  G104SN03_VFP              ((uint16_t)1)     /* Vertical front porch       */

/** 
  * @brief  G104SN03 frequency divider  
  */    
#define  G104SN03_FREQUENCY_DIVIDER     4              /* LCD Frequency divider      */
/**
  * @}
  */
  
/** @defgroup G104SN03_Exported_Functions
  * @{
  */    

/**
  * @}
  */    
#ifdef __cplusplus
}
#endif

#endif /* __G104SN03_H */
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
