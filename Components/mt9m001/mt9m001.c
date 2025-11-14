/**
  ******************************************************************************
  * @file    mt9m001.c
  * @author  MCD Application Team
  * @version V1.0.2
  * @date    02-December-2014
  * @brief   This file provides the mt9m001 camera driver
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

/* Includes ------------------------------------------------------------------*/
#include "mt9m001.h"
#include "stm32f7xx_eval.h"
extern I2C_HandleTypeDef hEvalI2c;
uint16_t reg_value;
/** @addtogroup BSP
  * @{
  */ 

/** @addtogroup Components
  * @{
  */ 
  
/** @addtogroup mt9m001
  * @brief     This file provides a set of functions needed to drive the 
  *            mt9m001 Camera module.
  * @{
  */

/** @defgroup mt9m001_Private_TypesDefinitions
  * @{
  */ 

/**
  * @}
  */ 

/** @defgroup mt9m001_Private_Defines
  * @{
  */

/**
  * @}
  */ 
  
/** @defgroup mt9m001_Private_Macros
  * @{
  */
     
/**
  * @}
  */  
  
/** @defgroup mt9m001_Private_FunctionPrototypes
  * @{
  */
//static uint32_t mt9m001_ConvertValue(uint32_t feature, uint32_t value);
/**
  * @}
  */ 
  
/** @defgroup mt9m001_Private_Variables
  * @{
  */        

CAMERA_DrvTypeDef   mt9m001_drv = 
{
  mt9m001_Init,
  mt9m001_ReadID,  
  mt9m001_Config,
};

/**
  * @}
  */
  
/** @defgroup mt9m001_Private_Functions
  * @{
  */ 

/**
  * @brief  Write data to mt9m001 register.
  * @param  addr: Register address to be write.
  * @param  value: Value to be configured
	* @retval 0:OK,0xFF:KO	
  */
uint8_t mt9m001_WriteReg(uint16_t addr, uint16_t value)
{
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t buf[3];
	buf[0] = addr;
	buf[1] = (uint8_t)(value>>8);
	buf[2] = (uint8_t)value;
	
	status = HAL_I2C_Master_Transmit(&hEvalI2c, MT9M001_I2C_WRITE_ADDRESS, buf, 3,1000);
	if(status != HAL_OK)
	{
		/* Return KO */
		return ERROR; 
	}
	return 0;
}

/**
  * @brief  Read data from mt9m001 register.
  * @param  addr: Register address to be read.
	* @retval Register value
  */
uint16_t mt9m001_ReadReg(uint8_t addr)
{
	HAL_StatusTypeDef status;
	uint8_t buf[2];
	
	//
	status = HAL_I2C_Master_Transmit(&hEvalI2c, MT9M001_I2C_WRITE_ADDRESS, &addr, 1,1000);
	if(status != HAL_OK)
	{
		/* Return KO */
		return ERROR; 
	}
	
	HAL_I2C_Master_Receive(&hEvalI2c, MT9M001_I2C_READ_ADDRESS, buf, 2, 1000);
	if(status != HAL_OK)
	{
		/* Return KO */
		return ERROR; 
	}
	return ((buf[0]<<8)|buf[1]);
}

/**
  * @brief  Read the mt9m001 Camera identity.
  * @param  DeviceAddr: Device address on communication Bus.
  * @retval the mt9m001 ID
  */
uint16_t mt9m001_ReadID(uint16_t DeviceAddr)
{
	I2Cx_Init();
	reg_value = mt9m001_ReadReg(0x00);
	return reg_value;
}

/**
  * @brief  Initializes the mt9m001 CAMERA component.
  * @param  DeviceAddr: Device address on communication Bus.
  * @param  resolution: Camera resolution
  * @retval None
  */
void mt9m001_Init(uint16_t DeviceAddr, uint32_t resolution)
{
	uint32_t i;
  
  /* Initialize I2C */
  CAMERA_IO_Init();
	mt9m001_WriteReg(0x03,0x03ff);HAL_Delay(5);
	mt9m001_WriteReg(0x04,0x04ff);HAL_Delay(5);
	mt9m001_WriteReg(0x09,1000);HAL_Delay(5);
	mt9m001_WriteReg(0x07,2);HAL_Delay(5);
	
//	mt9m001_WriteReg(0x03,0x03ff);HAL_Delay(5);
//	mt9m001_WriteReg(0x04,0x04ff);HAL_Delay(5);
//	mt9m001_WriteReg(0x09,1000);HAL_Delay(5);
//	mt9m001_WriteReg(0x07,0x0002);HAL_Delay(5);
	
	/*Soft reset*/
}
//void mt9m001_Init(uint16_t DeviceAddr, uint32_t resolution)
//{
//       mt9m001_WriteReg(0x0D,1);HAL_Delay(50);
//       mt9m001_WriteReg(0x0D,0);HAL_Delay(50);
//       mt9m001_WriteReg(0x09,1000);//Number of rows of integration—default = 0x04C9 (1225).39.2us a line under 50M Clock;
//       //wrMT9XXXReg(0x62,0x849f);//Disable Cal
//       //wrMT9XXXReg(0x60,0);//Cal Green1
//       //wrMT9XXXReg(0x61,0);//Cal Green2
//      // wrMT9XXXReg(0x63,0);//Cal red
//       //wrMT9XXXReg(0x64,0);//Cal Blue
//      // wrMT9XXXReg(0xf1,1);//Chip Enable

//      mt9m001_WriteReg(0x2b,33);//GREEN1 Gain
//      mt9m001_WriteReg(0x2c,40);//Blue Gain
//      mt9m001_WriteReg(0x2d,40);//Red Gain
//      mt9m001_WriteReg(0x2e,33);//GREEN2 Gain
//     // wrMT9XXXReg(0x35,40);//Globle Gain for Mono Type
//      
//      mt9m001_WriteReg(0x03,1023);//Rowsize
//      mt9m001_WriteReg(0x04,1279);//Col Size
////      mt9m001_WriteReg(0x03,479);//Rowsize
////      mt9m001_WriteReg(0x04,639);//Col Size
////			mt9m001_WriteReg(0x07,0x0040);
//			
//      HAL_Delay(5);
//}

void mt9m001_Config(uint16_t DeviceAddr, uint32_t feature, uint32_t value, uint32_t brightness_value)
{
}
         
/**
  * @}
  */ 
  
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
