/**
  ******************************************************************************
  * @file    ov5640.c
  * @author  MCD Application Team
  * @version V1.0.2
  * @date    02-December-2014
  * @brief   This file provides the OV5640 camera driver
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
#include "ov5640.h"
#include "ov5640_config.h"
#include "stm32f7xx_eval.h"

/** @addtogroup BSP
  * @{
  */ 

/** @addtogroup Components
  * @{
  */ 
  
/** @addtogroup OV5640
  * @brief     This file provides a set of functions needed to drive the 
  *            OV5640 Camera module.
  * @{
  */

/** @defgroup OV5640_Private_TypesDefinitions
  * @{
  */ 

/**
  * @}
  */ 

/** @defgroup OV5640_Private_Defines
  * @{
  */

/**
  * @}
  */ 
  
/** @defgroup OV5640_Private_Macros
  * @{
  */
     
/**
  * @}
  */  
  
/** @defgroup OV5640_Private_FunctionPrototypes
  * @{
  */
//static uint32_t ov5640_ConvertValue(uint32_t feature, uint32_t value);
/**
  * @}
  */ 
  
/** @defgroup OV5640_Private_Variables
  * @{
  */        

CAMERA_DrvTypeDef   ov5640_drv = 
{
  ov5640_Init,
  ov5640_ReadID,  
  ov5640_Config,
};

/**
  * @}
  */
  
/** @defgroup OV5640_Private_Functions
  * @{
  */ 

/**
  * @brief  Write data to OV5640 register.
  * @param  addr: Register address to be write.
  * @param  value: Value to be configured
	* @retval 0:OK,0xFF:KO	
  */
uint8_t OV5640_WriteReg(uint16_t addr, uint8_t value)
{
	HAL_StatusTypeDef status = HAL_OK;
	
	status = I2Cx_WriteMultiple(OV5640_I2C_ADDRESS, addr, I2C_MEMADD_SIZE_16BIT, &value, 1);
	/* Check the communication status */
  if(status != HAL_OK)
  {
    /* Re-Initiaize the I2C Bus */
    I2Cx_Error(OV5640_I2C_ADDRESS);
		return 0xFF;
  }
	return 0;
}

/**
  * @brief  Read data from OV5640 register.
  * @param  addr: Register address to be read.
	* @retval Register value
  */
uint8_t OV5640_ReadReg(uint16_t addr)
{
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t value = 0;
	
	status = I2Cx_ReadMultiple(OV5640_I2C_ADDRESS, addr, I2C_MEMADD_SIZE_16BIT, &value, 1);
	/* Check the communication status */
  if(status != HAL_OK)
  {
    /* Re-Initiaize the I2C Bus */
    I2Cx_Error(OV5640_I2C_ADDRESS);
  }
	return value;
}

/**
  * @brief  Read the OV5640 Camera identity.
  * @param  DeviceAddr: Device address on communication Bus.
  * @retval the OV5640 ID
  */
uint16_t ov5640_ReadID(uint16_t DeviceAddr)
{
	uint8_t buffer[2];
	I2Cx_Init();
	buffer[0] = OV5640_ReadReg(OV5640_SENSOR_PIDH);
  buffer[1] = OV5640_ReadReg(OV5640_SENSOR_PIDL);
  /* Get the camera ID */
  return ((buffer[0]<<8) | buffer[1]);
}

/**
  * @brief  Resets the OV5640 camera.
  * @param  None
  * @retval None
  */
void OV5640_Reset(void)
{
	//Soft reset OV5640
  OV5640_WriteReg(0x3008, 0x80);
}

/**
  * @brief  Set the resolution of the output image.
  * @param  width: image's width.
  * @param  height: image's height.
	* @retval None
  */
void OV5640_Set_OutSize(uint16_t width,uint16_t height)
{ 
	OV5640_WriteReg(0x3808,width>>8);		//set hight byte
	OV5640_WriteReg(0x3809,width&0xff);	//set low byte
	OV5640_WriteReg(0x380a,height>>8);	//set hight byte
	OV5640_WriteReg(0x380b,height&0xff);//set low byte
}

/**
  * @brief  Write Auto Focus firmware.
  * @param  None.
	* @retval OK:0,KO:0xFF
  */
uint8_t OV5640_Auto_Focus_Init(void)
{
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t value;
	uint32_t i;	
	
	//Enable the MCU clock by setting register 0x3000[5] to 1'b1
	OV5640_WriteReg(0x3000, 0x20);
	
//	for(i = 0;i< sizeof(Auto_Focus_Init_Table); i++)
//	{
//		value = Auto_Focus_Init_Table[i];
//		OV5640_WriteReg((0x8000+i),value);
//	}

	status = I2Cx_WriteMultiple(OV5640_I2C_ADDRESS, OV5640_FIRMWARE_ADDR, I2C_MEMADD_SIZE_16BIT, (uint8_t *)Auto_Focus_Init_Table, sizeof(Auto_Focus_Init_Table));
	/* Check the communication status */
  if(status != HAL_OK)
  {
    /* Re-Initiaize the I2C Bus */
    I2Cx_Error(OV5640_I2C_ADDRESS);
		return 0xFF;
  }
	//After downloading the firmware set register 0x3000[5] to 1'b0
	OV5640_WriteReg(0x3022, 0x00);
	OV5640_WriteReg(0x3023, 0x00);
	OV5640_WriteReg(0x3024, 0x00);
	OV5640_WriteReg(0x3025, 0x00);
	OV5640_WriteReg(0x3026, 0x00);
	OV5640_WriteReg(0x3027, 0x00);
	OV5640_WriteReg(0x3028, 0x00);
	OV5640_WriteReg(0x3029, 0xFF);
	OV5640_WriteReg(0x3000, 0x00);
	OV5640_WriteReg(0x3004, 0xFF);
	OV5640_WriteReg(0x0000, 0x00);
	OV5640_WriteReg(0x0000, 0x00);
	OV5640_WriteReg(0x0000, 0x00);
	OV5640_WriteReg(0x0000, 0x00);
	//Check Status of focus
	do 
	{
		value = (uint8_t)OV5640_ReadReg(0x3029);
		HAL_Delay(500);
		i++;
		if(i > 1000)
		{
			return 0xFF;
		}
	} 
	while(value != 0x70);	
	return 0;
}
/**
  * @brief  Set OV5640 constant focus.
  * @param  None.
	* @retval OK:0,KO:0xFF
  */
uint8_t OV5640_Focus_Constant(void)
{
	uint8_t state = 0x8F;
	uint32_t i;

	//send constant focus mode command to firmware
	OV5640_WriteReg(0x3023,0x01);
	OV5640_WriteReg(0x3022,0x04);
	do 
	{
		state = (uint8_t)OV5640_ReadReg(0x3023);
		i++;
		if(i > 1000)
		{
			return 0xFF;
		}
		HAL_Delay(500);
	} while(state!=0x00);//0x0 : focused 0x01: is focusing
	return 0;
}



/**
  * @brief  Initializes the OV5640 CAMERA component.
  * @param  DeviceAddr: Device address on communication Bus.
  * @param  resolution: Camera resolution
  * @retval None
  */
void ov5640_Init(uint16_t DeviceAddr, uint32_t resolution)
{
	uint32_t i;
  
  /* Initialize I2C */
  CAMERA_IO_Init();
	
	/*Soft reset*/
	OV5640_Reset();
	
	HAL_Delay(50);
	
	//initialize OV5640 necessary register
  for(i=0; i<(sizeof(REG_Init_Table)/4); i++)
  {
    OV5640_WriteReg(REG_Init_Table[i][0], REG_Init_Table[i][1]);
		HAL_Delay(2);
  }
	
	//initialize OV5640 to RGB565 format,default resolution is 800x480
	for(i=0; i<(sizeof(RGB565_Init_Table)/4); i++)
  {
    OV5640_WriteReg(RGB565_Init_Table[i][0], RGB565_Init_Table[i][1]);
		HAL_Delay(2);
  }
  
  /* Initialize the resolution */
  switch (resolution)
  {
		case CAMERA_R160x120:
		{
			OV5640_Set_OutSize(160,120); 
			break;
		}    
		case CAMERA_R320x240:
		{
			OV5640_Set_OutSize(320,240); 
			break;
		}
		case CAMERA_R480x272:
		{
			OV5640_Set_OutSize(480,272); 
			break;
		}
		case CAMERA_R640x480:
		{
			OV5640_Set_OutSize(640,480);
			break;
		}
		case CAMERA_R800x480:
		{
			OV5640_Set_OutSize(800,480);
			break;
		} 
		case CAMERA_R1024x600:
		{
			OV5640_Set_OutSize(1024,600);
			break;
		} 		
		default:
    {
      break;
    }
  }
	//LED On
	OV5640_LED_Ctrl(1);

	//Write Auto Focus firmware
	OV5640_Auto_Focus_Init();
	//Set constant focus
	OV5640_Focus_Constant();
	//LED Off
	OV5640_LED_Ctrl(0);
}



/**
  * @brief  Configures the camera brightness.
  * @param  bright: brightness level
  *          This parameter can be one of the following values:
	*            @arg  0:-4
	*            @arg  1:-3
	*            @arg  2:-2
	*            @arg  3:-1
	*            @arg  4:0
	*            @arg  5:1
	*            @arg  6:2
	*            @arg  7:3
	*            @arg  8:4
  * @retval None
  */
void OV5640_Set_Brightness(uint8_t bright)
{
	uint8_t value;
	if(bright < 4)
		value = 4 - bright;
	else 
		value = bright - 4;
	OV5640_WriteReg(0x3212,0x03);	//start group 3
	OV5640_WriteReg(0x5587,value<<4);
	if(bright < 4)
		OV5640_WriteReg(0x5588,0x09);
	else 
		OV5640_WriteReg(0x5588,0x01);
	OV5640_WriteReg(0x3212,0x13); //end group 3
	OV5640_WriteReg(0x3212,0xa3); //launch group 3
}

/**
  * @brief  Configures the camera contrast.
  * @param  contrast: contrast level
  *          This parameter can be one of the following values:
	*            @arg  0:-3
	*            @arg  1:-2
	*            @arg  2:-1
	*            @arg  3:0
	*            @arg  4:1
	*            @arg  5:2
	*            @arg  6:3
  * @retval None
  */
void OV5640_Set_Contrast(uint8_t contrast)
{
	uint8_t value1 = 0x00;//contrast=3
	uint8_t value2 = 0x20;
	switch(contrast)
	{
		case 0://-3
			value2 = value1 = 0x14;	 	 
			break;	
		case 1://-2
			value2 = value1 = 0x18; 	 
			break;	
		case 2://-1
			value2 = value1 = 0x1C;	 
			break;	
		case 4://1
			value2 = 0x10;	 	 
			value2 = 0x24;	 	 
			break;	
		case 5://2
			value1 =0x18;	 	 
			value2 = 0x28;	 	 
			break;	
		case 6://3
			value1 = 0x1C;	 	 
			value2 = 0x2C;	 	 
			break;	
	} 
	OV5640_WriteReg(0x3212,0x03); //start group 3
	OV5640_WriteReg(0x5585,value1);
	OV5640_WriteReg(0x5586,value2); 
	OV5640_WriteReg(0x3212,0x13); //end group 3
	OV5640_WriteReg(0x3212,0xa3); //launch group 3
}

/**
  * @brief  Configures the camera color effect.
  * @param  value: color effect
  *          This parameter can be one of the following values:
	*            @arg  0:Normal
	*            @arg  1:Blueish (cool light)
	*            @arg  2:Redish (warm)
	*            @arg  3:Black and white
	*            @arg  4:Sepia
	*            @arg  5:Negative
	*            @arg  6:Greenish
  * @retval None
  */
void OV5640_Set_Color_Effect(uint8_t value)
{
	OV5640_WriteReg(0x3212,0x03); //start group 3
	OV5640_WriteReg(0x5580,Color_Effect_Table[value][0]);
	OV5640_WriteReg(0x5583,Color_Effect_Table[value][1]);// sat U
	OV5640_WriteReg(0x5584,Color_Effect_Table[value][2]);// sat V
	OV5640_WriteReg(0x5003,0x08);
	OV5640_WriteReg(0x3212,0x13); //end group 3
	OV5640_WriteReg(0x3212,0xa3); //launch group 3
}

/**
  * @brief  Configures the OV5640 camera Black and white mode.
  * @param  BlackWhite: BlackWhite value, where BlackWhite can be: 
  *         0x18 for B&W,
  *         0x40 for Negative,
  *         0x58 for B&W negative,
  *         0x00 for Normal,
  * @retval None
  */
void OV5640_Set_BandW(uint8_t BlackWhite)
{
  OV5640_WriteReg(0xff, 0x00);
  OV5640_WriteReg(0x7c, 0x00);
  OV5640_WriteReg(0x7d, BlackWhite);
  OV5640_WriteReg(0x7c, 0x05);
  OV5640_WriteReg(0x7d, 0x80);
  OV5640_WriteReg(0x7d, 0x80);
}

/**
  * @brief  Configures the OV5640 camera feature.
  * @param  DeviceAddr: Device address on communication Bus.
  * @param  feature: Camera feature to be configured
  * @param  value: Value to be configured
  * @param  brightness_value: Brightness value to be configured
  * @retval None
  */
void ov5640_Config(uint16_t DeviceAddr, uint32_t feature, uint32_t value, uint32_t brightness_value)
{
  switch(feature)
  {
  case CAMERA_BLACK_WHITE:
    {  
			OV5640_Set_BandW(value);
      break;
    }
  case CAMERA_CONTRAST_BRIGHTNESS:
    {
			OV5640_Set_Contrast(value);
			OV5640_Set_Brightness(brightness_value);
      break;
    }
  case CAMERA_COLOR_EFFECT:
    {     
			OV5640_Set_Color_Effect(value);
      break;
    }     
  default:
    {
      break;
    }
  }
}
/**
  * @brief  Turns the LED On or Off.
  * @param  led: LED to be set on or Off.
  *          This parameter can be one of the following values:
	*            @arg  0:LED Off
	*            @arg  1:LED ON
  * @retval None
  */
void OV5640_LED_Ctrl(uint8_t led)
{
	OV5640_WriteReg(0x3016,0x02);
	OV5640_WriteReg(0x301C,0x02); 
	if(led == 1)
		OV5640_WriteReg(0x3019,0x02); 
	else 
		OV5640_WriteReg(0x3019,0x00);
}

/******************************************************************************
                            Static Functions
*******************************************************************************/
/**
  * @brief  Convert input values into ov5640 parameters.
  * @param  feature: Camera feature to be configured
  * @param  value: Value to be configured
  * @retval The converted value
  */
//not used
//static uint32_t ov5640_ConvertValue(uint32_t feature, uint32_t value)
//{
//  uint32_t ret = 0;
//  
//  switch(feature)
//  {
//  case CAMERA_BLACK_WHITE:
//    {
//      switch(value)
//      {
//      case CAMERA_BLACK_WHITE_BW:
//        {
//          ret =  OV5640_BLACK_WHITE_BW;
//          break;
//        }
//      case CAMERA_BLACK_WHITE_NEGATIVE:
//        {
//          ret =  OV5640_BLACK_WHITE_NEGATIVE;
//          break;
//        }
//      case CAMERA_BLACK_WHITE_BW_NEGATIVE:
//        {
//          ret =  OV5640_BLACK_WHITE_BW_NEGATIVE;
//          break;
//        }
//      case CAMERA_BLACK_WHITE_NORMAL:
//        {
//          ret =  OV5640_BLACK_WHITE_NORMAL;
//          break;
//        }
//      default:
//        {
//          ret =  OV5640_BLACK_WHITE_NORMAL;
//          break;
//        }
//      }
//      break;
//    }
//  case CAMERA_CONTRAST_BRIGHTNESS:
//    {
//      switch(value)
//      {
//      case CAMERA_BRIGHTNESS_LEVEL0:
//        {
//          ret =  OV5640_BRIGHTNESS_LEVEL0;
//          break;
//        }
//      case CAMERA_BRIGHTNESS_LEVEL1:
//        {
//          ret =  OV5640_BRIGHTNESS_LEVEL1;
//          break;
//        }
//      case CAMERA_BRIGHTNESS_LEVEL2:
//        {
//          ret =  OV5640_BRIGHTNESS_LEVEL2;
//          break;
//        }
//      case CAMERA_BRIGHTNESS_LEVEL3:
//        {
//          ret =  OV5640_BRIGHTNESS_LEVEL3;
//          break;
//        }
//      case CAMERA_BRIGHTNESS_LEVEL4:
//        {
//          ret =  OV5640_BRIGHTNESS_LEVEL4;
//          break;
//        }        
//      case CAMERA_CONTRAST_LEVEL0:
//        {
//          ret =  OV5640_CONTRAST_LEVEL0;
//          break;
//        }
//      case CAMERA_CONTRAST_LEVEL1:
//        {
//          ret =  OV5640_CONTRAST_LEVEL1;
//          break;
//        }
//      case CAMERA_CONTRAST_LEVEL2:
//        {
//          ret =  OV5640_CONTRAST_LEVEL2;
//          break;
//        }
//      case CAMERA_CONTRAST_LEVEL3:
//        {
//          ret =  OV5640_CONTRAST_LEVEL3;
//          break;
//        }
//      case CAMERA_CONTRAST_LEVEL4:
//        {
//          ret =  OV5640_CONTRAST_LEVEL4;
//          break;
//        }
//      default:
//        {
//          ret =  OV5640_CONTRAST_LEVEL0;
//          break;
//        }
//      }
//      break;
//    }
//  case CAMERA_COLOR_EFFECT:
//    {
//      switch(value)
//      {
//      case CAMERA_COLOR_EFFECT_ANTIQUE:
//        {
//          ret =  OV5640_COLOR_EFFECT_ANTIQUE;
//          break;
//        }
//      case CAMERA_COLOR_EFFECT_BLUE:
//        {
//          ret =  OV5640_COLOR_EFFECT_BLUE;
//          break;
//        }
//      case CAMERA_COLOR_EFFECT_GREEN:
//        {
//          ret =  OV5640_COLOR_EFFECT_GREEN;
//          break;
//        }
//      case CAMERA_COLOR_EFFECT_RED:
//        {
//          ret =  OV5640_COLOR_EFFECT_RED;
//          break;
//        }
//      default:
//        {
//          ret =  OV5640_COLOR_EFFECT_RED;
//          break;
//        }
//      }
//      break;
//    default:
//      {
//        ret = 0;
//        break;
//      }    
//    }
//  }
//  
//  return ret;
//}
         
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
