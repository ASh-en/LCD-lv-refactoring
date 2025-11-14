/**
  ******************************************************************************
  * @file    fontchinese.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    18-February-2014
  * @brief   This file provides text font24 for STM32xx-EVAL's LCD driver. 
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
#include "fontchines.h"
#include "stm32f7xx_eval_lcd.h"

/* FatFs includes component */
#include "ff_gen_drv.h"
#include "sd_diskio.h"

static FIL Font_File;		/* file objects */
static FATFS fs;				/* Work area (file system object) for logical drives */
static FRESULT res_sd; 
static UINT br;         /* File R/W count */

uint8_t SD_mount = 0;

extern LTDC_HandleTypeDef  hLtdcEval;
extern uint32_t            ActiveLayer;
extern LCD_DrawPropTypeDef DrawProp[MAX_LAYER_NUMBER];

/** @addtogroup Utilities
  * @{
  */
  
/** @addtogroup STM32_EVAL
  * @{
  */ 

/** @addtogroup Common
  * @{
  */

/** @addtogroup FONTS
  * @brief      This file provides text font24 for STM32xx-EVAL's LCD driver.
  * @{
  */  

/** @defgroup FONTS_Private_Types
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup FONTS_Private_Defines
  * @{
  */
/**
  * @}
  */ 


/** @defgroup FONTS_Private_Macros
  * @{
  */
/**
  * @}
  */ 
  

/** @defgroup FONTS_Private_Variables
  * @{
  */
sFONTCHINES FontSong12 = {
  "0:/Font/宋体12.DZK",
  16, /* Width */
  16, /* Height */
};

sFONTCHINES FontSong18 = {
  "0:/Font/宋体18.DZK",
  24, /* Width */
  24, /* Height */
};

sFONTCHINES FontYouYuan18 = {
  "0:/Font/幼圆18.DZK",
  24, /* Width */
  24, /* Height */
};

sFONTCHINES FontCaiYun17 = {
  "0:/Font/华文彩云17.DZK",
  24, /* Width */
  24, /* Height */
};

sFONTCHINES FontCaiYun23 = {
  "0:/Font/华文彩云23.DZK",
  32, /* Width */
  32, /* Height */
};

/**
  * @brief  Get the Chinese Font from SD card
  * @param  pBuffer: Pointer to source buffer for the character array
  * @param  Code: GB code
	* @param  FileName: Name of the Font File
	* @param  FileSize: Size of one chinese character
	* @retval 1:OK,0:Fail
  */
uint32_t GetChineseFont_From_SD(uint8_t *pBuffer, uint16_t Code, char *FileName,uint32_t FileSize)
{ 
	uint8_t Internal_Code_H;
	uint8_t Internal_Code_L;
	uint32_t position;

	Internal_Code_H = Code >> 8;    	//Chinese Character Internal Code1
	Internal_Code_L = Code & 0x00FF; 	//Chinese Character Internal Code2

	//Get character array location
	//GB2312
//	position = ((Internal_Code_H-0xa1)*94+(Internal_Code_L-0xa1))*FileSize;	
	//GBK
	position = ((Internal_Code_H-0x81)*190+(Internal_Code_L-0x41))*FileSize;	

	//Mount the SD card at the first time
	if(SD_mount == 0)
	{
		res_sd = f_mount(&fs,"0:",1);
		SD_mount = 1;
	}
	res_sd = f_open(&Font_File, FileName, FA_OPEN_EXISTING | FA_READ); 
	if(res_sd == FR_OK) 
	{
		f_lseek(&Font_File, position);		//Move read/write pointer by offset
		res_sd = f_read(&Font_File, pBuffer, FileSize, &br);
		f_close(&Font_File);
			return 1;  
	}    
	else
		return 0;    
}

//RGB565              RRRRR   -GGGGGG  -BBBBB
//ARGB8888   11111111-RRRRRRRR-GGGGGGGG-BBBBBBBB
/**
  * @brief  Draws a chines character on LCD.
  * @param  Xpos: Line where to display the character shape
  * @param  Ypos: Start column address
  * @param  c: Pointer to the character data
  * @retval None
  */
void BSP_LCD_DisplayChinesChar(uint16_t Xpos,uint16_t Ypos, uint16_t usChar)
{
	uint32_t i = 0, j = 0;
	uint32_t TextColor,BackColor;
	uint32_t line;
	uint32_t Display_Offset =0;
  uint32_t Pixel_Offset = 0;
	uint16_t height, width;
	char *FileName;
	
	height = DrawProp[ActiveLayer].pFontChines->Height;
  width  = DrawProp[ActiveLayer].pFontChines->Width;
	TextColor = DrawProp[ActiveLayer].TextColor;
	BackColor = DrawProp[ActiveLayer].BackColor;
	FileName = DrawProp[ActiveLayer].pFontChines->FileName;
	
	uint8_t Pixel_Buffer[height*width/8];	
  
	//Current display line offset
	if(hLtdcEval.LayerCfg[ActiveLayer].PixelFormat == LTDC_PIXEL_FORMAT_ARGB8888)
	{
		Display_Offset = Ypos*BSP_LCD_GetXSize()*4;	//RGB888
	}
	else
		Display_Offset = Ypos*BSP_LCD_GetXSize()*2;	//RGB565
	
	//Offset of the character piex
  Pixel_Offset += Xpos;
	   
  GetChineseFont_From_SD(Pixel_Buffer,usChar,FileName,height*width/8);//Get piex datas
	
	//Current line,Total line in one character=height
	for(i = 0; i < height; i++)
	{
		//Get the piex datas of one line,depend on the width
		switch(width/8) 
    {  
			case 1:
				line =  Pixel_Buffer[i];      
				break;				
			case 2:
				line =  (Pixel_Buffer[i*2]<<8) | Pixel_Buffer[i*2+1];      
				break;				
			case 3:
				line =  (Pixel_Buffer[i*3]<<16) | (Pixel_Buffer[i*3+1]<<8) | Pixel_Buffer[i*3+2];      
				break;
			case 4:
				line =  (Pixel_Buffer[i*4]<<24) | (Pixel_Buffer[i*4+1]<<16) | (Pixel_Buffer[i*4+2]<<8) | Pixel_Buffer[i*4+3];      
				break;
			default:
				line =  (Pixel_Buffer[i*3]<<16) | (Pixel_Buffer[i*3+1]<<8) | Pixel_Buffer[i*3+2];      
				break;
    }
		//Start to draw one line
		for(j = 0; j < width;j++ ) 
		{			
			if(line & (0x01 << (width-1)))	//1:TextColor;0:BackColor
			{
				//Color of the character
				if(hLtdcEval.LayerCfg[ActiveLayer].PixelFormat == LTDC_PIXEL_FORMAT_ARGB8888)
					*(__IO uint32_t*)(hLtdcEval.LayerCfg[ActiveLayer].FBStartAdress + Display_Offset + (4*Pixel_Offset)) = TextColor;//ARGB8888
				else
					*(__IO uint16_t*)(hLtdcEval.LayerCfg[ActiveLayer].FBStartAdress + Display_Offset + (2*Pixel_Offset)) = ARGB8888_To_RGB565(TextColor);//RGB565
			}				
			else	
			{
				//Color of the character
				if(hLtdcEval.LayerCfg[ActiveLayer].PixelFormat == LTDC_PIXEL_FORMAT_ARGB8888)
					*(__IO uint32_t*)(hLtdcEval.LayerCfg[ActiveLayer].FBStartAdress + Display_Offset + (4*Pixel_Offset)) = BackColor;//ARGB8888
				else
					*(__IO uint16_t*)(hLtdcEval.LayerCfg[ActiveLayer].FBStartAdress + Display_Offset + (2*Pixel_Offset)) = ARGB8888_To_RGB565(BackColor);//RGB565
			}	
			//Next piex
			Pixel_Offset++;			
			line <<= 1;
		}
		//End of one line,point to the first piex of next line
		Pixel_Offset += (BSP_LCD_GetXSize() - width);		
	}
}

/**
  * @brief  Displays chines characters on the LCD.
  * @param  Xpos: X position (in pixel)
  * @param  Ypos: Y position (in pixel)   
  * @param  ptr: Pointer to string to display on LCD
  * @retval None
  */
void BSP_LCD_DisplayChineseStringAt(uint16_t Xpos, uint16_t Ypos, uint8_t *ptr)
{
	uint16_t ChinesCharacter;
	
	uint16_t cnheight, cnwidth;
	uint16_t enwidth;
	//Chines character height and width
	cnheight = DrawProp[ActiveLayer].pFontChines->Height;
  cnwidth  = DrawProp[ActiveLayer].pFontChines->Width;
	//English character width
  enwidth  = DrawProp[ActiveLayer].pFont->Width;
	
	while(*ptr != '\0')
	{
		if (*ptr <= 126)		//English Character
		{
			//Check the end of the line
			if((Xpos + enwidth) > BSP_LCD_GetXSize())
			{
				Xpos = 0;
				//Alignment to the height of chines character
				Ypos += cnheight;
			}
			//Check the and of the height,Alignment to the height of chines character
			if((Ypos + cnheight) > BSP_LCD_GetYSize())
			{
				Xpos = 0;
				Ypos = 0;
			}			
			BSP_LCD_DisplayChar(Xpos,Ypos,*ptr);
			Xpos += enwidth;
		  ptr++;
		}		
		else		//Chines Character
		{
			//Check the end of the line
			if((Xpos + cnwidth) > BSP_LCD_GetXSize())
			{
				Xpos = 0;
				//Alignment to the height of chines character
				Ypos += cnheight;
			}
			//Check the and of the height,Alignment to the height of chines character
			if((Ypos + cnheight) > BSP_LCD_GetYSize())
			{
				Xpos = 0;
				Ypos = 0;
			}				
			//Two bytes in one chines character 
			ChinesCharacter = *(uint16_t *)ptr;				
			ChinesCharacter = (ChinesCharacter << 8) + (ChinesCharacter >> 8);		

			BSP_LCD_DisplayChinesChar(Xpos,Ypos,ChinesCharacter);
			
			Xpos += cnwidth;
			
			ptr += 2;    
    }		
  }
}

/**
  * @brief  Display chines characters on the LCD with wrap.
  * @param  Line: Line where to display the character shape
  * @param  ptr: Pointer to string to display on LCD
  * @retval None
  */
void BSP_LCD_DisplayChineseStringAtLine(uint16_t Line, uint8_t *ptr)
{
	BSP_LCD_DisplayChineseStringAt(0,LINE(Line),ptr);
}

/**
  * @}
  */ 


/** @defgroup FONTS_Private_Function_Prototypes
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup FONTS_Private_Functions
  * @{
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

/**
  * @}
  */

/**
  * @}
  */  
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
