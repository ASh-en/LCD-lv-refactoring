/**
  ******************************************************************************
  * @file    gt911.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    03-August-2015
  * @brief   This file provides a set of functions needed to manage the gt911
  *          IO Expander devices.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
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
#include "gt911.h"
#include "stm32f7xx_eval.h"
#include "stm32f7xx_hal.h"
#include "stdio.h"
#include "string.h"
#include "gt911_firmware.h"

extern I2C_HandleTypeDef hEvalI2c;
struct goodix_ts_data ts;
uint8_t i2c_opr_buf[FL_PACK_SIZE] = {0};
uint8_t chk_cmp_buf[FL_PACK_SIZE] = {0};

uint16_t version_info;
uint8_t cfg[240] = {0};
uint8_t CTP_CFG_GROUP[] = {\
0x00,0xE0,0x01,0x20,0x03,0x0A,0x05,0x00,0x01,0x0F,\
0x28,0x0F,0x50,0x32,0x03,0x05,0x00,0x00,0x00,0x00,\
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x89,0x29,0x0A,\
0x52,0x50,0x0C,0x08,0x00,0x00,0x00,0x00,0x03,0x1D,\
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x32,0x00,0x00,\
0x00,0x48,0x70,0x94,0xC5,0x02,0x07,0x00,0x00,0x04,\
0x87,0x4B,0x00,0x7D,0x52,0x00,0x74,0x59,0x00,0x6B,\
0x62,0x00,0x64,0x6B,0x00,0x64,0x00,0x00,0x00,0x00,\
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
0x00,0x00,0x08,0x0A,0x0C,0x0E,0x10,0x12,0x14,0x16,\
0x18,0x1A,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
0x00,0x00,0x2A,0x29,0x28,0x24,0x22,0x20,0x1F,0x1E,\
0x1D,0x0E,0x0C,0x0A,0x08,0x06,0x05,0x04,0x02,0x00,\
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
0x00,0x00,0x00,0x00,0x00,0x00,0xAA,0x01};
/** @addtogroup BSP
  * @{
  */

/** @addtogroup Component
  * @{
  */

/** @defgroup gt911
  * @{
  */

/* Private typedef -----------------------------------------------------------*/


/**
  * @brief  Software Reset the gt911.
  *         @note : Not applicable to FT5206.
  * @param  DeviceAddr: Device address on communication Bus (I2C slave address of FT5206).
  * @retval None
  */
void gt911_Init(void)
{
  I2Cx_Init();
	gtp_get_chip_type();
	gtp_gt9xxf_init();
	gt911_read_version(&version_info);
}
ErrorStatus gt911_read_reg(uint16_t reg_addr, uint32_t cnt, uint8_t *value)
{
	HAL_StatusTypeDef status;
	uint8_t buf[2];
	
	buf[0] = (uint8_t)(reg_addr>>8);	
	buf[1] = (uint8_t)reg_addr;
	//
	status = HAL_I2C_Master_Transmit(&hEvalI2c, GT811_I2C_ADDR_W, buf, 2,1000);
	if(status != HAL_OK)
	{
		/* Return KO */
		return ERROR; 
	}
	
	HAL_I2C_Master_Receive(&hEvalI2c, GT811_I2C_ADDR_R, value, cnt, 1000);
	if(status != HAL_OK)
	{
		/* Return KO */
		return ERROR; 
	}
	return SUCCESS;
}

//uint8_t gt911_Read_REG(uint16_t RegAddr)
//{
////  HAL_StatusTypeDef status;
//	uint8_t buf[2];
//	
//	buf[0] = (uint8_t)(RegAddr>>8);	
//	buf[1] = (uint8_t)RegAddr;
//	//
//	HAL_I2C_Master_Transmit(&hEvalI2c, GT811_I2C_ADDR_W, buf, 2,1000);
//	
//	HAL_I2C_Master_Receive(&hEvalI2c, GT811_I2C_ADDR_R, buf, 1, 1000);
//	return buf[0];
//}

ErrorStatus gt911_write_reg(uint16_t reg_addr, uint32_t cnt, uint8_t *value)
{
  HAL_StatusTypeDef status = HAL_OK;

  status = HAL_I2C_Mem_Write(&hEvalI2c, GT811_I2C_ADDR_W, (uint16_t)reg_addr, I2C_MEMADD_SIZE_16BIT, value, cnt, 1000); 

  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Execute user timeout callback */
    I2Cx_Error(GT811_I2C_ADDR_W);
		return ERROR; 
  }
	return SUCCESS;
}
//void gt911_Write_REG(uint16_t RegAddr,uint8_t Value)
//{
////  HAL_StatusTypeDef status;
//	uint8_t buf[3];
//	
//	buf[0] = (uint8_t)(RegAddr>>8);	
//	buf[1] = (uint8_t)RegAddr;
//	buf[2] = Value;
//	//
//	HAL_I2C_Master_Transmit(&hEvalI2c, GT811_I2C_ADDR_W, buf, 3, 1000);
//}
void gtp_get_chip_type(void)
{
	uint8_t temp[16] = {0};
	gt911_read_reg(GTP_REG_CHIP_TYPE,16,temp);
	if (!memcmp(temp,"GOODIX_GT9",10))
	{
		ts.chip_type = CHIP_TYPE_GT9;
	}
	else
	{
		ts.chip_type = CHIP_TYPE_GT9F;
	}
}

ErrorStatus gt911_read_version(uint16_t* version)
{
	ErrorStatus ret;
	uint8_t buf[8] = {0};

	ret = gt911_read_reg(GTP_REG_VERSION,6,buf);
	if (ret == ERROR)
	{

		return ret;
	}

	if (version)
	{
		*version = (buf[5] << 8) | buf[4];
	}
	
	return SUCCESS;
}

ErrorStatus gtp_send_cfg(void)
{
	ErrorStatus ret;


	int32_t retry = 0;
	
	for (retry = 0; retry < 5; retry++)
	{
		//ret = gtp_i2c_wr ite(client, config , 240GTP_CONFIG_MAX_LENGTH + GTP_ADDR_LENGTH);
		ret = gt911_write_reg(GTP_REG_CONFIG_DATA,228,CTP_CFG_GROUP);//cfg
		if (ret == SUCCESS)
		{
			break;
		}
	}

	return ret;
}

static ErrorStatus gup_check_update_file_fl(st_fw_head* fw_head)
{
	ErrorStatus ret;
	int32_t i = 0;
	int32_t fw_checksum = 0;

	memcpy(fw_head, gtp_default_FW_fl, FW_HEAD_LENGTH);

	fw_head->vid = ((fw_head->vid & 0xFF00) >> 8) + ((fw_head->vid & 0x00FF) << 8);


	//check firmware legality
	fw_checksum = 0;
	for(i = FW_HEAD_LENGTH; i < (FW_HEAD_LENGTH+FW_SECTION_LENGTH*4+FW_DSP_LENGTH); i += 2)
	{
		fw_checksum += (gtp_default_FW_fl[i] << 8) + gtp_default_FW_fl[i+1];
	}
	ret = SUCCESS;
	if (fw_checksum & 0xFFFF)
	{
		ret = ERROR;
	}

	return ret;
}

ErrorStatus gup_fw_download_proc( uint8_t dwn_mode)
{
	ErrorStatus ret;
	uint8_t  retry = 0;
	st_fw_head fw_head;
	

	ret = gup_check_update_file_fl(&fw_head);
	//    show_len = 10;

	if (ERROR == ret)
	{
		goto file_fail;
	}

	if (!memcmp(fw_head.pid, "950", 3))
	{
		ts.is_950 = 1;
	}
	else
	{
		ts.is_950 = 0;
	}


#if GTP_ESD_PROTECT
	if (NULL != dir)
	{
		gtp_esd_switch(SWITCH_OFF);
	}
#endif

	ret = gup_enter_update_mode_fl();   
	//show_len = 20;    
	if (ERROR == ret)    
	{ 
		goto download_fail;    
	}

	while (retry++ < 5)
	{
		ret = gup_download_fw_ss51(dwn_mode);
		
		if (ERROR == ret)
		{
			continue;
		}

		ret = gup_download_fw_dsp(dwn_mode);
		// show_len = 80;
		if (ERROR == ret)
		{
			continue;
		}
		break;
	}

	if (retry >= 5)
	{
		goto download_fail;
	}

	return SUCCESS;

download_fail:

	//GT9XX_Resume();
file_fail:
	//show_len = 200;

	//ts->enter_udpate = 0;
	return ERROR;
}

ErrorStatus gtp_gt9xxf_init()
{
	ErrorStatus ret;
	

	ret = gup_fw_download_proc( GTP_FL_FW_BURN);

	if (ERROR == ret)
	{
		return ERROR;
	}

	ret = gtp_fw_startup();
	if (ERROR == ret)
	{
		return ERROR;
	}

	return SUCCESS;
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
