/**
 ******************************************************************************
 * @file    gt911.h
 * @author  MCD Application Team
  * @version V1.0.0
  * @date    03-August-2015
 * @brief   This file contains all the functions prototypes for the
 *          gt911.c IO expander driver.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GT911_H
#define __GT911_H

#ifdef __cplusplus
extern "C" {
#endif
  
/* Includes ------------------------------------------------------------------*/
#include "../Common/ts.h"
#include "stm32f7xx_hal.h"	
#define GT811_I2C_ADDR_W	0x28
#define GT811_I2C_ADDR_R	0x29
	
// Registers define
#define GTP_REG_CHIP_TYPE     0x8000
#define PRODUCT_ID1						0x8140
#define PRODUCT_ID2						0x8141
#define PRODUCT_ID3						0x8142
#define PRODUCT_ID4						0x8143
#define FIRMWARE_VERSION_L		0x8144
#define FIRMWARE_VERSION_H		0x8145	
#define GTP_READ_COOR_ADDR    0x814E
#define GTP_REG_SLEEP         0x8040
#define GTP_REG_SENSOR_ID     0x814A
#define GTP_REG_CONFIG_DATA   0x8047
#define GTP_REG_VERSION       0x8140
	
#define X1_COORDINATE_L				0x8150
#define X1_COORDINATE_H				0x8151
#define Y1_COORDINATE_L				0x8152
#define Y1_COORDINATE_H				0x8153

#define FL_PACK_SIZE          1024
#define GUP_FW_CHK_SIZE       FL_PACK_SIZE    //FL_PACK_SIZE

/* Macros --------------------------------------------------------------------*/
typedef enum
{
	CHIP_TYPE_GT9  = 0,
	CHIP_TYPE_GT9F = 1,
} CHIP_TYPE_T;
struct goodix_ts_data 
{
	uint16_t bak_ref_len;
	uint32_t ref_chk_fs_times;
	uint32_t clk_chk_fs_times;
	CHIP_TYPE_T chip_type;
	uint8_t rqst_processing;
	uint8_t is_950;    
};
typedef struct 
{
	uint8_t  hw_info[4];          //hardware info//
	uint8_t  pid[8];              //product id   //
	uint16_t vid;                 //version id   //
}st_fw_head;
/** @typedef gt911_handle_TypeDef
 *  gt911 Handle definition.
 */
typedef struct
{
  uint8_t i2cInitialized;

  /* field holding the current number of simultaneous active touches */
  uint8_t currActiveTouchNb;

  /* field holding the touch index currently managed */
  uint8_t currActiveTouchIdx;

} gt911_handle_TypeDef;

void gt911_Init(void);
void gt911_Write_Config(void);
ErrorStatus gt911_read_reg(uint16_t reg_addr, uint32_t cnt, uint8_t *value);
ErrorStatus gt911_write_reg(uint16_t reg_addr, uint32_t cnt, uint8_t *value);
ErrorStatus gt911_read_version(uint16_t* version);
ErrorStatus gtp_send_cfg(void);
void gtp_get_chip_type(void);

#ifdef __cplusplus
}
#endif
#endif /* __GT911_H */


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
