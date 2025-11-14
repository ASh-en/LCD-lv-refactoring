/**
  ******************************************************************************
  * @file    stm32756g_eval_sram.c
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    22-April-2016
  * @brief   This file includes the SRAM driver for the IS61WV102416BLL-10M memory 
  *          device mounted on STM32756G-EVAL and STM32746G-EVAL evaluation boards.
  @verbatim
  How To use this driver:
  -----------------------
   - This driver is used to drive the IS61WV102416BLL-10M SRAM external memory mounted
     on STM32756G-EVAL evaluation board.
   - This driver does not need a specific component driver for the SRAM device
     to be included with.

  Driver description:
  ------------------
  + Initialization steps:
     o Initialize the SRAM external memory using the BSP_SRAM_Init() function. This 
       function includes the MSP layer hardware resources initialization and the
       FMC controller configuration to interface with the external SRAM memory.
  
  + SRAM read/write operations
     o SRAM external memory can be accessed with read/write operations once it is
       initialized.
       Read/write operation can be performed with AHB access using the functions
       BSP_SRAM_ReadData()/BSP_SRAM_WriteData(), or by DMA transfer using the functions
       BSP_SRAM_ReadData_DMA()/BSP_SRAM_WriteData_DMA().
     o The AHB access is performed with 16-bit width transaction, the DMA transfer
       configuration is fixed at single (no burst) halfword transfer 
       (see the SRAM_MspInit() static function).
     o User can implement his own functions for read/write access with his desired 
       configurations.
     o If interrupt mode is used for DMA transfer, the function BSP_SRAM_DMA_IRQHandler()
       is called in IRQ handler file, to serve the generated interrupt once the DMA 
       transfer is complete.
  @endverbatim
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
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
#include "stm32f7xx_eval_sram.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup STM32756G_EVAL
  * @{
  */ 
  
/** @defgroup STM32756G_EVAL_SRAM
  * @{
  */ 

/** @defgroup STM32756G_EVAL_SRAM_Private_Types_Definitions
  * @{
  */ 
/**
  * @}
  */ 

/** @defgroup STM32756G_EVAL_SRAM_Private_Defines
  * @{
  */
/**
  * @}
  */ 

/** @defgroup STM32756G_EVAL_SRAM_Private_Macros
  * @{
  */  
/**
  * @}
  */ 

/** @defgroup STM32756G_EVAL_SRAM_Private_Variables
  * @{
  */       
SRAM_HandleTypeDef sramHandle;
static FMC_NORSRAM_TimingTypeDef FSMC_ReadTim;
static FMC_NORSRAM_TimingTypeDef FSMC_WriteTim;
void LCD_ShowxNum(uint16_t x,uint16_t y,uint32_t num,uint8_t len,uint32_t ARGB);
extern 	int16_t w_buf[10];
extern  int16_t CH1_BUF[15000];
extern  int16_t CH2_BUF[488];
extern  int16_t CH3_BUF[488];
/**
  * @}
  */ 

/** @defgroup STM32756G_EVAL_SRAM_Private_Function_Prototypes
  * @{
  */ 
/**
  * @}
  */
    
/** @defgroup STM32756G_EVAL_SRAM_Private_Functions
  * @{
  */
//配置MPU的region(SRAM区域为透写模式)
void LCD_MPU_Config(void)
{	
	MPU_Region_InitTypeDef MPU_Initure;

	HAL_MPU_Disable();							//配置MPU之前先关闭MPU,配置完成以后在使能MPU	
	//外部SRAM为region0，大小为2MB，此区域可读写
	MPU_Initure.Enable=MPU_REGION_ENABLE;	    //使能region
	MPU_Initure.Number=MPU_REGION_NUMBER0;		//设置region，外部SRAM使用的region0
	MPU_Initure.BaseAddress=SRAM_DEVICE_ADDR;	//region基地址
	MPU_Initure.Size=MPU_REGION_SIZE_256MB;			//region大小
	MPU_Initure.SubRegionDisable=0X00;
	MPU_Initure.TypeExtField=MPU_TEX_LEVEL0;
	MPU_Initure.AccessPermission=MPU_REGION_FULL_ACCESS;	//此region可读写
	MPU_Initure.DisableExec=MPU_INSTRUCTION_ACCESS_ENABLE;	//允许读取此区域中的指令
	MPU_Initure.IsShareable=MPU_ACCESS_NOT_SHAREABLE;
	MPU_Initure.IsCacheable=MPU_ACCESS_NOT_CACHEABLE;
	MPU_Initure.IsBufferable=MPU_ACCESS_BUFFERABLE;
	HAL_MPU_ConfigRegion(&MPU_Initure);
	HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);     //开启MPU
}  
  
/**
  * @brief  Initializes the SRAM device.
  * @retval SRAM status
  */
uint8_t BSP_SRAM_Init(void)
{ 
  static uint8_t sram_status = SRAM_ERROR;
  /* SRAM device configuration */
  sramHandle.Instance = FMC_NORSRAM_DEVICE;
  sramHandle.Extended = FMC_NORSRAM_EXTENDED_DEVICE;

	LCD_MPU_Config();                       						//使能MPU保护LCD区域
  /* SRAM device configuration */
  /* Timing configuration derived from system clock (up to 216Mhz)
     for 108Mhz as SRAM clock frequency */
  FSMC_ReadTim.AddressSetupTime      = 5;  //地址建立时间(ADDSET)为15个HCLK 1/200M=5ns*15=75ns
  FSMC_ReadTim.AddressHoldTime       = 5;
  FSMC_ReadTim.DataSetupTime         = 8;
  FSMC_ReadTim.BusTurnAroundDuration = 1;
  FSMC_ReadTim.CLKDivision           = 1;
  FSMC_ReadTim.DataLatency           = 2;
  FSMC_ReadTim.AccessMode            = FMC_ACCESS_MODE_A;
	
	FSMC_WriteTim.AddressSetupTime      = 1;  //地址建立时间(ADDSET)为15个HCLK 1/200M=5ns*15=75ns
  FSMC_WriteTim.AddressHoldTime       = 0;
  FSMC_WriteTim.DataSetupTime         = 1;
  FSMC_WriteTim.AccessMode            = FMC_ACCESS_MODE_A;
	
  
  sramHandle.Init.NSBank             = FMC_NORSRAM_BANK1;
  sramHandle.Init.DataAddressMux     = FMC_DATA_ADDRESS_MUX_DISABLE;
  sramHandle.Init.MemoryType         = FMC_MEMORY_TYPE_SRAM;
  sramHandle.Init.MemoryDataWidth    = FMC_NORSRAM_MEM_BUS_WIDTH_16;
  //sramHandle.Init.BurstAccessMode    = SRAM_BURSTACCESS;
  //sramHandle.Init.WaitSignalPolarity = FMC_WAIT_SIGNAL_POLARITY_LOW;
  //sramHandle.Init.WaitSignalActive   = FMC_WAIT_TIMING_BEFORE_WS;
  sramHandle.Init.WriteOperation     = FMC_WRITE_OPERATION_ENABLE;
  //sramHandle.Init.WaitSignal         = FMC_WAIT_SIGNAL_DISABLE;
  sramHandle.Init.ExtendedMode       = FMC_EXTENDED_MODE_ENABLE;
  //sramHandle.Init.AsynchronousWait   = FMC_ASYNCHRONOUS_WAIT_DISABLE;
  //sramHandle.Init.WriteBurst         = SRAM_WRITEBURST;
  //sramHandle.Init.ContinuousClock    = CONTINUOUSCLOCK_FEATURE;
    
  /* SRAM controller initialization */
  //BSP_SRAM_MspInit(&sramHandle, NULL); /* __weak function can be rewritten by the application */
  if(HAL_SRAM_Init(&sramHandle, &FSMC_ReadTim, &FSMC_WriteTim) != HAL_OK)
  {
    sram_status = SRAM_ERROR;
  }
  else
  {
    sram_status = SRAM_OK;
  }
  return sram_status;
		
}
//void HAL_SRAM_DMA_XferCpltCallback(DMA_HandleTypeDef *hdma)
//{
//	 int16_t i;
//	 
////	printf(" DMA Compled buf=%x\r\n",w_buf[0]); //打印LCD ID
//	//w_buf[0]=0xac;
//	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
//	 //HAL_GPIO_WritePin(GPIOH, GPIO_PIN_6, GPIO_PIN_SET);		//键盘灯
//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
//	//	LCD_ShowxNum(268 ,266, w_buf[0] ,4,0xFF00FF00);
////	if(w_buf[0]==31)
////			BSP_LCD_DisplayStringAtLine(10, (uint8_t*)"  31 ");
////		else
////			BSP_LCD_DisplayStringAtLine(10, (uint8_t*)"  0 ");
////		if(w_buf[0]==31)
////			BSP_LCD_DisplayStringAtLine(10, (uint8_t*)"  31 ");
////		else
////			BSP_LCD_DisplayStringAtLine(10, (uint8_t*)"  0 ");
////	  for(i=0;i<488;i++)
////	{
////	  CH2_BUF[i]=w_buf[i*2+200]/200+233;	
////	  CH3_BUF[i]=233;	
////	}
////	 diswave(CH2_BUF,CH3_BUF);	
//}

/**
  * @brief  DeInitializes the SRAM device.
  * @retval SRAM status
  */
uint8_t BSP_SRAM_DeInit(void)
{ 
  static uint8_t sram_status = SRAM_ERROR;
  /* SRAM device de-initialization */
  sramHandle.Instance = FMC_NORSRAM_DEVICE;
  sramHandle.Extended = FMC_NORSRAM_EXTENDED_DEVICE;

  if(HAL_SRAM_DeInit(&sramHandle) != HAL_OK)
  {
    sram_status = SRAM_ERROR;
  }
  else
  {
    sram_status = SRAM_OK;
  }
  
  /* SRAM controller de-initialization */
  BSP_SRAM_MspDeInit(&sramHandle, NULL);
  
  return sram_status;
}

/**
  * @brief  Reads an amount of data from the SRAM device in polling mode.
  * @param  uwStartAddress: Read start address
  * @param  pData: Pointer to data to be read
  * @param  uwDataSize: Size of read data from the memory   
  * @retval SRAM status
  */
uint8_t BSP_SRAM_ReadData(uint32_t uwStartAddress, uint16_t *pData, uint32_t uwDataSize)
{ 
  if(HAL_SRAM_Read_16b(&sramHandle, (uint32_t *)uwStartAddress, pData, uwDataSize) != HAL_OK)
  {
    return SRAM_ERROR;
  }
  else
  {
    return SRAM_OK;
  }
}

/**
  * @brief  Reads an amount of data from the SRAM device in DMA mode.
  * @param  uwStartAddress: Read start address
  * @param  pData: Pointer to data to be read
  * @param  uwDataSize: Size of read data from the memory   
  * @retval SRAM status
  */
uint8_t BSP_SRAM_ReadData_DMA(uint32_t uwStartAddress, uint16_t *pData, uint32_t uwDataSize)
{
  if(HAL_SRAM_Read_DMA(&sramHandle, (uint32_t *)uwStartAddress, (uint32_t *)pData, uwDataSize) != HAL_OK)
  {
    return SRAM_ERROR;
  }
  else
  {
    return SRAM_OK;
  }
}

/**
  * @brief  Writes an amount of data from the SRAM device in polling mode.
  * @param  uwStartAddress: Write start address
  * @param  pData: Pointer to data to be written
  * @param  uwDataSize: Size of written data from the memory   
  * @retval SRAM status
  */
uint8_t BSP_SRAM_WriteData(uint32_t uwStartAddress, uint16_t *pData, uint32_t uwDataSize) 
{ 
  if(HAL_SRAM_Write_16b(&sramHandle, (uint32_t *)uwStartAddress, pData, uwDataSize) != HAL_OK)
  {
    return SRAM_ERROR;
  }
  else
  {
    return SRAM_OK;
  }
}

/**
  * @brief  Writes an amount of data from the SRAM device in DMA mode.
  * @param  uwStartAddress: Write start address
  * @param  pData: Pointer to data to be written
  * @param  uwDataSize: Size of written data from the memory   
  * @retval SRAM status
  */
uint8_t BSP_SRAM_WriteData_DMA(uint32_t uwStartAddress, uint16_t *pData, uint32_t uwDataSize) 
{
  if(HAL_SRAM_Write_DMA(&sramHandle, (uint32_t *)uwStartAddress, (uint32_t *)pData, uwDataSize) != HAL_OK)
  {
    return SRAM_ERROR;
  }
  else
  {
    return SRAM_OK;
  } 
}

/**
  * @brief  Initializes SRAM MSP.
  * @param  hsram: SRAM handle
  * @retval None
  */
 void HAL_SRAM_MspInit(SRAM_HandleTypeDef  *hsram)
{  
	  static DMA_HandleTypeDef dma_handle;
	    GPIO_InitTypeDef GPIO_Initure;
	
	__HAL_RCC_FMC_CLK_ENABLE();				//使能FMC时钟
	__HAL_RCC_GPIOD_CLK_ENABLE();			//使能GPIOD时钟
	__HAL_RCC_GPIOE_CLK_ENABLE();			//使能GPIOE时钟
	__HAL_RCC_GPIOF_CLK_ENABLE();			//使能GPIOF时钟
	
	//初始化PD0,1,4,5,7,8,9,10,13,14,15
	GPIO_Initure.Pin=GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_7|GPIO_PIN_8|\
                     GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
	GPIO_Initure.Mode=GPIO_MODE_AF_PP; 		//推挽复用
	GPIO_Initure.Pull=GPIO_PULLUP;			//上拉
	GPIO_Initure.Speed=GPIO_SPEED_HIGH;		//高速
	GPIO_Initure.Alternate=GPIO_AF12_FMC;	//复用为FMC
	HAL_GPIO_Init(GPIOD,&GPIO_Initure);     //初始化
	
	//初始化PE7,8,9,10,11,12,13,14,15
	GPIO_Initure.Pin=GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|\
                     GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
	HAL_GPIO_Init(GPIOE,&GPIO_Initure);
	
	GPIO_Initure.Pin=GPIO_PIN_2;
	HAL_GPIO_Init(GPIOF,&GPIO_Initure);
	

//  GPIO_InitTypeDef gpio_init_structure;
//  
//  /* Enable FMC clock */
//  __HAL_RCC_FMC_CLK_ENABLE();
//  
  /* Enable chosen DMAx clock */
  __SRAM_DMAx_CLK_ENABLE();

//  /* Enable GPIOs clock */
//  __HAL_RCC_GPIOD_CLK_ENABLE();
//  __HAL_RCC_GPIOE_CLK_ENABLE();
//  __HAL_RCC_GPIOF_CLK_ENABLE();
//  __HAL_RCC_GPIOG_CLK_ENABLE();
//  
//  /* Common GPIO configuration */
//  gpio_init_structure.Mode      = GPIO_MODE_AF_PP;
//  gpio_init_structure.Pull      = GPIO_PULLUP;
//  gpio_init_structure.Speed     = GPIO_SPEED_HIGH;
//  gpio_init_structure.Alternate = GPIO_AF12_FMC;
//  
//  /* GPIOD configuration */
//  gpio_init_structure.Pin   = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_7 | GPIO_PIN_8     |\
//                              GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 |\
//                              GPIO_PIN_14 | GPIO_PIN_15;
//  HAL_GPIO_Init(GPIOD, &gpio_init_structure);

//  /* GPIOE configuration */  
//  gpio_init_structure.Pin   = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_3| GPIO_PIN_4 | GPIO_PIN_7     |\
//                              GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 |\
//                              GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
//  HAL_GPIO_Init(GPIOE, &gpio_init_structure);
//  
//  /* GPIOF configuration */  
//  gpio_init_structure.Pin   = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2| GPIO_PIN_3 | GPIO_PIN_4     |\
//                              GPIO_PIN_5 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
//  HAL_GPIO_Init(GPIOF, &gpio_init_structure);
//  
//  /* GPIOG configuration */  
//  gpio_init_structure.Pin   = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2| GPIO_PIN_3 | GPIO_PIN_4     |\
//                              GPIO_PIN_5 | GPIO_PIN_10;
//  HAL_GPIO_Init(GPIOG, &gpio_init_structure);  
//  
  /* Configure common DMA parameters */
  dma_handle.Init.Channel             = SRAM_DMAx_CHANNEL;
  dma_handle.Init.Direction           = DMA_MEMORY_TO_MEMORY;
  dma_handle.Init.PeriphInc           = DMA_PINC_ENABLE;
  dma_handle.Init.MemInc              = DMA_MINC_ENABLE;
  dma_handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  dma_handle.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
  dma_handle.Init.Mode                = DMA_NORMAL;
  dma_handle.Init.Priority            = DMA_PRIORITY_HIGH;
  dma_handle.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
  dma_handle.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
  dma_handle.Init.MemBurst            = DMA_MBURST_SINGLE;
  dma_handle.Init.PeriphBurst         = DMA_PBURST_SINGLE;
  
  dma_handle.Instance = SRAM_DMAx_STREAM;
  
   /* Associate the DMA handle */
  __HAL_LINKDMA(hsram, hdma, dma_handle);
  
  /* Deinitialize the Stream for new transfer */
  HAL_DMA_DeInit(&dma_handle);
  
  /* Configure the DMA Stream */
  HAL_DMA_Init(&dma_handle);
	
	  /*##-5- Select Callbacks functions called after Transfer complete and Transfer error */
  HAL_DMA_RegisterCallback(&dma_handle, HAL_DMA_XFER_CPLT_CB_ID, HAL_SRAM_DMA_XferCpltCallback);
  
  /* NVIC configuration for DMA transfer complete interrupt */
  HAL_NVIC_SetPriority(SRAM_DMAx_IRQn, 15, 10);
  HAL_NVIC_EnableIRQ(SRAM_DMAx_IRQn);   
}


/**
  * @brief  DeInitializes SRAM MSP.
  * @param  hsram: SRAM handle
  * @retval None
  */
__weak void BSP_SRAM_MspDeInit(SRAM_HandleTypeDef  *hsram, void *Params)
{  
    static DMA_HandleTypeDef dma_handle;
  
    /* Disable NVIC configuration for DMA interrupt */
    HAL_NVIC_DisableIRQ(SRAM_DMAx_IRQn);

    /* Deinitialize the stream for new transfer */
    dma_handle.Instance = SRAM_DMAx_STREAM;
    HAL_DMA_DeInit(&dma_handle);

    /* GPIO pins clock, FMC clock and DMA clock can be shut down in the applications
       by surcharging this __weak function */ 
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
