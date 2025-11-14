#ifndef __QSPI_H
#define __QSPI_H
//#include "sys.h"
#include <stdint.h>
#include "stm32f7xx_hal_qspi.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F7开发板
//QSPI驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2015/11/30
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	
extern QSPI_HandleTypeDef QSPI_Handler;    //QSPI句柄

uint8_t QSPI_Init(void);												//初始化QSPI
void QSPI_Send_CMD(uint32_t instruction,uint32_t address,uint32_t dummyCycles,uint32_t instructionMode,uint32_t addressMode,uint32_t addressSize,uint32_t dataMode);			//QSPI发送命令
uint8_t QSPI_Receive(uint8_t* buf,uint32_t datalen);							//QSPI接收数据
uint8_t QSPI_Transmit(uint8_t* buf,uint32_t datalen);							//QSPI发送数据
#endif
