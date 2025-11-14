#ifndef __uart6_H
#define __uart6_H	 
 
/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"
#include "stm32f7xx_eval.h"
#include "stdarg.h"
#include "string.h"
#include "stm32f7xx_hal_tim.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* User can use this section to tailor USARTx/UARTx instance used and associated
   resources */
	 
/* Definition for USARTx clock resources */
//#define RS485_REDE_RCC			RCC_AHB1ENR_GPIOGEN
//#define RS485_REDE_PORT			GPIOG
//#define RS485_REDE_PIN			GPIO_PIN_12

//#define RS485_REDE_LOW			RS485_REDE_PORT->BSRR = ((uint32_t)RS485_REDE_PIN << 16);
//#define RS485_REDE_HIGH			RS485_REDE_PORT->BSRR = RS485_REDE_PIN

#define USARTx                           USART6
#define USARTx_CLK_ENABLE()              __HAL_RCC_USART6_CLK_ENABLE()
#define USARTx_RX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOC_CLK_ENABLE()
#define USARTx_TX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOC_CLK_ENABLE()

#define USARTx_FORCE_RESET()             __HAL_RCC_USART6_FORCE_RESET()
#define USARTx_RELEASE_RESET()           __HAL_RCC_USART6_RELEASE_RESET()

/* Definition for USARTx Pins */
#define USARTx_TX_PIN                    GPIO_PIN_6
#define USARTx_TX_GPIO_PORT              GPIOC
#define USARTx_TX_AF                     GPIO_AF8_USART6
#define USARTx_RX_PIN                    GPIO_PIN_7
#define USARTx_RX_GPIO_PORT              GPIOC
#define USARTx_RX_AF                     GPIO_AF8_USART6

/* Definition for USARTx's NVIC IRQ and IRQ Handlers */
#define USARTx_IRQn                      USART6_IRQn
#define USARTx_IRQHandler                USART6_IRQHandler

/* Size of Transmission buffer */
#define TXSTARTMESSAGESIZE                   (COUNTOF(aTxStartMessage) - 1)
#define TXENDMESSAGESIZE                     (COUNTOF(aTxEndMessage) - 1)

/* Size of Reception buffer */
#define RXBUFFERSIZE                      1
#define USART6_MAX_SEND_LEN               200
#define USART6_MAX_RECV_LEN               30
////////////////////////////////////////////////////////////////////////////
 
extern uint8_t  USART6_RX_BUF[USART6_MAX_RECV_LEN]; //接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 
extern __IO uint16_t USART6_RX_STA;         		//接收状态标记	
extern UART_HandleTypeDef UartHandle; //UART句柄
extern __align(8) uint8_t USART6_TX_BUF[USART6_MAX_SEND_LEN]; 	//发送缓冲,最大USART3_MAX_SEND_LEN字节


void Uart6_Init(void);
 
void u6_printf(char* fmt,...) ;
#endif  
















