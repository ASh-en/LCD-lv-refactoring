#include "stm32f7xx_hal.h"
#include "stm32f7xx_eval.h"			 
#include "uart6.h" 
#include "string.h"	 
#include "math.h"
//uint8_t aRxBuffer;//HAL库使用的串口接收缓冲
	uint8_t res[1];
/* Private functions ---------------------------------------------------------*/
//void RS485_REDE_InitPins(void) 
//	{
//	GPIO_InitTypeDef GPIO_InitStruct;
//	
//	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOGEN;

//	//Common settings
//	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
//	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
//	
//	//REDE pin
////	GPIO_InitStruct.Pin = RS485_REDE_PIN;
////	HAL_GPIO_Init(RS485_REDE_PORT, &GPIO_InitStruct);
//	
////	RS485_REDE_LOW;
//}

void u6_printf(char* fmt,...)  
{  
	uint16_t i,j;
	va_list ap;
	va_start(ap,fmt);
	vsprintf((char*)USART6_TX_BUF,fmt,ap);
	va_end(ap);
	i=strlen((const char*)USART6_TX_BUF);//此次发送数据的长度
	//RS485_REDE_HIGH;
	for(j=0;j<i;j++)//循环发送数据
	{
		while((USART6->ISR&0X40)==0);//循环发送,直到发送完毕   
		USART6->TDR=USART6_TX_BUF[j];  
	}
//	RS485_REDE_LOW;
}
////////////////////////////////////////////////////
//通过判断接收连续2个字符之间的时间差不大于10ms来决定是不是一次连续的数据.
//如果2个字符接收间隔超过10ms,则认为不是1次连续数据.也就是超过10ms没有接收到
//任何数据,则表示此次接收完毕.
//接收到的数据状态
//[15]:0,没有接收到数据;1,接收到了一批数据.
//[14:0]:接收到的数据长度
//__IO uint16_t  USART6_RX_STA=0;   	 
//void USART6_IRQHandler(void)
//{
//	uint8_t res;

//	if((__HAL_UART_GET_FLAG(&UartHandle,UART_FLAG_RXNE)!=RESET))  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
//	{
//       HAL_UART_Receive(&UartHandle,&res,1,1000); 
//		if((USART6_RX_STA&(1<<15))==0)//接收完的一批数据,还没有被处理,则不再接收其他数据
//		{ 
//			if(USART6_RX_STA<USART6_MAX_RECV_LEN)	//还可以接收数据
//			 {
//         // __HAL_TIM_SET_COUNTER(&TIM7_Handler,0);	//计数器清空			
////        if(USART6_RX_STA==0) 				//使能定时器7的中断 
////			   {
////              __HAL_TIM_ENABLE(&TIM7_Handler); //使能定时器7
////				  }
//				 USART6_RX_BUF[USART6_RX_STA++]=res;	//记录接收到的值	 
//				 if(USART6_RX_STA==USART6_MAX_RECV_LEN)	
//						USART6_RX_STA|=1<<15;				//强制标记接收完成 
//			 }
//			else 
//			{
//				USART6_RX_STA|=1<<15;				//强制标记接收完成
//			} 
//		}
//		 //__HAL_UART_SEND_REQ(&UartHandle, UART_RXDATA_FLUSH_REQUEST);
//		// else
//		//	 u6_printf("RS485 RX OK\r\n");	//查询角色 
//    } 
//} 
////////////////////////////////////////////////////////////////////////////
//void USART6_IRQHandler(void)
//{
//	uint8_t res;

//	if((__HAL_UART_GET_FLAG(&UartHandle,UART_FLAG_RXNE)!=RESET))  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
//	{
//     HAL_UART_Receive(&UartHandle,&res,1,1000); 
//    if((USART6_RX_STA&0x8000)==0)//接收未完成
//		{
//			if(USART6_RX_STA&0x4000)//接收到了0x0d
//			{
//				if(res!=0x0a)USART6_RX_STA=0;//接收错误,重新开始
//				else USART6_RX_STA|=0x8000;	//接收完成了 
//			}
//			else //还没收到0X0D
//			{	
//				if(res==0x0d)USART6_RX_STA|=0x4000;
//				else
//				{
//					USART6_RX_BUF[USART6_RX_STA&0X3FFF]=res ;
//					USART6_RX_STA++;
//					if(USART6_RX_STA>(USART6_MAX_RECV_LEN-1))
//						USART6_RX_STA=0;//接收数据错误,重新开始接收	  
//				}		 
//			}
//		 }
//		else
//			USART6_RX_STA|=1<<15;				//强制标记接收完成
//  } 

//}
void USART6_IRQHandler(void)   
{
	 HAL_UART_IRQHandler(&UartHandle);
	 HAL_UART_Receive_IT(&UartHandle, (uint8_t *)res, 1);
	// while(HAL_UART_Transmit_IT(&UartHandle, (uint8_t*)aTxStartMessage, TXSTARTMESSAGESIZE)!= HAL_OK);
}
///////////////////////////////////////////


/***************************************************************************************/ 
//////////////////////////////////////////////////////////
//初始化Uart6_Init模块
//返回值:0,成功;1,失败.
void Uart6_Init(void)
{
	
  /*##-1- Configure the UART peripheral ######################################*/
  /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
  /* UART configured as follows:
      - Word Length = 8 Bits
      - Stop Bit    = One Stop bit
      - Parity      = NONE parity
      - BaudRate    = 115200 baud
      - Hardware flow control disabled (RTS and CTS signals) */
  UartHandle.Instance        = USARTx;

  UartHandle.Init.BaudRate   = 9600;//9600;//115200;
  UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
  UartHandle.Init.StopBits   = UART_STOPBITS_1;
  UartHandle.Init.Parity     = UART_PARITY_NONE;
  UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  UartHandle.Init.Mode       = UART_MODE_TX_RX;


  if(HAL_UART_Init(&UartHandle) != HAL_OK)
  {
    /* In case of Initialization Error */
    while(1)
    {
    }
  }
	 __HAL_UART_ENABLE_IT(&UartHandle,UART_IT_RXNE);  //开启接收中断
	HAL_NVIC_EnableIRQ(USART6_IRQn);		//使能USART6中断通道
	HAL_NVIC_SetPriority(USART6_IRQn,1,1);	//抢占优先级3，子优先级3
  HAL_UART_Receive_IT(&UartHandle, (uint8_t *)res, 1);		
	//TIM7_Init(500-1,10800-1);	        //50ms中断一次
  // __HAL_TIM_DISABLE(&TIM7_Handler);   //关闭定时器7 
	
	//USART6_RX_STA=0;  
}
	
/////////////////////////////////////////////////////////////////
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
    // HAL_UART_Receive(&UartHandle,&res,1,1000); 
    if((USART6_RX_STA&0x8000)==0)//接收未完成
		{
			if(USART6_RX_STA&0x4000)//接收到了0x0d
			{
				if(res[0]!=0x0a)
					 USART6_RX_STA=0;//接收错误,重新开始
				else 
					 USART6_RX_STA|=0x8000;	//接收完成了 
			}
			else //还没收到0X0D
			{	
				if(res[0]==0x0d)
					USART6_RX_STA|=0x4000;
				else
				{
					USART6_RX_BUF[USART6_RX_STA&0X3FFF]=res[0] ;
					USART6_RX_STA++;
					if(USART6_RX_STA>(USART6_MAX_RECV_LEN-1))
						USART6_RX_STA=0;//接收数据错误,重新开始接收	  
				}		 
			}
		 }
	//	 HAL_UART_Receive_IT(UartHandle, (uint8_t *)res, 1);

  
}