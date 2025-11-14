#include "stm32f7xx_hal.h"
#include "stm32f7xx_eval.h"			 
#include "hc05.h" 
#include "uart6.h" 
#include "string.h"	 
#include "math.h"
//uint8_t aRxBuffer;//HAL库使用的串口接收缓冲
/* Private functions ---------------------------------------------------------*/


/***************************************************************************************/
//////////////////////////////////////////
void HC05_Role_Show(void)
{
	if(HC05_Get_Role()==1)
		//LCD_ShowString(30,140,200,16,16,"ROLE:Master");	//主机
	  BSP_LED_On(LED2);
	else 
	//	LCD_ShowString(30,140,200,16,16,"ROLE:Slave ");			 		//从机
	 BSP_LED_Off(LED2);
}
/***************************************************************************************/
//显示ATK-HC05模块的连接状态
void HC05_Sta_Show(void)
{												 
	if(HC05_LED)
	//	LCD_ShowString(120,140,120,16,16,"STA:Connected ");			//连接成功
	BSP_LED_On(LED2);
	else 
		BSP_LED_Off(LED2);
		//LCD_ShowString(120,140,120,16,16,"STA:Disconnect");	 			//未连接				 
}
/***************************************************************************************/ 
//////////////////////////////////////////////////////////
//初始化ATK-HC05模块
//返回值:0,成功;1,失败.
uint8_t HC05_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure; 
	uint8_t retry=4,t;	  		 
	uint8_t temp=1;
	
    __HAL_RCC_GPIOG_CLK_ENABLE();           //开启GPIOI时钟    
    GPIO_InitStructure.Pin=GPIO_PIN_6;            //PG6
    GPIO_InitStructure.Mode=GPIO_MODE_INPUT;      //输入
    GPIO_InitStructure.Pull=GPIO_PULLUP;          //上拉
    GPIO_InitStructure.Speed=GPIO_SPEED_HIGH;     //高速
    HAL_GPIO_Init(GPIOG,&GPIO_InitStructure);
    
    GPIO_InitStructure.Pin=GPIO_PIN_9;           //PG9
    GPIO_InitStructure.Mode=GPIO_MODE_OUTPUT_PP;  //推挽输出
    GPIO_InitStructure.Pull=GPIO_PULLUP;          //上拉
    GPIO_InitStructure.Speed=GPIO_SPEED_HIGH;     //高速
    HAL_GPIO_Init(GPIOG,&GPIO_InitStructure);
    HAL_GPIO_WritePin(GPIOG,GPIO_PIN_9,GPIO_PIN_SET);	//PG9置0 
  /*##-1- Configure the UART peripheral ######################################*/
    Uart6_Init();		
	//TIM7_Init(500-1,10800-1);	        //50ms中断一次
  // __HAL_TIM_DISABLE(&TIM7_Handler);   //关闭定时器7 
	BSP_LED_On(LED1);
	USART6_RX_STA=0;  
	//	HAL_Delay(100);
	while(retry--)
	{
		HC05_KEY(1);					//KEY置高,进入AT模式
		//delay_ms(10);
		HAL_Delay(10);
		u6_printf("AT\r\n");			//发送AT测试指令
		HAL_Delay(1);
		HC05_KEY(0);					//KEY拉低,退出AT模式
		for(t=0;t<10;t++) 				//最长等待50ms,来接收HC05模块的回应
		{
			if(USART6_RX_STA&0X8000)break;
			//delay_ms(5);
			HAL_Delay(5);
		}		
		if(USART6_RX_STA&0X8000)	//接收到一次数据了
		{
			temp=USART6_RX_STA&0X7FFF;	//得到数据长度
			USART6_RX_STA=0;			 
			if(temp==2&&USART6_RX_BUF[0]=='O'&&USART6_RX_BUF[1]=='K')
			{
				temp=0;//接收到OK响应
				break;
			}
		}			    		
	}		    
	if(retry==0)temp=1;	//检测失败
	return temp;	 
}	
/***************************************************************************************/
////////////////////////////////////////////////////////
//获取ATK-HC05模块的角色
//返回值:0,从机;1,主机;0XFF,获取失败.							  
uint8_t HC05_Get_Role(void)
{	 		    
	uint8_t retry=0X0F;
	uint8_t temp,t;
	while(retry--)
	{
		HC05_KEY(1);				//KEY置高,进入AT模式
		//delay_ms(10);
		HAL_Delay(5);
		u6_printf("AT+ROLE?\r\n");	//查询角色
		for(t=0;t<20;t++) 			//最长等待200ms,来接收HC05模块的回应
		{
			//delay_ms(10);
			HAL_Delay(1);
			if(USART6_RX_STA&0X8000)break;
		}		
		HC05_KEY(0);				//KEY拉低,退出AT模式
		HAL_Delay(5);
		if(USART6_RX_STA&0X8000)	//接收到一次数据了
		{
			temp=USART6_RX_STA&0X7FFF;	//得到数据长度
			USART6_RX_STA=0;			 
			if(USART6_RX_BUF[0]=='+')//接收到正确的应答了
			{
				temp=USART6_RX_BUF[6]-'0';//得到主从模式值
				break;
			}
		}		
	}
	if(retry==0)temp=0XFF;//查询失败.
	return temp;
} 			
///***************************************************************************************/				   
//ATK-HC05设置命令
//此函数用于设置ATK-HC05,适用于仅返回OK应答的AT指令
//atstr:AT指令串.比如:"AT+RESET"/"AT+UART=9600,0,0"/"AT+ROLE=0"等字符串
//返回值:0,设置成功;其他,设置失败.							  
uint8_t HC05_Set_Cmd(uint8_t* atstr)
{	 		    
	uint8_t retry=0X0F;
	uint8_t temp,t;
	while(retry--)
	{
		HC05_KEY(1);				//KEY置高,进入AT模式
		//delay_ms(10);
		HAL_Delay(5);
		u6_printf("%s\r\n",atstr);	//发送AT字符串
		HC05_KEY(0);				//KEY拉低,退出AT模式
		for(t=0;t<100;t++) 			//最长等待100ms,来接收HC05模块的回应
		{
			if(USART6_RX_STA&0X8000)break;
			//delay_ms(5);
			HAL_Delay(5);
		}		
		if(USART6_RX_STA&0X8000)	//接收到一次数据了
		{
			temp=USART6_RX_STA&0X7FFF;	//得到数据长度
			USART6_RX_STA=0;			 
			if(temp==4&&USART6_RX_BUF[0]=='O')//接收到正确的应答了
			{			
				temp=0;
				break;			 
			}
		}		
	}
	if(retry==0)temp=0XFF;//设置失败.
	return temp;
} 
/***************************************************************************************/				   
//ATK-HC05设置命令 修改名称 查询名称
//此函数用于设置ATK-HC05,适用于仅返回OK应答的AT指令
//atstr:AT指令串.比如:"AT+NAME=beijing"/"AT+NAME?="等字符串
//返回值:0,设置成功;其他,设置失败.							  
uint8_t HC05_SetName_Cmd(uint8_t* atstr)
{	 		    
	uint8_t retry=0X0F;
	uint8_t temp,t;
	USART6_RX_STA=0;  
	while(retry--)
	{
		HC05_KEY(1);				//KEY置高,进入AT模式
		//delay_ms(10);
		HAL_Delay(5);
		u6_printf("%s\r\n",atstr);	//发送AT字符串
	//	HC05_KEY(0);				//KEY拉低,退出AT模式
		for(t=0;t<100;t++) 			//最长等待100ms,来接收HC05模块的回应
		{
			if(USART6_RX_STA&0X8000)break;
			//delay_ms(5);
			HAL_Delay(5);
		}		
		if(USART6_RX_STA&0X8000)	//接收到一次数据了
		{
			temp=USART6_RX_STA&0X7FFF;	//得到数据长度
			USART6_RX_STA=0;			 
			if(USART6_RX_BUF[0]=='+')//接收到正确的应答了
			{			
				temp=0;
				break;			 
			}
			break;
		}		
	}
	HC05_KEY(0);	
	if(retry==0)temp=0XFF;//设置失败.
	
	return temp;
} 











