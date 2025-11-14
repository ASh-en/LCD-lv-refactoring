/*
**************************************************************************************************
* @file    		w5500_conf.c
* @author  		WIZnet Software Team 
* @version 		V1.0
* @date    		2015-02-14
* @brief  		配置MCU，移植W5500程序需要修改的文件，配置W5500的MAC和IP地址
**************************************************************************************************
*/
#include <stdio.h> 
#include <string.h>

#include "w5500_conf.h"
//#include "bsp_i2c_ee.h"
#include "utility.h"
#include "w5500.h"
#include "dhcp.h"
//#include "bsp_TiMbase.h"

CONFIG_MSG  ConfigMsg;																	/*配置结构体*/
EEPROM_MSG_STR EEPROM_MSG;															/*EEPROM存储信息结构体*/
SPI_HandleTypeDef SPI2_Handler;  //SPI2句柄	
static DMA_HandleTypeDef hdmat;
static DMA_HandleTypeDef hdmar;
/*定义MAC地址,如果多块W5500网络适配板在同一现场工作，请使用不同的MAC地址*/
uint8 mac[6]={0x00,0x08,0xdc,0x11,0x11,0x11};

/*定义默认IP信息*/
uint8 local_ip[4]  ={192,168,1,88};											/*定义W5500默认IP地址*/
uint8 subnet[4]    ={255,255,255,0};										/*定义W5500默认子网掩码*/
uint8 gateway[4]   ={192,168,1,1};											/*定义W5500默认网关*/
uint8 dns_server[4]={114,114,114,114};									/*定义W5500默认DNS*/

uint16 local_port=5000;	                       					/*定义本地端口*/

/*定义远端IP信息*/
uint8  remote_ip[4]={192,168,1,102};											/*远端IP地址*/
uint16 remote_port=5000;																/*远端端口号*/

/*IP配置方法选择，请自行选择*/
uint8	ip_from=IP_FROM_DEFINE;				

uint8   dhcp_ok   = 0;													   			/*dhcp成功获取IP*/
uint32	ms        = 0;															  	/*毫秒计数*/
uint32	dhcp_time = 0;															  	/*DHCP运行计数*/
vu8	    ntptimer  = 0;															  	/*NPT秒计数*/
//选择SPI口

/**
*@brief		配置W5500的IP地址
*@param		无
*@return	无
*/
uint8_t SPI_SendByte(uint8_t byte);
void set_w5500_ip(void)
{	
		
   /*复制定义的配置信息到配置结构体*/
	memcpy(ConfigMsg.mac, mac, 6);
	memcpy(ConfigMsg.lip,local_ip,4);
	memcpy(ConfigMsg.sub,subnet,4);
	memcpy(ConfigMsg.gw,gateway,4);
	memcpy(ConfigMsg.dns,dns_server,4);
	if(ip_from==IP_FROM_DEFINE)	
	//	printf(" 使用定义的IP信息配置W5500\r\n");
	
/*以下配置信息，根据需要选用*/	
	ConfigMsg.sw_ver[0]=FW_VER_HIGH;
	ConfigMsg.sw_ver[1]=FW_VER_LOW;	

	/*将IP配置信息写入W5500相应寄存器*/	
	setSUBR(ConfigMsg.sub);
	setGAR(ConfigMsg.gw);
	setSIPR(ConfigMsg.lip);
	
	getSIPR (local_ip);			
//	printf(" W5500 IP地址   : %d.%d.%d.%d\r\n", local_ip[0],local_ip[1],local_ip[2],local_ip[3]);
	getSUBR(subnet);
	//printf(" W5500 子网掩码 : %d.%d.%d.%d\r\n", subnet[0],subnet[1],subnet[2],subnet[3]);
	getGAR(gateway);
	//printf(" W5500 网关     : %d.%d.%d.%d\r\n", gateway[0],gateway[1],gateway[2],gateway[3]);
}

/**
*@brief		配置W5500的MAC地址
*@param		无
*@return	无
*/
void set_w5500_mac(void)
{
	memcpy(ConfigMsg.mac, mac, 6);
	setSHAR(ConfigMsg.mac);	/**/

}

/**
*@brief		配置W5500的GPIO接口
*@param		无
*@return	无
*/
void gpio_for_w5500_config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	//SPI引脚的定义已经在M8266的初始化中完成	
  //定义RESET引脚
  GPIO_InitStructure.Pin = WIZ_RESET;					       /*选择要控制的GPIO引脚*/		 
  GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;		     /*设置引脚速率为50MHz */		
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;		     /*设置引脚模式为通用推挽输出*/	
  HAL_GPIO_Init(WIZ_SPIx_RESET_PORT, &GPIO_InitStructure);		 /*调用库函数，初始化GPIO*/
	HAL_GPIO_WritePin(WIZ_SPIx_RESET_PORT, WIZ_RESET, GPIO_PIN_SET);	
  //定义INT引脚	
  GPIO_InitStructure.Pin = WIZ_INT;						       /*选择要控制的GPIO引脚*/		 
  GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;		     /*设置引脚速率为50MHz*/		
  GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING ;				     /*设置引脚模式为通用推挽模拟上拉输入*/		
	GPIO_InitStructure.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(WIZ_SPIx_INT_PORT, &GPIO_InitStructure);			 /*调用库函数，初始化GPIO*/
  //定义cs引脚
  GPIO_InitStructure.Pin = WIZ_SPIx_SCS;					       /*选择要控制的GPIO引脚*/		 
  GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;		     /*设置引脚速率为50MHz */		
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;		     /*设置引脚模式为通用推挽输出*/	
  HAL_GPIO_Init(WIZ_SPIx_SCS_PORT, &GPIO_InitStructure);		 /*调用库函数，初始化GPIO*/
	HAL_GPIO_WritePin(WIZ_SPIx_SCS_PORT, WIZ_SPIx_SCS, GPIO_PIN_SET);	
	SPI2_Init();
}
void SPI2_Init(void)
{

    SPI2_Handler.Instance=SPI2;                      //SP2
    SPI2_Handler.Init.Mode=SPI_MODE_MASTER;          //设置SPI工作模式，设置为主模式
    SPI2_Handler.Init.Direction=SPI_DIRECTION_2LINES;//设置SPI单向或者双向的数据模式:SPI设置为双线模式
    SPI2_Handler.Init.DataSize=SPI_DATASIZE_8BIT;    //设置SPI的数据大小:SPI发送接收8位帧结构
    SPI2_Handler.Init.CLKPolarity=SPI_POLARITY_HIGH; //串行同步时钟的空闲状态为高电平
    SPI2_Handler.Init.CLKPhase=SPI_PHASE_2EDGE;      //串行同步时钟的第二个跳变沿（上升或下降）数据被采样
    SPI2_Handler.Init.NSS=SPI_NSS_SOFT;              //NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理:内部NSS信号有SSI位控制
    SPI2_Handler.Init.BaudRatePrescaler=SPI_BAUDRATEPRESCALER_2;//SPI_BAUDRATEPRESCALER_256;//定义波特率预分频的值:波特率预分频值为256
    SPI2_Handler.Init.FirstBit=SPI_FIRSTBIT_MSB;     //指定数据传输从MSB位还是LSB位开始:数据传输从MSB位开始
    SPI2_Handler.Init.TIMode=SPI_TIMODE_DISABLE;     //关闭TI模式
    SPI2_Handler.Init.CRCCalculation=SPI_CRCCALCULATION_DISABLE;//关闭硬件CRC校验
    SPI2_Handler.Init.CRCPolynomial=7;               //CRC值计算的多项式
	  if(HAL_SPI_Init(&SPI2_Handler) != HAL_OK)
  {
    /* Initialization Error */
   
  }
 
    
    __HAL_SPI_ENABLE(&SPI2_Handler);                 //使能SPI2
   // SPI_SendByte(0Xff);                        //启动传输
}
//SPI2底层驱动，时钟使能，引脚配置
//此函数会被HAL_SPI_Init()调用
//hspi:SPI句柄
void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
    GPIO_InitTypeDef GPIO_Initure;
	
     __HAL_RCC_GPIOA_CLK_ENABLE();                   //使能GPIOA时钟   
    __HAL_RCC_GPIOC_CLK_ENABLE();                   //使能GPIOB时钟
    __HAL_RCC_SPI2_CLK_ENABLE();                    //使能SPI2时钟
     __HAL_RCC_DMA1_CLK_ENABLE();
	//PC1,2  pc1:mosi   pc2:miso
    GPIO_Initure.Pin=GPIO_PIN_1|GPIO_PIN_2;
    GPIO_Initure.Mode=GPIO_MODE_AF_PP;              //复用推挽输出
    GPIO_Initure.Pull=GPIO_PULLUP;                  //上拉
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;             //快速    
    GPIO_Initure.Alternate=GPIO_AF5_SPI2;           //复用为SPI2
    HAL_GPIO_Init(GPIOC,&GPIO_Initure);             //初始化
	  GPIO_Initure.Pin=GPIO_PIN_9;//pa9  LCK
	    HAL_GPIO_Init(GPIOA,&GPIO_Initure);             //初始化

	
  /*##-4- Configure the DMA channel ###########################################*/
  /* QSPI DMA channel configuration */
	/*
  hdmat.Init.Channel             = SPI_TX_DMA_CHANNEL;                     
  hdmat.Init.PeriphInc           = DMA_PINC_DISABLE;
  hdmat.Init.MemInc              = DMA_MINC_ENABLE;
  hdmat.Init.Direction           = DMA_MEMORY_TO_PERIPH;	
  hdmat.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdmat.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
  hdmat.Init.Mode                = DMA_NORMAL;
  hdmat.Init.Priority            = DMA_PRIORITY_LOW;
  hdmat.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;       
  hdmat.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
  hdmat.Init.MemBurst            = DMA_MBURST_SINGLE;//DMA_MBURST_INC4;//DMA_MBURST_SINGLE;           
  hdmat.Init.PeriphBurst         = DMA_PBURST_SINGLE;//DMA_PBURST_INC4;//DMA_PBURST_SINGLE;           
  hdmat.Instance                 = SPI_TX_DMA_INSTANCE;
	HAL_DMA_DeInit(&hdmat);
  HAL_DMA_Init(&hdmat);
  __HAL_LINKDMA(hspi, hdmatx, hdmat);
	
    hdmar.Instance                 = SPI_RX_DMA_INSTANCE;
    hdmar.Init.Channel             = SPI_RX_DMA_CHANNEL;
    hdmar.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    hdmar.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    hdmar.Init.MemBurst            = DMA_MBURST_SINGLE;
    hdmar.Init.PeriphBurst         = DMA_PBURST_SINGLE;
    hdmar.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdmar.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdmar.Init.MemInc              = DMA_MINC_ENABLE;
    hdmar.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdmar.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdmar.Init.Mode                = DMA_NORMAL;
    hdmar.Init.Priority            = DMA_PRIORITY_LOW;
	HAL_DMA_DeInit(&hdmar);
    HAL_DMA_Init(&hdmar);

    __HAL_LINKDMA(hspi, hdmarx, hdmar);



  HAL_NVIC_SetPriority(SPI_RX_DMA_IRQ, 0x01, 6);
  HAL_NVIC_EnableIRQ(SPI_RX_DMA_IRQ);
	  HAL_NVIC_SetPriority(SPI_TX_DMA_IRQ, 0x01, 5);
  HAL_NVIC_EnableIRQ(SPI_TX_DMA_IRQ);	*/
	  HAL_NVIC_SetPriority(SPI2_IRQn, 0x01, 7);
  HAL_NVIC_EnableIRQ(SPI2_IRQn);	

}
/**
*@brief		W5500片选信号设置函数
*@param		val: 为“0”表示片选端口为低，为“1”表示片选端口为高
*@return	无
*/
void wiz_cs(uint8_t val)
{
	if (val == LOW) 
	{
	//  GPIO_ResetBits(WIZ_SPIx_SCS_PORT, WIZ_SPIx_SCS); 
		HAL_GPIO_WritePin(WIZ_SPIx_SCS_PORT, WIZ_SPIx_SCS, GPIO_PIN_RESET);
	}
	else if (val == HIGH)
	{
	//  GPIO_SetBits(WIZ_SPIx_SCS_PORT, WIZ_SPIx_SCS); 
		HAL_GPIO_WritePin(WIZ_SPIx_SCS_PORT, WIZ_SPIx_SCS, GPIO_PIN_SET);
	}
}

/**
*@brief		设置W5500的片选端口SCSn为低
*@param		无
*@return	无
*/
void iinchip_csoff(void)
{
	wiz_cs(LOW);
}

/**
*@brief		设置W5500的片选端口SCSn为高
*@param		无
*@return	无
*/
void iinchip_cson(void)
{	
   wiz_cs(HIGH);
}

/**
*@brief		W5500复位设置函数
*@param		无
*@return	无
*/
void reset_w5500(void)
{
	HAL_GPIO_WritePin(WIZ_SPIx_RESET_PORT, WIZ_RESET, GPIO_PIN_RESET);
	delay_us(2000);  
	HAL_GPIO_WritePin(WIZ_SPIx_RESET_PORT, WIZ_RESET, GPIO_PIN_SET);
	delay_ms(1600);
}

uint8_t SPI_SendByte(uint8_t byte)
{
	
	uint8_t rx_buf;
//	HAL_SPI_TransmitReceive_DMA(&SPI2_Handler,&byte,&rx_buf,1);
	HAL_SPI_TransmitReceive(&SPI2_Handler,&byte,&rx_buf,1, 1000);    
  return rx_buf;
	
}
uint8_t SPI_SendData(uint8_t *byte,uint16_t len)
{
	

//	HAL_SPI_TransmitReceive_DMA(&SPI2_Handler,&byte,&rx_buf,1);
	HAL_SPI_Transmit(&SPI2_Handler,byte,len, 1000);    	
}
uint8_t SPI_ReceiveData(uint8_t *byte,uint16_t len)
{
	

//	HAL_SPI_TransmitReceive_DMA(&SPI2_Handler,&byte,&rx_buf,1);
	HAL_SPI_Receive(&SPI2_Handler,byte,len, 1000);    	
}
/**
*@brief		STM32 SPI1读写8位数据
*@param		dat：写入的8位数据
*@return	无
*/
uint8  IINCHIP_SpiSendData(uint8 dat)
{
   return(SPI_SendByte(dat));
}

/**
*@brief		写入一个8位数据到W5500
*@param		addrbsb: 写入数据的地址
*@param   data：写入的8位数据
*@return	无
*/
void IINCHIP_WRITE( uint32 addrbsb,  uint8 data)
{
   iinchip_csoff();                              		
   IINCHIP_SpiSendData( (addrbsb & 0x00FF0000)>>16);	
   IINCHIP_SpiSendData( (addrbsb & 0x0000FF00)>> 8);
   IINCHIP_SpiSendData( (addrbsb & 0x000000F8) + 4);  
   IINCHIP_SpiSendData(data);                   
   iinchip_cson();                            
}

/**
*@brief		从W5500读出一个8位数据
*@param		addrbsb: 写入数据的地址
*@param   data：从写入的地址处读取到的8位数据
*@return	无
*/
uint8 IINCHIP_READ(uint32 addrbsb)
{
   uint8 data = 0;
   iinchip_csoff();                            
   IINCHIP_SpiSendData( (addrbsb & 0x00FF0000)>>16);
   IINCHIP_SpiSendData( (addrbsb & 0x0000FF00)>> 8);
   IINCHIP_SpiSendData( (addrbsb & 0x000000F8))    ;
   data = IINCHIP_SpiSendData(0x00);            
   iinchip_cson();                               
   return data;    
}

/**
*@brief		向W5500写入len字节数据
*@param		addrbsb: 写入数据的地址
*@param   buf：写入字符串
*@param   len：字符串长度
*@return	len：返回字符串长度
*/
uint16 wiz_write_buf(uint32 addrbsb,uint8* buf,uint16 len)
{
   uint16 idx = 0;
   //if(len == 0)// printf("Unexpected2 length 0\r\n");
   iinchip_csoff();                               
   IINCHIP_SpiSendData( (addrbsb & 0x00FF0000)>>16);
   IINCHIP_SpiSendData( (addrbsb & 0x0000FF00)>> 8);
   IINCHIP_SpiSendData( (addrbsb & 0x000000F8) + 4); 
	 /*
   for(idx = 0; idx < len; idx++)
   {
     IINCHIP_SpiSendData(buf[idx]);
   }
	 */
	 SPI_SendData(buf,len);
   iinchip_cson();                           
   return len;  
}

/**
*@brief		从W5500读出len字节数据
*@param		addrbsb: 读取数据的地址
*@param 	buf：存放读取数据
*@param		len：字符串长度
*@return	len：返回字符串长度
*/
uint16 wiz_read_buf(uint32 addrbsb, uint8* buf,uint16 len)
{
  uint16 idx = 0;
//  if(len == 0)
//  {
//   // printf("Unexpected2 length 0\r\n");
//  }
  iinchip_csoff();                                
  IINCHIP_SpiSendData( (addrbsb & 0x00FF0000)>>16);
  IINCHIP_SpiSendData( (addrbsb & 0x0000FF00)>> 8);
  IINCHIP_SpiSendData( (addrbsb & 0x000000F8));    
//  for(idx = 0; idx < len; idx++)                   
//  {
//    buf[idx] = IINCHIP_SpiSendData(0x00);
//  }
	SPI_ReceiveData(buf,len);
  iinchip_cson();                                  
  return len;
}





