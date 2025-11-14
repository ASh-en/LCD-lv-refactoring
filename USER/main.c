/**
  ******************************************************************************
  * @file    Display/LTDC_PicturesFromSDCard/Src/main.c
  * @author  MCD Application Team
  * @version V1.0.4
  * @date    22-April-2016
  * @brief   This file provides main program functions
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics International N.V. 
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "math.h"
#include "main.h"
#include "stm32f7xx_eval_sd.h"
#include "stm32f7xx_eval_sram.h"
#include "stm32f7xx_eval_lcd.h"
#include "w25qxx.h"
#include "sys.h"
#include "delay.h"
#include "M8266HostIf.h"
#include "led.h"
#include "M8266WIFIDrv.h"
#include "M8266WIFI_ops.h"
#include "brd_cfg.h"
#include "test_m8266wifi.h"
#include "usart.h"
#include "stm32f7xx_hal_uart.h"
#include "gpio.h"
#include "lcd_init.h"
#include "lcd.h"
//#include "../../../Utilities/Fonts/fonts.h"
/** @addtogroup STM32F7xx_HAL_Applications
  * @{
  */

/** @addtogroup LTDC_PicturesFromSDCard
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
//////////////////////////////////////////
/* ADC handler declaration */
ADC_HandleTypeDef    AdcHandle;

/* Variable used to get converted value */
__IO uint16_t uhADCxConvertedValue = 0;
FATFS SD_FatFs;  /* File system object for SD card logical drive */
FIL MyFile;
static char SD_Path[4]; /* SD card logical drive path */
static char* pDirectoryFiles[MAX_BMP_FILES];
static char* pDirectoryFiles1[40];
static uint32_t  ubNumberOfFiles,ubNumberOftxtFiles,ubNumberOfkaiji;
uint32_t uwBmplen = 0 ;
uint32_t  avg_adc ;
uint16_t Sum1[3000];
uint8_t  lcd_flag,lcd_flag1 ,v,year,month,day;
uint8_t  Hist_data = 0;
uint8_t aShowTime[50] = {0};
static uint16_t x,y,i,m;
static	uint8_t str[30];
static	uint32_t byteswritten, bytesread;                     /* File write/read counts */
int16_t w_buf[16000];
int16_t CH1_BUF[16000],baoluoxianshi[2000],Dagc;
		uint16_t SCAN_BUF[20000];
static int16_t CH2_BUF[489],DISSCAN_BUF[488]={408};
static int16_t CH3_BUF[489]={34};	
static uint8_t TEXT_Buffer[200],LCDDISBUF[240],LCDDISBUFPRE[240];
static	uint8_t datatemp[200];
static	uint32_t FLASH_SIZE,wifisendmun;
#define SIZE sizeof(TEXT_Buffer)
 u8 wifirevdata[100];
 u8 wifisenddata[30004];
 u8 wavestrodata[4000];
 u8 uartsenddata[310];
/* Internal Buffer defined in SDRAM memory */
uint8_t *uwInternelBuffer;
static FLASH_EraseInitTypeDef EraseInitStruct;
uint32_t FirstSector = 0, NbOfSectors = 0;	
uint32_t Address = 0, SECTORError = 0;
uint16_t encoder;
static int32_t  A1, A2, B1, B2;
__IO uint32_t data32 = 0 , MemoryProgramStatus = 0;
#define FLASH_USER_START_ADDR   ADDR_FLASH_SECTOR_6   /* Start @ of user Flash area */
#define FLASH_USER_END_ADDR     (ADDR_FLASH_SECTOR_7-1)   /* End @ of user Flash area */	
#define DATA_32                 ((uint32_t)0x12345678)		
/* UART handler declaration */
UART_HandleTypeDef UartHandle;

RTC_HandleTypeDef RtcHandle;
DMA_HandleTypeDef hdma_usart3_rx;
//USBD 
USBD_HandleTypeDef USBD_Device;
/* Private function prototypes -----------------------------------------------*/
#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */



/* Private function prototypes -----------------------------
------------------*/
static void LCD_Config(void);
static void SystemClock_Config(void);
static void Error_Handler(void);
static void CPU_CACHE_Enable(void);
static void MX_DMA_Init(void);
//void HAL_SRAM_DMA_XferCpltCallback(DMA_HandleTypeDef *hdma);
void Start_Init (void);
static void dishanzi(uint16_t x,uint16_t y,uint16_t mun,uint32_t color);
void disdashuzi(uint16_t x,uint16_t y,uint16_t mun,uint32_t color);
void LCD_ShowxNum(uint16_t x,uint16_t y,uint32_t num,uint8_t len,uint32_t ARGB);
void LCD_ShowxNum1(uint16_t x,uint16_t y,uint32_t num,uint8_t len,uint32_t ARGB);
void LCD_shouBigMun_pix(uint16_t x,uint16_t y,uint32_t num,uint8_t len, uint8_t pix,uint8_t bit,uint32_t ARGB);
void LCD_ShowxNum_pix(uint16_t x,uint16_t y,uint32_t num,uint8_t len, uint8_t pix,uint8_t bit,uint32_t ARGB);
static void distupian(uint16_t x,uint16_t y,uint32_t color);
void disdeploy(void);
void disfirstwaveindex(uint16_t coordinate);
u8 calfirstwaveindex(uint16_t coordinate);
u8 calsecondwaveindex(uint16_t coordinate);
void clearfirstwaveindex(uint16_t coordinate);
void dissecondwaveindex(uint16_t coordinate);
void clearsecondwaveindex(uint16_t coordinate);
void disarrowup(uint16_t x,uint16_t y);
void disarrowdown(uint16_t x,uint16_t y);
void dishisdeploy(void);
void dismeun(uint16_t page);
void disgongnengmeun(void);
void disconfirm(uint16_t xpos,uint16_t ypos,uint16_t yn);
void dissaved(uint16_t xpos,uint16_t ypos);
void disparament(uint16_t gongneng1,uint16_t mun,uint32_t color);
void dispageparament(uint32_t col);
void disinformationunit(void);
void LCDinformationunit(void);
void disinformationdata(void);
void LCDinformationdata(void);
void diswavedeploy(void);//显示波形外框
void LCDxcoordinate(void);//显示时间坐标

void disxcoordinate(void);//显示时间坐标
void diswave(int16_t *data,int16_t *data1);
void drawblock(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height);
void drawcolorblock(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height,uint32_t discolor);//画有颜色的块
void drawkeybroad(uint16_t Xpos,uint16_t Ypos);
char getkeymun(uint16_t Xpos,uint16_t Ypos);
char gethiskeymun(void);
void drawmainbroad(void);
void Dosomething(uint16_t key);
void Gate_display(void); //闸门显示 
void Gate_drawRect(void);
void Gate_drawRectpeak(void);
//BSCan 
void disbscandeploy(void);
void bscandisxcoordinate(uint16_t percent);//
void disbat(uint16_t percent);//dis bat vot
void dishisdata(void);
void paramint(void);
void eraseflash(uint32_t strataddress,uint32_t endaddress);
void programparamentflash(void);
void readparamentflash(void);
void programIDflash(void);
void readIDflash(void);
uint16_t Cmd2Frm(uint8_t *pfrm, uint8_t *pcmd, uint16_t nByteLen);
uint8_t FBufferHasExFrame(uint8_t *pcmd, uint8_t *buffer);
uint8_t GCMDFbuffer(uint8_t *pcmd, uint8_t *buffer);
void ProcPrmFrame(uint8_t *pcmd);
uint8_t ProcParaCmd(uint8_t *pcmd);
uint8_t ProcWaveCmd(uint8_t *pcmd);
uint8_t ProcBanbenCmd(uint8_t *pcmd);
uint8_t ProcBatCmd(uint8_t *pcmd);
uint8_t ProcSettimeCmd(uint8_t *pcmd);
uint8_t ProchouduCmd(uint8_t *pcmd);

uint8_t BSP_SRAM_ReadData_DMA(uint32_t uwStartAddress, uint16_t *pData, uint32_t uwDataSize);
 uint32_t GetSector(uint32_t Address);
 void sendparament(void);
 char calibration(void);
void dispagehisname(uint16_t mun);
void getstoremun(void);
 void dishisname(uint16_t mun,uint16_t sel);
 void disscandata(void);
 void savehisdata(void);
 void savehistxtdata(void);
 void delhisdata(void);
 void delhistxtdata(void);
 void readhisdata(void);
 void readhistxtdata(void);  
 void jisuanyingli(void);
void RTC_set(char data); 
char transformHtoB(char data);
 void POWER_Config(void);
void GPIO_Config(void);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
uint16_t Get_ADC(uint32_t ch);
static void MX_ADC1_Init(void);
void RTC_Init(void);
char transformHtoB(char data);
void RTC_set(char data);
 void RTC_TimeShow(uint8_t* showtime);
 void FPGA_PAR_EN_Init(void);
 void delay(void);
 void wuxianshezhi(void);
/* Private functions ---------------------------------------------------------*/

static uint16_t paramvaulelimitup[4][16]={  {140,150,800,10,390,400,42500,9500,6200,980,9,6,1,99,1800,6200},
																						{140,150,980,1490,1500,1490,1500,62500,7,500,0,0,0,0,0,0},
																						{99,7,5,1,100,1,1,10000,10000,10000,15,2,1,1,1,1},
                                            {1,1,20000,1,150,2,500,500,1,1,2,1,1,1,1,1},  //B扫描界面 显示内容限制
																					};
static uint16_t paramvaule[4][16]={
																		{0,30,200,0,40,300,32200,1200,2470,24,6,5,0,70,800,2470},//显示开始结束，增益，波形，闸门开始结束，声速，标定厚度，零点，温度，材料，平滑，算法，亮度 ,扫描显示长度， 厚度
																		{0,30,200,50,150,160,250,32450,1,150,55,55,55,55,55,240},//
																		{50,5,2,0,70,0,0,55,0,1152,0,0,55,55,55,40},//????,????,????,?? 
																		{0,0,500,0,30,0,300,170,0,1,270,24,10,1,1,200},//通信模式 1：WIFI  0：有线   ；测量模式：0固定PP  1自动切换；频率 ；算法：0自乘 1无	，平滑次数，铝/钛版本选择0：铝，1：钛，铝合金增益上限，钛合金增益上限；厚度补偿使能：0，不补偿，1：补偿；探头选择：1：带方向；0：不带方向
																	};
static uint16_t paramvauledefault[4][16]={
																		{0,30,200,0,40,300,32498,1200,2470,24,0,5,0,70,800,2470},//显示开始结束，增益，波形，闸门开始结束，声速，标定厚度，零点，温度，材料，平滑，算法，亮度 ,扫描显示长度， 厚度
																		{0,30,200,50,150,160,250,32450,1,150,55,55,55,55,55,240},//?????????
																		{50,5,2,0,70,0,0,55,0,1152,0,0,55,55,55,40},//????,????,????,?? 
																		{0,0,500,0,30,0,300,170,0,1,270,24,10,1,1,200},
																	};
static char paramenttype[4][17]={ //{2,3,3,1,3,3,24,24,4,4,3,3,2,9,9,4},//9???????????,10??????`
																	{3,3,13,30,14,14,15,24,34,3,30,30,30,3,4,25,30}, //30 表示整数
																	{3,3,13,14,14,14,14,15,30,13,0,0,0,0,0,0,0},//15各位表示有几位书，十位表示几个小数点 例如24 表示2位小数，4表示4个数字
																	{13,30,1,30,3,30,0,0,0,0,0,0,0,0,0,0,0},
																	{3,4,25,30,14,14,14,14,3,4,30,3,2,9,9,4,0}, //30 表示整数
																};
static char parament_bit[4][16]={ //{2,3,3,1,3,3,24,24,4,4,3,3,2,9,9,4},//9???????????,10??????`
																	{3,3,3,1,4,4,4,4,4,4,3,3,2,9,5,4},
																	{5,4,4,3,10,9,10,9,10,0,0,0,0,0,0,0},
																	{2,2,1,30,3,30,2,0,0,0,0,0,0,0,0,0},
																	{3,4,5,4,4,4,4,4,4,4,3,3,2,9,9,4},//page3
																};	
static uint16_t shengsu[10]={32250,32100,22600,30800,32300,31130,0};
static uint16_t shengsutemp[10],yingli[1000];
uint16_t bobanpeakloc[40];
int16_t bobanbokuan[40];
	uint16_t  datalocs1[3],datalocm1[3];
static uint16_t senddata[100];
static uint16_t YB,HB,Y,H;
static uint16_t paramvauletemp[4][16];
int32 ziji;
static uint32_t prethickdis,thickdis,histhick,scanthickdis,thickavg,thickavg3,thickavgtemp;
static uint32_t thickavg1[200],thickavg2[200],thickbaocun[200];
char xinhaoindexs[30];
 char recivenew,modifyparament,findway,modelindex,error,calibrationindex,calibrationokindex,prekeymun,keymun,dosomething,draw,scanhisindex,
	hisindex,findpeakindex,saveindex,delhisdataindex,HL,timemodify,timemodifything,settime,SCAN_cursorindex,readwave,
readhisdataindex,pause,deleteindex,confirmindex,hiswavedis,scandirection,firstmun,defaultindex,wavetempstore,
prekeyindex,keyindex,pagekeyindex,paramentmodifybit,paramentmodifydisbit,gongneng,hispage,discalibrationindex,avgcount=0,preavgcount=0
,consel,CheckRes,reset,sendindex,sendthickindex,lcd_flag2,xinhaoindex,uartreceive,uartreceiveindex,tcpreceiveindex,lcddisindex,shezhiwuxian=0;
 u8 lcdfirstcoordinate,prelcdfirstcoordinate,lcdsecondcoordinate,prelcdsecondcoordinate,avgcount1,processBuffer[40],cmdBuffer[40],batpercent,boxingduan,xinhaoindexmun,xinhaocount,inposition,waveshuaxin=0;
 char banbenlv[]="v25-02-15-L";
 char banbentai[]="v25-02-15-T";
//findway寻找峰值的方法
//error寻找峰值是否有找到的指示
//modifyparament:修改参数的指示
//modelindex;手动自动模式下的计算 0自动 1手动;
//calibrationindex校准的指示
//draw显示波形指示0表示还在显示中，1表示可以刷新波形
//findpeakindex正在找峰值的指示1,：还在找；0：结束了
//hisindex历史数据显示指示，0正常，1在历史数据显示界面
//saveindex保存数据指示
//readhisdataindex读取历史数据指示 0：没有 1：读取
//pause暂停显示指示 0正常 1暂停
//deleteindex指示是删除还是不删除
//confirmindex指示是在确认删除显示界面 0不在，1在
//hiswavedis读取历史数据中的波形模式
//scandirection扫描的方向，0右 1左
//scanhisindex扫描状态下历史数据界面的显示指示
//firstmun用数字键输入的时候第一次按键的指示
//pagekeyindex指示现在的键盘转态，0没有调出，1调出键盘
//paramentmodifybit键盘显示的整数位
//paramentmodifydisbit显示的
//gongneng 1：测厚；2：探伤；3：应力
//hispage历史数据 页码
static int16_t peakm;
static float peakb,peak1b,yinglixishu,KP,KI,KD,T,xiangsidu;
 uint16_t page,paramentmun,paramback,modifybit,peakc1,timeA2,peakc,timeA1,adc_num,
	peakst,zeropointd,time,timed,countinuemun,hisgain,hisv,hiszero,hist,hismun,hiskeymun,usedv,agctemp,agctempstore,scanindex,
start,length,Gate_start,Gate_length,paramentmodifyval,key_x,key_y,caltouch_x,caltouch_y,dbpeak,
dbpeakloc,sbpeak,sbpeakloc,yinglicount,yinglimin,yinglimax,yinglizhi,calibrationcount,wifisendcount,
Gate_startA,Gate_startS,Gate_lengthA,Gate_lengthS,SCAN_cursor,preSCAN_cursor,wifirevmun,wifistatus,heartcount,paramentmun_r,paramentdatatemp,sendpyhcount,calpeakloc,firstloc,secondloc,disfirstcoordinate,prefirstcoordinate
,dissecondcoordinate,presecondcoordinate,loctemp,peakmb,taipinlv,senden,storecount,timecount=0,timeset=0,caltime;
//paramentmodifyval为修改参数的暂时存储
static	uint16_t Gate_start_parA=100,Gate_start_parS=200,Gate_length_parA=20,Gate_length_parS=20,storemun,bscanstoremun;
//thickdis用于显示的厚度
//speak,speakloc界面上用于显示第一个峰的大小和位置；
//peakc1,timeA2界面上用于显示第二个峰的大小和位置
//peakb,peak1b找峰值算法中的中间比较量
//peakst，time，timed用于校准函数中的两个峰之间的时间差
//zeropointd下波形的零点
//countinuemun用于连续按键
//hisgain,hisv,hiszero,hist,histhick历史数据中的信息值
//hismun历史数据显示的位置指示
//usedv现在使用的声速/
//agctemp自动状态下的增益  
//scanindex扫描状态指示 0不在 1在
//
static float sn,snavg,kp,lcdkp,distance;
static TS_StateTypeDef  TS_State1;  
//sn,snavg 信噪比 -平均值
//kp取显示点的系数

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)


{
	uint32_t i ,counter;
	//int64_t v ;
	uint8_t j,k,success;
uwInternelBuffer = (uint8_t *)0xC0600000;	//1024x768x4x2

  ADC_ChannelConfTypeDef sConfig;

	  /* Fill EraseInit structure*/
	

	//GPIO_InitTypeDef  gpio_init_structure;'
  /* Enable the CPU Cache */
  CPU_CACHE_Enable();

  /* STM32F7xx HAL library initialization:
       - Configure the Flash ART accelerator on ITCM interface
       - Configure the Systick to generate an interrupt each 1 msec
       - Set NVIC Group Priority to 4
       - Global MSP (MCU Support Package) initialization
     */
  HAL_Init();
  POWER_Config();

  /* Configure the system clock to 200 MHz */
  SystemClock_Config();
	   delay_init(200); 



	 
	FPGA_PAR_EN_Init();
	BSP_SRAM_Init();
		W25QXX_Init();		            //初始化W25QXX
	uart_init(115200);		        //串口初始化
	//MX_DMA_Init();
	MX_ADC1_Init();	  
	paramint();
	readparamentflash();	
	if(paramvauletemp[2][15]==paramvaule[2][15])//不是第一次启动，把FLASH中的参数读出来
		{
			for(j=0;j<4;j++)
			{
				for(k=0;k<16;k++)
					paramvaule[j][k]=paramvauletemp[j][k];
				//paramvaule[j][k]=paramvaule[j][k];
			}
							for(k=0;k<10;k++)
					shengsu[k]=shengsutemp[k];	
		}
	else//是第一次启动把默认参数写进FLASH
		{
			
			programparamentflash();	
		}
	
if(paramvaule[3][6]<200||paramvaule[3][6]>500)
	paramvaule[3][6]=300;


if(paramvaule[3][7]<200||paramvaule[3][7]>500)
	paramvaule[3][7]=420;

	

		
		//读取存储的指示
		HAL_NVIC_DisableIRQ(EXTI15_10_IRQn); //解决开机过程中干扰导致的死机  2020-11-26
	 	BSP_SD_Init();
		


			
FLASH_SIZE=32*1024*1024;	//FLASH 大小为32M字节
	if(W25QXX_ReadID()==W25Q256)								//检测不到W25Q256	
	{
	W25QXX_Write((uint8_t*)TEXT_Buffer,FLASH_SIZE-300,SIZE);		//从倒数第100个地址处开始,写入SIZE长度的数据
	W25QXX_Read(datatemp,FLASH_SIZE-300,SIZE);					//从倒数第100个地址处开始,读出SIZE个字节
	}
//programIDflash();
readIDflash();

		paramvaule[3][15]=0;
		paramvaule[0][3]=0;
		HL=1;
	paramvaule[0][10]=6;

		modifyparament=0;//正常显示状态	

					GPIO_Config();
		for(i=0;i<200;i++)
			thickavg1[i]=0;
		for(i=0;i<30;i++)
			xinhaoindexs[i]=10;
		m=0;	
if(paramvaule[2][8]	>50&&paramvaule[2][8]<200)
taipinlv=500;
else
	taipinlv=340;
	
//		key_x=155;
//		key_y=481;
//		drawmainbroad();		
	
/*
	if (paramvaule[3][0]==0)	//0 选择有线，1	 选择WIFI	
	{
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_RESET);
			HAL_Delay(400);
			consel=0;
		gpio_for_w5500_config();						        
	reset_w5500();											
	set_w5500_mac();										
	set_w5500_ip();										
	socket_buf_init(txsize, rxsize);		
		reset=1;
	}
		else
		{HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_SET);
				HAL_Delay(400);
			consel=1;
			M8266HostIf_Init();
			
				success = M8266WIFI_Module_Init_Via_SPI();
				 if(success)
				 {
				 M8266WIFI_Test();
				 }
			 } 
  */
			 MX_GPIO_Init();
			 HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_SET);//无线电源打开
			 HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_SET);//无线SET脚高
			 
delay_ms(200);
		LCD_Init();
		delay_ms(200);
	LCD_Fill(0,0,LCD_W,LCD_H,BLACK);
		delay_ms(200);

if(lcddisindex==0)//信息
LCDinformationunit();
else
LCDwavedeploy();
		 HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
//paramvaule[3][2]=500;
//paramvaule[2][9]=1152;
//paramvaule[2][10]=0;
//paramvaule[2][11]=0;
//wuxianshezhi();
	 for(x=0;x<40;x++)
	{
	processBuffer[x]=0;

	}
	while(1)	
 {
	 if(shezhiwuxian==1)
	 {
	 wuxianshezhi();
		 shezhiwuxian=0;
	 }
	 
		 if(uartreceive==1||(UART3_Handler.RxState == HAL_UART_STATE_READY))
		 {
			 UART_EndRxTransfer(&UART3_Handler);
			 UART3_Handler.RxState = HAL_UART_STATE_READY;
		 HAL_UART_Receive_IT(&UART3_Handler, (u8 *)aRxBuffer, 40);
		//	HAL_UART_Receive_DMA(&UART3_Handler, (u8 *)aRxBuffer, 40) ;	 
		 uartreceive=0;
		 }
  if(aRxBuffer[35]!=0||aRxBuffer[36]!=0||aRxBuffer[37]!=0||aRxBuffer[38]!=0||aRxBuffer[39]!=0)
  {
      uartreceive=1;
      for(x=0;x<40;x++)
        aRxBuffer[x]=0;
  }		 

	 senden++;
if(senden>5)
	senden=0;


//无线串口
/*
if(aRxBuffer[11]!=0)	
{
	 for(x=0;x<40;x++)
	{
	processBuffer[x]=aRxBuffer[x];
		aRxBuffer[x]=0;
	}
	 uartreceive=1;

}	
*/
if((aRxBuffer[0]!=0||aRxBuffer[1]!=0||aRxBuffer[2]!=0||aRxBuffer[3]!=0||aRxBuffer[4]!=0||aRxBuffer[5]!=0||aRxBuffer[6]!=0
	||aRxBuffer[7]!=0||aRxBuffer[8]!=0||aRxBuffer[9]!=0)&&processBuffer[0]==0)//接收到数据，看是否有合适的并转移到命令数组
{

	
	FBufferHasExFrame(processBuffer,aRxBuffer);	
	 for(x=0;x<40;x++)
	{
		aRxBuffer[x]=0;
	}
	 uartreceive=1;		

}

if(processBuffer[0]!=0)
{
 if(GCMDFbuffer(cmdBuffer,processBuffer)==1)
 {
 switch(cmdBuffer[4])
 {
	 case 0x11 ://参数
		 ProcParaCmd(cmdBuffer);
		 break;
	 case 0x22 ://读波形
		 ProcWaveCmd(cmdBuffer);
		 break;	
	 	 case 0x44 ://获取电池
		ProcBatCmd(cmdBuffer);
		 break;	
	 case 0x33 ://发送厚度
		  ProchouduCmd(cmdBuffer);
		 break;	
	 	 case 0x41 ://时间校准
			ProcSettimeCmd(cmdBuffer);
		 break;	
		 	 case 0x42 ://版本号
			ProcBanbenCmd(cmdBuffer);
		 break;	
		default:
		break;
 }
 }
}
					if(dosomething==1)
		{dosomething=0;
			// Dosomething(keymun);	
		}	
		if(heartcount>800)
		{
				heartcount=0;
		
    }

		 if(lcd_flag1==0)
		  uhADCxConvertedValue=Get_ADC(ADC_CHANNEL_3);	
			v =uhADCxConvertedValue*100*3.3/4096 ;
		 if(lcd_flag1==0)
		   {
		   	if(adc_num>30)
				{
		    	adc_num=0;
					avg_adc=0;
						for(i=0;i<30;i++)
					avg_adc =Sum1[i]+avg_adc;
					avg_adc=avg_adc/30;
				}
		    else
		     adc_num=adc_num+1;	
		    Sum1[adc_num]=uhADCxConvertedValue;
				
	if(avg_adc>= 3300)
		 avg_adc=3300;
	if(avg_adc<= 2511)
		 avg_adc=2511;
     batpercent=(avg_adc-2511)*100/790;
		  }	
			
			   	 if(findpeakindex==0&&recivenew==1)//
		 {
			 	if(paramvaule[0][10]==6)//自定义声速
			usedv=paramvaule[0][6];
					else
			usedv=shengsu[paramvaule[0][10]];
						 HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
			
					
						 if(sendthickindex==1)//&&lcd_flag2==2&&senden>=2
	 {
		
								timecount++;
	
		 if(timecount>=4)
		 {
			 xinhaocount++;
			 if(xinhaocount>254)
				 xinhaocount=0;
									wifisenddata[0]=0x66;
									wifisenddata[1]=0x5a;
			 						wifisenddata[2]=6>>8;
			 						wifisenddata[3]=6;
									wifisenddata[4]=0x35;
									wifisenddata[5]=xinhaocount;
									wifisenddata[6]=thickavg>>8;
									wifisenddata[7]=thickavg;
									wifisenddata[8]=timeset>>8;
									wifisenddata[9]=timeset;	 
			CheckRes = 0;
			for(i = 0; i <10; i++)			
				CheckRes ^= wifisenddata[i];
			wifisenddata[10]=CheckRes;			
			wifisenddata[11]=0x99;
		 HAL_UART_Transmit(&UART3_Handler,wifisenddata,12,10);
			 timecount=0;
		 }
		 

	 
	 }
	 	 HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
					recivenew=0;
			 if(lcddisindex==0&&lcd_flag1==30)
			 LCDinformationdata();

				 if(avgcount>=200)
				 avgcount=0;
				thickavg1[avgcount]=thickdis;
			 thickavg3=0;
			 avgcount1=0;
			 thickavg=0;
				 //去除错误的算法



if(paramvaule[3][4]>150)
	paramvaule[3][4]=150;
if(paramvaule[3][4]<5)
paramvaule[3][4]=5;
thickavg3=0;
			 avgcount1=0;
			 for(i=0;i<paramvaule[3][4];i++)
			 {
						if(i<=avgcount)
						{
							
							if(thickavg1[avgcount-i]>20)
							{
								if(i==avgcount)
								{
										if((thickavg1[199]<20||thickavg1[1]<20)||(abs(thickavg1[0]-thickavg1[1])<1000&&abs(thickavg1[0]-thickavg1[199])<1000))
										{
						thickavg3=thickavg1[avgcount-i]+thickavg3;
						avgcount1++;	
										}								
								}
									else
									{
										if((thickavg1[avgcount-i-1]<20||thickavg1[avgcount-i+1]<20)||(abs(thickavg1[avgcount-i]-thickavg1[avgcount-i-1])<1000&&abs(thickavg1[avgcount-i]-thickavg1[avgcount-i+1])<1000))
										{
						thickavg3=thickavg1[avgcount-i]+thickavg3;
						avgcount1++;	
										}
									}										
							}
						}
					else
					{if(thickavg1[avgcount-i+199]>20)
						{
							if((avgcount-i)==-1)
							{
							if((thickavg1[198]<20||thickavg1[0]<20)||(abs(thickavg1[199]-thickavg1[0])<1000&&abs(thickavg1[199]-thickavg1[198])<1000))
							{
						thickavg3=thickavg1[avgcount-i+199]+thickavg3; 
							avgcount1++;							
							}
							}
							else
							{
										if((thickavg1[avgcount-i+199-1]<20||thickavg1[avgcount-i+199+1]<20)||(abs(thickavg1[avgcount-i+199]-thickavg1[avgcount-i+199-1])<1000&&abs(thickavg1[avgcount-i+199]-thickavg1[avgcount-i+199+1])<1000))
										{
						thickavg3=thickavg1[avgcount-i+199]+thickavg3; 
							avgcount1++;										
										}							
							}
								

						}	
					}			 
			 }
if(avgcount1>0)			 
			thickavg3=thickavg3/avgcount1;
			thickavgtemp=	thickavg3;
thickavg3=0;	
avgcount1=0;			 
			 for(i=0;i<paramvaule[3][4];i++)
			 {
						if(i<=avgcount)
						{
							if(abs(thickavg1[avgcount-i]-thickavgtemp)<thickavgtemp*0.3)
							{
						thickavg3=thickavg1[avgcount-i]+thickavg3;
						avgcount1++;								
							}
						}
					else
					{if(abs(thickavg1[avgcount-i+200]-thickavgtemp)<thickavgtemp*0.3)
						{
						thickavg3=thickavg1[avgcount-i+200]+thickavg3; 
							avgcount1++;
						}	
					}			 
			 }	
				if(avgcount1>0)			 
			thickavg3=thickavg3/avgcount1;			 
			 for(i=0;i<paramvaule[3][4];i++)
			 {
						if(i<=avgcount)
						{
							if(abs(thickavg1[avgcount-i]-thickavg3)<250)
						thickavg2[i]=thickavg1[avgcount-i];
							else
							thickavg2[i]=0;	
						}
					else
					{if(abs(thickavg1[avgcount-i+200]-thickavg3)<250)
						thickavg2[i]=thickavg1[avgcount-i+200]; 
					else
							thickavg2[i]=0;	
					}
			 
			 }
				thickavg=0;
			 avgcount1=0;
					for(i=0;i<paramvaule[3][4];i++)
				{
					if(thickavg2[i]>10)
					{
						thickavg=thickavg2[i]+thickavg;
						avgcount1++;
					}


				
				}
					if(avgcount1>0)
						thickavg=thickavg/avgcount1;
										////去除错误算法结束

				avgcount++;			 

			  findpeakindex=0;

		 }

	
 
 
 }
 }
 
 
 
 
 
 


		 


/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
  HAL_UART_Transmit(&UartHandle, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}

/**
  * @brief  LCD configuration
  * @param  None
  * @retval None
  */
static void LCD_Config(void)
{
  /* LCD Initialization */ 
  /* Two layers are used in this application but not simultaneously 
     so "LCD_MAX_PCLK" is recommended to programme the maximum PCLK = 25,16 MHz */
  BSP_LCD_Init();

  /* LCD Initialization */ 
 // BSP_LCD_LayerDefaultInit(0, LCD_FB_START_ADDRESS);
	  BSP_LCD_LayerDefaultInit(0, LCD_FB_START_ADDRESS+(BSP_LCD_GetXSize()*BSP_LCD_GetYSize()*4));
  BSP_LCD_LayerDefaultInit(1, LCD_FB_START_ADDRESS+(BSP_LCD_GetXSize()*BSP_LCD_GetYSize()*4));

  /* Enable the LCD */ 
  BSP_LCD_DisplayOn(); 
  
  /* Select the LCD Background Layer  */
  BSP_LCD_SelectLayer(0);

  /* Clear the Background Layer */ 
  BSP_LCD_Clear(LCD_COLOR_BLACK);  
  
  /* Select the LCD Foreground Layer  */
  BSP_LCD_SelectLayer(1);

  /* Clear the Foreground Layer */ 
  BSP_LCD_Clear(LCD_COLOR_BLACK);
  
  /* Configure the transparency for foreground and background :
     Increase the transparency */
  BSP_LCD_SetTransparency(0, 10);
  BSP_LCD_SetTransparency(1, 200);
}

/**
  * @brief  Clock Config.
  * @param  hltdc: LTDC handle
  * @note   This API is called by BSP_LCD_Init()
  * @retval None
  */
void BSP_LCD_ClockConfig(LTDC_HandleTypeDef *hltdc, void *Params)
{
  static RCC_PeriphCLKInitTypeDef  periph_clk_init_struct;
	/* Select the used LCD */
#ifdef USE_480x272
    /* The AMPIRE LCD 480x272 is selected */
    /* LCD clock configuration */
    /* PLLSAI_VCO Input = HSE_VALUE/PLL_M = 1 Mhz */
    /* PLLSAI_VCO Output = PLLSAI_VCO Input * PLLSAIN = 192 Mhz */
    /* PLLLCDCLK = PLLSAI_VCO Output/PLLSAIR = 192/5 = 38.4 Mhz */
    /* LTDC clock frequency = PLLLCDCLK / LTDC_PLLSAI_DIVR_4 = 38.4/4 = 9.6Mhz */
    periph_clk_init_struct.PeriphClockSelection = RCC_PERIPHCLK_LTDC;
    periph_clk_init_struct.PLLSAI.PLLSAIN = 192;
    periph_clk_init_struct.PLLSAI.PLLSAIR = AMPIRE480272_FREQUENCY_DIVIDER;
    periph_clk_init_struct.PLLSAIDivR = RCC_PLLSAIDIVR_4;
    HAL_RCCEx_PeriphCLKConfig(&periph_clk_init_struct); 
#endif	
#ifdef USE_640x480
		/* AMPIRE640480 LCD clock configuration */
    /* PLLSAI_VCO Input = HSE_VALUE/PLL_M = 1 Mhz */
    /* PLLSAI_VCO Output = PLLSAI_VCO Input * PLLSAIN = 120 Mhz */
    /* PLLLCDCLK = PLLSAI_VCO Output/PLLSAIR = 120/3 = 40 Mhz */
    /* LTDC clock frequency = PLLLCDCLK / LTDC_PLLSAI_DIVR_2 = 40/2 = 20 Mhz */
//    periph_clk_init_struct.PeriphClockSelection = RCC_PERIPHCLK_LTDC;
//    periph_clk_init_struct.PLLSAI.PLLSAIN = 120;
//    periph_clk_init_struct.PLLSAI.PLLSAIR = 4;
//    periph_clk_init_struct.PLLSAIDivR = RCC_PLLSAIDIVR_2;
//    HAL_RCCEx_PeriphCLKConfig(&periph_clk_init_struct);
/*
    periph_clk_init_struct.PeriphClockSelection = RCC_PERIPHCLK_LTDC;
		
    periph_clk_init_struct.PLLSAI.PLLSAIN = 120;
    periph_clk_init_struct.PLLSAI.PLLSAIR = 4;
    periph_clk_init_struct.PLLSAIDivR = RCC_PLLSAIDIVR_2;
    HAL_RCCEx_PeriphCLKConfig(&periph_clk_init_struct);
	*/
		    periph_clk_init_struct.PeriphClockSelection =RCC_PERIPHCLK_LTDC;//RCC_PERIPHCLK_CLK48; //
    periph_clk_init_struct.PLLSAI.PLLSAIN =384; //120;//
    periph_clk_init_struct.PLLSAI.PLLSAIR =7;//7; //
    periph_clk_init_struct.PLLSAIDivR =RCC_PLLSAIDIVR_4;//RCC_PLLSAIP_DIV8; //
    HAL_RCCEx_PeriphCLKConfig(&periph_clk_init_struct);
			
		
#endif
#ifdef USE_800x480
    /* The 640x480 LCD is selected */
		/* LCD clock configuration */
		/* PLLSAI_VCO Input = HSE_VALUE/PLL_M = 1 Mhz */
		/* PLLSAI_VCO Output = PLLSAI_VCO Input * PLLSAIN = 120 Mhz */
		/* PLLLCDCLK = PLLSAI_VCO Output/PLLSAIR = 120/2 = 60 Mhz */
		/* LTDC clock frequency = PLLLCDCLK / LTDC_PLLSAI_DIVR_2 = 60/2 = 30 Mhz */
		periph_clk_init_struct.PLLSAI.PLLSAIN = 100;//140 35MHz			
		periph_clk_init_struct.PeriphClockSelection = RCC_PERIPHCLK_LTDC;    
		periph_clk_init_struct.PLLSAI.PLLSAIR = 2;    
		periph_clk_init_struct.PLLSAIDivR = RCC_PLLSAIDIVR_2;
		HAL_RCCEx_PeriphCLKConfig(&periph_clk_init_struct);
#endif
#ifdef USE_800x600
    /* The 800x600 LCD is selected */
		/* LCD clock configuration */
		/* PLLSAI_VCO Input = HSE_VALUE/PLL_M = 1 Mhz */
		/* PLLSAI_VCO Output = PLLSAI_VCO Input * PLLSAIN = 120 Mhz */
		/* PLLLCDCLK = PLLSAI_VCO Output/PLLSAIR = 120/2 = 60 Mhz */
		/* LTDC clock frequency = PLLLCDCLK / LTDC_PLLSAI_DIVR_2 = 60/2 = 30 Mhz */
		periph_clk_init_struct.PLLSAI.PLLSAIN = 120;//140 35MHz			
		periph_clk_init_struct.PeriphClockSelection = RCC_PERIPHCLK_LTDC;    
		periph_clk_init_struct.PLLSAI.PLLSAIR = 2;    
		periph_clk_init_struct.PLLSAIDivR = RCC_PLLSAIDIVR_2;
		HAL_RCCEx_PeriphCLKConfig(&periph_clk_init_struct);
#endif
#ifdef USE_1024x600
		/* The 1024x600 LCD is selected */
		/* LCD clock configuration */
		/* PLLSAI_VCO Input = HSE_VALUE/PLL_M = 1 Mhz */
		/* PLLSAI_VCO Output = PLLSAI_VCO Input * PLLSAIN = 192 Mhz */
		/* PLLLCDCLK = PLLSAI_VCO Output/PLLSAIR = 192/3 = 64 Mhz */
		/* LTDC clock frequency = PLLLCDCLK / LTDC_PLLSAI_DIVR_2 = 64/2 = 32 Mhz */
		periph_clk_init_struct.PLLSAI.PLLSAIN = 192;//		
		periph_clk_init_struct.PeriphClockSelection = RCC_PERIPHCLK_LTDC;    
		periph_clk_init_struct.PLLSAI.PLLSAIR = 3;    
		periph_clk_init_struct.PLLSAIDivR = RCC_PLLSAIDIVR_2;
		HAL_RCCEx_PeriphCLKConfig(&periph_clk_init_struct);  
#endif
#ifdef USE_1024x768
		/* The 1024x768 LCD is selected */
		/* LCD clock configuration */
		/* PLLSAI_VCO Input = HSE_VALUE/PLL_M = 1 Mhz */
		/* PLLSAI_VCO Output = PLLSAI_VCO Input * PLLSAIN = 144 Mhz */
		/* PLLLCDCLK = PLLSAI_VCO Output/PLLSAIR = 144/2 = 72 Mhz */
		/* LTDC clock frequency = PLLLCDCLK / LTDC_PLLSAI_DIVR_2 = 72/2 = 36 Mhz */
		periph_clk_init_struct.PLLSAI.PLLSAIN = 192;//		
		periph_clk_init_struct.PeriphClockSelection = RCC_PERIPHCLK_LTDC;    
		periph_clk_init_struct.PLLSAI.PLLSAIR = 3;    
		periph_clk_init_struct.PLLSAIDivR = RCC_PLLSAIDIVR_2;
		HAL_RCCEx_PeriphCLKConfig(&periph_clk_init_struct);
#endif
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* Turn LED3 on */
  BSP_LED_On(LED3);
  while(1)
  {
  }
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 200000000
  *            HCLK(Hz)                       = 200000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 25000000
  *            PLL_M                          = 25
  *            PLL_N                          = 400
  *            PLL_P                          = 2
  *            PLL_Q                          = 8
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 6
  * @param  None
  * @retval None
  */
void SystemClock_Config(void)
{
   RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;
  HAL_StatusTypeDef ret = HAL_OK;

  /* Enable HSE Oscillator and activate PLL with HSE as source */
	
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 400;  
  //RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 9;//8

  ret = HAL_RCC_OscConfig(&RCC_OscInitStruct);
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }
/*
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_OFF;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 432;  
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  
	*/
  /* Activate the OverDrive to reach the 200 MHz Frequency */
  ret = HAL_PWREx_EnableOverDrive();
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }
    /* Select PLLSAI output as USB clock source */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLLSAIP;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 384;
  PeriphClkInitStruct.PLLSAI.PLLSAIQ = 7; 
  PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV8;
  if(HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct)  != HAL_OK)
  {
    Error_Handler();
  }
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers */
//  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
//  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
//  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;  
//  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  ret = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7);
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }
}
//    HAL_StatusTypeDef ret = HAL_OK;
//    RCC_OscInitTypeDef RCC_OscInitStructure; 
//    RCC_ClkInitTypeDef RCC_ClkInitStructure;
//	
//    __HAL_RCC_PWR_CLK_ENABLE(); //使能PWR时钟
// 
//    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);//设置调压器输出电压级别，以便在器件未以最大频率工作
//      
//    RCC_OscInitStructure.OscillatorType=RCC_OSCILLATORTYPE_HSE;    //时钟源为HSE
//    RCC_OscInitStructure.HSEState=RCC_HSE_ON;                      //打开HSE
//    RCC_OscInitStructure.PLL.PLLState=RCC_PLL_ON;				   //打开PLL
//    RCC_OscInitStructure.PLL.PLLSource=RCC_PLLSOURCE_HSE;          //PLL时钟源选择HSE
//    RCC_OscInitStructure.PLL.PLLM=25;	//主PLL和音频PLL分频系数(PLL之前的分频)
//    RCC_OscInitStructure.PLL.PLLN=432; //主PLL倍频系数(PLL倍频)
//    RCC_OscInitStructure.PLL.PLLP=2; //系统时钟的主PLL分频系数(PLL之后的分频)
//    RCC_OscInitStructure.PLL.PLLQ=9; //USB/SDIO/随机数产生器等的主PLL分频系数(PLL之后的分频)
//    ret=HAL_RCC_OscConfig(&RCC_OscInitStructure);//初始化
//    if(ret!=HAL_OK) while(1);
//    
//    ret=HAL_PWREx_EnableOverDrive(); //开启Over-Driver功能
//    if(ret!=HAL_OK) while(1);
//    
//    //选中PLL作为系统时钟源并且配置HCLK,PCLK1和PCLK2
//    RCC_ClkInitStructure.ClockType=(RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2);
//    RCC_ClkInitStructure.SYSCLKSource=RCC_SYSCLKSOURCE_PLLCLK;//设置系统时钟时钟源为PLL
//    RCC_ClkInitStructure.AHBCLKDivider=RCC_SYSCLK_DIV1;//AHB分频系数为1
//    RCC_ClkInitStructure.APB1CLKDivider=RCC_HCLK_DIV4;//APB1分频系数为4
//    RCC_ClkInitStructure.APB2CLKDivider=RCC_HCLK_DIV2;//APB2分频系数为2
//    
//    ret=HAL_RCC_ClockConfig(&RCC_ClkInitStructure,FLASH_LATENCY_7);//同时设置FLASH延时周期为7WS，也就是8个CPU周期。
//    if(ret!=HAL_OK) while(1);


/**
  * @brief  CPU L1-Cache enable.
  * @param  None
  * @retval None
  */
static void CPU_CACHE_Enable(void)
{
  /* Enable I-Cache */
  SCB_EnableICache();

  /* Enable D-Cache */
  SCB_EnableDCache();
	
		SCB->CACR|=1<<2;   //强制D-Cache透写,如不开启,实际使用中可能遇到各种问题	
}
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

}
////////////////////////////////////////////////////////////////
 void HAL_SRAM_DMA_XferCpltCallback(DMA_HandleTypeDef *hdma)
{
	 uint16_t i,findstrat,z,startb ;
uint32_t pinjun;



	sendpyhcount++;

	timeset=timeset+10;
	if(timeset>60000)
	
		timeset=timeset%60000;

	

		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
    sendparament();

	 

//PID运算
		if(findpeakindex==0&&hisindex==0&&pause==0)//
		{
			recivenew=1;
			if(w_buf[0]==0xf&&w_buf[1]==0xf)
			{
			if(paramvaule[3][3]==0)
			{
//6M  520开始  5.5M 540  5M  540    4.5M  530   4M  540  3.5M  600  3.3 620
		 if(paramvaule[3][2]<399&&paramvaule[3][2]>345)
		 {
			 findstrat=600;
			 startb=findstrat+30;
		 }
		 else if(paramvaule[3][2]<=345&&paramvaule[3][2]>330)
		 {
			 findstrat=620;
			 startb=findstrat+30;
		 }
		    else if(paramvaule[3][2]>=400)
				{
					findstrat=470;
					startb=findstrat+30;
				}
					findstrat=290;
					startb=findstrat+30;
				for(z=0;z<2000;z++)//找第一串波的最大值
				baoluoxianshi[z]=w_buf[z+4000];
			for(z=findstrat;z<findstrat+100;z++)//找第一串波的最大值
				{

					pinjun=(abs(w_buf[z])+abs(w_buf[z+1])+abs(w_buf[z+2])+abs(w_buf[z+3])+abs(w_buf[z+4])+abs(w_buf[z+5])+abs(w_buf[z+6])+abs(w_buf[z+7])+abs(w_buf[z+8])+abs(w_buf[z+10])+abs(w_buf[z+11])+abs(w_buf[z+12])+abs(w_buf[z+13])+abs(w_buf[z+14])
					+abs(w_buf[z+15])+abs(w_buf[z+16])+abs(w_buf[z+17])+abs(w_buf[z+18])+abs(w_buf[z+19])+abs(w_buf[z+20])+abs(w_buf[z+21])
					+abs(w_buf[z+22])+abs(w_buf[z+24])+abs(w_buf[z+25])+abs(w_buf[z+26])+abs(w_buf[z+27])+abs(w_buf[z+28])
					+abs(w_buf[z+29])+abs(w_buf[z+31])+abs(w_buf[z+32]))/33;
					if(pinjun<1000)
					{
							startb=z;
						break;
					}	
				} 
				if(z>=findstrat+99)//7M
					startb=paramvaule[0][4]*10;					

				peakm=w_buf[startb];
			//	peakm=w_buf[paramvaule[0][4]*10];
			for(i=startb;i<4000;i++)
				{
				if(w_buf[i]>peakm)
						peakm=w_buf[i];
				}
				if(peakm<1)
					peakm=1;
			for(i=0;i<4000;i++)
					{				if(w_buf[i]>=0)
						{
										ziji=w_buf[i]*w_buf[i]/peakm;
										if(ziji>32766)
										CH1_BUF[i]=32766;
									else
										CH1_BUF[i]=ziji;
						}
									else
									{
										ziji=-w_buf[i]*w_buf[i]/peakm;
										if(ziji<-32766)
											CH1_BUF[i]=-32766;
											else
										CH1_BUF[i]=ziji;
									}
						SCAN_BUF[i]=w_buf[i];			
								
					}
					peakmb=peakm*1.2;
											for(i=4000;i<8000;i++)
					{
					//CH1_BUF[i]=w_buf[i];
						
												ziji=w_buf[i]*w_buf[i];
						     ziji=ziji/peakmb;
										if(ziji>32766)
										CH1_BUF[i]=32766;
									else
										CH1_BUF[i]=ziji;				
					}
//					peakm=peakm*1.4;
//			for(i=4000;i<8000;i++)
//					{				if(w_buf[i]>=0)
//						{
//										ziji=w_buf[i]*w_buf[i]/peakm;
//										if(ziji>32766)
//										CH1_BUF[i]=32766;
//									else
//										CH1_BUF[i]=ziji;
//						}
//									else
//									{
//										ziji=-w_buf[i]*w_buf[i]/peakm;
//										if(ziji<-32766)
//											CH1_BUF[i]=-32766;
//											else
//										CH1_BUF[i]=ziji;
//									}																
//					}
			}
			else
			{
			for(i=0;i<8000;i++)		
				CH1_BUF[i]=w_buf[i];
			
			}
			}
								if(lcd_flag2>0)  //显示波形刷新
				     lcd_flag2=0;
			    else
					   lcd_flag2=1;

		if(paramvaule[3][3]==0)//自乘算法波形
		{

			if(peakc>21000&&agctemp>8&&(modelindex==0||modelindex==3||modelindex==4))
			{
				if(abs(peakc-18000)<4000)
							agctemp=agctemp-1;
				else
						agctemp=agctemp-2;
			}
			else
			{	
		if(paramvaule[3][5]==0)//铝合金	
		{			
			 if(peakc<16000&&agctemp<paramvaule[3][6]&&(modelindex==0||modelindex==3||modelindex==4))
					{
						if(abs(peakc-18000)<4000)
									agctemp=agctemp+1;
						else
								agctemp=agctemp+2;
					}	
				}			
			else//钛合金
			{
						 if(peakc<16000&&agctemp<paramvaule[3][7]&&(modelindex==0||modelindex==3||modelindex==4))
								{
									if(abs(peakc-18000)<4000)
												agctemp=agctemp+1;
									else
											agctemp=agctemp+2;
								}	
			}
			}	
		}
else//原始波形
{	
			if(peakc>21000&&agctemp>8&&(modelindex==0||modelindex==3||modelindex==4))
			{
				if(abs(peakc-18000)<4000)
							agctemp=agctemp-1;
				else
						agctemp=agctemp-3;
			}

			else
			{	
		if(paramvaule[3][5]==0)//铝合金	
		{			
			 if(peakc<16000&&agctemp<paramvaule[3][6]&&(modelindex==0||modelindex==3||modelindex==4))
					{
						if(abs(peakc-18000)<4000)
									agctemp=agctemp+1;
						else
								agctemp=agctemp+3;
					}
					}
			else//钛合金
			{
			 if(peakc<16000&&agctemp<paramvaule[3][7]&&(modelindex==0||modelindex==3||modelindex==4))
					{
						if(abs(peakc-18000)<4000)
									agctemp=agctemp+1;
						else
								agctemp=agctemp+3;
					}

					}	
				}					
						
}

		

		

if(inposition==0)
{
	if(paramvaule[3][5]==0)//铝合金
	agctemp=180;
	else//钛合金
		agctemp=250;
}
/*
			if(peakc>24000||peakc<16000)
			{
				KP=0.001;
				KI=10;
				KD=0;
				T=0.020;
				e[2]=e[1];
				e[1]=e[0];	
				e[0]=20000-peakc;		
				Dagc=	KP*(1+T/KI+KD/T)*e[0]+KP*(1+2*KD/T)*e[1]+(KP*KD/T)*e[2];	
				if((agctemp+Dagc)>0&&(agctemp+Dagc)<340)
				agctemp=agctemp+Dagc;
			}
				*/
		}
	 if(calibrationindex==1)
			 {
				 if(calibrationcount<80)
				 {
				 paramvaule[0][3]=0;//原始波
					 if(calibrationcount>77)
							calibration();
				 }
				 else
				 {
				 paramvaule[0][3]=1;//包洛波
					 if(calibrationcount>157)
							calibration();
				 }
				 calibrationcount++;
				 if(calibrationcount>160)
				 {
				 calibrationindex=0;
				 paramvaule[0][3]=wavetempstore;
				dispageparament(LCD_COLOR_WHITE);	
				 }					 
			 }

if(lcd_flag==1)
 {	
	 keymun=w_buf[1];
	 if(keymun==15)
				countinuemun=0;
  
		        
			
	/*	
 
		if(draw==1)
		{		
				if(gongneng==0||gongneng==2)//测厚和应力功能的KP计算方法
				{
			if(paramvaule[3][15]==0)
			{
			       kp=(float)(paramvaule[0][1]-paramvaule[0][0])*100.0/477.0;
					lcdkp=(float)(paramvaule[0][1]-paramvaule[0][0])*100.0/240.0;
			}
					else
							kp=((float)paramvaule[0][1]*1000000/usedv-(float)paramvaule[0][0]*1000000/usedv)/477.0;
				}else if(gongneng==1)//探伤下面，只按照mm显示来计算KP
								kp=((float)paramvaule[1][1]*1000000/usedv-(float)paramvaule[1][0]*1000000/usedv)/477.0;
			if(hisindex==0)//没有在历史数据显示界面
				{
					if(scanindex==0)//不在扫描状态下
					{
						if(gongneng==0||gongneng==2)
						{
            if(paramvaule[0][3]==1)//包络显示
            {

						for(i=0;i<240;i++)//1.05寸屏的显示						
								LCDDISBUF[i]=CH1_BUF[(int)(i*lcdkp)+paramvaule[0][0]*100]/280;
													
            }
            else
            {

						for(i=0;i<240;i++)//1.05寸屏的显示						
								LCDDISBUF[i]=60+CH1_BUF[(int)(i*lcdkp)+paramvaule[0][0]*100]/420;							
            }
					}

					 }	
	}
	if(draw==1)
				{
	    	
if(lcddisindex==1)
{
	LCDwavedeploy();
					 lcdfirstcoordinate=calfirstwaveindex(disfirstcoordinate);
					 lcdsecondcoordinate=calsecondwaveindex(dissecondcoordinate);
	
					LCD_ShowWave (LCDDISBUF,LCDDISBUFPRE,YELLOW);
						LCDxcoordinate();
					LCD_firstwaveindex(lcdfirstcoordinate,prelcdfirstcoordinate);
					LCD_dissecondwaveindex(lcdsecondcoordinate,prelcdsecondcoordinate);
					prelcdfirstcoordinate=lcdfirstcoordinate;
					prelcdsecondcoordinate=lcdsecondcoordinate;
}
				}
				 
		}

if(scanindex!=1)
						disbat(avg_adc);
		 
	 */
  }	
//HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}
////////////////////////////////////////////////////////////
void LCD_shouBigMun_pix(uint16_t x,uint16_t y,uint32_t num,uint8_t len, uint8_t pix,uint8_t bit,uint32_t ARGB)
{
	uint8_t t,temp ;
	  uint32_t temp1,temp2;
  uint32_t color;
  color=BSP_LCD_GetTextColor();
temp1=num;
		  disdashuzi(x-pix*bit,y,0xa,ARGB);
		for(t=0;t<len;t++)
		{
	
		temp=temp1%10;
			temp1=temp1/10;
	
		if(t<bit)
		 disdashuzi(x-pix*t,y,temp,ARGB); 
		else if(t>=bit)
	 	 disdashuzi(x-pix*(t+1),y,temp,ARGB); 
	 	 
	
	 	  
		 }
	  
	BSP_LCD_SetTextColor(color);	
}
uint32_t LCD_Pow(uint16_t m,uint16_t n)
{
	uint32_t result=1;	 
	while(n--)result*=m;    
	return result;
}	
//显示带小数点的数字,高位为0,还是显示
//x,y:起点坐标
//num:数值(0~999999999);	 
//len:长度(即要显示的位数)
//pix :像素大小
// bit 小数点为啥
//void LCD_ShowChar(unsigned char X,unsigned char Y,char c)//8行6列  ,x 0-83 ,y 0-5
void LCD_ShowxNum_pix(uint16_t x,uint16_t y,uint32_t num,uint8_t len, uint8_t pix,uint8_t bit,uint32_t ARGB)
{  
	uint8_t t,temp ;

  uint32_t color;
  color=BSP_LCD_GetTextColor();
	BSP_LCD_SetTextColor(ARGB);	
	for(t=0;t<len;t++)
	{
		temp=(num/LCD_Pow(10,len-t-1))%10;
	//	if(t==0 && temp==0)
		// BSP_LCD_DisplayChar(x+pix*t,y,' ');	
		//else 
		if(t<bit)
		 BSP_LCD_DisplayChar(x+pix*t,y,temp+'0'); 
		else if(t>bit)
	 	 BSP_LCD_DisplayChar(x+pix*(t+1),y,temp+'0'); 
	 	 else 
		 { BSP_LCD_DisplayChar(x+pix*t,y,'.');
	 	   BSP_LCD_DisplayChar(x+pix*(t+1),y,temp+'0'); 
		 }
	 }  
	BSP_LCD_SetTextColor(color);	
}
//////////////////////////////////////////////////
//显示数字,高位为0,还是显示
//x,y:起点坐标
//num:数值(0~999999999);	 
//len:长度(即要显示的位数)
//void LCD_ShowChar(unsigned char X,unsigned char Y,char c)//8行6列  ,x 0-83 ,y 0-5
void LCD_ShowxNum(uint16_t x,uint16_t y,uint32_t num,uint8_t len,uint32_t ARGB)
{  
	uint8_t t,temp ;

  uint32_t color;
  color=BSP_LCD_GetTextColor();
	BSP_LCD_SetTextColor(ARGB);	
	for(t=0;t<len;t++)
	{
		
		temp=(num/LCD_Pow(10,len-t-1))%10;

	   BSP_LCD_DisplayChar(x+12*t,y,temp+'0'); 
	}
	BSP_LCD_SetTextColor(color);	
}

void LCD_ShowxNum1(uint16_t x,uint16_t y,uint32_t num,uint8_t len,uint32_t ARGB)
{  
	uint8_t t,temp ;

  uint32_t color;
  color=BSP_LCD_GetTextColor();
	BSP_LCD_SetTextColor(ARGB);	
	for(t=0;t<len;t++)
	{
		
		temp=(num/LCD_Pow(10,len-t-1))%10;
	   BSP_LCD_DisplayChar(x+12*t,y,temp+'0'); 
	}
	BSP_LCD_SetTextColor(color);	
}
//////////////////////////////////////////////////
void Start_Init (void)
{  int16_t delay_time=200;
   BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	 BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
	 BSP_LCD_SetFont(&Font24_1);
	 BSP_LCD_DisplayStringAtLine(0, (uint8_t*)"Starting: >");
	 HAL_Delay(delay_time);
	 BSP_LCD_DisplayStringAtLine(0, (uint8_t*)"Starting: > > ");
	 HAL_Delay(delay_time);
   BSP_LCD_DisplayStringAtLine(0, (uint8_t*)"Starting: > > >");
	 HAL_Delay(delay_time);
   BSP_LCD_DisplayStringAtLine(0, (uint8_t*)"Starting: > > > >");
	 HAL_Delay(delay_time);
	 BSP_LCD_DisplayStringAtLine(0, (uint8_t*)"Starting: > > > > >");
	 HAL_Delay(delay_time);
	 BSP_LCD_DisplayStringAtLine(0, (uint8_t*)"Starting: > > > > > >");
	 HAL_Delay(delay_time);
	 BSP_LCD_DisplayStringAtLine(0, (uint8_t*)"Starting: > > > > > > >");
	  HAL_Delay(delay_time);
	 BSP_LCD_DisplayStringAtLine(0, (uint8_t*)"Starting: > > > > > > > >");
	 HAL_Delay(delay_time);
	 BSP_LCD_DisplayStringAtLine(0, (uint8_t*)"Starting: > > > > > > > > >");
	 HAL_Delay(delay_time);
	 BSP_LCD_DisplayStringAtLine(0, (uint8_t*)"Starting: > > > > > > > > > >");
	 HAL_Delay(delay_time);
	 BSP_LCD_DisplayStringAtLine(0, (uint8_t*)"Starting: > > > > > > > > > > >");
	 HAL_Delay(delay_time);
	 BSP_LCD_DisplayStringAtLine(0, (uint8_t*)"Starting: > > > > > > > > > > > >");
	 HAL_Delay(delay_time);
	 BSP_LCD_DisplayStringAtLine(0, (uint8_t*)"Starting: > > > > > > > > > > > > >");
	 HAL_Delay(delay_time);
	 BSP_LCD_DisplayStringAtLine(0, (uint8_t*)"Starting: > > > > > > > > > > > > > >");	 
	 HAL_Delay(delay_time);
	 BSP_LCD_DisplayStringAtLine(0, (uint8_t*)"Starting: > > > > > > > > > > > > > > >");	 
	 HAL_Delay(delay_time);
	 BSP_LCD_DisplayStringAtLine(0, (uint8_t*)"Starting: > > > > > > > > > > > > > > > >");	 
	 HAL_Delay(delay_time);
	 BSP_LCD_DisplayStringAtLine(0, (uint8_t*)"Starting: > > > > > > > > > > > > > > > > >");
	 HAL_Delay(delay_time);
   BSP_LCD_DisplayStringAtLine(0, (uint8_t*)"Starting: > > > > > > > > > > > > > > > > > > >");
   HAL_Delay(delay_time);
   BSP_LCD_DisplayStringAtLine(0, (uint8_t*)"Starting: > > > > > > > > > > > > > > > > > > > >");
   HAL_Delay(delay_time);
   BSP_LCD_DisplayStringAtLine(0, (uint8_t*)"Starting: > > > > > > > > > > > > > > > > > > > > >");
   HAL_Delay(delay_time);
   BSP_LCD_DisplayStringAtLine(0, (uint8_t*)"Starting: > > > > > > > > > > > > > > > > > > > > > >");
	 HAL_Delay(1200);
	 BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
	// BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
	 BSP_LCD_SetFont(&Font24_1);
	 BSP_LCD_DisplayStringAtLine(6, (uint8_t*)"      System initialization completed! ");
	 BSP_LCD_DisplayStringAtLine(8, (uint8_t*)"             Version number: ARM [001A] "); 
	 BSP_LCD_DisplayStringAtLine(9, (uint8_t*)"             Version number: FPGA[001B] "); 
	 HAL_Delay(4400);
 }

 void disdashuzi(uint16_t xpos,uint16_t ypos,uint16_t mun,uint32_t color)//xianshi hanzi 24*24
 {
 char x,y;
  for(y=0;y<64;y++)
    {
          for(x=0;x<8;x++)
          {
              if(((dashuzi[mun][y*4]>>(7-x))&0x0001)==0)
								BSP_LCD_DrawPixel(xpos+x,ypos+y ,LCD_COLOR_BLACK);
              else
                BSP_LCD_DrawPixel(xpos+x, ypos+y , color);
          }
          for(x=0;x<8;x++)
          {
              if(((dashuzi[mun][y*4+1]>>(7-x))&0x0001)==0)
								BSP_LCD_DrawPixel(xpos+x+8,ypos+y ,LCD_COLOR_BLACK);
              else
                BSP_LCD_DrawPixel(xpos+x+8, ypos+y , color);
          }	
          for(x=0;x<8;x++)
          {
              if(((dashuzi[mun][y*4+2]>>(7-x))&0x0001)==0)
								BSP_LCD_DrawPixel(xpos+x+16,ypos+y ,LCD_COLOR_BLACK);
              else
                BSP_LCD_DrawPixel(xpos+x+16, ypos+y , color);
          }
          for(x=0;x<8;x++)
          {
              if(((dashuzi[mun][y*4+3]>>(7-x))&0x0001)==0)
								BSP_LCD_DrawPixel(xpos+x+24,ypos+y ,LCD_COLOR_BLACK);
              else
                BSP_LCD_DrawPixel(xpos+x+24, ypos+y , color);
          }						
   } 
 
 }
/////////////////////////////////////////////////
void dishanzi(uint16_t xpos,uint16_t ypos,uint16_t mun,uint32_t color)//xianshi hanzi 24*24
{
char x,y;
  for(y=0;y<24;y++)
    {
          for(x=0;x<8;x++)
          {
              if(((hanzi[mun][y*3]>>(7-x))&0x0001)==0)
								BSP_LCD_DrawPixel(xpos+x,ypos+y ,LCD_COLOR_BLACK);
              else
                BSP_LCD_DrawPixel(xpos+x, ypos+y , color);
          }
          for(x=0;x<8;x++)
          {
              if(((hanzi[mun][y*3+1]>>(7-x))&0x0001)==0)
								BSP_LCD_DrawPixel(xpos+x+8,ypos+y ,LCD_COLOR_BLACK);
              else
                BSP_LCD_DrawPixel(xpos+x+8, ypos+y , color);
          }	
          for(x=0;x<8;x++)
          {
              if(((hanzi[mun][y*3+2]>>(7-x))&0x0001)==0)
								BSP_LCD_DrawPixel(xpos+x+16,ypos+y ,LCD_COLOR_BLACK);
              else
                BSP_LCD_DrawPixel(xpos+x+16, ypos+y , color);
          }						
   }        		
}
//////////////////////////////////////////////////////////////////////
void drawkeybroad(uint16_t Xpos,uint16_t Ypos)
{
	uint32_t color;
	color=BSP_LCD_GetTextColor();
if(gongneng==0&&scanindex==1&&paramentmun==16&& timemodify==0)
{
	BSP_LCD_DrawLine(Xpos+5,Ypos+5,Xpos+105,Ypos+5);
BSP_LCD_DrawLine(Xpos+5,Ypos+5,Xpos+5,Ypos+85);
BSP_LCD_DrawLine(Xpos+105,Ypos+5,Xpos+105,Ypos+85);	
BSP_LCD_DrawLine(Xpos+5,Ypos+85,Xpos+105,Ypos+85);
					  dishanzi(Xpos+10,Ypos+10,155,LCD_COLOR_YELLOW);
						dishanzi(Xpos+40,Ypos+10,156,LCD_COLOR_YELLOW);
						dishanzi(Xpos+70,Ypos+10,157,LCD_COLOR_YELLOW);	//右扫描
						dishanzi(Xpos+10,Ypos+50,154,LCD_COLOR_YELLOW);
						dishanzi(Xpos+40,Ypos+50,156,LCD_COLOR_YELLOW);
						dishanzi(Xpos+70,Ypos+50,157,LCD_COLOR_YELLOW);//左扫描
}
	else
if(gongneng==0&&paramentmun==3&&page==1&& timemodify==0)
{
	drawblock(Xpos,Ypos,77,85);
BSP_LCD_DrawLine(Xpos+5,Ypos+5,Xpos+77,Ypos+5);
BSP_LCD_DrawLine(Xpos+5,Ypos+5,Xpos+5,Ypos+85);
BSP_LCD_DrawLine(Xpos+77,Ypos+5,Xpos+77,Ypos+85);	
BSP_LCD_DrawLine(Xpos+5,Ypos+85,Xpos+77,Ypos+85);	

	dishanzi(Xpos+10,Ypos+10,121,LCD_COLOR_WHITE); //原始
	dishanzi(Xpos+40,Ypos+10,122,LCD_COLOR_WHITE);	
	dishanzi(Xpos+10,   Ypos+50,119,LCD_COLOR_WHITE); //包络
	dishanzi(Xpos+40,Ypos+50,120,LCD_COLOR_WHITE);
}
else if((gongneng==0&&paramentmun==10&&page==1&& timemodify==0)||(gongneng==1&&paramentmun==8))
{
	drawblock(Xpos,Ypos,130,285);
BSP_LCD_DrawLine(Xpos+5,Ypos+5,Xpos+130,Ypos+5);
BSP_LCD_DrawLine(Xpos+5,Ypos+5,Xpos+5,Ypos+285);
BSP_LCD_DrawLine(Xpos+130,Ypos+5,Xpos+130,Ypos+285);	
BSP_LCD_DrawLine(Xpos+5,Ypos+285,Xpos+130,Ypos+285);	

	dishanzi(Xpos+10,Ypos+10,141,LCD_COLOR_WHITE); //碳钢
	dishanzi(Xpos+40,Ypos+10,103,LCD_COLOR_WHITE);	
	dishanzi(Xpos+10,   Ypos+50,104,LCD_COLOR_WHITE); //合金钢
	dishanzi(Xpos+40,Ypos+50,105,LCD_COLOR_WHITE);
	dishanzi(Xpos+70,Ypos+50,103,LCD_COLOR_WHITE);	
	dishanzi(Xpos+10,   Ypos+90,142,LCD_COLOR_WHITE); //铜
	dishanzi(Xpos+10,   Ypos+130,143,LCD_COLOR_WHITE); //铝
	dishanzi(Xpos+10,   Ypos+170,144,LCD_COLOR_WHITE); //不锈钢
	dishanzi(Xpos+40,Ypos+170,145,LCD_COLOR_WHITE);
	dishanzi(Xpos+70,Ypos+170,103,LCD_COLOR_WHITE);	
	dishanzi(Xpos+10,   Ypos+210,150,LCD_COLOR_WHITE); //球磨铸铁
	dishanzi(Xpos+40,Ypos+210,151,LCD_COLOR_WHITE);
	dishanzi(Xpos+70,Ypos+210,152,LCD_COLOR_WHITE);
	dishanzi(Xpos+100,Ypos+210,153,LCD_COLOR_WHITE);	
	dishanzi(Xpos+10,   Ypos+250,61,LCD_COLOR_WHITE); //自定义
	dishanzi(Xpos+40,Ypos+250,95,LCD_COLOR_WHITE);
	dishanzi(Xpos+70,Ypos+250,112,LCD_COLOR_WHITE);	
}
else
if(gongneng==0&&paramentmun==16&&page==1&& timemodify==0)
{
	drawblock(Xpos,Ypos,77,85);
BSP_LCD_DrawLine(Xpos+5,Ypos+5,Xpos+77,Ypos+5);
BSP_LCD_DrawLine(Xpos+5,Ypos+5,Xpos+5,Ypos+85);
BSP_LCD_DrawLine(Xpos+77,Ypos+5,Xpos+77,Ypos+85);	
BSP_LCD_DrawLine(Xpos+5,Ypos+85,Xpos+77,Ypos+85);	

	dishanzi(Xpos+10,Ypos+10,175,LCD_COLOR_WHITE); //低频
	dishanzi(Xpos+40,Ypos+10,52,LCD_COLOR_WHITE);
	dishanzi(Xpos+10,Ypos+50,174,LCD_COLOR_WHITE); //高频
	dishanzi(Xpos+40,Ypos+50,52,LCD_COLOR_WHITE);
}
else
if(gongneng==0&&paramentmun==12&&page==1&& timemodify==0)
{
	drawblock(Xpos,Ypos,77,85);
BSP_LCD_DrawLine(Xpos+5,Ypos+5,Xpos+77,Ypos+5);
BSP_LCD_DrawLine(Xpos+5,Ypos+5,Xpos+5,Ypos+85);
BSP_LCD_DrawLine(Xpos+77,Ypos+5,Xpos+77,Ypos+85);	
BSP_LCD_DrawLine(Xpos+5,Ypos+85,Xpos+77,Ypos+85);	

BSP_LCD_DisplayStringAt(Xpos+10,Ypos+10,(uint8_t*)" P-P ", LEFT_MODE);
BSP_LCD_DisplayStringAt(Xpos+10,Ypos+50,(uint8_t*)" Z-P ", LEFT_MODE);
}
else
if(gongneng==0&&paramentmun==11&&page==1&& timemodify==0)
{
	drawblock(Xpos,Ypos,77,285);
BSP_LCD_DrawLine(Xpos+5,Ypos+5,Xpos+77,Ypos+5);
BSP_LCD_DrawLine(Xpos+5,Ypos+5,Xpos+5,Ypos+285);
BSP_LCD_DrawLine(Xpos+77,Ypos+5,Xpos+77,Ypos+285);	
BSP_LCD_DrawLine(Xpos+5,Ypos+285,Xpos+77,Ypos+285);	
BSP_LCD_DisplayStringAt(Xpos+10,Ypos+10,(uint8_t*)" 1  ", LEFT_MODE);
BSP_LCD_DisplayStringAt(Xpos+10,Ypos+50,(uint8_t*)" 2  ", LEFT_MODE);
BSP_LCD_DisplayStringAt(Xpos+10,Ypos+90,(uint8_t*)" 4  ", LEFT_MODE);
BSP_LCD_DisplayStringAt(Xpos+10,Ypos+130,(uint8_t*)" 8  ", LEFT_MODE);	
BSP_LCD_DisplayStringAt(Xpos+10,Ypos+170,(uint8_t*)" 16 ", LEFT_MODE);
BSP_LCD_DisplayStringAt(Xpos+10,Ypos+210,(uint8_t*)" 32 ", LEFT_MODE);
BSP_LCD_DisplayStringAt(Xpos+10,Ypos+250,(uint8_t*)" 64 ", LEFT_MODE);
	

}
	else{
		drawblock(Xpos,Ypos,328,318);
	BSP_LCD_SetTextColor(LCD_COLOR_CYAN);
BSP_LCD_DrawLine(Xpos,Ypos,Xpos+320,Ypos);
BSP_LCD_DrawLine(Xpos,Ypos+80,Xpos+320,Ypos+80);
BSP_LCD_DrawLine(Xpos,Ypos+160,Xpos+320,Ypos+160);
BSP_LCD_DrawLine(Xpos,Ypos+240,Xpos+320,Ypos+240);
BSP_LCD_DrawLine(Xpos,Ypos+317,Xpos+320,Ypos+317);
BSP_LCD_DrawLine(Xpos,Ypos,Xpos,Ypos+318);
BSP_LCD_DrawLine(Xpos+80,Ypos+80,Xpos+80,Ypos+318);
	BSP_LCD_DrawLine(Xpos+160,Ypos,Xpos+160,Ypos+318);
	BSP_LCD_DrawLine(Xpos+240,Ypos,Xpos+240,Ypos+318);
	BSP_LCD_DrawLine(Xpos+320,Ypos,Xpos+320,Ypos+317);
	BSP_LCD_SetFont(&Font48);
		BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
		//第一行
		dishanzi(Xpos+178,Ypos+8,94,LCD_COLOR_WHITE);
dishanzi(Xpos+178,Ypos+48,95,LCD_COLOR_WHITE);//确定
dishanzi(Xpos+258,Ypos+8,44,LCD_COLOR_WHITE);
dishanzi(Xpos+258,Ypos+48,149,LCD_COLOR_WHITE);//清除
		//第二行
LCD_ShowxNum_pix(Xpos+22,Ypos+96,0,1,24,1,LCD_COLOR_WHITE);
LCD_ShowxNum_pix(Xpos+102,Ypos+96,1,1,24,1,LCD_COLOR_WHITE);	
LCD_ShowxNum_pix(Xpos+182,Ypos+96,2,1,24,1,LCD_COLOR_WHITE);
LCD_ShowxNum_pix(Xpos+262,Ypos+96,3,1,24,1,LCD_COLOR_WHITE);
		//第三行
LCD_ShowxNum_pix(Xpos+22,Ypos+176,4,1,24,1,LCD_COLOR_WHITE);
LCD_ShowxNum_pix(Xpos+102,Ypos+176,5,1,24,1,LCD_COLOR_WHITE);	
LCD_ShowxNum_pix(Xpos+182,Ypos+176,6,1,24,1,LCD_COLOR_WHITE);
LCD_ShowxNum_pix(Xpos+262,Ypos+176,7,1,24,1,LCD_COLOR_WHITE);
		//第四行
LCD_ShowxNum_pix(Xpos+22,Ypos+256,8,1,24,1,LCD_COLOR_WHITE);
LCD_ShowxNum_pix(Xpos+102,Ypos+256,9,1,24,1,LCD_COLOR_WHITE);	
BSP_LCD_DisplayChar(Xpos+182,Ypos+256,'.');
dishanzi(Xpos+258,Ypos+248,170,LCD_COLOR_WHITE);
dishanzi(Xpos+258,Ypos+288,171,LCD_COLOR_WHITE);//返回
}
	BSP_LCD_SetTextColor(color);
	BSP_LCD_SetFont(&Font24_1);
BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
}

void drawmainbroad(void)
{
		uint32_t color;
	color=BSP_LCD_GetTextColor();
	BSP_LCD_SetTextColor(LCD_COLOR_LIGHTMAGENTA);



	if(gongneng==0)
	{
	drawcolorblock(1,481,118,64,LCD_COLOR_LIGHTGREEN);
	drawblock(18,494,83,36);
	dishanzi(28,500,127,LCD_COLOR_ORANGE);
	dishanzi(68,500,5,LCD_COLOR_ORANGE);//测厚
	}
	else if(gongneng==1)
	{
	drawcolorblock(1,481,118,64,LCD_COLOR_LIGHTGREEN);
	drawblock(18,494,83,36);
	dishanzi(28,500,22,LCD_COLOR_ORANGE);
	dishanzi(68,500,169,LCD_COLOR_ORANGE);//探伤
	}
	else if(gongneng==2)
	{
		drawcolorblock(1,481,118,64,LCD_COLOR_LIGHTGREEN);
		drawblock(18,494,83,36);
	 dishanzi(28,500,128,LCD_COLOR_ORANGE);
	dishanzi(68,500,129,LCD_COLOR_ORANGE);//应力
	}
	BSP_LCD_SetFont(&Font24_1);
if(pause==1)
{
dishanzi(148,500,177,LCD_COLOR_RED);
dishanzi(188,500,178,LCD_COLOR_RED);	//冻结
}
else
{
dishanzi(148,500,177,LCD_COLOR_LIGHTMAGENTA);
dishanzi(188,500,178,LCD_COLOR_LIGHTMAGENTA);	//冻结
}	
if(gongneng==0)
{
	if(modelindex==0)
	{

drawblock(8,550,110,60);
			if(paramvaule[0][3]==0)//原始波形
			{
dishanzi(10,550,121,LCD_COLOR_LIGHTMAGENTA);//原始
dishanzi(45,550,122,LCD_COLOR_LIGHTMAGENTA);	
			}
			else
			{
dishanzi(10,550,119,LCD_COLOR_LIGHTMAGENTA);
dishanzi(45,550,120,LCD_COLOR_LIGHTMAGENTA);	//包洛			
			
			}
dishanzi(80,550,123,LCD_COLOR_LIGHTMAGENTA);	//波		
dishanzi(28,580,61,LCD_COLOR_LIGHTMAGENTA);
dishanzi(68,580,47,LCD_COLOR_LIGHTMAGENTA);	//自动	
		
	}
	else	if(modelindex==3)
	{
drawblock(8,550,110,60);
dishanzi(28,565,184,LCD_COLOR_LIGHTMAGENTA);
dishanzi(68,565,185,LCD_COLOR_LIGHTMAGENTA);			//腐蚀	
		
	}
	else if(modelindex==1)
	{
drawblock(8,550,110,60);		
dishanzi(22,565,46,LCD_COLOR_LIGHTMAGENTA);
dishanzi(57,565,47,LCD_COLOR_LIGHTMAGENTA);	//手动	1
BSP_LCD_DisplayStringAt(88,565,(uint8_t*)"1", LEFT_MODE);		
	}
		else if(modelindex==2)
	{
drawblock(8,550,110,60);
dishanzi(22,565,46,LCD_COLOR_LIGHTMAGENTA);
dishanzi(57,565,47,LCD_COLOR_LIGHTMAGENTA);	//手动	2
BSP_LCD_DisplayStringAt(88,565,(uint8_t*)"2", LEFT_MODE);		
	}
			else if(modelindex==4)
	{
drawblock(8,550,110,60);
dishanzi(22,565,186,LCD_COLOR_LIGHTMAGENTA);
dishanzi(57,565,102,LCD_COLOR_LIGHTMAGENTA);	//薄板
	
	}
	if(scanindex==1)
BSP_LCD_DisplayStringAt(140,565,(uint8_t*)"B", LEFT_MODE);
	else
BSP_LCD_DisplayStringAt(140,565,(uint8_t*)"A", LEFT_MODE);		
dishanzi(163,565,156,LCD_COLOR_LIGHTMAGENTA);
dishanzi(200,565,157,LCD_COLOR_LIGHTMAGENTA);	//扫描


dishanzi(268,500,166,LCD_COLOR_LIGHTMAGENTA);
dishanzi(308,500,167,LCD_COLOR_LIGHTMAGENTA);//查询
	
dishanzi(268,565,179,LCD_COLOR_LIGHTMAGENTA);
dishanzi(308,565,135,LCD_COLOR_LIGHTMAGENTA);//保存

	
dishanzi(388,565,63,LCD_COLOR_LIGHTMAGENTA);
dishanzi(428,565,64,LCD_COLOR_LIGHTMAGENTA);//校准
}
else 
{
		BSP_LCD_SetTextColor(LCD_COLOR_GRAY);
		if(modelindex==0)
	{

drawblock(8,550,110,60);
			if(paramvaule[0][3]==0)//原始波形
			{
dishanzi(10,550,121,LCD_COLOR_GRAY);//原始
dishanzi(45,550,122,LCD_COLOR_GRAY);	
			}
			else
			{
dishanzi(10,550,119,LCD_COLOR_GRAY);
dishanzi(45,550,120,LCD_COLOR_GRAY);	//包洛			
			
			}
dishanzi(80,550,123,LCD_COLOR_GRAY);	//波		
dishanzi(28,580,61,LCD_COLOR_GRAY);
dishanzi(68,580,47,LCD_COLOR_GRAY);	//自动	
		
	}
	else	if(modelindex==3)
	{
drawblock(8,550,110,60);
dishanzi(28,565,184,LCD_COLOR_GRAY);
dishanzi(68,565,185,LCD_COLOR_GRAY);			//腐蚀	
		
	}
	else if(modelindex==1)
	{
drawblock(8,550,110,60);		
dishanzi(22,565,46,LCD_COLOR_GRAY);
dishanzi(57,565,47,LCD_COLOR_GRAY);	//手动	1
BSP_LCD_DisplayStringAt(88,565,(uint8_t*)"1", LEFT_MODE);		
	}
		else if(modelindex==2)
	{
drawblock(8,550,110,60);
dishanzi(22,565,46,LCD_COLOR_GRAY);
dishanzi(57,565,47,LCD_COLOR_GRAY);	//手动	2
BSP_LCD_DisplayStringAt(88,565,(uint8_t*)"2", LEFT_MODE);		
	}
	BSP_LCD_SetTextColor(LCD_COLOR_GRAY);
	
	if(scanindex==1)
BSP_LCD_DisplayStringAt(140,565,(uint8_t*)"B", LEFT_MODE);
	else
BSP_LCD_DisplayStringAt(140,565,(uint8_t*)"A", LEFT_MODE);		
dishanzi(163,565,156,LCD_COLOR_GRAY);
dishanzi(200,565,157,LCD_COLOR_GRAY);	//扫描


dishanzi(268,500,166,LCD_COLOR_GRAY);
dishanzi(308,500,167,LCD_COLOR_GRAY);//查询
	
dishanzi(268,565,179,LCD_COLOR_GRAY);
dishanzi(308,565,135,LCD_COLOR_GRAY);//保存

dishanzi(388,565,63,LCD_COLOR_GRAY);
dishanzi(428,565,64,LCD_COLOR_GRAY);//校准
}
BSP_LCD_SetTextColor(color);
}
void diskeymun(uint16_t Xpos,uint16_t Ypos)//显示键盘按下的数字
{
	BSP_LCD_SetFont(&Font24_1);
		BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
LCD_ShowxNum_pix(Xpos+26,Ypos+10,paramentmodifyval,1,24,1,LCD_COLOR_WHITE);
}
void distupian(uint16_t xpos,uint16_t ypos,uint32_t color)//xianshi hanzi 24*24
{
char x,y;
  for(y=0;y<104;y++)
    {
          for(x=0;x<8;x++)
          {
              if(((tupian[y*3]>>(7-x))&0x0001)==0)
								BSP_LCD_DrawPixel(xpos+x,ypos+y ,LCD_COLOR_BLACK);
              else
                BSP_LCD_DrawPixel(xpos+x, ypos+y , color);
          }
          for(x=0;x<8;x++)
          {
              if(((tupian[y*3+1]>>(7-x))&0x0001)==0)
								BSP_LCD_DrawPixel(xpos+x+8,ypos+y ,LCD_COLOR_BLACK);
              else
                BSP_LCD_DrawPixel(xpos+x+8, ypos+y , color);
          }	
          for(x=0;x<8;x++)
          {
              if(((tupian[y*3+2]>>(7-x))&0x0001)==0)
								BSP_LCD_DrawPixel(xpos+x+16,ypos+y ,LCD_COLOR_BLACK);
              else
                BSP_LCD_DrawPixel(xpos+x+16, ypos+y , color);
          }		
       for(x=0;x<8;x++)
          {
              if(((tupian[y*3+4]>>(7-x))&0x0001)==0)
								BSP_LCD_DrawPixel(xpos+x+16,ypos+y ,LCD_COLOR_BLACK);
              else
                BSP_LCD_DrawPixel(xpos+x+16, ypos+y , color);
          }	
       for(x=0;x<8;x++)
          {
              if(((tupian[y*3+5]>>(7-x))&0x0001)==0)
								BSP_LCD_DrawPixel(xpos+x+16,ypos+y ,LCD_COLOR_BLACK);
              else
                BSP_LCD_DrawPixel(xpos+x+16, ypos+y , color);
          }		
       for(x=0;x<8;x++)
          {
              if(((tupian[y*3+6]>>(7-x))&0x0001)==0)
								BSP_LCD_DrawPixel(xpos+x+16,ypos+y ,LCD_COLOR_BLACK);
              else
                BSP_LCD_DrawPixel(xpos+x+16, ypos+y , color);
          }							
   }        		
}
void dishisdeploy(void)//显示历史数据界面
{
	uint32_t color;
	color=BSP_LCD_GetTextColor();
	BSP_LCD_SetTextColor(LCD_COLOR_CYAN);
		drawblock(0,481,479,318);//清除显示区域
	HAL_Delay(30);
BSP_LCD_DrawLine(1,481,1,799);
	BSP_LCD_DrawLine(0,799,479,799);
	BSP_LCD_DrawLine(479,481,479,799);//画外框
BSP_LCD_DrawLine(0,520,360,520);
BSP_LCD_DrawLine(0,560,360,560);
BSP_LCD_DrawLine(0,600,360,600);
	BSP_LCD_DrawLine(0,640,360,640);
	BSP_LCD_DrawLine(0,680,360,680);
	BSP_LCD_DrawLine(0,720,479,720);
		BSP_LCD_DrawLine(0,760,479,760);//画横线
		BSP_LCD_DrawLine(120,480,120,799);
		BSP_LCD_DrawLine(240,480,240,799);
		BSP_LCD_DrawLine(360,480,360,799);
dishanzi(380,725,170,LCD_COLOR_WHITE);
dishanzi(420,725,171,LCD_COLOR_WHITE);//返回	
dishanzi(380,765,148,LCD_COLOR_WHITE);
dishanzi(420,765,149,LCD_COLOR_WHITE);//删除	
				disarrowup(420,485);
			disarrowdown(420,635);
	BSP_LCD_SetTextColor(color);
}

void disarrowup(uint16_t x,uint16_t y)
{
BSP_LCD_DrawLine(x,y,x-25,y+50);//顶点260，520
BSP_LCD_DrawLine(x,y,x+25,y+50);	
BSP_LCD_DrawLine(x-25,y+50,x+25,y+50);
BSP_LCD_DrawLine(x-15,y+50,x-15,y+80);	
BSP_LCD_DrawLine(x+15,y+50,x+15,y+80);	
BSP_LCD_DrawLine(x-15,y+80,x+15,y+80);		
}

void disarrowdown(uint16_t x,uint16_t y)
{
BSP_LCD_DrawLine(x,y,x+25,y-50);//顶点260，520
BSP_LCD_DrawLine(x,y,x-25,y-50);	
BSP_LCD_DrawLine(x+25,y-50,x-25,y-50);
BSP_LCD_DrawLine(x+15,y-50,x+15,y-80);	
BSP_LCD_DrawLine(x-15,y-50,x-15,y-80);	
BSP_LCD_DrawLine(x+15,y-80,x-15,y-80);		
}
void disdeploy(void) //显示外框
{uint32_t color;
	color=BSP_LCD_GetTextColor();
	BSP_LCD_SetTextColor(LCD_COLOR_CYAN);
BSP_LCD_DrawLine(1,1,480,1);
BSP_LCD_DrawLine(1,1,1,800);
		BSP_LCD_DrawLine(1,799,480,799);
//功能区划线
	BSP_LCD_DrawLine(1,34,480,34);//画横线
BSP_LCD_DrawLine(1,610,480,610);//画横线
BSP_LCD_DrawLine(1,545,480,545);//画横线
BSP_LCD_DrawLine(1,480,480,480);//画横线
	BSP_LCD_DrawLine(120,480,120,610);
	BSP_LCD_DrawLine(240,480,240,610);
	BSP_LCD_DrawLine(360,480,360,610);//竖线
	//
	//参数区域

BSP_LCD_DrawLine(1,610,319,610);//画横线	
BSP_LCD_DrawLine(1,673,319,673);//画横线
BSP_LCD_DrawLine(1,736,319,736);//画横线	
BSP_LCD_DrawLine(55,610,55,799);
BSP_LCD_DrawLine(159,610,159,799);
BSP_LCD_DrawLine(214,610,214,799);
BSP_LCD_DrawLine(319,610,319,799);//竖线
BSP_LCD_DrawLine(480,800,480,1);
		
	

	
	drawcolorblock(361,481,118,63,LCD_COLOR_CYAN);
	//waikuan
	if(scanindex==0)
	{
BSP_LCD_DrawLine(1,113,480,113);
//BSP_LCD_DrawLine(124,0,124,113);
BSP_LCD_DrawLine(353,34,353,113);		
	}

	//上下翻页的标识
	if(hisindex==0&&(gongneng==0||gongneng==1))
	{
	disarrowup(400,620);
	disarrowdown(400,790);
	}
	else if(hisindex==1)
			{
	//		disarrowup(420,490);
	//		disarrowdown(420,720);
			}
	//扫描显示框
	
	BSP_LCD_SetTextColor(LCD_COLOR_ORANGE);
			BSP_LCD_SetFont(&Font24);
			
BSP_LCD_SetTextColor(color);	
}

 void disscandata(void)
 {
 BSP_LCD_SetFont(&Font24_1);
	 if(SCAN_BUF[19999]<10000&&SCAN_BUF[19999]>0)
	 {	 
		LCD_ShowxNum(499 ,279,0,3,LCD_COLOR_WHITE);	
		LCD_ShowxNum(499 ,309,(10000-SCAN_BUF[19999])*2/40 ,3,LCD_COLOR_WHITE);
		LCD_ShowxNum(499 ,339,(10000-SCAN_BUF[19999])*4/40 ,3,LCD_COLOR_WHITE);			
		LCD_ShowxNum(499 ,369,(10000-SCAN_BUF[19999])*6/40 ,3,LCD_COLOR_WHITE);			
		LCD_ShowxNum(499 ,398,(10000-SCAN_BUF[19999])*2/10 ,3,LCD_COLOR_WHITE);

		LCD_ShowxNum_pix(559 ,279,SCAN_BUF[9999]/10 ,5,12,3,LCD_COLOR_WHITE);	
		LCD_ShowxNum_pix(559 ,309,SCAN_BUF[10000-(10000-SCAN_BUF[19999])/4]/10 ,5,12,3,LCD_COLOR_WHITE);
		LCD_ShowxNum_pix(559 ,339,SCAN_BUF[10000-(10000-SCAN_BUF[19999])*2/4]/10,5,12,3,LCD_COLOR_WHITE);			
		LCD_ShowxNum_pix(559 ,369,SCAN_BUF[10000-(10000-SCAN_BUF[19999])*3/4]/10,5,12,3,LCD_COLOR_WHITE);			
		LCD_ShowxNum_pix(559 ,398,SCAN_BUF[10000-(10000-SCAN_BUF[19999])]/10,5,12,3,LCD_COLOR_WHITE);			 
	 }
	 else if(SCAN_BUF[19999]>=10000)
	 {
		LCD_ShowxNum(499 ,279,0,3,LCD_COLOR_WHITE);	
		LCD_ShowxNum(499 ,309,(SCAN_BUF[19999]-10000)*2/40 ,3,LCD_COLOR_WHITE);
		LCD_ShowxNum(499 ,339,(SCAN_BUF[19999]-10000)*4/40 ,3,LCD_COLOR_WHITE);			
		LCD_ShowxNum(499 ,369,(SCAN_BUF[19999]-10000)*6/40 ,3,LCD_COLOR_WHITE);			
		LCD_ShowxNum(499 ,398,(SCAN_BUF[19999]-10000)*2/10 ,3,LCD_COLOR_WHITE);

		LCD_ShowxNum_pix(559 ,279,SCAN_BUF[10001]/10 ,5,12,3,LCD_COLOR_WHITE);	
		LCD_ShowxNum_pix(559 ,309,SCAN_BUF[(SCAN_BUF[19999]-10000)/4+10000]/10 ,5,12,3,LCD_COLOR_WHITE);
		LCD_ShowxNum_pix(559 ,339,SCAN_BUF[(SCAN_BUF[19999]-10000)*2/4+10000]/10,5,12,3,LCD_COLOR_WHITE);			
		LCD_ShowxNum_pix(559 ,369,SCAN_BUF[(SCAN_BUF[19999]-10000)*3/4+10000]/10,5,12,3,LCD_COLOR_WHITE);			
		LCD_ShowxNum_pix(559 ,398,SCAN_BUF[(SCAN_BUF[19999]-10000)+10000]/10,5,12,3,LCD_COLOR_WHITE);			 
	 }
	 else if(SCAN_BUF[19999]==0)
		 	 {
		LCD_ShowxNum(499 ,279,0,3,LCD_COLOR_WHITE);	
		LCD_ShowxNum(499 ,309,0,3,LCD_COLOR_WHITE);
		LCD_ShowxNum(499 ,339,0,3,LCD_COLOR_WHITE);			
		LCD_ShowxNum(499 ,369,0,3,LCD_COLOR_WHITE);			
		LCD_ShowxNum(499 ,398,0,3,LCD_COLOR_WHITE);

		LCD_ShowxNum_pix(559 ,279,0,5,12,3,LCD_COLOR_WHITE);	
		LCD_ShowxNum_pix(559 ,309,0 ,5,12,3,LCD_COLOR_WHITE);
		LCD_ShowxNum_pix(559 ,339,0,5,12,3,LCD_COLOR_WHITE);			
		LCD_ShowxNum_pix(559 ,369,0,5,12,3,LCD_COLOR_WHITE);			
		LCD_ShowxNum_pix(559 ,398,0,5,12,3,LCD_COLOR_WHITE);			 
	 }
 }
 

void dismeun(uint16_t page1)
{uint32_t color;
	color=BSP_LCD_GetTextColor();
	BSP_LCD_SetFont(&Font24_1);
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
if(gongneng==0)
{
 if(scanindex==0&&hisindex==0)	
{
		if(page1==0)
		{
		BSP_LCD_DisplayStringAt(4,618,(uint8_t*)"    ", LEFT_MODE);
		BSP_LCD_DisplayStringAt(4,642,(uint8_t*)"    ", LEFT_MODE);	
		BSP_LCD_DisplayStringAt(4,683,(uint8_t*)"    ", LEFT_MODE);
		BSP_LCD_DisplayStringAt(4,707,(uint8_t*)"    ", LEFT_MODE);	
		BSP_LCD_DisplayStringAt(4,744,(uint8_t*)"    ", LEFT_MODE);
		BSP_LCD_DisplayStringAt(4,768,(uint8_t*)"    ", LEFT_MODE);				
		dishanzi(3,629,158,LCD_COLOR_RED);
		dishanzi(27,629,159,LCD_COLOR_RED);//范围

		if(paramvaule[3][15]==0)
			BSP_LCD_DisplayStringAt(60,629,(uint8_t*)"   us   ", LEFT_MODE);//xianshi:   us
			else
		BSP_LCD_DisplayStringAt(60,629,(uint8_t*)"     mm ", LEFT_MODE);//xianshi:   us	
			BSP_LCD_DisplayStringAt(161,629,(uint8_t*)" -- ", LEFT_MODE);
			if(paramvaule[3][15]==0)
			BSP_LCD_DisplayStringAt(219,629,(uint8_t*)"   us   ", LEFT_MODE);//xianshi:   us
			else
		BSP_LCD_DisplayStringAt(219,629,(uint8_t*)"     mm ", LEFT_MODE);//xianshi:   us	
		
	

		  dishanzi(4,692,78,LCD_COLOR_RED);
			dishanzi(28,692,79,LCD_COLOR_RED);//闸门
					if(paramvaule[3][15]==0)
				BSP_LCD_DisplayStringAt(60,692,(uint8_t*)"     us ", LEFT_MODE);//zhamen A:   us
			else
				BSP_LCD_DisplayStringAt(60,692,(uint8_t*)"     mm ", LEFT_MODE);//zhamen A:   mm	
			BSP_LCD_DisplayStringAt(161,692,(uint8_t*)" -- ", LEFT_MODE);//闸门
			if(paramvaule[3][15]==0)
				BSP_LCD_DisplayStringAt(219,692,(uint8_t*)"     us ", LEFT_MODE);//zhamen A:   us
			else
				BSP_LCD_DisplayStringAt(219,692,(uint8_t*)"     mm ", LEFT_MODE);//zhamen A:   mm	

			
			dishanzi(4,755,59,LCD_COLOR_RED);
			dishanzi(28,755,60,LCD_COLOR_RED);//增益	
		BSP_LCD_DisplayStringAt(60,755,(uint8_t*)"    dB  ", LEFT_MODE);	
			dishanzi(161,755,5,LCD_COLOR_RED);
			dishanzi(185,755,6,LCD_COLOR_RED);//厚度

			BSP_LCD_DisplayStringAt(219,755,(uint8_t*)"     mm ", LEFT_MODE);//houdu   mm
			


		//			dishanzi(4,768,45,LCD_COLOR_RED);
			//		dishanzi(28,768,62,LCD_COLOR_RED);
		//	BSP_LCD_DisplayStringAt(52,768,(uint8_t*)":  .   us", LEFT_MODE);//lingdian us
			
			BSP_LCD_SetTextColor(LCD_COLOR_CYAN);

		}
			else
				if(page1==1)
				{
					//		drawblock(2,611,476,188);
		BSP_LCD_DisplayStringAt(4,618,(uint8_t*)"    ", LEFT_MODE);
		BSP_LCD_DisplayStringAt(4,642,(uint8_t*)"    ", LEFT_MODE);	
		BSP_LCD_DisplayStringAt(4,683,(uint8_t*)"    ", LEFT_MODE);
		BSP_LCD_DisplayStringAt(4,707,(uint8_t*)"    ", LEFT_MODE);	
		BSP_LCD_DisplayStringAt(4,744,(uint8_t*)"    ", LEFT_MODE);
		BSP_LCD_DisplayStringAt(4,768,(uint8_t*)"    ", LEFT_MODE);						
						dishanzi(4,629,9,LCD_COLOR_RED);
					dishanzi(28,629,10,LCD_COLOR_RED);//材质
			BSP_LCD_DisplayStringAt(60,629,(uint8_t*)"        ", LEFT_MODE);
			
						dishanzi(161,629,80,LCD_COLOR_RED);
					dishanzi(185,629,81,LCD_COLOR_RED);//温度	
			BSP_LCD_DisplayStringAt(219,629,(uint8_t*)"        ", LEFT_MODE);
				//	drawblock(1,521,220,278);
					dishanzi(4,692,138,LCD_COLOR_RED);
					dishanzi(28,692,139,LCD_COLOR_RED);	//算法	
					BSP_LCD_DisplayStringAt(60,692,(uint8_t*)"        ", LEFT_MODE);					
					
					dishanzi(161,692,75,LCD_COLOR_RED);
					dishanzi(185,692,76,LCD_COLOR_RED);//caizhi
					BSP_LCD_DisplayStringAt(219,692,(uint8_t*)"        ", LEFT_MODE);//平滑
					
					dishanzi(4,755,123,LCD_COLOR_RED);
					dishanzi(28,755,124,LCD_COLOR_RED);
					BSP_LCD_DisplayStringAt(60,755,(uint8_t*)"        ", LEFT_MODE);//波形
					dishanzi(161,755,52,LCD_COLOR_RED);
					dishanzi(185,755,53,LCD_COLOR_RED);
					BSP_LCD_DisplayStringAt(219,755,(uint8_t*)"        ", LEFT_MODE);	//频率
					/////////////
		
				}
			if(page1==2)
			{
					//	drawblock(2,611,476,188);
		BSP_LCD_DisplayStringAt(4,618,(uint8_t*)"    ", LEFT_MODE);
		BSP_LCD_DisplayStringAt(4,642,(uint8_t*)"    ", LEFT_MODE);	
		BSP_LCD_DisplayStringAt(4,683,(uint8_t*)"    ", LEFT_MODE);
		BSP_LCD_DisplayStringAt(4,707,(uint8_t*)"    ", LEFT_MODE);
		BSP_LCD_DisplayStringAt(4,744,(uint8_t*)"    ", LEFT_MODE);
		BSP_LCD_DisplayStringAt(4,768,(uint8_t*)"    ", LEFT_MODE);					
					dishanzi(4,629,140,LCD_COLOR_RED);
					dishanzi(28,629,6,LCD_COLOR_RED);//亮度
					BSP_LCD_DisplayStringAt(60,629,(uint8_t*)"   %    ", LEFT_MODE);//	
					dishanzi(161,629,11,LCD_COLOR_RED);
					dishanzi(185,629,12,LCD_COLOR_RED);//声速
					BSP_LCD_DisplayStringAt(219,629,(uint8_t*)"      m", LEFT_MODE);//		
					dishanzi(4,692,45,LCD_COLOR_RED);
					dishanzi(28,692,62,LCD_COLOR_RED);//零点
	        BSP_LCD_DisplayStringAt(60,692,(uint8_t*)"     us ", LEFT_MODE);//lingdian us					
					BSP_LCD_DisplayStringAt(161,692,(uint8_t*)"    ", LEFT_MODE);
					BSP_LCD_DisplayStringAt(4,755,(uint8_t*)"    ", LEFT_MODE);
					BSP_LCD_DisplayStringAt(161,755,(uint8_t*)"    ", LEFT_MODE);					
			
					BSP_LCD_DisplayStringAt(219,692,(uint8_t*)"        ", LEFT_MODE);
					BSP_LCD_DisplayStringAt(60,755,(uint8_t*)"        ", LEFT_MODE);
					BSP_LCD_DisplayStringAt(219,755,(uint8_t*)"        ", LEFT_MODE);					
			}
				BSP_LCD_SetTextColor(color);
					}
else  if(scanindex==1&&hisindex==0)	//在测厚功能的扫描状态下
				{
							//		drawblock(2,611,476,188);
							
							dishanzi(4,618,156,LCD_COLOR_RED);
							dishanzi(28,618,157,LCD_COLOR_RED);				
							dishanzi(4,642,158,LCD_COLOR_RED);
							dishanzi(28,642,159,LCD_COLOR_RED);					
					BSP_LCD_DisplayStringAt(60,629,(uint8_t*)"    mm  ", LEFT_MODE);	//扫描范围:      mm										
							dishanzi(161,629,5,LCD_COLOR_RED);
							dishanzi(185,629,6,LCD_COLOR_RED);		
					BSP_LCD_DisplayStringAt(219,629,(uint8_t*)"      mm", LEFT_MODE);		//厚度:     mm	
							dishanzi(4,683,156,LCD_COLOR_RED);
							dishanzi(28,683,157,LCD_COLOR_RED);				
							dishanzi(4,707,27,LCD_COLOR_RED);
							dishanzi(28,707,28,LCD_COLOR_RED);					
					BSP_LCD_DisplayStringAt(60,692,(uint8_t*)"        ", LEFT_MODE);	//扫描方式: 
							dishanzi(161,692,29,LCD_COLOR_RED);
							dishanzi(185,692,30,LCD_COLOR_RED);		
					BSP_LCD_DisplayStringAt(219,692,(uint8_t*)"    mm  ", LEFT_MODE);		//间距:     mm	
							dishanzi(4,744,125,LCD_COLOR_RED);
							dishanzi(28,744,126,LCD_COLOR_RED);						
							dishanzi(4,768,5,LCD_COLOR_RED);
							dishanzi(28,768,6,LCD_COLOR_RED);		
					BSP_LCD_DisplayStringAt(60,755,(uint8_t*)"      mm", LEFT_MODE);		//显示厚度:     mm		
					BSP_LCD_DisplayStringAt(161,755,(uint8_t*)"    ", LEFT_MODE);	//去除没有显示的参数
					BSP_LCD_DisplayStringAt(219,755,(uint8_t*)"        ", LEFT_MODE);						
						if(scandirection==0)//右扫描
						{	
						dishanzi(60,692,155,LCD_COLOR_YELLOW);
						dishanzi(60+24,692,156,LCD_COLOR_YELLOW);
						dishanzi(60+48,692,157,LCD_COLOR_YELLOW);
						}
						else//左扫描
						{
						dishanzi(60,692,154,LCD_COLOR_YELLOW);
						dishanzi(60+24,692,156,LCD_COLOR_YELLOW);
						dishanzi(60+48,692,157,LCD_COLOR_YELLOW);	
						}					
				}
			}
else if(gongneng==1)//探伤界面显示
				{
		BSP_LCD_DisplayStringAt(4,618,(uint8_t*)"    ", LEFT_MODE);
		BSP_LCD_DisplayStringAt(4,642,(uint8_t*)"    ", LEFT_MODE);	
		BSP_LCD_DisplayStringAt(4,683,(uint8_t*)"    ", LEFT_MODE);
		BSP_LCD_DisplayStringAt(4,707,(uint8_t*)"    ", LEFT_MODE);
		BSP_LCD_DisplayStringAt(4,744,(uint8_t*)"    ", LEFT_MODE);
		BSP_LCD_DisplayStringAt(4,768,(uint8_t*)"    ", LEFT_MODE);	
	if(page1==0)	
	{		
					dishanzi(4,629,158,LCD_COLOR_RED);
					dishanzi(28,629,159,LCD_COLOR_RED);//范围			
					BSP_LCD_DisplayStringAt(161,629,(uint8_t*)" -- ", LEFT_MODE);
					BSP_LCD_DisplayStringAt(60,629,(uint8_t*)"   mm   ", LEFT_MODE);//
					BSP_LCD_DisplayStringAt(219,629,(uint8_t*)"   mm   ", LEFT_MODE);//				

					
							dishanzi(4,683,176,LCD_COLOR_RED);
							dishanzi(28,683,123,LCD_COLOR_RED);				
							dishanzi(4,707,78,LCD_COLOR_RED);
							dishanzi(28,707,79,LCD_COLOR_RED);					//底波闸门
					BSP_LCD_SetTextColor(LCD_COLOR_RED);
					BSP_LCD_DisplayStringAt(161,692,(uint8_t*)" -- ", LEFT_MODE);//闸门
					BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
					BSP_LCD_DisplayStringAt(60,692,(uint8_t*)"     mm ", LEFT_MODE);//
					BSP_LCD_DisplayStringAt(219,692,(uint8_t*)"     mm ", LEFT_MODE);//	
					
							dishanzi(4,744,169,LCD_COLOR_RED);
							dishanzi(28,744,123,LCD_COLOR_RED);						
							dishanzi(4,768,78,LCD_COLOR_RED);
							dishanzi(28,768,79,LCD_COLOR_RED);//伤波闸门
					BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
					BSP_LCD_DisplayStringAt(161,755,(uint8_t*)" -- ", LEFT_MODE);//闸门

					BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
					BSP_LCD_DisplayStringAt(60,755,(uint8_t*)"     mm ", LEFT_MODE);//
					BSP_LCD_DisplayStringAt(219,755,(uint8_t*)"     mm ", LEFT_MODE);//				
				} else if(page1==1)
				{
		BSP_LCD_DisplayStringAt(4,618,(uint8_t*)"    ", LEFT_MODE);
		BSP_LCD_DisplayStringAt(4,642,(uint8_t*)"    ", LEFT_MODE);	
		BSP_LCD_DisplayStringAt(4,683,(uint8_t*)"    ", LEFT_MODE);
		BSP_LCD_DisplayStringAt(4,707,(uint8_t*)"    ", LEFT_MODE);	
		BSP_LCD_DisplayStringAt(4,744,(uint8_t*)"    ", LEFT_MODE);
		BSP_LCD_DisplayStringAt(4,768,(uint8_t*)"    ", LEFT_MODE);	
					dishanzi(4,629,59,LCD_COLOR_RED);
					dishanzi(28,629,60,LCD_COLOR_RED);
					BSP_LCD_DisplayStringAt(60,629,(uint8_t*)"    dB  ", LEFT_MODE);//增益
					dishanzi(161,629,11,LCD_COLOR_RED);
					dishanzi(185,629,12,LCD_COLOR_RED);//声速
					BSP_LCD_DisplayStringAt(219,629,(uint8_t*)"      m ", LEFT_MODE);
			
					dishanzi(4,692,9,LCD_COLOR_RED);
					dishanzi(28,692,10,LCD_COLOR_RED);//caizhi
					BSP_LCD_DisplayStringAt(60,692,(uint8_t*)"        ", LEFT_MODE);//材质
					dishanzi(161,692,172,LCD_COLOR_RED);
					dishanzi(185,692,173,LCD_COLOR_RED);
					BSP_LCD_DisplayStringAt(219,692,(uint8_t*)"-    dB ", LEFT_MODE);//阈值
					BSP_LCD_DisplayStringAt(161,755,(uint8_t*)"    ", LEFT_MODE);					
					BSP_LCD_DisplayStringAt(60,755,(uint8_t*)"        ", LEFT_MODE);
					BSP_LCD_DisplayStringAt(219,755,(uint8_t*)"        ", LEFT_MODE);	
						}		
				}

}

void dissaved(uint16_t xpos,uint16_t ypos)
{
drawblock(xpos,ypos,150,60);
		BSP_LCD_DrawLine(xpos-1,ypos-1,xpos+150,ypos-1);
	BSP_LCD_DrawLine(xpos-1,ypos-1,xpos-1,ypos+60);
	BSP_LCD_DrawLine(xpos+150,ypos+60,xpos+150,ypos-1);
	BSP_LCD_DrawLine(xpos+150,ypos+60,xpos-1,ypos+60);
	if(scanindex==0)
	{
		if(ubNumberOftxtFiles<MAX_BMP_FILES)
		{
		dishanzi(xpos+15,ypos+25,135,LCD_COLOR_GREEN);
		dishanzi(xpos+45,ypos+25,136,LCD_COLOR_GREEN);	
		dishanzi(xpos+75,ypos+25,67,LCD_COLOR_GREEN);
		dishanzi(xpos+105,ypos+25,68,LCD_COLOR_GREEN);	//存储成功
		}
		else
		{
		dishanzi(xpos+15,ypos+25,135,LCD_COLOR_GREEN);
		dishanzi(xpos+45,ypos+25,136,LCD_COLOR_GREEN);	
		dishanzi(xpos+75,ypos+25,164,LCD_COLOR_GREEN);
		dishanzi(xpos+105,ypos+25,165,LCD_COLOR_GREEN);	//存储已满	
		}
   }
	else
	{
		if(ubNumberOfFiles<MAX_BMP_FILES)
		{
		dishanzi(xpos+15,ypos+25,135,LCD_COLOR_GREEN);
		dishanzi(xpos+45,ypos+25,136,LCD_COLOR_GREEN);	
		dishanzi(xpos+75,ypos+25,67,LCD_COLOR_GREEN);
		dishanzi(xpos+105,ypos+25,68,LCD_COLOR_GREEN);	//存储成功
		}
		else
		{
		dishanzi(xpos+15,ypos+25,135,LCD_COLOR_GREEN);
		dishanzi(xpos+45,ypos+25,136,LCD_COLOR_GREEN);	
		dishanzi(xpos+75,ypos+25,164,LCD_COLOR_GREEN);
		dishanzi(xpos+105,ypos+25,165,LCD_COLOR_GREEN);	//存储已满	
		}	
	}
	HAL_Delay(1000);
	drawblock(xpos-2,ypos-1,153,63);
}
void disconfirm(uint16_t xpos,uint16_t ypos,uint16_t yn)
{
	uint32_t color;
	color=BSP_LCD_GetTextColor();
	BSP_LCD_SetTextColor(LCD_COLOR_CYAN);
	
	BSP_LCD_DrawLine(xpos-1,ypos-1,xpos+150,ypos-1);
	BSP_LCD_DrawLine(xpos-1,ypos-1,xpos-1,ypos+80);
	BSP_LCD_DrawLine(xpos+150,ypos+80,xpos+150,ypos-1);
	BSP_LCD_DrawLine(xpos+150,ypos+80,xpos-1,ypos+80);
	
drawcolorblock(xpos,ypos,148,79,LCD_COLOR_BLACK);

	dishanzi(xpos+15,ypos+5,94,LCD_COLOR_RED);
	dishanzi(xpos+45,ypos+5,95,LCD_COLOR_RED);	
	dishanzi(xpos+75,ypos+5,148,LCD_COLOR_RED);
	dishanzi(xpos+105,ypos+5,149,LCD_COLOR_RED);	//确定删除
if(yn==0)
{	
	dishanzi(xpos+45,ypos+45,86,LCD_COLOR_WHITE);
	dishanzi(xpos+85,ypos+45,87,LCD_COLOR_GREEN);
}
else
{
	dishanzi(xpos+45,ypos+45,86,LCD_COLOR_GREEN);
	dishanzi(xpos+85,ypos+45,87,LCD_COLOR_WHITE);

}
BSP_LCD_SetTextColor(color);
}
void dishisdata(void)
{
	BSP_LCD_DrawLine(490,140,639,140);	
drawblock(491,141,147,192);
BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
BSP_LCD_SetBackColor(LCD_COLOR_BLUE);	
	
BSP_LCD_DisplayStringAt(491,144,(uint8_t*)"181105-098", LEFT_MODE);	
	BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
BSP_LCD_DisplayStringAt(491,168,(uint8_t*)"181105-083", LEFT_MODE);	
BSP_LCD_DisplayStringAt(491,192,(uint8_t*)"181105-073", LEFT_MODE);	
BSP_LCD_DisplayStringAt(491,216,(uint8_t*)"181105-017", LEFT_MODE);	
BSP_LCD_DisplayStringAt(491,240,(uint8_t*)"181103-092", LEFT_MODE);	
BSP_LCD_DisplayStringAt(491,264,(uint8_t*)"181103-023", LEFT_MODE);	
BSP_LCD_DisplayStringAt(491,288,(uint8_t*)"181103-004", LEFT_MODE);	
BSP_LCD_DisplayStringAt(491,312,(uint8_t*)"181101-043", LEFT_MODE);
BSP_LCD_DisplayStringAt(491,336,(uint8_t*)"181101-033", LEFT_MODE);
BSP_LCD_DisplayStringAt(491,360,(uint8_t*)"181101-024", LEFT_MODE);
BSP_LCD_DisplayStringAt(491,384,(uint8_t*)"181101-001", LEFT_MODE);
}


void disparament(uint16_t gongneng1,uint16_t mun,uint32_t col)
{
uint16_t x,xloc,yloc,y,m,l;
	
uint32_t color,discolor;
color=BSP_LCD_GetTextColor();
BSP_LCD_SetFont(&Font24_1);
if(col==1)
{
if(modifyparament==1)//canshuxiuai
  discolor=LCD_COLOR_GREEN;
else
  if(modifyparament==2)//guanbiaoxianshi
      discolor=LCD_COLOR_ORANGE;//
  else
    if(modifyparament==0)//zhengchangxianshi
        discolor=LCD_COLOR_WHITE;
}
else
	discolor=col;
	BSP_LCD_SetTextColor(discolor);
	
if(gongneng1==0)
{
if(mun==16)
{
				
					xloc=219;
					yloc=755;			
				

}
	else
	{	
switch(mun)
{
	case 0:
		xloc=60;
		yloc=629;
		break;
	case 1:
		xloc=219;
		yloc=629;		
		break;
	case 2:
		xloc=60;
		yloc=755;			
		break;
	case 3:
		xloc=60;
		yloc=755;			
		break;
	case 4:
		xloc=60;
		yloc=692;	
		break;
	case 5:
		xloc=219;
		yloc=692;			
		break;
	case 6:
		xloc=219;
		yloc=629;			
		break;	
	case 7:
		xloc=219;
		yloc=755;			
		break;
	case 8:
		xloc=60;
		yloc=692;			
		break;	
				case 9:
					xloc=219;
					yloc=629;
					break;
				case 10:
					xloc=60;
					yloc=629;		
					break;
				case 11:
					xloc=219;
					yloc=692;			
					break;
				case 12:
					xloc=60;
					yloc=692;			
					break;
				case 13:
					xloc=60;
					yloc=629;	
					break;
				case 14:
					xloc=60;
					yloc=629;			
					break;
				case 15:
					xloc=219;
					yloc=629;					
					break;				

	
	default:
		break;
		
}
}
		}
		
				else if(gongneng1==1)
					{
						if(mun>9)
								mun=9;
						switch(mun)
						{
							case 0:
										xloc=60;
										yloc=629;
								break;
							case 1:
										xloc=219;
										yloc=629;		
								break;
							case 2:
										xloc=60;
										yloc=629;			
								break;
							case 3:
									xloc=60;
									yloc=692;			
								break;
							case 4:
								xloc=219;
								yloc=692;			
								break;
							case 5:
								xloc=60;
								yloc=755;			
								break;
							case 6:
								xloc=219;
								yloc=755;			
								break;
							case 7:
								xloc=219;
								yloc=629;			
								break;	
							case 8:
								xloc=60;
								yloc=692;			
								break;	
							case 9:
								xloc=219;
								yloc=692;			
								break;							
							default:
								break;
								
						}		
					}
			else if(gongneng1==2)
					{
						if(mun>2)
								mun=2;
						switch(mun)
						{
							case 0:
								xloc=170;
								yloc=424;
								break;
							case 1:
								xloc=170;
								yloc=452;		
								break;
							case 2:
								xloc=473;
								yloc=424;			
								break;
							case 3:
								xloc=423;
								yloc=452;			
								break;
							
							default:
								break;								
						}							
					}	
	if(gongneng1==2&&mun==2)
x=paramvaule[gongneng1][mun]+1;
else	
x=paramvaule[gongneng1][mun];
//	LCD_ShowxNum(580 ,330, x ,4,LCD_COLOR_GREEN);
//LCD_ShowxNum_pix(xloc ,yloc, x ,3,12,2,discolor);	
if(paramenttype[gongneng1][mun]>10)
	y=paramenttype[gongneng1][mun]%10;
else
{

		y=paramenttype[gongneng1][mun];
}
if(paramenttype[gongneng1][mun]>30&&paramenttype[gongneng1][mun]<40)// xianshi sanwei xiaoshu
{
	BSP_LCD_DisplayStringAt(xloc+(y-3)*12,yloc,(uint8_t*)".", LEFT_MODE);
	                for(m=y;m>0;m--)
                  {   l=x%10;
                      x=x/10;	
                        
                        if(m<(y-2))
                          BSP_LCD_DisplayChar(xloc+(m-1)*12,yloc,48+l);  
                        else
                          BSP_LCD_DisplayChar(xloc+m*12,yloc,48+l); 
		    							if(modifyparament==1&&(y-m+1)==modifybit)
		    								{
		    								BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
                        if(m<(y-2))
                          BSP_LCD_DisplayChar(xloc+(m-1)*12,yloc,48+l);  
                        else
                          BSP_LCD_DisplayChar(xloc+m*12,yloc,48+l); 
		    								BSP_LCD_SetTextColor(discolor);
		    								}
                  } 
}
else if(paramenttype[gongneng1][mun]>20&&paramenttype[gongneng1][mun]<30)//liang wei xiao shu两位小数显示 
{
	BSP_LCD_DisplayStringAt(xloc+(y-2)*12,yloc,(uint8_t*)".", LEFT_MODE);
            for(m=y;m>0;m--)
              {   l=x%10;
                  x=x/10;

                    if(m<(y-1))
                      BSP_LCD_DisplayChar(xloc+(m-1)*12,yloc,48+l);  
                    else
                      BSP_LCD_DisplayChar(xloc+m*12,yloc,48+l); 
										
									if(modifyparament==1&&(y-m+1)==modifybit)
										{
										BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
                    if(m<(y-1))
                      BSP_LCD_DisplayChar(xloc+(m-1)*12,yloc,48+l);  
                    else
                      BSP_LCD_DisplayChar(xloc+m*12,yloc,48+l); 
										BSP_LCD_SetTextColor(discolor);
										}										

							}

}
	else if(paramenttype[gongneng1][mun]>10&&paramenttype[gongneng1][mun]<20)//一位小数显示 
	{
	  BSP_LCD_DisplayStringAt(xloc+(y-1)*12,yloc,(uint8_t*)".", LEFT_MODE);
	  if(y>3)
	    {
	      if(x>=100)
	         {
                for(m=y;m>0;m--)
                  {   l=x%10;
                      x=x/10;	
                        
                        if(m<y)
                          BSP_LCD_DisplayChar(xloc+(m-1)*12,yloc,48+l);  
                        else
                          BSP_LCD_DisplayChar(xloc+m*12,yloc,48+l); 
		    							if(modifyparament==1&&(y-m+1)==modifybit)
		    								{
		    								BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
                        if(m<y)
                          BSP_LCD_DisplayChar(xloc+(m-1)*12,yloc,48+l);  
                        else
                          BSP_LCD_DisplayChar(xloc+m*12,yloc,48+l); 
		    								BSP_LCD_SetTextColor(discolor);
		    								}
		    							if(m==1&&l==0)
                        BSP_LCD_DisplayChar(xloc+0,yloc,' ');  //最高位是0不显示
                  }      	
		    			}
		    	else
		    		{
                for(m=y;m>0;m--)
                  {   l=x%10;
                      x=x/10;
                        
                        if(m<y)
                          BSP_LCD_DisplayChar(xloc+(m-1)*12,yloc,48+l);  
                        else
                          BSP_LCD_DisplayChar(xloc+m*12,yloc,48+l); 
		    							if(modifyparament==1&&(y-m+1)==modifybit)
		    								{
		    								BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
                        if(m<y)
                          BSP_LCD_DisplayChar(xloc+(m-1)*12,yloc,48+l);  
                        else
                          BSP_LCD_DisplayChar(xloc+m*12,yloc,48+l); 
		    								BSP_LCD_SetTextColor(discolor);
		    								}								
                  } 
                  
                    BSP_LCD_DisplayChar(xloc+0,yloc,' ');  //最高位是0不显示 
                    BSP_LCD_DisplayChar(xloc+12,yloc,' ');  //最高位是0不显示     	
		     			}	
		  }			
	  else
	  {
	    if(x>=100)
	       {
              for(m=y;m>0;m--)
                {   l=x%10;
                    x=x/10;
                      
                      if(m<y)
                        BSP_LCD_DisplayChar(xloc+(m-1)*12,yloc,48+l);  
                      else
                        BSP_LCD_DisplayChar(xloc+m*12,yloc,48+l); 
		  							if(modifyparament==1&&(y-m+1)==modifybit)
		  								{
		  								BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
                      if(m<y)
                        BSP_LCD_DisplayChar(xloc+(m-1)*12,yloc,48+l);  
                      else
                        BSP_LCD_DisplayChar(xloc+m*12,yloc,48+l); 
		  								BSP_LCD_SetTextColor(discolor);
		  								}
		  							if(m==1&&l==0)
                      BSP_LCD_DisplayChar(xloc+0,yloc,' ');  //最高位是0不显示
                }      	
		  			}
		  	else
		  		{
              for(m=y;m>0;m--)
                {   l=x%10;
                    x=x/10;
                      
                      if(m<y)
                        BSP_LCD_DisplayChar(xloc+(m-1)*12,yloc,48+l);  
                      else
                        BSP_LCD_DisplayChar(xloc+m*12,yloc,48+l); 
		  							if(modifyparament==1&&(y-m+1)==modifybit)
		  								{
		  								BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
                      if(m<y)
                        BSP_LCD_DisplayChar(xloc+(m-1)*12,yloc,48+l);  
                      else
                        BSP_LCD_DisplayChar(xloc+m*12,yloc,48+l); 
		  								BSP_LCD_SetTextColor(discolor);
		  								}								
                } 
                
                  BSP_LCD_DisplayChar(xloc+0,yloc,' ');  //最高位是0不显示 
                     	
		  			}	
									
	   }
	}
		else if(paramenttype[gongneng1][mun]<10)//整数显示 
		{
			if(scanindex==1)// 在扫描状态下
			{
				if(gongneng1==0&&mun==0)//第一个参数为0
					x=0;
			}
			if(x<10)	
			  {	BSP_LCD_DisplayStringAt(xloc,yloc,(uint8_t*)"   ",LEFT_MODE); 
			  	BSP_LCD_DisplayChar(xloc+(y-1)*12,yloc,48+x); 
			  	if(modifyparament==1&&(y-1-1)==modifybit)
			    	{
			    		BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
              BSP_LCD_DisplayChar(xloc+(y-1)*12,yloc,48+x);      
			    		BSP_LCD_SetTextColor(discolor);
			    	}					
			   } 			
			else if (x>=10 && x<100 )
			    {
			    		BSP_LCD_DisplayStringAt(xloc,yloc,(uint8_t*)"   ",LEFT_MODE); 
			    	 for(m=y;m>1;m--)
			     {
			    	l=x%10;
			    	x=x/10;
			    	BSP_LCD_DisplayChar(xloc+(m-1)*12,yloc,48+l);
			    	if(modifyparament==1&&(y-m+1)==modifybit)
			    		{
			    		BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
              BSP_LCD_DisplayChar(xloc+(m-1)*12,yloc,48+l);      
			    		BSP_LCD_SetTextColor(discolor);
			    		}				
		    	 if(m==2&&l==0)
		    	 {
            BSP_LCD_DisplayChar(xloc+0,yloc,' ');  //最高位是0不	
            BSP_LCD_DisplayChar(xloc+12,yloc,' ');  //最高位是0不
           }		
         }	    	 
			    }
			else if (x>=100 && x<1000 )  
				
			  {BSP_LCD_DisplayStringAt(xloc,yloc,(uint8_t*)"   ",LEFT_MODE);
			  	  for(m=y;m>0;m--)
			    {
			    	l=x%10;
			    	x=x/10;
			    	BSP_LCD_DisplayChar(xloc+(m-1)*12,yloc,48+l);
			    	if(modifyparament==1&&(y-m+1)==modifybit)
			    		{
			    		BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
              BSP_LCD_DisplayChar(xloc+(m-1)*12,yloc,48+l);      
			    		BSP_LCD_SetTextColor(discolor);
			    		}				
			    	
			    	if(m==1&&l==0)
           BSP_LCD_DisplayChar(xloc+0,yloc,' ');  //最高位是0不	          
          }	
			  }  	
			 else if (x>=1000 && x<10000 )  
				
			  {BSP_LCD_DisplayStringAt(xloc,yloc,(uint8_t*)"   ",LEFT_MODE);
			  	  for(m=y;m>0;m--)
			    {
			    	l=x%10;
			    	x=x/10;
			    	BSP_LCD_DisplayChar(xloc+(m-1)*12,yloc,48+l);
			    	if(modifyparament==1&&(y-m+1)==modifybit)
			    		{
			    		BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
              BSP_LCD_DisplayChar(xloc+(m-1)*12,yloc,48+l);      
			    		BSP_LCD_SetTextColor(discolor);
			    		}				
			    	
			    		if(m==1&&l==0)
           BSP_LCD_DisplayChar(xloc+0,yloc,' ');  //最高位是0不	
           }
			  }  	 
	}
			else if(paramenttype[gongneng1][mun]==30) //特殊参数显示
						{
							/*
							if(page1==0&&mun==10)//jiangpandeng
							{
								if(paramvaule[page1][mun]==1)
										dishanzi(xloc,yloc,42,discolor);//kai
								else
									dishanzi(xloc,yloc,43,discolor);//guan
							}
							*/
							if(gongneng1==0&&mun==11)//相关系数
								{
										switch(paramvaule[gongneng1][mun])
										{
											case 0:
												BSP_LCD_DisplayStringAt(xloc,yloc,(uint8_t*)" 01", LEFT_MODE);
												break;
											case 1:
												BSP_LCD_DisplayStringAt(xloc,yloc,(uint8_t*)" 02", LEFT_MODE);	
												break;
											case 2:
												BSP_LCD_DisplayStringAt(xloc,yloc,(uint8_t*)" 04", LEFT_MODE);			
												break;
											case 3:
												BSP_LCD_DisplayStringAt(xloc,yloc,(uint8_t*)" 08", LEFT_MODE);			
												break;
											case 4:
												BSP_LCD_DisplayStringAt(xloc,yloc,(uint8_t*)" 16", LEFT_MODE);			
												break;
											case 5:
												BSP_LCD_DisplayStringAt(xloc,yloc,(uint8_t*)" 32", LEFT_MODE);			
												break;	
											case 6:
												BSP_LCD_DisplayStringAt(xloc,yloc,(uint8_t*)" 64", LEFT_MODE);			
												break;
											case 7:
												BSP_LCD_DisplayStringAt(xloc,yloc,(uint8_t*)"128", LEFT_MODE);			
												break;												
											default:
												break;												
										}																			
								}
							if(gongneng1==0&&mun==12)//ZP PP算法
								{
								 if(paramvaule[gongneng1][mun]==0)
										BSP_LCD_DisplayStringAt(xloc,yloc,(uint8_t*)"PP", LEFT_MODE);
								 else		
										BSP_LCD_DisplayStringAt(xloc,yloc,(uint8_t*)"ZP", LEFT_MODE);									 
								}	
								if(gongneng1==0&&mun==16)//高频  低频   0低 1高
								{
								 if(HL==0)
								 {
										dishanzi(xloc,   yloc,175,discolor); //低频
										dishanzi(xloc+24,yloc,52,discolor);
								 }
								 else		
								 {
										dishanzi(xloc,   yloc,174,discolor); //高频
										dishanzi(xloc+24,yloc,52,discolor);
								 }								 
								}								
							if(gongneng1==0&&mun==3)//波形选择
								{
								 if(paramvaule[gongneng1][mun]==1) //1 是包络 0是原始
									{
										dishanzi(xloc,   yloc,119,discolor); //包络
										dishanzi(xloc+24,yloc,120,discolor);
									}	
								 else		
									{
										dishanzi(xloc,   yloc,121,discolor); //原始
										dishanzi(xloc+24,yloc,122,discolor);
									}								 
								}								
								
							if((gongneng1==0&&mun==10)||(gongneng1==1&&mun==8))//材料
								{
									BSP_LCD_DisplayStringAt(xloc,yloc,(uint8_t*)"        ", LEFT_MODE);
										switch(paramvaule[gongneng1][mun])
										{
											case 0:
												dishanzi(xloc,yloc,141,discolor);
												dishanzi(xloc+24,yloc,103,discolor);//碳钢
												break;
											case 1:
												dishanzi(xloc,yloc,104,discolor);
												dishanzi(xloc+24,yloc,105,discolor);
												dishanzi(xloc+48,yloc,103,discolor);//合金钢	
												break;
											case 2:
												dishanzi(xloc,yloc,142,discolor);//铜	
												break;	
											case 3:
												dishanzi(xloc,yloc,143,discolor);//铝
												break;											
											case 4:
												dishanzi(xloc,yloc,144,discolor);
												dishanzi(xloc+24,yloc,145,discolor);
												dishanzi(xloc+48,yloc,103,discolor);//不锈钢	
												break;	
											case 5:
												dishanzi(xloc,yloc,150,discolor);
												dishanzi(xloc+24,yloc,151,discolor);
												dishanzi(xloc+48,yloc,152,discolor);
												dishanzi(xloc+72,yloc,153,discolor);//球墨铸铁	
												break;											
											case 6:
												dishanzi(xloc,yloc,61,discolor);
												dishanzi(xloc+24,yloc,95,discolor);
												dishanzi(xloc+48,yloc,112,discolor);//自定义	
												break;											
											default:
												break;												
										}																			
								}								
						}
		
BSP_LCD_SetTextColor(color);
}

void dispageparament(uint32_t col)
{
	uint16_t x;
	if(gongneng==0)
	{
  if(page==0&&scanindex==0)
    {
	 //	  for(x=0;x<9;x++)
		  disparament(gongneng,0,col);
			disparament(gongneng,1,col);
			disparament(gongneng,4,col);
			disparament(gongneng,5,col);
			disparament(gongneng,2,col);
			disparament(gongneng,7,col);
				
     }
	else if(page==1&&scanindex==0)
	        {
		      //  for(x=9;x<14;x++)
							 disparament(gongneng,10,col);	
			         disparament(gongneng,9,col);
			         disparament(gongneng,12,col);	
						   disparament(gongneng,11,col);	
						   disparament(gongneng,3,col);	
						   disparament(gongneng,16,col);	

	        }
					else if(page==2&&scanindex==0)
					{
						   disparament(gongneng,13,col);	
						   disparament(gongneng,6,col);	
						   disparament(gongneng,8,col);	
							 dishanzi(219,755,180,LCD_COLOR_RED); 
							 dishanzi(243,755,181,LCD_COLOR_RED);
							 dishanzi(267,755,182,LCD_COLOR_RED); 
							 dishanzi(291,755,183,LCD_COLOR_RED);	//恢复出厂				
					}
					else if(scanindex==1)
						{
		        for(x=14;x<16;x++)
			         disparament(gongneng,x,col);							
						}
				}
	else if(gongneng==1)
		     {
					 if(page==0)
					 {
					 disparament(gongneng,0,col);
					 disparament(gongneng,1,col);	
					 disparament(gongneng,3,col);	
					 disparament(gongneng,4,col);	
					 disparament(gongneng,5,col);	
					 disparament(gongneng,6,col);							 
					 }
					else if(page==1)
					{
					 disparament(gongneng,2,col);
					 disparament(gongneng,7,col);	
					 disparament(gongneng,8,col);	
					 disparament(gongneng,9,col);				
					}
		     }
	else if(gongneng==2)
		     {
			      for(x=0;x<3;x++)
				       disparament(gongneng,x,col);		
		     }		     
}
void disinformationdata(void)
{
	uint32_t color,disshengsu;
	uint32_t thickdisin,histhickin;
	
	float m;
	if(modifyparament==0&&timemodify==0)
	{
	disscandata();
color=BSP_LCD_GetTextColor();
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	BSP_LCD_SetFont(&Font24_1);
	if(scanindex==0)
	{

					if(error==1&&modelindex!=1&&modelindex!=2)
						thickavg=0;

	
//	if(((thickavg%1000)<20)||((thickavg%1000)>980))
//	{
//	if((thickavg%1000)<20)
//			thickavg=(thickavg/100)*100;
//	 else if((thickavg%1000)>980)
//			thickavg=((thickavg+22)/100)*100;	
//	}
					if(paramvaule[2][2]==0)
			{
				BSP_LCD_SetFont(&Font48);

				if((thickavg%100)<50)
					thickdisin=thickavg/100;
				else
					thickdisin=thickavg/100+1;
				if((histhick%10)<50)
					histhickin=histhick/100;
				else
					histhickin=histhick/100+1;
				if(hisindex==0)
					LCD_shouBigMun_pix(240,45,thickdisin,4,32,3,LCD_COLOR_YELLOW);
				//	LCD_ShowxNum_pix(130 ,5, thickdisin,4,24,3,LCD_COLOR_YELLOW);//小数点在第2位，字符大小24pixLCD_ShowxNum_pix(150 ,5, thickdisin,4,16,3,LCD_COLOR_YELLOW)
				else
					LCD_shouBigMun_pix(240,45,histhickin,4,32,3,LCD_COLOR_YELLOW);
				//	LCD_ShowxNum_pix(130 ,5, histhickin,4,24,3,LCD_COLOR_YELLOW);//小数点在第2位，字符大小24pix
					BSP_LCD_SetFont(&Font24_1);	
			}
				else if(paramvaule[2][2]==1)
				{
					BSP_LCD_SetFont(&Font48);
				if((thickdis%10)<5)
					thickdisin=thickdis/10;
				else
					thickdisin=thickdis/10+1;
				if((histhick%10)<5)
					histhickin=histhick/10;
				else
					histhickin=histhick/10+1;
					if(hisindex==0)
							LCD_shouBigMun_pix(240,45,thickdisin,5,32,3,LCD_COLOR_YELLOW);
				//	LCD_ShowxNum_pix(130 ,5, thickdisin,5,24,3,LCD_COLOR_YELLOW);//小数点在第2位，字符大小24pix
				else
					LCD_shouBigMun_pix(240,45,histhickin,5,32,3,LCD_COLOR_YELLOW);
				//	LCD_ShowxNum_pix(130 ,5, histhickin,5,24,3,LCD_COLOR_YELLOW);//小数点在第2位，字符大小24pix
				BSP_LCD_SetFont(&Font24_1);
				}
				else
				{
					BSP_LCD_SetFont(&Font48);
					thickdisin=thickavg;
					histhickin=histhick;
				if(hisindex==0)
					LCD_shouBigMun_pix(240,45,thickdisin,6,32,3,LCD_COLOR_YELLOW);
				//	LCD_ShowxNum_pix(130 ,5, thickdisin,6,24,3,LCD_COLOR_YELLOW);//小数点在第2位，字符大小24pix
				else
					LCD_shouBigMun_pix(240,45,histhickin,6,32,3,LCD_COLOR_YELLOW);
				//	LCD_ShowxNum_pix(130 ,5, histhickin,6,24,3,LCD_COLOR_YELLOW);//小数点在第2位，字符大小24pix
				BSP_LCD_SetFont(&Font24_1);
			}
	}
	else
	{
		if(scanhisindex==0)
	{
		if(scandirection==0)//右扫描
	{
	

	distance=abs(((encoder-10000)*2));
	}
	else//左扫描
	{
	
	distance=abs(((10000-encoder)*2));
	}
		LCD_ShowxNum_pix(219 ,692, distance/10,4,12,4,LCD_COLOR_YELLOW);//小数点在第2位，字符大小24pix

	

								if(scandirection==0)//扫描方向右
										thickdisin=SCAN_BUF[(int)(SCAN_cursor*kp/(0.2))+10000];	//57
							else //扫描方向左
							  	thickdisin=SCAN_BUF[(int)(10000-SCAN_cursor*kp/(0.2))];
						LCD_ShowxNum_pix(60,755, thickdisin/10,5,12,3,LCD_COLOR_RED);

						}	
					else
					{
								if(scandirection==0)//扫描方向右
										thickdisin=SCAN_BUF[(int)(SCAN_cursor*kp/(0.2))+10000];	//57
							else //扫描方向左
							  	thickdisin=SCAN_BUF[(int)(10000-SCAN_cursor*kp/(0.2))];
						LCD_ShowxNum_pix(366,670, thickdisin/10,5,12,3,LCD_COLOR_RED);

					}	
	}
	

	BSP_LCD_SetFont(&Font24_1);	
if(scanhisindex==0&&scanindex==0)
{
//显示计算的算法
	if(modelindex==0||modelindex==3)
	{
	if(error==1)
	{
			BSP_LCD_SetTextColor(LCD_COLOR_RED);
		BSP_LCD_DisplayStringAt(370,89,(uint8_t*)"ER", LEFT_MODE);
	}
		else
		{
			BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
			if(modelindex==0)
			{
				if(findway==1)
			BSP_LCD_DisplayStringAt(370,89,(uint8_t*)"PP", LEFT_MODE);
				else if(findway==2||findway==3||findway==4)
					BSP_LCD_DisplayStringAt(370,89,(uint8_t*)"ZP", LEFT_MODE);
			}
			else if(modelindex==3)
					BSP_LCD_DisplayStringAt(370,89,(uint8_t*)"ZP", LEFT_MODE);
	}
 }
	else
		 BSP_LCD_DisplayStringAt(370,89,(uint8_t*)"  ", LEFT_MODE);
		
	//T1
		BSP_LCD_SetTextColor(LCD_COLOR_YELLOW );
if(hisindex==0)
{
	if(gongneng==0)
	{
x=timeA1;//T1

LCD_ShowxNum_pix(38,2, timeA1,4,12,2,LCD_COLOR_RED);
//A1
if(peakc<0)
  peakc=1;
m=peakc;
x=abs(200*log10(m/32768.0));
if(x>999)
  x=999;
if(x<0)
  x=0;

 LCD_ShowxNum_pix(213,2, x,3,12,2,LCD_COLOR_RED);
x=timeA2;//T2

LCD_ShowxNum_pix(366,2, timeA2,4,12,2,LCD_COLOR_RED);
//A2
//if(peakc1<0)
//  peakc1=1;
//m=peakc1;
//x=abs(200*log10(m/32768.0));
//if(x>999)
//  x=999;
//if(x<0)
//  x=0;

//LCD_ShowxNum_pix(50,83, x,3,12,2,LCD_COLOR_RED);
}
	else if(gongneng==1)
	{
//SH		
x=(sbpeakloc-((float)(paramvaule[0][8]))/10)*(usedv-7.086*paramvaule[0][9])/200000;

LCD_ShowxNum_pix(38,2, x,4,12,3,LCD_COLOR_RED);
//SA/DA
if(sbpeak<0)
  sbpeak=1;
m=sbpeak;
if(200*log10(sbpeak/32768.0)-200*log10(dbpeak/32768.0)<0)
	BSP_LCD_DisplayStringAt(201,2,(uint8_t*)"-", LEFT_MODE);
else
	BSP_LCD_DisplayStringAt(201,2,(uint8_t*)" ", LEFT_MODE);
x=abs(200*log10(sbpeak/32768.0)-200*log10(dbpeak/32768.0));

if(x>999)
  x=999;
if(x<0)
  x=0;

 LCD_ShowxNum_pix(213,2, x,3,12,2,LCD_COLOR_RED);
//DA
x=(dbpeakloc-((float)(paramvaule[0][8]))/10)*(usedv-7.086*paramvaule[0][9])/200000;

LCD_ShowxNum_pix(366,2, x,4,12,3,LCD_COLOR_RED);
//DH
//if(dbpeak<0)
//  dbpeak=1;
//m=dbpeak;
//x=abs(200*log10(m/32768.0));
//if(x>999)
//  x=999;
//if(x<0)
//  x=0;

//LCD_ShowxNum_pix(50,83, x,3,12,2,LCD_COLOR_RED);	
	}
}
}
if(scanindex==1&&scanhisindex==0&&page==3 )
{
	if(scandirection==0)//右扫描
	{
//	dishanzi(420,30,155,LCD_COLOR_WHITE);
//	dishanzi(420+24,30,156,LCD_COLOR_WHITE);
//	dishanzi(420+48,30,157,LCD_COLOR_WHITE);	
	dishanzi(312,452,155,LCD_COLOR_YELLOW);
	dishanzi(312+24,452,156,LCD_COLOR_YELLOW);
	dishanzi(312+48,452,157,LCD_COLOR_YELLOW);
	distance=(float)((encoder-10000)*2);
	}
	else//左扫描
	{
	dishanzi(312,452,154,LCD_COLOR_YELLOW);
	dishanzi(312+24,452,156,LCD_COLOR_YELLOW);
	dishanzi(312+48,452,157,LCD_COLOR_YELLOW);	
	distance=(float)((10000-encoder)*2);
	}
	
	//BSP_LCD_DisplayStringAt(493,452,(uint8_t*)":", LEFT_MODE);
  LCD_ShowxNum_pix(460,452, distance,5,12,4,LCD_COLOR_BLUE);
  BSP_LCD_DisplayStringAt(440+132,452,(uint8_t*)"mm", LEFT_MODE);	
  		
}
//else
	//BSP_LCD_DisplayStringAt(420,30,(uint8_t*)"                ", LEFT_MODE);
BSP_LCD_SetTextColor(color);	
if(scanhisindex==0)
{
if(hisindex==0)//正常还是历史数据显示界面
{
	if(paramvaule[0][10]==6)//自定义声速
			usedv=paramvaule[0][6];
	else
			usedv=shengsu[paramvaule[0][10]];
//	disshengsu=usedv/10;
//	if(gongneng==0)
//	{
//if(modelindex==0)		
//LCD_ShowxNum_pix(390 ,650,agctemp,3,12,2,LCD_COLOR_WHITE);//增益
//else
//LCD_ShowxNum_pix(390 ,650,paramvaule[0][2],3,12,2,LCD_COLOR_WHITE);//增益	
//}
//	else
//		LCD_ShowxNum_pix(390 ,650,paramvaule[1][2],3,12,2,LCD_COLOR_WHITE);//增益	
//LCD_ShowxNum(390 ,680,disshengsu,4,LCD_COLOR_WHITE);//声速
//LCD_ShowxNum(390 ,710,paramvaule[0][9],3,LCD_COLOR_WHITE);//温度
//				switch(paramvaule[0][10])
//										{
//											case 0:
//												dishanzi(390,620,141,LCD_COLOR_WHITE);
//												dishanzi(390+24,620,103,LCD_COLOR_WHITE);//碳钢
//												BSP_LCD_DisplayStringAt(390+48,620,(uint8_t*)"  ", LEFT_MODE);
//												break;
//											case 1:
//												dishanzi(390,620,104,LCD_COLOR_WHITE);
//												dishanzi(390+24,620,105,LCD_COLOR_WHITE);
//												dishanzi(390+48,620,103,LCD_COLOR_WHITE);//合金钢	
//												break;
//											case 2:
//												dishanzi(390,620,142,LCD_COLOR_WHITE);//铜
//												BSP_LCD_DisplayStringAt(390+24,620,(uint8_t*)"    ", LEFT_MODE);											
//												break;	
//											case 3:
//												dishanzi(390,620,143,LCD_COLOR_WHITE);//铝
//												BSP_LCD_DisplayStringAt(390+24,620,(uint8_t*)"    ", LEFT_MODE);											
//												break;											
//											case 4:
//												dishanzi(390,620,144,LCD_COLOR_WHITE);
//												dishanzi(390+24,620,145,LCD_COLOR_WHITE);
//												dishanzi(390+48,620,103,LCD_COLOR_WHITE);//不锈钢	
//												break;	
//											case 5:
//												dishanzi(390,620,152,LCD_COLOR_WHITE);
//												dishanzi(390+24,620,153,LCD_COLOR_WHITE);//铸铁	
//												BSP_LCD_DisplayStringAt(390+48,620,(uint8_t*)"  ", LEFT_MODE);											
//												break;											
//											case 6:
//												dishanzi(390,620,61,LCD_COLOR_WHITE);
//												dishanzi(390+24,620,95,LCD_COLOR_WHITE);
//												dishanzi(390+48,620,112,LCD_COLOR_WHITE);//自定义	
//												break;											
//											default:
//												break;												
//										}		
}
else
{
dishanzi(366,636,59,LCD_COLOR_WHITE);
dishanzi(390,636,60,LCD_COLOR_WHITE);
BSP_LCD_DisplayStringAt(414,636,(uint8_t*)":    ", LEFT_MODE);//增益	
	
dishanzi(366,660,11,LCD_COLOR_WHITE);
dishanzi(390,660,12,LCD_COLOR_WHITE);
BSP_LCD_DisplayStringAt(414,660,(uint8_t*)":    ", LEFT_MODE);//声速

dishanzi(366,684,80,LCD_COLOR_WHITE);
dishanzi(390,684,81,LCD_COLOR_WHITE);
BSP_LCD_DisplayStringAt(414,684,(uint8_t*)":    ", LEFT_MODE);//wendu
LCD_ShowxNum_pix(426 ,636,hisgain,3,12,2,LCD_COLOR_WHITE);//增益
LCD_ShowxNum(426 ,660,hisv/10,4,LCD_COLOR_WHITE);//声速
LCD_ShowxNum(426 ,684,hist,3,LCD_COLOR_WHITE);//温度	
}
}
if(gongneng==2)
{
	BSP_LCD_SetFont(&Font48);
LCD_ShowxNum_pix(98 ,620, yinglimax,5,24,3,LCD_COLOR_YELLOW);
LCD_ShowxNum_pix(98 ,680, yinglimin,5,24,3,LCD_COLOR_YELLOW);
LCD_ShowxNum_pix(50 ,740, yinglizhi,5,24,3,LCD_COLOR_YELLOW);	
	BSP_LCD_SetFont(&Font24_1);
}
	}																
//if(scanindex==0)
//RTC_TimeShow(aShowTime);

}
void disinformationunit(void)
{
uint32_t color;

color=BSP_LCD_GetTextColor();
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	BSP_LCD_SetFont(&Font24_1);	
//dishanzi(120,5,5,LCD_COLOR_WHITE);
//dishanzi(144,5,6,LCD_COLOR_WHITE);

	if(scanindex==0)
	{BSP_LCD_SetFont(&Font48);	
BSP_LCD_DisplayStringAt(290,63,(uint8_t*)"mm", LEFT_MODE);//厚度
BSP_LCD_SetFont(&Font24_1);	



//		BSP_LCD_SetTextColor(LCD_COLOR_ORANGE);
//BSP_LCD_DisplayStringAt(410,88,(uint8_t*)"V3.81", LEFT_MODE);//版本号
		BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	}
if(hisindex==0)
{
//dishanzi(330,620,9,LCD_COLOR_WHITE);
//dishanzi(354,620,10,LCD_COLOR_WHITE);
//BSP_LCD_DisplayStringAt(378,620,(uint8_t*)":      ", LEFT_MODE);//材质	
//dishanzi(330,650,59,LCD_COLOR_WHITE);
//dishanzi(354,650,60,LCD_COLOR_WHITE);
//BSP_LCD_DisplayStringAt(378,650,(uint8_t*)":  . dB", LEFT_MODE);//增益
//	
//dishanzi(330,680,11,LCD_COLOR_WHITE);
//dishanzi(354,680,12,LCD_COLOR_WHITE);
//BSP_LCD_DisplayStringAt(378,680,(uint8_t*)":    m/s", LEFT_MODE);//声速


//dishanzi(330,710,80,LCD_COLOR_WHITE);
//dishanzi(354,710,81,LCD_COLOR_WHITE);
//BSP_LCD_DisplayStringAt(378,710,(uint8_t*)":   'C", LEFT_MODE);//wendu
		
if(gongneng==0&&scanindex==0)
{
BSP_LCD_SetTextColor(LCD_COLOR_YELLOW );
	BSP_LCD_DisplayStringAt(2,2,(uint8_t*)"T1:  .  us", LEFT_MODE);//
	BSP_LCD_DisplayStringAt(165,2,(uint8_t*)"A1:-  . dB", LEFT_MODE);//
	BSP_LCD_DisplayStringAt(330,2,(uint8_t*)"T2:  .  us", LEFT_MODE);//
//	BSP_LCD_DisplayStringAt(2,83,(uint8_t*)"A2:-  . dB", LEFT_MODE);//
}
else if(gongneng==1&&scanindex==0)
{
	BSP_LCD_SetTextColor(LCD_COLOR_RED );
	BSP_LCD_DisplayStringAt(2,2,(uint8_t*)"SH:   . mm", LEFT_MODE);//
	BSP_LCD_DisplayStringAt(165,2,(uint8_t*)"SA:-  . dB", LEFT_MODE);//
	BSP_LCD_SetTextColor(LCD_COLOR_GREEN );
	BSP_LCD_DisplayStringAt(330,2,(uint8_t*)"DH:   . mm", LEFT_MODE);//
//	BSP_LCD_DisplayStringAt(2,83,(uint8_t*)"DA:-  . dB", LEFT_MODE);//

}
else if(gongneng==2)
{
BSP_LCD_SetFont(&Font48);	
	BSP_LCD_DisplayStringAt(2,620,(uint8_t*)"Max:   .  mm", LEFT_MODE);//厚度
	BSP_LCD_DisplayStringAt(2,680,(uint8_t*)"Min:   .  mm", LEFT_MODE);//厚度
	BSP_LCD_DisplayStringAt(2,740,(uint8_t*)"     .  Mpa", LEFT_MODE);//应力	
BSP_LCD_SetFont(&Font24_1);	

}
}
else if(hisindex==1)
{
dishanzi(366,636,59,LCD_COLOR_WHITE);
dishanzi(390,636,60,LCD_COLOR_WHITE);
BSP_LCD_DisplayStringAt(414,636,(uint8_t*)":    ", LEFT_MODE);//增益	
	
dishanzi(366,660,11,LCD_COLOR_WHITE);
dishanzi(390,660,12,LCD_COLOR_WHITE);
BSP_LCD_DisplayStringAt(414,660,(uint8_t*)":    ", LEFT_MODE);//声速

dishanzi(366,684,80,LCD_COLOR_WHITE);
dishanzi(390,684,81,LCD_COLOR_WHITE);
BSP_LCD_DisplayStringAt(414,684,(uint8_t*)":    ", LEFT_MODE);//wendu

}
if(scanhisindex==1)
{
dishanzi(400,636,5,LCD_COLOR_WHITE);
dishanzi(424,636,6,LCD_COLOR_WHITE);//厚度
BSP_LCD_DisplayStringAt(366,670,(uint8_t*)"      mm", LEFT_MODE);//声速
}


BSP_LCD_SetTextColor(color);	

}


void LCDinformationdata(void)
{uint16_t mun;
LCD_ShowFloatNum1(12,0,((float)timeA1)/100.0,4,RED,BLACK,24);//显示第一个峰位置
LCD_ShowFloatNum1(132,0,((float)timeA2)/100.0,4,RED,BLACK,24);//显示第二个峰位置

	mun=thickavg%10;
	LCD_ShowIntNum(145,30,mun,1,RED,BLACK,48);
	mun=(thickavg/10)%10;
	LCD_ShowIntNum(121,30,mun,1,RED,BLACK,48);
		mun=(thickavg/100)%10;
	LCD_ShowIntNum(97,30,mun,1,RED,BLACK,48);	
	LCD_ShowChar(73,30,'.',RED,BLACK,48,0);
		mun=(thickavg/1000)%10;
	LCD_ShowIntNum(49,30,mun,1,RED,BLACK,48);	
			mun=(thickavg/10000)%10;
	LCD_ShowIntNum(25,30,mun,1,RED,BLACK,48);	
		if(modelindex==0||modelindex==3||modelindex==4)//手动
		LCD_ShowFloatNum1(12,90,(float)(agctemp*10)/100,4,RED,BLACK,24);//显示第一个峰位置	
	else
		LCD_ShowFloatNum1(12,90,(float)(paramvaule[0][2]*10)/100,4,RED,BLACK,24);//显示增益	
		LCD_ShowFloatNum1(112,90,((float)paramvaule[3][2])/100,3,RED,BLACK,24);//显示频率	

	
}
void LCDinformationunit(void)
{
LCD_ShowString(77,0,"us",RED,BLACK,24,0);
LCD_ShowString(197,0,"us",RED,BLACK,24,0);
LCD_ShowString(170,35,"mm",RED,BLACK,32,0);	
LCD_ShowString(77,90,"dB",RED,BLACK,24,0);
	LCD_ShowString(166,90,"MHz",RED,BLACK,24,0);
}
void diswavedeploy(void) //显示网格
{
	uint32_t color;
	BSP_LCD_SetTransparency(0, 10);
  color=BSP_LCD_GetTextColor();
	BSP_LCD_SetTextColor(LCD_COLOR_GRAY);
if(scanindex==0)
{
BSP_LCD_DrawHLine(0,463,490);
BSP_LCD_DrawHLine(0,428,490);
BSP_LCD_DrawHLine(0,393,490);
BSP_LCD_DrawHLine(0,358,490);
BSP_LCD_DrawHLine(0,323,490);	
BSP_LCD_DrawHLine(0,288,490);	
BSP_LCD_DrawHLine(0,253,490);	
BSP_LCD_DrawHLine(0,218,490);	
BSP_LCD_DrawHLine(0,183,490);	
BSP_LCD_DrawHLine(0,148,490);

BSP_LCD_DrawVLine(48,113,350);	
BSP_LCD_DrawVLine(96,113,350);	
BSP_LCD_DrawVLine(144,113,350);	
BSP_LCD_DrawVLine(192,113,350);
BSP_LCD_DrawVLine(240,113,350);
BSP_LCD_DrawVLine(288,113,350);
BSP_LCD_DrawVLine(336,113,350);
BSP_LCD_DrawVLine(384,113,350);
BSP_LCD_DrawVLine(432,113,350);	
}
else
{

	
	BSP_LCD_DrawVLine(441,218,245);
	/*
BSP_LCD_DrawHLine(0,408,441);
BSP_LCD_DrawHLine(0,373,441);
BSP_LCD_DrawHLine(0,338,441);
BSP_LCD_DrawHLine(0,303,441);
BSP_LCD_DrawHLine(0,268,441);	
BSP_LCD_DrawHLine(0,233,441);	
BSP_LCD_DrawHLine(0,198,441);	
BSP_LCD_DrawHLine(0,163,441);	
BSP_LCD_DrawHLine(0,128,441);	
BSP_LCD_DrawHLine(0,93,441);
	*/
}	
	


BSP_LCD_SetTextColor(color);	
}



void disbscandeploy(void)
{
	uint32_t color;
  color=BSP_LCD_GetTextColor();
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
BSP_LCD_DrawHLine(0,408,490);
BSP_LCD_DrawVLine(49,408,10);	
BSP_LCD_DrawVLine(147,408,10);
BSP_LCD_DrawVLine(196,408,10);
BSP_LCD_DrawVLine(245,408,10);
BSP_LCD_DrawVLine(294,408,10);
BSP_LCD_DrawVLine(343,408,10);
BSP_LCD_DrawVLine(392,408,10);
BSP_LCD_DrawVLine(441,408,10);

	BSP_LCD_SetTextColor(color);

}
void disxcoordinate(void)
{uint16_t x,y;
BSP_LCD_SetTextColor(LCD_COLOR_GRAY);
	BSP_LCD_SetFont(&Font12);
	if(scanindex==0)//不在扫描状态
	{
		if(gongneng==0)
		{
		if(paramvaule[3][15]==0)
		{
		BSP_LCD_DisplayStringAt(2,467,  (uint8_t*)"     us", LEFT_MODE);//	
		BSP_LCD_DisplayStringAt(82,467,(uint8_t*)"      us", LEFT_MODE);//	
		BSP_LCD_DisplayStringAt(182,467,(uint8_t*)"      us", LEFT_MODE);//
		BSP_LCD_DisplayStringAt(280,467,(uint8_t*)"      us", LEFT_MODE);//
		BSP_LCD_DisplayStringAt(378,467,(uint8_t*)"      us", LEFT_MODE);//
		}
		else
		{
		BSP_LCD_DisplayStringAt(2,467,  (uint8_t*)"     mm", LEFT_MODE);//	
		BSP_LCD_DisplayStringAt(82,467,(uint8_t*)"      mm", LEFT_MODE);//	
		BSP_LCD_DisplayStringAt(182,467,(uint8_t*)"      mm", LEFT_MODE);//
		BSP_LCD_DisplayStringAt(280,467,(uint8_t*)"      mm", LEFT_MODE);//
		BSP_LCD_DisplayStringAt(378,467,(uint8_t*)"      mm", LEFT_MODE);//		
		}
	}else if(gongneng==1)
		{
		BSP_LCD_DisplayStringAt(2,467,  (uint8_t*)"     mm", LEFT_MODE);//	
		BSP_LCD_DisplayStringAt(82,467,(uint8_t*)"      mm", LEFT_MODE);//	
		BSP_LCD_DisplayStringAt(182,467,(uint8_t*)"      mm", LEFT_MODE);//
		BSP_LCD_DisplayStringAt(280,467,(uint8_t*)"      mm", LEFT_MODE);//
		BSP_LCD_DisplayStringAt(378,467,(uint8_t*)"      mm", LEFT_MODE);//	
		}
	}
	else
	{
				BSP_LCD_SetTextColor(LCD_COLOR_GRAY);
				if(paramvaule[3][15]==0)
				{
				BSP_LCD_DisplayStringAt(2,197,  (uint8_t*)"     us", LEFT_MODE);//	
				BSP_LCD_DisplayStringAt(82,197,(uint8_t*)"      us", LEFT_MODE);//	
				BSP_LCD_DisplayStringAt(182,197,(uint8_t*)"      us", LEFT_MODE);//
				BSP_LCD_DisplayStringAt(280,197,(uint8_t*)"      us", LEFT_MODE);//
				BSP_LCD_DisplayStringAt(378,197,(uint8_t*)"      us", LEFT_MODE);//
				}
				else
				{
				BSP_LCD_DisplayStringAt(2,197,  (uint8_t*)"     mm", LEFT_MODE);//	
				BSP_LCD_DisplayStringAt(82,197,(uint8_t*)"      mm", LEFT_MODE);//	
				BSP_LCD_DisplayStringAt(182,197,(uint8_t*)"      mm", LEFT_MODE);//
				BSP_LCD_DisplayStringAt(280,197,(uint8_t*)"      mm", LEFT_MODE);//
				BSP_LCD_DisplayStringAt(378,197,(uint8_t*)"      mm", LEFT_MODE);//				
				}
		if(gongneng==0||gongneng==2)
		{
		y=paramvaule[0][1]-paramvaule[0][0];
		x=paramvaule[0][0]*10;
		}
		else
		{
		y=paramvaule[1][1]-paramvaule[1][0];
		x=paramvaule[1][0]*10;		
		}
		LCD_ShowxNum_pix(1 ,197, x ,3,8,2,LCD_COLOR_YELLOW);	
		x=paramvaule[0][0]*10+y*2;
		LCD_ShowxNum_pix(82 ,197, x ,4,8,3,LCD_COLOR_YELLOW);		
		x=paramvaule[0][0]*10+y*4;	
		LCD_ShowxNum_pix(180,197, x ,4,8,3,LCD_COLOR_YELLOW);
		x=paramvaule[0][0]*10+y*6;	
		LCD_ShowxNum_pix(280,197, x ,4,8,3,LCD_COLOR_YELLOW);
		x=paramvaule[0][0]*10+y*8;	
		LCD_ShowxNum_pix(380,197, x ,4,8,3,LCD_COLOR_YELLOW);
		BSP_LCD_SetTextColor(LCD_COLOR_CYAN);
		BSP_LCD_DrawLine(0,218,480,218);	
		BSP_LCD_DrawLine(0,0,480,0);
				BSP_LCD_DrawLine(480,1,480,800);
		BSP_LCD_SetTextColor(LCD_COLOR_GRAY);
		BSP_LCD_DrawLine(0,195,480,195);	
		BSP_LCD_DrawHLine(0,463,480);
		BSP_LCD_DisplayStringAt(2,467,  (uint8_t*)"     mm", LEFT_MODE);//	
		BSP_LCD_DisplayStringAt(82,467, (uint8_t*)"      mm", LEFT_MODE);//	
		BSP_LCD_DisplayStringAt(182,467,(uint8_t*)"      mm", LEFT_MODE);//
		BSP_LCD_DisplayStringAt(280,467,(uint8_t*)"       mm", LEFT_MODE);//
		BSP_LCD_DisplayStringAt(378,467,(uint8_t*)"       mm", LEFT_MODE);//
		//厚度方向的坐标
		BSP_LCD_DisplayStringAt(447,450,  (uint8_t*)"    mm", LEFT_MODE);//	
		BSP_LCD_DisplayStringAt(447,405,  (uint8_t*)"    mm", LEFT_MODE);//
		BSP_LCD_DisplayStringAt(447,360,  (uint8_t*)"    mm", LEFT_MODE);//		
		BSP_LCD_DisplayStringAt(447,315,  (uint8_t*)"    mm", LEFT_MODE);//
		BSP_LCD_DisplayStringAt(447,270,  (uint8_t*)"    mm", LEFT_MODE);//			
	}
if(scanindex==0)	
{
		if(gongneng==0||gongneng==2)
		{
		y=paramvaule[0][1]-paramvaule[0][0];
		x=paramvaule[0][0]*10;
		}
		else
		{
		y=paramvaule[1][1]-paramvaule[1][0];
		x=paramvaule[1][0]*10;		
		}
}
else
{
	//y=(float)paramvaule[0][8]/44; 
	  y=paramvaule[0][14]; 
	//y=400;
	x=0;	
}	
if(scanindex==0)
{
	BSP_LCD_SetFont(&Font12);
	LCD_ShowxNum_pix(1 ,467, x ,3,8,2,LCD_COLOR_YELLOW);
if(gongneng==0||gongneng==2)	
	x=paramvaule[0][0]*10+y*2;
else
	x=paramvaule[1][0]*10+y*2;
	LCD_ShowxNum_pix(82 ,467, x ,4,8,3,LCD_COLOR_YELLOW);
if(gongneng==0||gongneng==2)
	x=paramvaule[0][0]*10+y*4;	
else
		x=paramvaule[1][0]*10+y*4;	
	LCD_ShowxNum_pix(180,467, x ,4,8,3,LCD_COLOR_YELLOW);
if(gongneng==0||gongneng==2)
	x=paramvaule[0][0]*10+y*6;
else
		x=paramvaule[1][0]*10+y*6;
	LCD_ShowxNum_pix(280,467, x ,4,8,3,LCD_COLOR_YELLOW);
if(gongneng==0||gongneng==2)
	x=paramvaule[0][0]*10+y*8;	
else
		x=paramvaule[1][0]*10+y*8;
	LCD_ShowxNum_pix(380,467, x ,4,8,3,LCD_COLOR_YELLOW);
}
else
{
BSP_LCD_SetFont(&Font12);
	
	LCD_ShowxNum_pix(1 ,467, x ,3,8,2,LCD_COLOR_YELLOW);	
	x=y*2;	
	LCD_ShowxNum_pix(82 ,467, x ,4,8,3,LCD_COLOR_YELLOW);		
	x=y*4;	
	LCD_ShowxNum_pix(180,467, x ,4,8,3,LCD_COLOR_YELLOW);
	x=y*6;	
	LCD_ShowxNum_pix(280,467, x ,5,8,4,LCD_COLOR_YELLOW);
	x=y*8;	
	LCD_ShowxNum_pix(380,467, x ,5,8,4,LCD_COLOR_YELLOW);
	
	y=paramvaule[0][15]/100;
	x=0;	
	LCD_ShowxNum_pix(442 ,450, x ,3,8,2,LCD_COLOR_YELLOW);	
	x=y*2;
	LCD_ShowxNum_pix(442 ,405, x ,3,8,2,LCD_COLOR_YELLOW);
	x=y*4;
	LCD_ShowxNum_pix(442 ,360, x ,3,8,2,LCD_COLOR_YELLOW);	
	x=y*6;
	LCD_ShowxNum_pix(442 ,315, x ,3,8,2,LCD_COLOR_YELLOW);
	x=y*8;
	//LCD_ShowxNum(442 ,115, x ,2,LCD_COLOR_YELLOW);
	LCD_ShowxNum_pix(442 ,270, x ,3,8,2,LCD_COLOR_YELLOW);	
}
							
//BSP_LCD_SetFont(&Font24_1);
}


void LCDxcoordinate(void)//显示时间坐标
{
uint16_t m,y;
	
		m=paramvaule[0][0];
	
	LCD_ShowFloatNum1(0,107,m,4,LIGHTGREEN,BLACK,12);
	m=paramvaule[0][1];
		LCD_ShowFloatNum1(210,107,m,4,LIGHTGREEN,BLACK,12);
}
void Gate_drawRectpeak( )
{
			//s闸门
    	 if(firstloc<paramvaule[0][0]*100)
    	     Gate_startA=1;
    	 else
    	    Gate_startA=((float)(firstloc-paramvaule[0][0]*100))*488/((float)(paramvaule[0][1]*100-paramvaule[0][0]*100))+1;

				if((firstloc+120)<=paramvaule[0][1]*100)
						Gate_lengthA=120*488/((float)(paramvaule[0][1]*100-paramvaule[0][0]*100));//length=((paramvaule[0][4]*10+150)-paramvaule[0][0]*100-start);
						if((firstloc+120)<paramvaule[0][0]*100)
						Gate_lengthA=0;

			
    	 if(Gate_startA>489)
    	    Gate_startA=489;

				if((firstloc+120)>paramvaule[0][1]*100)
							Gate_lengthA=489-Gate_startA;
/////////////////////////第二个////////////////			 
    	 if(secondloc<paramvaule[0][0]*100)
    	     Gate_startS=1;
    	 else
    	    Gate_startS=((float)(secondloc-paramvaule[0][0]*100))*488/((float)(paramvaule[0][1]*100-paramvaule[0][0]*100))+1;

			 
    	 if((secondloc+120)<=paramvaule[0][1]*100)
    	    Gate_lengthS=120*488/((float)(paramvaule[0][1]*100-paramvaule[0][0]*100));

		 if((secondloc+120)<paramvaule[0][0]*100)
					Gate_lengthS=0;

    	 if(Gate_startS>489)
    	    Gate_startS=489;

			 	    	 if((secondloc+120)>paramvaule[0][1]*100)		 
											Gate_lengthS=489-Gate_startS;
    	  BSP_LCD_SetTextColor(LCD_COLOR_TRANSPARENT); //LCD_COLOR_TRANSPARENT 透明
    	  BSP_LCD_DrawRect(Gate_start_parA,185,Gate_length_parA,200);  
    	  Gate_start_parA = Gate_startA;
			 Gate_length_parA=Gate_lengthA;
	    	  BSP_LCD_DrawRect(Gate_start_parS,185,Gate_length_parS,200);  
    	  Gate_start_parS = Gate_startS;
			 Gate_length_parS=Gate_lengthS;			 

			   BSP_LCD_SetTextColor(LCD_COLOR_LIGHTGREEN);//闸门显示 
          BSP_LCD_DrawRect(Gate_startA,185,Gate_lengthA,200);
			 BSP_LCD_SetTextColor(LCD_COLOR_LIGHTBLUE);
          BSP_LCD_DrawRect(Gate_startS,185,Gate_lengthS,200);
}
//////////////////////////////////////////////////
void Gate_drawRect( ) //闸门显示方框 手动时候显示 BSP_LCD_DrawRect(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height); 
    {
    	 
			//s闸门
    	 if(paramvaule[0][4]*10<paramvaule[0][0]*100)
    	     Gate_startA=1;
    	 else
    	    Gate_startA=((float)(paramvaule[0][4]*10-paramvaule[0][0]*100))*488/((float)(paramvaule[0][1]*100-paramvaule[0][0]*100))+1;

				if((paramvaule[0][4]*10+150)<=paramvaule[0][1]*100)
						Gate_lengthA=150*488/((float)(paramvaule[0][1]*100-paramvaule[0][0]*100));//length=((paramvaule[0][4]*10+150)-paramvaule[0][0]*100-start);
						if((paramvaule[0][4]*10+150)<paramvaule[0][0]*100)
						Gate_lengthA=0;

			
    	 if(Gate_startA>489)
    	    Gate_startA=489;

				if((paramvaule[0][4]*10+150)>paramvaule[0][1]*100)
							Gate_lengthA=489-Gate_startA;
/////////////////////////第二个////////////////			 
    	 if(paramvaule[0][5]*10<paramvaule[0][0]*100)
    	     Gate_startS=1;
    	 else
    	    Gate_startS=((float)(paramvaule[0][5]*10-paramvaule[0][0]*100))*488/((float)(paramvaule[0][1]*100-paramvaule[0][0]*100))+1;

			 
    	 if((paramvaule[0][5]*10+150)<=paramvaule[0][1]*100)
    	    Gate_lengthS=150*488/((float)(paramvaule[0][1]*100-paramvaule[0][0]*100));

		 if((paramvaule[0][5]*10+150)<paramvaule[0][0]*100)
					Gate_lengthS=0;

    	 if(Gate_startS>489)
    	    Gate_startS=489;

			 	    	 if((paramvaule[0][5]*10+150)>paramvaule[0][1]*100)		 
											Gate_lengthS=489-Gate_startS;
    	  BSP_LCD_SetTextColor(LCD_COLOR_TRANSPARENT); //LCD_COLOR_TRANSPARENT 透明
    	  BSP_LCD_DrawRect(Gate_start_parA,185,Gate_length_parA,200);  
    	  Gate_start_parA = Gate_startA;
			 Gate_length_parA=Gate_lengthA;
	    	  BSP_LCD_DrawRect(Gate_start_parS,185,Gate_length_parS,200);  
    	  Gate_start_parS = Gate_startS;
			 Gate_length_parS=Gate_lengthS;			 

			   BSP_LCD_SetTextColor(LCD_COLOR_LIGHTGREEN);//闸门显示 
          BSP_LCD_DrawRect(Gate_startA,185,Gate_lengthA,200);
			 BSP_LCD_SetTextColor(LCD_COLOR_LIGHTBLUE);
          BSP_LCD_DrawRect(Gate_startS,185,Gate_lengthS,200);
    }
/////////////////////////////////////////
void Gate_display(void) //闸门显示 
    {
			if(gongneng==1)
			{
    	//伤闸门//  
    	 if(paramvaule[1][3]*10<paramvaule[1][0]*100)
    	     start=1;
    	 else
    	    start=paramvaule[1][3]*10-paramvaule[1][0]*100;
			 if(paramvaule[1][3]*10>paramvaule[1][1]*100)
			 {length=0;}
				 else
				 {
				 if(paramvaule[1][4]*10<=paramvaule[1][1]*100)
						length=paramvaule[1][4]*10-paramvaule[1][3]*10;
				 else 
						length=478-start;
				 if((paramvaule[1][4]*10<=paramvaule[1][0]*100)||(paramvaule[1][4]<=paramvaule[1][3]))
					 length=0;
		     }
    	 Gate_start=  (((float)start*478)/(float)(paramvaule[1][1]*100-paramvaule[1][0]*100))+1;
				 if(paramvaule[1][4]*10<=paramvaule[1][1]*100)
       Gate_length= (((float)length*478)/(float)(paramvaule[1][1]*100-paramvaule[1][0]*100))+1;
				 else
					 Gate_length=478-Gate_start;
    	 if(Gate_start>478)
    	    Gate_start=478;
    	  
    	  BSP_LCD_SetTextColor(LCD_COLOR_TRANSPARENT); //LCD_COLOR_BLACK
    	  BSP_LCD_DrawHLine(1, 165,488);  
        BSP_LCD_SetTextColor(LCD_COLOR_RED);//闸门显示 
			 
			 if(Gate_length!=0)
			 {
        if (Gate_length+Gate_start>477)  
            BSP_LCD_DrawHLine(Gate_start, 165,478-Gate_start);
        else
            BSP_LCD_DrawHLine(Gate_start, 165,Gate_length);
			}
			////////////////////// 
    	//底波闸门//  
     	 if(paramvaule[1][5]*10<paramvaule[1][0]*100)
    	     start=1;
    	 else
    	    start=paramvaule[1][5]*10-paramvaule[1][0]*100;
			 if(paramvaule[1][5]*10>paramvaule[1][1]*100)
			 {length=0;}
				 else
				 {
				 if(paramvaule[1][6]*10<=paramvaule[1][1]*100)
						length=paramvaule[1][6]*10-paramvaule[1][5]*10;
				 else 
						length=478-start;
				 if((paramvaule[1][6]*10<=paramvaule[1][0]*100)||(paramvaule[1][6]<=paramvaule[1][5]))
					 length=0;
		     }
    	 Gate_start=  (((float)start*478)/(float)(paramvaule[1][1]*100-paramvaule[1][0]*100))+1;
				 if(paramvaule[1][6]*10<=paramvaule[1][1]*100)
       Gate_length= (((float)length*478)/(float)(paramvaule[1][1]*100-paramvaule[1][0]*100))+1;
				 else
					 Gate_length=478-Gate_start;
    	 if(Gate_start>478)
    	    Gate_start=478;
    	  
    	  BSP_LCD_SetTextColor(LCD_COLOR_TRANSPARENT); //LCD_COLOR_BLACK
    	  BSP_LCD_DrawHLine(1,125,488);  
        BSP_LCD_SetTextColor(LCD_COLOR_GREEN);//闸门显示 
			 
			 if(Gate_length!=0)
			 {
        if (Gate_length+Gate_start>477)  
            BSP_LCD_DrawHLine(Gate_start, 125,478-Gate_start);
        else
            BSP_LCD_DrawHLine(Gate_start, 125,Gate_length);
			}
			////////////////////// 
		}
else if(gongneng==0)	
{
	if(scanindex==0)
	{
    	//底波闸门//  
    	 if(paramvaule[0][4]*10<paramvaule[0][0]*100)
    	     start=0;
    	 else
    	    start=paramvaule[0][4]*10-paramvaule[0][0]*100;
			 if(paramvaule[0][4]*10>paramvaule[0][1]*100)
			 {length=0;}
				 else
				 {
				 if(paramvaule[0][5]*10<=paramvaule[0][1]*100)
						length=(paramvaule[0][5]*10-paramvaule[0][0]*100-start);
				 else 
						length=(paramvaule[0][1]*100-start);
				 if(paramvaule[0][5]*10<=paramvaule[0][0]*100)
					 length=0;
		     }
    	 Gate_start=  (start/kp)+1;
       Gate_length= length/kp;
    	 if(Gate_start>488)
    	    Gate_start=488;
    	  
    	  BSP_LCD_SetTextColor(LCD_COLOR_TRANSPARENT); //LCD_COLOR_BLACK
    	  BSP_LCD_DrawHLine(1, 125,488);  
        BSP_LCD_SetTextColor(LCD_COLOR_GREEN);//闸门显示 
			 
			 if(Gate_length!=0)
			 {
        if (Gate_length+Gate_start>487)  
            BSP_LCD_DrawHLine(Gate_start, 125,488-Gate_start);
        else
            BSP_LCD_DrawHLine(Gate_start, 125,Gate_length);
			}
		}
	else
	{

	BSP_LCD_SetTextColor(LCD_COLOR_TRANSPARENT); //LCD_COLOR_BLACK
		BSP_LCD_SetTextColor(LCD_COLOR_TRANSPARENT); //LCD_COLOR_BLACK
		BSP_LCD_DrawVLine(preSCAN_cursor, 219,243);
		BSP_LCD_DrawVLine(preSCAN_cursor, 219,243);
		for(x=0;x<100;x++)
			i=0;
	BSP_LCD_SetTextColor(LCD_COLOR_RED);//B扫描光标红色显示
		BSP_LCD_SetTextColor(LCD_COLOR_RED);//B扫描光标红色显示
		BSP_LCD_DrawVLine(SCAN_cursor, 219,243);
BSP_LCD_DrawVLine(SCAN_cursor, 219,243);		
		SCAN_cursorindex=0;
	}
			////////////////////// 
}	
    }
void diswave(int16_t *data,int16_t *data1)
{
	
uint16_t mun,i,sub;
uint32_t color;

color=BSP_LCD_GetTextColor();
	i=0;
	draw=0;
if(scanindex==0)
{	
	clearfirstwaveindex(prefirstcoordinate);
	clearsecondwaveindex(presecondcoordinate);
for(mun=2;mun<479;mun++)
	{
BSP_LCD_SetTextColor(LCD_COLOR_TRANSPARENT);//LCD_COLOR_BLACK LCD_COLOR_TRANSPARENT
BSP_LCD_DrawLine(mun,data1[i],mun+1,data1[i+1]);
BSP_LCD_SetTextColor(LCD_COLOR_LIGHTYELLOW);
BSP_LCD_DrawLine(mun,data[i],mun+1,data[i+1]);
		data1[i]=data[i];
i++;		
}
		data1[i]=data[i];	
diswavedeploy();
	if(gongneng==0)
	{
	if(modelindex==0&&paramvaule[2][8]>100)
		Gate_drawRectpeak(); 
	if(modelindex==1)
		Gate_display();
	if(modelindex==2)
		Gate_drawRect(); 
	disfirstwaveindex(disfirstcoordinate);
	dissecondwaveindex(dissecondcoordinate);
	}
	else if(gongneng==1)
					Gate_display();
	
}
else
{
	for(mun=2;mun<479;mun++)
	{
BSP_LCD_SetTextColor(LCD_COLOR_TRANSPARENT);//LCD_COLOR_BLACK LCD_COLOR_TRANSPARENT
BSP_LCD_DrawLine(mun,data1[i],mun+1,data1[i+1]);
BSP_LCD_SetTextColor(LCD_COLOR_LIGHTYELLOW);
BSP_LCD_DrawLine(mun,data[i],mun+1,data[i+1]);
		data1[i]=data[i];
i++;
}
	Gate_display();
		data1[i]=data[i];	
	i=0;
BSP_LCD_SetTextColor(LCD_COLOR_LIGHTGREEN);
for(mun=2;mun<441;mun++)
	{
		sub=463-DISSCAN_BUF[i];
		if(sub<1)
			sub=1;
		if(sub>463)
			sub=463;				
		BSP_LCD_DrawVLine(mun,DISSCAN_BUF[i],463-DISSCAN_BUF[i]);		
i++;
}
	
diswavedeploy();
}
	
BSP_LCD_SetTextColor(color);
draw=1;
}
/////////////////////////////////////////////////

void drawblock(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height)
{
uint32_t color;
color=BSP_LCD_GetTextColor();
BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
BSP_LCD_FillRect(Xpos,Ypos,Width,Height);	
HAL_Delay(10);
BSP_LCD_SetTextColor(color);
}

void drawcolorblock(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height,uint32_t discolor)
{
uint32_t color;

color=BSP_LCD_GetTextColor();
BSP_LCD_SetTextColor(discolor);
BSP_LCD_FillRect(Xpos,Ypos,Width,Height);	
	HAL_Delay(10);
BSP_LCD_SetTextColor(color);
}
char getkeymun(uint16_t Xpos,uint16_t Ypos)
{char keymun1=0xff;
if(hisindex==0&&(modifyparament==1||timemodify==1))//不在B扫描和历史数据查看的状态下,在参数修改状态下
{
	 if((caltouch_x>Xpos+1&&caltouch_x<Xpos+80)&&(caltouch_y>Ypos+80&&caltouch_y<Ypos+160))	 
		keymun1=0;	 
	 else if((caltouch_x>Xpos+80&&caltouch_x<Xpos+160)&&(caltouch_y>Ypos+80&&caltouch_y<Ypos+160))
					keymun1=1;
			 	 else if
					((caltouch_x>Xpos+160&&caltouch_x<Xpos+240)&&(caltouch_y>Ypos+80&&caltouch_y<Ypos+160))
						keymun1=2;
			 	 else if
						((caltouch_x>Xpos+240&&caltouch_x<Xpos+320)&&(caltouch_y>Ypos+80&&caltouch_y<Ypos+160))
								keymun1=3;
							else if((caltouch_x>Xpos+1&&caltouch_x<Xpos+80)&&(caltouch_y>Ypos+160&&caltouch_y<Ypos+240))	 
										keymun1=4;	 
									 else if((caltouch_x>Xpos+80&&caltouch_x<Xpos+160)&&(caltouch_y>Ypos+160&&caltouch_y<Ypos+240))
													keymun1=5;
												 else if
													((caltouch_x>Xpos+160&&caltouch_x<Xpos+240)&&(caltouch_y>Ypos+160&&caltouch_y<Ypos+240))
														keymun1=6;
												 else if
														((caltouch_x>Xpos+240&&caltouch_x<Xpos+320)&&(caltouch_y>Ypos+160&&caltouch_y<Ypos+240))
																keymun1=7;
																	else if((caltouch_x>Xpos+1&&caltouch_x<Xpos+80)&&(caltouch_y>Ypos+240&&caltouch_y<Ypos+317))	 
																				keymun1=8;	 
																			 else if((caltouch_x>Xpos+80&&caltouch_x<Xpos+160)&&(caltouch_y>Ypos+240&&caltouch_y<Ypos+317))
																							keymun1=9;
																						 else if
																							((caltouch_x>Xpos+160&&caltouch_x<Xpos+240)&&(caltouch_y>Ypos+240&&caltouch_y<Ypos+317))//小数点
																								keymun1=10;
																						 else if
																								((caltouch_x>Xpos+240&&caltouch_x<Xpos+320)&&(caltouch_y>Ypos+240&&caltouch_y<Ypos+317))//返回
																										keymun1=13;	
																											else if((caltouch_x>Xpos+160&&caltouch_x<Xpos+240)&&(caltouch_y>Ypos+1&&caltouch_y<Ypos+80))	//确认
																														keymun1=11;	
																											else if((caltouch_x>Xpos+240&&caltouch_x<Xpos+320)&&(caltouch_y>Ypos+1&&caltouch_y<Ypos+80))	//清除 
																														keymun1=12;
																											else
																														keymun1=14;
																										}

		return keymun1;


}
char gethiskeymun(void)
{
char mun,x,y;
	if((caltouch_x>0&&caltouch_x<120))
		x=0;
	else if((caltouch_x>120&&caltouch_x<240))
			x=1;
		else if((caltouch_x>240&&caltouch_x<360))
				x=2;
		
		if(caltouch_y>480&&caltouch_y<520)
				y=0;
			else if((caltouch_y>520&&caltouch_y<560))	
					y=1;				
				else if((caltouch_y>560&&caltouch_y<600))	
						y=2;
				else if((caltouch_y>600&&caltouch_y<640))	
						y=3;	
				else if((caltouch_y>640&&caltouch_y<680))	
						y=4;		
				else if((caltouch_y>680&&caltouch_y<720))	
						y=5;
				else if((caltouch_y>720&&caltouch_y<760))	
						y=6;
				else if((caltouch_y>760&&caltouch_y<799))	
						y=7;				
mun=y*3+x;
return mun;				
}
void Dosomething(uint16_t key)//自动0x0E 暂停0x0D 保存0x0B
{
  uint16_t x,m,valback,modifydata;

	HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	
	if(hisindex==1||scanhisindex==1)//在历史数据界面下
	{
	if((caltouch_x>360&&caltouch_x<479)&&(caltouch_y>720&&caltouch_y<760)&&confirmindex==0)//历史数据，不在确认状态，返回
						{
									if(confirmindex==0&&modifyparament==0)
									{
										if(scanindex==0)//不在扫描状态
										{

														hisindex=0;
														drawblock(1,196,479,603);
														disdeploy();
														disinformationunit();
														disxcoordinate();
														diswavedeploy();
														drawmainbroad();
														dismeun(page);
														dispageparament(LCD_COLOR_WHITE);	
												
										}
										else
										{


													scanhisindex=0;
											drawblock(1,196,479,603);
												//	drawblock(1,481,479,318);
													disdeploy();
													disinformationunit();
													disxcoordinate();
													diswavedeploy();
													drawmainbroad();
													dismeun(page);
													dispageparament(LCD_COLOR_WHITE);															
												
											
										}
									}					
						}
else 	if((caltouch_x>0&&caltouch_x<360)&&(caltouch_y>480&&caltouch_y<799)&&confirmindex==0)//查看数据
			{
						HAL_Delay(200);
			hiskeymun=gethiskeymun();
				hismun=hispage*24+hiskeymun;
									dishisdeploy();
				if(hisindex==1||scanhisindex==1)
				{
				disinformationdata();
				disinformationunit();
				}
			//	HAL_Delay(100);
						dispagehisname(hispage);
		
				if(hisindex==1)
				{
				//if((hiskeymun)<ubNumberOfFiles)
				if((hismun)<ubNumberOfFiles)
				{
				 //dishisname(hiskeymun,1);
					dishisname(hismun,1);
					readhisdataindex=1;
				}
			}
				else if(scanhisindex==1)
				{
				//if((hiskeymun)<ubNumberOftxtFiles)
					if((hismun)<ubNumberOftxtFiles)
				{
				//dishisname(hiskeymun,1);
        dishisname(hismun,1);						
				readhisdataindex=1;
				}					
				}
				
			}
			else if((caltouch_x>360&&caltouch_x<479)&&(caltouch_y>760&&caltouch_y<799)&&confirmindex==0)//删除数据
			{
				if(hisindex==1||scanhisindex==1)//历史界面
					{
						deleteindex=1;//是
						confirmindex=1;//在确认显示界面，界面不刷新
						disconfirm(300,130,deleteindex);
						
					}
			}
			else if(confirmindex==1&&(caltouch_x>300&&caltouch_x<370)&&(caltouch_y>145&&caltouch_y<210))//确认删除
			{
				if(confirmindex==0)
				readhisdataindex=1;
			else 
			{
				confirmindex=0;
					drawblock(298,128,153,102);	
						delhisdataindex=1;						
				disxcoordinate();
		
			}				
			}
			else if(confirmindex==1&&(caltouch_x>370&&caltouch_x<450)&&(caltouch_y>145&&caltouch_y<210))//取消删除
			{
				if(confirmindex==0)
				readhisdataindex=1;
			else 
			{
				confirmindex=0;
					drawblock(298,128,153,102);									
				disxcoordinate();
				
			}			
			
			}
	else 	if((caltouch_x>390&&caltouch_x<450)&&(caltouch_y>490&&caltouch_y<565)&&confirmindex==0)//上翻
				{

				if(hispage>0)				
					hispage--;
				dishisdeploy();
				dispagehisname(hispage);
				if(scanindex==0)
				disinformationunit();
		     

				
				}
		else 	if((caltouch_x>390&&caltouch_x<450)&&(caltouch_y>570&&caltouch_y<640)&&confirmindex==0)//下翻
					{

					if((hispage+1)*24<=(ubNumberOfFiles-1))
						hispage++;
					dishisdeploy();
						dispagehisname(hispage);
					if(scanindex==0)
					disinformationunit();
				
          }
									
	}
	else
	if(hisindex==0&&pagekeyindex==0&&modifyparament==0&&timemodify==0)//不在B扫描和历史数据查看的状态下,不在参数修改状态下，如果按在参数上进入修改状态
	{
		if((caltouch_x>1&&caltouch_x<120)&&(caltouch_y>480&&caltouch_y<545)&&hisindex==0&&scanindex==0)//不在历史界面下切换到测厚,探伤，应力功能菜单切换
		{
			if(gongneng==2)
			{
		gongneng=0;
			agctemp=agctempstore;
			drawblock(2,611,477,188);
			//消除探伤闸门
			BSP_LCD_SetTextColor(LCD_COLOR_TRANSPARENT); //LCD_COLOR_BLACK
    	  BSP_LCD_DrawHLine(1, 165,488); 
				BSP_LCD_DrawHLine(1, 135,488);
			diswavedeploy();			
			disdeploy();
			disinformationunit();
			disxcoordinate();
			drawmainbroad();			
			dismeun(page);
			dispageparament(LCD_COLOR_WHITE);
			}else if(gongneng==1)
			{
			 if(gongneng==0)
					agctempstore=agctemp;
				gongneng=2;	
			
			diswavedeploy();
			disxcoordinate();	
					
			disdeploy();
			 drawblock(2,611,477,188);
				drawmainbroad();
			 disinformationunit();	
					for(yinglicount=0;yinglicount<1000;yinglicount++)
						yingli[yinglicount]=0;
					yinglicount=0;			
			}
			else if(gongneng==0)
			{
			if(gongneng==0)
					agctempstore=agctemp;
			gongneng=1;	
			page=0;
			drawblock(2,611,477,188);
			BSP_LCD_SetTextColor(LCD_COLOR_TRANSPARENT); //LCD_COLOR_BLACK
    	  BSP_LCD_DrawHLine(1, 110,488); 
				BSP_LCD_DrawHLine(1, 70,488);
			diswavedeploy();
			disxcoordinate();	
			disinformationunit();			
			disdeploy();
			dismeun(page);
			dispageparament(LCD_COLOR_WHITE);
			drawmainbroad();			
			
			}
		}
		else if((caltouch_x>120&&caltouch_x<240)&&(caltouch_y>480&&caltouch_y<545)||keymun==0x0D)//暂停
		{
						if(pause==0)
								pause=1;
						else
								pause=0;
						drawmainbroad();
		}
					else if(((caltouch_x>1&&caltouch_x<106)&&(caltouch_y>545&&caltouch_y<610)&&hisindex==0)||(keymun==0x0E&&hisindex==0))//手动自动切换
					{

							if(hisindex==0&&gongneng==0)//不在历史数据,在测厚界面
								{
								if(confirmindex==0)
								{
											if(keymun==0x0E) 
											{
												 if(modelindex==0)
											 {
												 	if(paramvaule[0][3]==0)//原始波形
													paramvaule[0][3]=1;
													else	
													{														
													modelindex=3;
													paramvaule[0][3]=1;
													}
													if(scanindex==0)
															drawblock(2,114,477,350);
												//	 else
														// drawblock (1,59,430,350);
														
											 }
											else if(modelindex==3)
											 {
												paramvaule[0][3]=0;				
												 modelindex=0;
												 if(scanindex==0)
															drawblock(2,114,477,350);
													// else
														// drawblock (1,58,430,350);
											 }
												else	
												{
												paramvaule[0][3]=0;				
												 modelindex=0;
												 if(scanindex==0)
															drawblock(2,114,477,350);												
												}													
											
											}
												else
												{	
													 if(modelindex==0)
											 {
												 	if(paramvaule[0][3]==0)//原始波形
													paramvaule[0][3]=1;
													else	
													{														
													modelindex=3;
													paramvaule[0][3]=1;
													}
													if(scanindex==0)
															drawblock(2,114,477,350);
												//	 else
														// drawblock (1,59,430,350);
														
											 }
											else if(modelindex==3)
											 {
												paramvaule[0][3]=0;				
												 modelindex=1;
												 if(scanindex==0)
															drawblock(2,114,477,350);
													// else
														// drawblock (1,58,430,350);
											 }
											 else if(modelindex==1)
											 {
												 modelindex=2;
												 if(scanindex==0)
															drawblock(2,114,477,350);
													// else
														// drawblock (1,58,430,350);
											 }
											 else
											 {
												 modelindex=0;
													if(scanindex==0)
															drawblock(2,114,477,350);
													// else
													//	 drawblock (1,58,430,350);					 
											 }
										 }
										 }
									 }
							drawmainbroad();					
					}
						else if((caltouch_x>106&&caltouch_x<212)&&(caltouch_y>545&&caltouch_y<610)&&hisindex==0&&gongneng==0)//测厚模式下A扫描 B扫描切换
						{
							drawblock(1,1,478,474);

						if(scanindex==0)//切换到B扫
						{
								scanindex=1;
							
							for(i=0;i<488;i++)
							CH3_BUF[i]=233;
							for(x=0;x<20000;x++)
								 SCAN_BUF[x]=0;
							encoder=10000;	
							if(FATFS_LinkDriver(&SD_Driver, SD_Path) == 0)
								{
								ubNumberOftxtFiles = Storage_GetDirectoryBittxtFiles("", pDirectoryFiles);
								}

								FATFS_UnLinkDriver(SD_Path);
								 f_close(&MyFile);							
						}
						else//切换到A扫
						{
						for(i=0;i<488;i++)
							CH3_BUF[i]=466;
								scanindex=0;
							disdeploy();
							disinformationunit();
							if(FATFS_LinkDriver(&SD_Driver, SD_Path) == 0)
								{
								ubNumberOfFiles = Storage_GetDirectoryBitmapFiles("", pDirectoryFiles);
								}

								FATFS_UnLinkDriver(SD_Path);
								 f_close(&MyFile);
						}
						disdeploy();
						disxcoordinate();
						diswavedeploy();
						      drawmainbroad();
											dismeun(page);
										dispageparament(LCD_COLOR_WHITE);
									
				
					//	disbat(avg_adc);
								//		disparament(gongneng,paramentmun,1);
							
						}	
				else if(((caltouch_x>240&&caltouch_x<360)&&(caltouch_y>545&&caltouch_y<610)&&hisindex==0&&gongneng==0)||(keymun==0x0b&&hisindex==0&&gongneng==0))//保存数据	,测厚模式下
				{
					if(confirmindex==0)		
							saveindex=1;	
				}	
				else if((caltouch_x>360&&caltouch_x<479)&&(caltouch_y>545&&caltouch_y<610)&&hisindex==0&&gongneng==0)//校准	测厚模式下
				{
						if(modifyparament==0&&scanindex==0&&hisindex==0)//不在扫描，修改参数，历史数据显示界面
							{
								if(confirmindex==0)
								{
								hisindex=0;
								calibrationindex=1;
									wavetempstore=paramvaule[0][3];
									calibrationcount=0;
									modelindex=0;
									
								}
							}
				}
				else if((caltouch_x>360&&caltouch_x<479)&&(caltouch_y>480&&caltouch_y<545)&&hisindex==0)//屏校准
				{
						if(modifyparament==0&&scanindex==0&&hisindex==0)//不在扫描，修改参数，历史数据显示界面
							{
								if(confirmindex==0)
								{
								hisindex=0;
								discalibrationindex=1;
									modelindex=0;
							HAL_Delay(1000);

									  HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
								}
							}
				}										
						else if((caltouch_x>240&&caltouch_x<360)&&(caltouch_y>480&&caltouch_y<545)&&gongneng==0)//历史数据测厚模式下
						{
									if(confirmindex==0&&modifyparament==0)
									{
										if(scanindex==0)//不在扫描状态
										{
												if(hisindex==0)
												{
													hisindex=1;
													hispage=0;												
													dishisdeploy();
													disinformationunit();
													dispagehisname(hispage);
												}
	
										}
										else
										{
											if(scanhisindex==0)
											{
													scanhisindex=1;
													hispage=0;
													dishisdeploy();
												disinformationunit();
													dispagehisname(hispage);
															
											}

											
										}
									}					
						}	
							else	if((caltouch_x>340&&caltouch_x<460)&&(caltouch_y>620&&caltouch_y<710)&&hisindex==0&&(gongneng==0||gongneng==1))		//上翻测厚功能
							{
								if(gongneng==0&&scanindex==0)//测厚功能下且不在扫描状态
								{
									if(page==0)
									{
										page=2;
										
									}
									else
									{
										page--;
										
									
								  }
										dismeun(page);
										dispageparament(LCD_COLOR_WHITE);
										
								}
								else if(gongneng==0&&scanindex==1)
								{
									if(scandirection==0)
										scandirection=1;
									else
										scandirection=0;
									drawblock(1,219,440,244);	
								encoder=10000;
								for(x=0;x<20000;x++)
									 SCAN_BUF[x]=0;
									if(FATFS_LinkDriver(&SD_Driver, SD_Path) == 0)
											{
											ubNumberOftxtFiles = Storage_GetDirectoryBittxtFiles("", pDirectoryFiles);
											}

											FATFS_UnLinkDriver(SD_Path);
											 f_close(&MyFile);									
										dismeun(page);
										dispageparament(LCD_COLOR_WHITE);
								} else if(gongneng==1)
								{
								
										if(page==0)
											page=1;
										else
											page=0;
										dismeun(page);
										dispageparament(LCD_COLOR_WHITE);
								}
							}	
							else	if((caltouch_x>340&&caltouch_x<460)&&(caltouch_y>710&&caltouch_y<790)&&hisindex==0&&(gongneng==0||gongneng==1))		//下翻测厚功能
							{
								if(gongneng==0&&scanindex==0)//测厚功能下且不在扫描状态
								{
									if(page>=2)
									{
										page=0;
										
									}
									else
									{
										page++;
										
									}
											dismeun(page);
										dispageparament(LCD_COLOR_WHITE);
														
								}	
								else if(gongneng==0&&scanindex==1)
								{
									if(scandirection==0)
										scandirection=1;
									else
										scandirection=0;	
								encoder=10000;
									drawblock(1,219,440,244);	
								for(x=0;x<20000;x++)
									 SCAN_BUF[x]=0;	
									if(FATFS_LinkDriver(&SD_Driver, SD_Path) == 0)
											{
											ubNumberOftxtFiles = Storage_GetDirectoryBittxtFiles("", pDirectoryFiles);
											}

											FATFS_UnLinkDriver(SD_Path);
											 f_close(&MyFile);
										dismeun(page);
										dispageparament(LCD_COLOR_WHITE);
								}else if(gongneng==1)
								{
								
										if(page==0)
											page=1;
										else
											page=0;
											dismeun(page);
										dispageparament(LCD_COLOR_WHITE);
								}
								
							}									
//以下是参数的按键
	if(gongneng==0||gongneng==1)
	{
	 if((caltouch_x>355&&caltouch_x<400)&&(caltouch_y>64&&caltouch_y<90))	//修改年	
	 {
	 if(gongneng==0&&scanindex==0&&modifyparament==0)	//在测厚功能下，不在扫描和参数修改状态下
		{
		timemodify=1;
		timemodifything=1;//1年2月 3日 4时 5分 6秒
			paramentmodifyval=0;
							paramentmodifybit=0;
				paramentmodifydisbit=0;
		}
	 }
	 else 	 if((caltouch_x>400&&caltouch_x<445)&&(caltouch_y>64&&caltouch_y<90))	//修改月
	 {
	 if(gongneng==0&&scanindex==0&&modifyparament==0)	//在测厚功能下，不在扫描和参数修改状态下
		{
		timemodify=1;
		timemodifything=2;//1年2月 3日 4时 5分 6秒
			paramentmodifyval=0;
							paramentmodifybit=0;
				paramentmodifydisbit=0;
		}	 
	 }	
	 else 	 if((caltouch_x>445&&caltouch_x<479)&&(caltouch_y>64&&caltouch_y<90))	//修改日
	 {
	 if(gongneng==0&&scanindex==0&&modifyparament==0)	//在测厚功能下，不在扫描和参数修改状态下
		{
		timemodify=1;
		timemodifything=3;//1年2月 3日 4时 5分 6秒
			paramentmodifyval=0;
							paramentmodifybit=0;
				paramentmodifydisbit=0;
		}	 
	 }
	 else 	 if((caltouch_x>325&&caltouch_x<375)&&(caltouch_y>31&&caltouch_y<62))	//修改时
	 {
	 if(gongneng==0&&scanindex==0&&modifyparament==0)	//在测厚功能下，不在扫描和参数修改状态下
		{
		timemodify=1;
		timemodifything=4;//1年2月 3日 4时 5分 6秒
			paramentmodifyval=0;
							paramentmodifybit=0;
				paramentmodifydisbit=0;
		}	 
	 }
	 else 	 if((caltouch_x>375&&caltouch_x<425)&&(caltouch_y>31&&caltouch_y<62))	//修改分
	 {
	 if(gongneng==0&&scanindex==0&&modifyparament==0)	//在测厚功能下，不在扫描和参数修改状态下
		{
		timemodify=1;
		timemodifything=5;//1年2月 3日 4时 5分 6秒
			paramentmodifyval=0;
							paramentmodifybit=0;
				paramentmodifydisbit=0;
		}	 
	 }	
	 else 	 if((caltouch_x>425&&caltouch_x<470)&&(caltouch_y>31&&caltouch_y<62))	//修改秒
	 {
	 if(gongneng==0&&scanindex==0&&modifyparament==0)	//在测厚功能下，不在扫描和参数修改状态下
		{
		timemodify=1;
		timemodifything=6;//1年2月 3日 4时 5分 6秒
			paramentmodifyval=0;
		}	 
	 }	 
  else
	 if((caltouch_x>55&&caltouch_x<159)&&(caltouch_y>610&&caltouch_y<673)&&timemodify==0)
	 {
		 if(gongneng==0&&scanindex==0)		
		{	
			if(page==0)				
				paramentmun=0;	
			else if(page==1)
				paramentmun=10;
			else if(page==2)
				paramentmun=13;
		}
		else if(gongneng==1&&scanindex==0)
		{
			if(page==0)				
				paramentmun=0;	
			else if(page==1)
				paramentmun=2;		
		
		}
		else if(gongneng==0&&scanindex==1)
			paramentmun=14;
				
		paramentmodifyval=0;
		modifyparament=1;	
		 paramentmodifybit=0;
		 paramentmodifydisbit=0;
	 }		 
	 else if((caltouch_x>214&&caltouch_x<319)&&(caltouch_y>610&&caltouch_y<673)&&timemodify==0)
	 {
			if(gongneng==0&&scanindex==0)		
		{	
			if(page==0)								
				paramentmun=1;
			else if(page==1)	
							paramentmun=9;
					else if(page==2)	
									paramentmun=6;				
				paramentmodifyval=0;
				modifyparament=1;	
				paramentmodifybit=0;
				paramentmodifydisbit=0;			
		}	
			else if(gongneng==0&&scanindex==1)//在测厚扫描状态下
			{
			paramentmun=15;
						paramentmodifyval=0;
		modifyparament=1;	
		 paramentmodifybit=0;
		 paramentmodifydisbit=0;
			}		else if(gongneng==1&&scanindex==0)
		{
			if(page==0)				
				paramentmun=1;	
			else if(page==1)
				paramentmun=7;		
						paramentmodifyval=0;
		modifyparament=1;	
		 paramentmodifybit=0;
		 paramentmodifydisbit=0;		
		}	
		
	 }	
	 else if((caltouch_x>55&&caltouch_x<159)&&(caltouch_y>673&&caltouch_y<736)&&timemodify==0)
	 {	
		if(gongneng==0&&scanindex==0)	
		{	
		if(page==0)
			paramentmun=4;	
		else if(page==1)
			paramentmun=12;
				else if(page==2)
								paramentmun=8;
						paramentmodifyval=0;
		modifyparament=1;	
		 paramentmodifybit=0;
		 paramentmodifydisbit=0;
	 }
	else if(gongneng==1&&scanindex==0)
		{
			if(page==0)				
				paramentmun=3;	
			else if(page==1)
				paramentmun=8;		
		paramentmodifyval=0;
		modifyparament=1;	
		 paramentmodifybit=0;
		 paramentmodifydisbit=0;				
	
		}	

	 }		
	 else if((caltouch_x>214&&caltouch_x<319)&&(caltouch_y>673&&caltouch_y<736)&&timemodify==0)
	 {		 
		if(gongneng==0&&scanindex==0)	
		{	
		if(page==0)
			paramentmun=5;	
		else if(page==1)
			paramentmun=11;
		if(page==0||page==1)
		{
							paramentmodifyval=0;
					modifyparament=1;	
					 paramentmodifybit=0;
					 paramentmodifydisbit=0;
		}
	  }
		else if(gongneng==1&&scanindex==0)
		{
			if(page==0)				
				paramentmun=4;	
			else if(page==1)
				paramentmun=9;		
		if(page==0||page==1)
		{
							paramentmodifyval=0;
					modifyparament=1;	
					 paramentmodifybit=0;
					 paramentmodifydisbit=0;
		}
		}	

	 }		
	 else if((caltouch_x>55&&caltouch_x<159)&&(caltouch_y>736&&caltouch_y<799)&&timemodify==0)
			 {		 
		if(gongneng==0&&scanindex==0)	
		{	
		if(page==0)
			paramentmun=2;	
		else if(page==1)
			paramentmun=3;
			if(page==0||page==1)
			{
					paramentmodifyval=0;
					modifyparament=1;	
					 paramentmodifybit=0;
					 paramentmodifydisbit=0;
			}		
	 }if(gongneng==1&&scanindex==0)
		{
			if(page==0)	
			{				
				paramentmun=5;
							paramentmodifyval=0;
					modifyparament=1;	
					 paramentmodifybit=0;
					 paramentmodifydisbit=0;
			}				
		}		
			 
	 }		
	 else if((caltouch_x>214&&caltouch_x<319)&&(caltouch_y>736&&caltouch_y<799)&&timemodify==0)
	 {		 
			if(gongneng==0&&scanindex==0)		
		{	
			if(page==0)						
				paramentmun=7;		
				else if(page==1)	
							paramentmun=16;
				else if(page==2)
				{
								defaultindex=1;
					paramentmun=8;
				}
			
			if(page==0||page==1||page==2)
			{
					paramentmodifyval=0;
					modifyparament=1;	
					 paramentmodifybit=0;
					 paramentmodifydisbit=0;
			}
			
		}else if(gongneng==1&&scanindex==0)
			{
		if(page==0)
		{
			paramentmun=6;	
					paramentmodifyval=0;
					modifyparament=1;	
					 paramentmodifybit=0;
					 paramentmodifydisbit=0;
		}
		}		
	 }	

							if(modifyparament==1||timemodify==1)//选择参数，状态改变
					{
						if(modifyparament==1&&timemodify==0)
						{
						dispageparament(LCD_COLOR_WHITE);  
					  disparament(gongneng,paramentmun,1); 
						}
					//	drawblock(230,481,248,318);
						drawkeybroad(key_x,key_y);
					}
	
				}
	
	}
	else
if(hisindex==0&&(modifyparament==1||timemodify==1))//不在B扫描和历史数据查看的状态下,在参数修改状态下
{
	
if(page==1&&paramentmun==3&&scanindex==0&&gongneng==0&&modifyparament==1)//测厚下面的波形选择菜单键盘
{
	if((caltouch_x>key_x+5&&caltouch_x<key_x+60)&&(caltouch_y>key_y+5&&caltouch_y<key_y+45))	//原始波
	{
		paramvaule[0][paramentmun]=0;
		if(paramvaule[0][4]>5)
				paramvaule[0][4]=paramvaule[0][4]-5;
		else
				paramvaule[0][4]=0;
					modifyparament=0;

	}
	else if((caltouch_x>key_x+5&&caltouch_x<key_x+60)&&(caltouch_y>key_y+45&&caltouch_y<key_y+85))//包洛波
	{
		paramvaule[0][paramentmun]=1;
		if((paramvaule[0][5]-paramvaule[0][4])>5)
				paramvaule[0][4]=paramvaule[0][4]+5;
		else
				paramvaule[0][4]=paramvaule[0][5]-5;
		modifyparament=0;
	}
	if(modifyparament==0)
	{
			
			drawblock(key_x-1,key_y,321,320);
				dismeun(page);
							dispageparament(LCD_COLOR_WHITE);  
					  disparament(gongneng,paramentmun,1); 
			disinformationunit();
			drawmainbroad();
			disdeploy();
					
			programparamentflash();
	}
}
else if(page==1&&(paramentmun==10||paramentmun==11||paramentmun==12||paramentmun==16)&&scanindex==0&&gongneng==0&&modifyparament==1)//测厚下面的材料选择
{
	if(paramentmun==10)
	{
		if((caltouch_x>key_x+5&&caltouch_x<key_x+100)&&(caltouch_y>key_y+5&&caltouch_y<key_y+45))	//碳钢
		{
		paramvaule[0][paramentmun]=0;
		modifyparament=0;		
		}
		else 		if((caltouch_x>key_x+5&&caltouch_x<key_x+100)&&(caltouch_y>key_y+45&&caltouch_y<key_y+85))	//合金钢
			{
			paramvaule[0][paramentmun]=1;
			modifyparament=0;		
			}
					else 		if((caltouch_x>key_x+5&&caltouch_x<key_x+100)&&(caltouch_y>key_y+85&&caltouch_y<key_y+125))	//铜
						{
						paramvaule[0][paramentmun]=2;
						modifyparament=0;		
						}
						else 		if((caltouch_x>key_x+5&&caltouch_x<key_x+100)&&(caltouch_y>key_y+125&&caltouch_y<key_y+165))	//铝
							{
							paramvaule[0][paramentmun]=3;
							modifyparament=0;		
							}		
							else 		if((caltouch_x>key_x+5&&caltouch_x<key_x+100)&&(caltouch_y>key_y+165&&caltouch_y<key_y+205))	//不锈钢
								{
								paramvaule[0][paramentmun]=4;
								modifyparament=0;		
								}	
								else 		if((caltouch_x>key_x+5&&caltouch_x<key_x+100)&&(caltouch_y>key_y+205&&caltouch_y<key_y+245))	//球墨铸铁
									{
									paramvaule[0][paramentmun]=5;
									modifyparament=0;		
									}								
									else 		if((caltouch_x>key_x+5&&caltouch_x<key_x+100)&&(caltouch_y>key_y+245&&caltouch_y<key_y+285))	//自定义
										{
										paramvaule[0][paramentmun]=6;
										modifyparament=0;		
										}								
								
	}
	else if(paramentmun==11)
		{
	if((caltouch_x>key_x+5&&caltouch_x<key_x+100)&&(caltouch_y>key_y+5&&caltouch_y<key_y+45))	//1
		{
		paramvaule[0][paramentmun]=0;
		modifyparament=0;		
		}
		else 		if((caltouch_x>key_x+5&&caltouch_x<key_x+100)&&(caltouch_y>key_y+45&&caltouch_y<key_y+85))	//2
			{
			paramvaule[0][paramentmun]=1;
			modifyparament=0;		
			}
					else 		if((caltouch_x>key_x+5&&caltouch_x<key_x+100)&&(caltouch_y>key_y+85&&caltouch_y<key_y+125))	//4
						{
						paramvaule[0][paramentmun]=2;
						modifyparament=0;		
						}
						else 		if((caltouch_x>key_x+5&&caltouch_x<key_x+100)&&(caltouch_y>key_y+125&&caltouch_y<key_y+165))	//8
							{
							paramvaule[0][paramentmun]=3;
							modifyparament=0;		
							}		
							else 		if((caltouch_x>key_x+5&&caltouch_x<key_x+100)&&(caltouch_y>key_y+165&&caltouch_y<key_y+205))	//16
								{
								paramvaule[0][paramentmun]=4;
								modifyparament=0;		
								}	
								else 		if((caltouch_x>key_x+5&&caltouch_x<key_x+100)&&(caltouch_y>key_y+205&&caltouch_y<key_y+245))	//32
									{
									paramvaule[0][paramentmun]=5;
									modifyparament=0;		
									}								
									else 		if((caltouch_x>key_x+5&&caltouch_x<key_x+100)&&(caltouch_y>key_y+245&&caltouch_y<key_y+285))	//64
										{
										paramvaule[0][paramentmun]=6;
										modifyparament=0;		
										}	
												
		}
		else if(paramentmun==12)
		{
			if((caltouch_x>key_x+5&&caltouch_x<key_x+100)&&(caltouch_y>key_y+5&&caltouch_y<key_y+45))	//PP
				{
				paramvaule[0][paramentmun]=0;
				modifyparament=0;		
				}
				else 		if((caltouch_x>key_x+5&&caltouch_x<key_x+100)&&(caltouch_y>key_y+45&&caltouch_y<key_y+85))	//PZ
					{
					paramvaule[0][paramentmun]=1;
					modifyparament=0;		
					}		
		}
				else if(paramentmun==16)
		{
			if((caltouch_x>key_x+5&&caltouch_x<key_x+100)&&(caltouch_y>key_y+5&&caltouch_y<key_y+45))	//低频
				{
				HL=0;
				modifyparament=0;		
				}
				else 		if((caltouch_x>key_x+5&&caltouch_x<key_x+100)&&(caltouch_y>key_y+45&&caltouch_y<key_y+85))	//高频
					{
					HL=1;
					modifyparament=0;		
					}		
		}
			if(modifyparament==0)
	{
			drawblock(key_x-1,key_y,321,320);
				dismeun(page);
							dispageparament(LCD_COLOR_WHITE);  
					  disparament(gongneng,paramentmun,1); 
			disinformationunit();
			drawmainbroad();
			disdeploy();
			
			programparamentflash();
	}
}
else if(paramentmun==16&&gongneng==0)//测厚扫描界面下
{
	if((caltouch_x>key_x+5&&caltouch_x<key_x+60)&&(caltouch_y>key_y+5&&caltouch_y<key_y+45))	//右扫描
	{
			
			scandirection=0;
			encoder=10000;
			for(x=0;x<20000;x++)
				 SCAN_BUF[x]=0;
			modifyparament=0;
			drawblock(key_x-1,key_y,321,320);
			disdeploy();
		drawmainbroad();
			disinformationunit();
			dismeun(page) ; 
		dispageparament(LCD_COLOR_WHITE);
	}
	else if((caltouch_x>key_x+5&&caltouch_x<key_x+60)&&(caltouch_y>key_y+45&&caltouch_y<key_y+85))//左扫描
	{
				
			scandirection=1;
			encoder=10000;
			for(x=0;x<20000;x++)
				 SCAN_BUF[x]=0;
				modifyparament=0;
			drawblock(key_x-1,key_y,321,320);
			disdeploy();
		drawmainbroad();
			disinformationunit();
		dismeun(page); 
			dispageparament(LCD_COLOR_WHITE);
	}

}
//下面是探伤下面的参数选择界面
else if(paramentmun==8&&gongneng==1)//探伤功能下面的材料选择界面
	{
		if((caltouch_x>key_x+5&&caltouch_x<key_x+100)&&(caltouch_y>key_y+5&&caltouch_y<key_y+45))	//碳钢
		{
		paramvaule[gongneng][paramentmun]=0;
		modifyparament=0;		
		}
		else 		if((caltouch_x>key_x+5&&caltouch_x<key_x+100)&&(caltouch_y>key_y+45&&caltouch_y<key_y+85))	//合金钢
			{
			paramvaule[gongneng][paramentmun]=1;
			modifyparament=0;		
			}
					else 		if((caltouch_x>key_x+5&&caltouch_x<key_x+100)&&(caltouch_y>key_y+85&&caltouch_y<key_y+125))	//铜
						{
						paramvaule[gongneng][paramentmun]=2;
						modifyparament=0;		
						}
						else 		if((caltouch_x>key_x+5&&caltouch_x<key_x+100)&&(caltouch_y>key_y+125&&caltouch_y<key_y+165))	//铝
							{
							paramvaule[gongneng][paramentmun]=3;
							modifyparament=0;		
							}		
							else 		if((caltouch_x>key_x+5&&caltouch_x<key_x+100)&&(caltouch_y>key_y+165&&caltouch_y<key_y+205))	//不锈钢
								{
								paramvaule[gongneng][paramentmun]=4;
								modifyparament=0;		
								}	
								else 		if((caltouch_x>key_x+5&&caltouch_x<key_x+100)&&(caltouch_y>key_y+205&&caltouch_y<key_y+245))	//球墨铸铁
									{
									paramvaule[gongneng][paramentmun]=5;
									modifyparament=0;		
									}								
									else 		if((caltouch_x>key_x+5&&caltouch_x<key_x+100)&&(caltouch_y>key_y+245&&caltouch_y<key_y+285))	//自定义
										{
										paramvaule[gongneng][paramentmun]=6;
										modifyparament=0;		
										}								
			if(modifyparament==0)
	{
				disparament(gongneng,paramentmun,1); 
			drawblock(key_x-1,key_y,321,320);
			disinformationunit();
			disdeploy();
		drawmainbroad();
		if(gongneng==1)
		{
						dismeun(page);
			dispageparament(LCD_COLOR_WHITE);
		}
			programparamentflash();
	}								
	}
	else//有数字键盘
	{
	keymun=getkeymun(key_x,key_y);		
	if(keymun<10)//数字键
	{if((paramentmodifyval*10+keymun)<paramvaulelimitup[gongneng][paramentmun])
		{
			paramentmodifyval=paramentmodifyval*10+keymun;
		if(paramentmodifyval==0&&paramentmodifydisbit!=0)
					paramentmodifybit++;
			else if(paramentmodifyval!=0)
							paramentmodifybit++;
		}
	}//paramentmodifydisbit整数位位数
	else if(keymun==10&&paramentmodifydisbit==0)//第一次按小数点,按了一次以后就没有用了
	{
		if(paramentmodifybit==0&&paramentmodifyval==0)//
		{
				paramentmodifydisbit=1;
		paramentmodifybit++;
			BSP_LCD_SetFont(&Font24_1);
			BSP_LCD_DisplayStringAt(key_x+20+12*(paramentmodifybit),key_y+20,(uint8_t*)".", LEFT_MODE);
		}
		else
		{
				paramentmodifydisbit=paramentmodifybit;
			BSP_LCD_SetFont(&Font24_1);
		BSP_LCD_DisplayStringAt(key_x+20+12*(paramentmodifybit),key_y+20,(uint8_t*)".", LEFT_MODE);
		}
	
	}
	
	else if(keymun==11)//确认键
	{
		if(timemodify==0)
		{	
		m=1;
		if(paramentmodifydisbit!=0)
		{
		for(x=0;x<(paramentmodifybit-paramentmodifydisbit);x++)
		m=m*10;
		}

	if(defaultindex==0)	
	{
			if(paramenttype[gongneng][paramentmun]<10)//整数
				modifydata=paramentmodifyval/m;		
			else
			{
				modifydata=paramentmodifyval/m;//取整数部分
				valback=paramentmodifyval%m;//取小数部分
							m=1;
					for(x=0;x<paramenttype[gongneng][paramentmun]/10;x++)
							m=m*10;
				modifydata=modifydata*m;
				
					if((paramentmodifybit-paramentmodifydisbit)<=paramenttype[gongneng][paramentmun]/10)
						{m=1;
						for(x=0;x<(paramenttype[gongneng][paramentmun]/10-(paramentmodifybit-paramentmodifydisbit));x++)
						m=m*10;
						valback=valback*m;
						modifydata=modifydata+valback;
					
						}
						else
						{
						m=1;
						for(x=0;x<((paramentmodifybit-paramentmodifydisbit)-paramenttype[gongneng][paramentmun]/10);x++)
						m=m*10;
						valback=valback/m;
						modifydata=modifydata+valback;
						
						}				
			}
			
		
			modifyparament=0;
			if(gongneng==0&&paramentmun==0)
			{
				if(modifydata>=paramvaule[gongneng][paramentmun+1]&&paramvaule[gongneng][paramentmun+1]>0)
					modifydata=paramvaule[gongneng][paramentmun+1]-1;
			}
			if(gongneng==0&&paramentmun==4)
			{
				if(modifydata>=paramvaule[gongneng][paramentmun+1]&&paramvaule[gongneng][paramentmun+1]>0)
					modifydata=paramvaule[gongneng][paramentmun+1]-1;
			}		
			if(gongneng==0&&paramentmun==1)
			{
				if(modifydata<=paramvaule[gongneng][paramentmun-1])
					modifydata=paramvaule[gongneng][paramentmun-1]+1;
			}	
			if(gongneng==0&&paramentmun==5)
			{
				if(modifydata<=paramvaule[gongneng][paramentmun-1])
					modifydata=paramvaule[gongneng][paramentmun-1]+1;
			}	

			if(gongneng==1&&paramentmun==0)
			{
				if(modifydata>=paramvaule[gongneng][paramentmun+1]&&paramvaule[gongneng][paramentmun+1]>0)
					modifydata=paramvaule[gongneng][paramentmun+1]-1;
			}	
			if(gongneng==1&&paramentmun==1)
			{
				if(modifydata<=paramvaule[gongneng][paramentmun-1])
					modifydata=paramvaule[gongneng][paramentmun-1]+1;
			}	
			if(gongneng==1&&paramentmun==3)
			{
				if(modifydata>=paramvaule[gongneng][paramentmun+1]&&paramvaule[gongneng][paramentmun+1]>0)
					modifydata=paramvaule[gongneng][paramentmun+1]-1;
			}
			if(gongneng==1&&paramentmun==4)
			{
				if(modifydata<=paramvaule[gongneng][paramentmun-1])
					modifydata=paramvaule[gongneng][paramentmun-1]+1;
			}
			if(gongneng==1&&paramentmun==5)
			{
				if(modifydata>=paramvaule[gongneng][paramentmun+1]&&paramvaule[gongneng][paramentmun+1]>0)
					modifydata=paramvaule[gongneng][paramentmun+1]-1;
			}
			if(gongneng==1&&paramentmun==6)
			{
				if(modifydata<=paramvaule[gongneng][paramentmun-1])
					modifydata=paramvaule[gongneng][paramentmun-1]+1;
			}
			if(modifydata<paramvaulelimitup[gongneng][paramentmun])//判断有没有超限
				paramvaule[gongneng][paramentmun]=modifydata;	
			else	
				paramvaule[gongneng][paramentmun]=paramvaulelimitup[gongneng][paramentmun];	
		}
else
{
defaultindex=0;
			modifyparament=0;
	HL=1;
	if(paramentmodifyval==1234)
	{
		for(x=0;x<2;x++)
		{
			for(y=0;y<16;y++)
		paramvaule[x][y]=paramvauledefault[x][y];
		}

	}
}	
			drawblock(key_x-1,key_y,321,320);
				
		 
			
			
			disdeploy();
			disinformationunit();
	//				if(gongneng==1)
	//	{
						dismeun(page);
			dispageparament(LCD_COLOR_WHITE);
	//	}
				disparament(gongneng,paramentmun,1);
			drawmainbroad();
				if(scanindex==1||(gongneng==0&&(paramentmun==0||paramentmun==1)))						
						disxcoordinate();	
				
							if(gongneng==1&&(paramentmun==0||paramentmun==1))
								disxcoordinate();	
							if(gongneng==0&&paramentmun==14)
									drawblock(2,219,441,244);
			
			programparamentflash();	
						}	
				else	
				{
						m=1;
						if(paramentmodifydisbit!=0)
						{
						for(x=0;x<(paramentmodifybit-paramentmodifydisbit);x++)
						m=m*10;
						}	
						x=((paramentmodifyval/m)%100)/10;						
						x=x*0x10;
					settime=(paramentmodifyval/m)%10+x;	
				RTC_set(settime);
									timemodify=0;
									drawblock(key_x-1,key_y,321,320);
										dismeun(page);
			dispageparament(LCD_COLOR_WHITE);		
			disparament(gongneng,paramentmun,1); 

			disinformationunit();
			disdeploy();
						drawmainbroad();
				}					
	}
	
		else if(keymun==12)//清除
		{
		paramentmodifyval=0;
		paramentmodifybit=0;
			paramentmodifydisbit=0;
			BSP_LCD_DisplayStringAt(key_x+20,key_y+20,(uint8_t*)"        ", LEFT_MODE);
		}
		else if(keymun==13)//返回
		{defaultindex=0;
			timemodify=0;
		paramentmodifyval=0;
		paramentmodifybit=0;
			paramentmodifydisbit=0;			
			modifyparament=0;
			drawblock(key_x-1,key_y,321,320);
						dismeun(page);
			dispageparament(LCD_COLOR_WHITE);
				disparament(gongneng,paramentmun,1);			
			disdeploy();
			drawmainbroad();
			disinformationunit();
					if(gongneng==1)
		{
						dismeun(page);
			dispageparament(LCD_COLOR_WHITE);
		}
			disparament(gongneng,paramentmun,1); 
		}
	if(keymun<11)
	{if(paramentmodifyval==0)
		{
		BSP_LCD_DisplayStringAt(key_x+20,key_y+20,(uint8_t*)"0", LEFT_MODE);
		}
			else
			{
		if(paramentmodifydisbit==0)
					LCD_ShowxNum_pix(key_x+20,key_y+20,paramentmodifyval,paramentmodifybit,12,paramentmodifybit,LCD_COLOR_WHITE);
		else
					LCD_ShowxNum_pix(key_x+20,key_y+20,paramentmodifyval,paramentmodifybit,12,paramentmodifydisbit,LCD_COLOR_WHITE);
	    }

	}
}

}
	



if(scanindex==1&&SCAN_cursorindex==0)//在B扫描下面，可光标显示
{

	if(caltouch_y>210&&caltouch_y<465)
	{
		preSCAN_cursor=SCAN_cursor;
		SCAN_cursor=caltouch_x;
		
	}
	if(SCAN_cursor<2)
			SCAN_cursor=2;
	if(SCAN_cursor>439)
			SCAN_cursor=439;	

}
dosomething=0;
SCAN_cursorindex=1;
	 HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}


void bscandisxcoordinate( uint16_t percent )//BSCAN
{
	uint32_t color;
 // uint16_t Y,H;
//percent=24-24*percent/100;	
  percent=24-24*(percent-2333)/795;
  Y=60+percent;
  H=24-percent;
	color=BSP_LCD_GetTextColor();
  BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
	BSP_LCD_DrawRect(620,56,6,4);
	BSP_LCD_DrawRect(616,60,14,24);
	if(H>=6)
  {
	 BSP_LCD_SetTextColor(LCD_COLOR_GREEN);	
	 BSP_LCD_FillRect(616,Y,14,H);	 																		
	}
 else
 {
	 BSP_LCD_SetTextColor(LCD_COLOR_RED);	
	 BSP_LCD_FillRect(616,Y,14,H);	
 }
//BSP_LCD_FillRect(617,11,12,24);	//满格
BSP_LCD_SetTextColor(color);

	
}

	
void disbat(uint16_t percent)  //0-23  23表示电量为零 0%  0标示电量满 100%  23-INT  12.6v=3128 充满，9.6V=2333 关机
{
uint32_t color;
 //uint16_t Y,H;
//percent=24-24*percent/100;	
		if(percent>= 3300)
		 percent=3300;
	if(percent<= 2511)
		 percent=2511;
     percent=24-24*(percent-2511)/790;
  if(percent<=1) 
     percent=1;
  if(percent>23) 
     percent=23;       
YB=7+percent;
HB=24-percent;
color=BSP_LCD_GetTextColor();
BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
BSP_LCD_DrawHLine(468,2,5);
BSP_LCD_DrawHLine(464,6,13);
BSP_LCD_DrawHLine(464,31,13);
BSP_LCD_DrawVLine(468,2,4);	
BSP_LCD_DrawVLine(473,2,4);	
BSP_LCD_DrawVLine(464,6,25);
BSP_LCD_DrawVLine(477,6,26);	
	
if(HB>=6)
  {
	 BSP_LCD_SetTextColor(LCD_COLOR_GREEN);	
	 BSP_LCD_FillRect(464,YB,12,HB);
	 BSP_LCD_SetTextColor(LCD_COLOR_BLACK);		
	 BSP_LCD_FillRect(464,7,12,percent);		
	}
 else
 {
	 BSP_LCD_SetTextColor(LCD_COLOR_RED);	
	 BSP_LCD_FillRect(464,YB,11,HB);
	 BSP_LCD_SetTextColor(LCD_COLOR_BLACK);		
	 BSP_LCD_FillRect(464,7,11,percent);		 
 }
 

BSP_LCD_SetTextColor(color);

}
 uint32_t GetSector(uint32_t Address)
{
  uint32_t sector = 0;

  if((Address < ADDR_FLASH_SECTOR_1) && (Address >= ADDR_FLASH_SECTOR_0))
  {
    sector = FLASH_SECTOR_0;
  }
  else if((Address < ADDR_FLASH_SECTOR_2) && (Address >= ADDR_FLASH_SECTOR_1))
  {
    sector = FLASH_SECTOR_1;
  }
  else if((Address < ADDR_FLASH_SECTOR_3) && (Address >= ADDR_FLASH_SECTOR_2))
  {
    sector = FLASH_SECTOR_2;
  }
  else if((Address < ADDR_FLASH_SECTOR_4) && (Address >= ADDR_FLASH_SECTOR_3))
  {
    sector = FLASH_SECTOR_3;
  }
  else if((Address < ADDR_FLASH_SECTOR_5) && (Address >= ADDR_FLASH_SECTOR_4))
  {
    sector = FLASH_SECTOR_4;
  }
  else if((Address < ADDR_FLASH_SECTOR_6) && (Address >= ADDR_FLASH_SECTOR_5))
  {
    sector = FLASH_SECTOR_5;
  }
  else if((Address < ADDR_FLASH_SECTOR_7) && (Address >= ADDR_FLASH_SECTOR_6))
  {
    sector = FLASH_SECTOR_6;
  }
  else /* (Address < FLASH_END_ADDR) && (Address >= ADDR_FLASH_SECTOR_7) */
  {
    sector = FLASH_SECTOR_7;
  }

  return sector;
}
void paramint(void)
{ uint16_t i;
	inposition=1;
	xinhaoindexmun=0;
	boxingduan=0;
	lcddisindex=0;
	uartreceiveindex=0;
	tcpreceiveindex=0;
uartreceive=1;	
	reset=1;
	senden=0;
	sendthickindex=0;
	sendpyhcount=0;
  dosomething=0;
	defaultindex=0;
	prethickdis=0;
	keymun=31;
	prekeymun=31;
	modelindex=0;
	calibrationindex=0;
	draw=1;
	findpeakindex=0;
	page=0;
	paramentmun=0;
	SCAN_cursorindex=0;
	countinuemun=0;
	hisindex=0;
	saveindex=0;
	delhisdataindex=0;
	readhisdataindex=0;
	pause=0;
	agctemp=150;
	confirmindex=0;
	deleteindex=1;
	encoder=10000;
	scanindex=0;
	scandirection=0;
	prekeyindex=0;
	keyindex=0;
	pagekeyindex=0;
	gongneng=0;
	A1=1092;
	B1=-15577;
	A2=1076;
	B1=-29110;
	dissecondcoordinate=500;
	presecondcoordinate=500;
	disfirstcoordinate=500;
	prefirstcoordinate=500;
	discalibrationindex=0;
	SCAN_cursor=200;
	preSCAN_cursor=200;
	timemodify=0;
	readwave=0;
	heartcount=0;
	for(i=0;i<488;i++)
	{
		CH3_BUF[i]=233;
		}
		for(i=0;i<240;i++)
	{
		LCDDISBUF[i]=60;
		LCDDISBUFPRE[i]=60;
		}
}



void eraseflash(uint32_t strataddress,uint32_t endaddress)
{
	HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
	  FirstSector = GetSector(strataddress);
  /* Get the number of sector to erase from 1st sector*/
  NbOfSectors = GetSector(endaddress) - FirstSector + 1;
  EraseInitStruct.TypeErase     = FLASH_TYPEERASE_SECTORS;
  EraseInitStruct.VoltageRange  = FLASH_VOLTAGE_RANGE_3;
  EraseInitStruct.Sector        = FirstSector;
  EraseInitStruct.NbSectors     = NbOfSectors;
	HAL_FLASH_Unlock();
	HAL_FLASHEx_Erase(&EraseInitStruct, &SECTORError);
	HAL_FLASH_Lock();
		HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}
void programIDflash(void)
{
HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
	TEXT_Buffer[0]=0x4d;//量产/商业版（M），研发内测版（R）
	TEXT_Buffer[1]=0x41;//STM32F7 系列（A），DSP 系列 （B）
	TEXT_Buffer[2]=0x53;//串口（S） / WiFi（W）
	TEXT_Buffer[3]=2;//烧录方式
	TEXT_Buffer[4]=25;//年	
	TEXT_Buffer[5]=8;//月
	TEXT_Buffer[6]=18;//日
	TEXT_Buffer[7]=1;//流水号	
	TEXT_Buffer[8]=0;//固定8字符	
		TEXT_Buffer[9]=0;//固定8字符	
		TEXT_Buffer[10]=0;//固定8字符	
		TEXT_Buffer[11]=0;//固定8字符	
		TEXT_Buffer[12]=0;//固定8字符	
		TEXT_Buffer[13]=0;//固定8字符	
		TEXT_Buffer[14]=0;//固定8字符	
		TEXT_Buffer[15]=0;//固定8字符	
		W25QXX_Write((uint8_t*)TEXT_Buffer,4096,16);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}
void readIDflash(void)
{
	HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
	W25QXX_Read(datatemp,4096,16);
		HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}
void programparamentflash(void)
{
	uint32_t n;
HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
	if(paramvaule[0][14]>1800)
		 paramvaule[0][14]=1800;
		if(paramvaule[0][15]>20000)
		 paramvaule[0][15]=20000;
	for(n=0;n<16;n++)
	{TEXT_Buffer[n*2]=paramvaule[0][n];
		TEXT_Buffer[n*2+1]=paramvaule[0][n]>>8;
	}

	for(n=0;n<16;n++)
	{TEXT_Buffer[n*2+32]=paramvaule[1][n];
		TEXT_Buffer[n*2+1+32]=paramvaule[1][n]>>8;
	}
	for(n=0;n<16;n++)
	{TEXT_Buffer[n*2+64]=paramvaule[2][n];
		TEXT_Buffer[n*2+1+64]=paramvaule[2][n]>>8;
	}
	for(n=0;n<16;n++)
	{TEXT_Buffer[n*2+96]=paramvaule[3][n];
		TEXT_Buffer[n*2+1+96]=paramvaule[3][n]>>8;
	}	
	for(n=0;n<9;n++)
		{TEXT_Buffer[n*2+128]=shengsu[n];
		TEXT_Buffer[n*2+1+128]=shengsu[n]>>8;
	}
		TEXT_Buffer[148+3]=A1>>24;
		TEXT_Buffer[148+2]=A1>>16;
		TEXT_Buffer[148+1]=A1>>8;
		TEXT_Buffer[148]=A1;
		TEXT_Buffer[152+3]=A2>>24;
		TEXT_Buffer[152+2]=A2>>16;
		TEXT_Buffer[152+1]=A2>>8;
		TEXT_Buffer[152]=A2;
		TEXT_Buffer[156+3]=B1>>24;
		TEXT_Buffer[156+2]=B1>>16;
		TEXT_Buffer[156+1]=B1>>8;
		TEXT_Buffer[156]=B1;
		TEXT_Buffer[160+3]=B2>>24;
		TEXT_Buffer[160+2]=B2>>16;
		TEXT_Buffer[160+1]=B2>>8;
		TEXT_Buffer[160]=B2;	

	W25QXX_Write((uint8_t*)TEXT_Buffer,1024,200);
	readparamentflash();
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void getstoremun(void)
{
  RTC_DateTypeDef storedatestructureget;
	  HAL_RTC_GetDate(&RtcHandle, &storedatestructureget, RTC_FORMAT_BIN);
	W25QXX_Read(datatemp,3072,20);
	
		year = datatemp[0];	
		month =datatemp[1];		
		day = datatemp[2];		
	storemun=datatemp[5]<<8;
  storemun=storemun|datatemp[4];		
	bscanstoremun=datatemp[7]<<8;	
	bscanstoremun=bscanstoremun|datatemp[6];
	if(year==storedatestructureget.Year&&month==storedatestructureget.Month&&day==storedatestructureget.Date)
	{
	if(scanindex==0)
	{
		if(storemun<9999)
				storemun=storemun+1;
		if(storemun==9998)
				storemun=0;
	}
	else
		if(scanindex==1)
				{
					if(bscanstoremun<999)
							bscanstoremun=bscanstoremun+1;
					if(bscanstoremun==0xffff)
							bscanstoremun=0;
				}
		TEXT_Buffer[0]=storedatestructureget.Year;
		TEXT_Buffer[1]=storedatestructureget.Month;	
		TEXT_Buffer[2]=storedatestructureget.Date;
		TEXT_Buffer[4]=storemun;
		TEXT_Buffer[5]=storemun>>8;
		TEXT_Buffer[6]=bscanstoremun;
		TEXT_Buffer[7]=bscanstoremun>>8;					
		W25QXX_Write((uint8_t*)TEXT_Buffer,3072,20);
					
	}
	else
	{
		storemun=0;
		bscanstoremun=0;
		TEXT_Buffer[0]=storedatestructureget.Year;
		TEXT_Buffer[1]=storedatestructureget.Month;	
		TEXT_Buffer[2]=storedatestructureget.Date;
		TEXT_Buffer[4]=storemun;
		TEXT_Buffer[5]=storemun>>8;
		TEXT_Buffer[6]=bscanstoremun;
		TEXT_Buffer[7]=bscanstoremun>>8;					
		W25QXX_Write((uint8_t*)TEXT_Buffer,3072,20);			
		
	}
		
}


void readparamentflash(void)
{
	uint16_t n;
	HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
	W25QXX_Read(datatemp,1024,200);
	for(n=0;n<16;n++)
	{
		paramvauletemp[0][n] = datatemp[n*2+1]<<8;
		paramvauletemp[0][n]=paramvauletemp[0][n]|datatemp[n*2];
	}
	for(n=0;n<16;n++)
	{
		paramvauletemp[1][n] = datatemp[n*2+1+32]<<8;
		paramvauletemp[1][n]=paramvauletemp[1][n]|datatemp[n*2+32];
	}
	for(n=0;n<16;n++)
	{
		paramvauletemp[2][n] = datatemp[n*2+1+64]<<8;
		paramvauletemp[2][n]=paramvauletemp[2][n]|datatemp[n*2+64];
	}
	for(n=0;n<16;n++)
	{
		paramvauletemp[3][n] = datatemp[n*2+1+96]<<8;
		paramvauletemp[3][n]=paramvauletemp[3][n]|datatemp[n*2+96];
	}
	for(n=0;n<9;n++)
	{

		shengsutemp[n] = shengsutemp[n]|(datatemp[n*2+128+1]<<8);
		shengsutemp[n] = shengsutemp[n]|datatemp[n*2+128];
	}
	A1=datatemp[148+3]<<24;
	A1=A1|(datatemp[148+2]<<16);;
	A1=A1|(datatemp[148+1]<<8);;
	A1=A1|datatemp[148];
	A2=datatemp[152+3]<<24;
	A2=A2|(datatemp[152+2]<<16);;
	A2=A2|(datatemp[152+1]<<8);;
	A2=A2|datatemp[152];
	B1=datatemp[156+3]<<24;
	B1=B1|(datatemp[156+2]<<16);;
	B1=B1|(datatemp[156+1]<<8);;
	B1=B1|datatemp[156];
	B2=datatemp[160+3]<<24;
	B2=B2|(datatemp[160+2]<<16);;
	B2=B2|(datatemp[160+1]<<8);;
	B2=B2|datatemp[160];
HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}
void sendparament(void)
{
	
char x;
uint32_t k;
	k=50000;
	senddata[0]=k;
	senddata[1]=k>>16;
	//if(paramvaule[2][8]<50)
	senddata[2]=paramvaule[3][2];//脉冲的频率	
//	else
//	senddata[2]=taipinlv;//脉冲的频率			
	if(paramvaule[2][2]>5)
		paramvaule[2][2]=3;
	senddata[3]=paramvaule[2][2];//脉冲的个数	
	
	senddata[4]=400*10/2;//采集深度
	senddata[5]=200;//ad_pre开始有多少不采集2us	
	senddata[6]=500;//采集起始点
	senddata[7]=1500;//采集结束点
//	senddata[8]=165;//增益
	if(gongneng==0||gongneng==2)
	{
	if(modelindex==0||modelindex==3||modelindex==4)//手动
		senddata[8]=agctemp*3.5608+655;
	else
		senddata[8]=paramvaule[0][2]*3.5608+655;//增益
	}
	else if(gongneng==1)
		senddata[8]=paramvaule[1][2]*3.5608+655;//增益
	
	if(gongneng==0||gongneng==2)
	senddata[9]=paramvaule[0][3];//包络 原始波形 1：包络 0 ：原始
	else if(gongneng==1)
						senddata[9]=1;
	//senddata[10]=4;	//平滑次数
	senddata[10]=paramvaule[0][11];	//平滑次数
	senddata[11]=paramvaule[2][6];	//无线下载程序的开关0：关闭 1：打开
	senddata[12]=0;	//高低频切换 0:160-320us 1:0-160us      //senddata[12]=HL;	//高低频切换 0低 1高
	if(paramvaule[0][13]<20)
		paramvaule[0][13]=20;
	senddata[13]=paramvaule[0][13];	//亮度调节
	//senddata[13]=100;	//亮度调节
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET);
BSP_SRAM_WriteData(SRAM_DEVICE_ADDR,senddata,14);
//	    for (x= 0;x<10; x++)
 //  {
 //     *(__IO uint16_t*) (SRAM_DEVICE_ADDR+ 2*x) = senddata[x];
 //   }
}


//校准
char calibration(void)
{
	uint16_t susedv;
if(paramvaule[0][3]==0)//原始波形零点和声速的计算
	{
		susedv=usedv;
		paramvaule[0][6]=(paramvaule[0][7]*200000/caltime)+7.086*paramvaule[0][9];
		if(paramvaule[0][6]>40000||paramvaule[0][6]<20000)
		{
			paramvaule[0][6]=susedv;
			calibrationokindex=0;
		}	
		else
			usedv=paramvaule[0][6];


paramvaule[0][8]=time-peakst;
if(paramvaule[0][8]>paramvaulelimitup[0][8])
{
				paramvaule[0][8]=2800;
	calibrationokindex=0;
}
if(paramvaule[0][8]<0)
{
	paramvaule[0][8]=2800;
	calibrationokindex=0;
}
}
	else//包络波形的零点计算
	{
		paramvaule[0][15]=time-(timed-time);
		if(paramvaule[0][15]>paramvaulelimitup[0][15])
		{
			paramvaule[0][15]=2800;
			calibrationokindex=0;
		}
if(paramvaule[0][15]<0)
{
	paramvaule[0][15]=2800;		
	calibrationokindex=0;
}
	}
			
			programparamentflash();
}

	void dishisname(uint16_t mun,uint16_t sel)
{
uint32_t color,discolor;
uint16_t xloc,yloc,mun1;
uint8_t distemp;
color=BSP_LCD_GetTextColor();
BSP_LCD_SetFont(&Font16);//设置字体，11个点宽，9个字符99宽，1行显示5个
if(sel==1)
  discolor=LCD_COLOR_CYAN;//天蓝色
else
	discolor=LCD_COLOR_WHITE;//白色
BSP_LCD_SetTextColor(discolor);
		sprintf ((char*)str, "%-11.11s", pDirectoryFiles[mun]);
mun1=mun%24;
switch(mun1)
{
	case 0:
		xloc=2;
		yloc=485;
		break;
	case 1:
		xloc=122;
		yloc=485;		
		break;
	case 2:
		xloc=242;
		yloc=485;			
		break;
	case 3:
		xloc=2;
		yloc=525;			
		break;
	case 4:
		xloc=122;
		yloc=525;		
		break;
	case 5:
		xloc=242;
		yloc=525;
		break;
	case 6:
		xloc=2;
		yloc=565;		
		break;
	case 7:
		xloc=121;
		yloc=565;			
		break;
	case 8:
		xloc=242;
		yloc=565;			
		break;
	case 9:
		xloc=2;
		yloc=605;		
		break;
	case 10:
		xloc=122;
		yloc=605;
		break;
	case 11:
		xloc=242;
		yloc=605;		
		break;
	case 12:
		xloc=2;
		yloc=645;			
		break;
	case 13:
		xloc=122;
		yloc=645;			
		break;
	case 14:
		xloc=242;
		yloc=645;		
		break;	
	case 15:
		xloc=2;
		yloc=685;			
		break;
	case 16:
		xloc=122;
		yloc=685;			
		break;
	case 17:
		xloc=242;
		yloc=685;		
		break;
	case 18:
		xloc=2;
		yloc=725;			
		break;
	case 19:
		xloc=122;
		yloc=725;			
		break;
	case 20:
		xloc=242;
		yloc=725;		
		break;	
	case 21:
		xloc=2;
		yloc=765;			
		break;
	case 22:
		xloc=122;
		yloc=765;			
		break;
	case 23:
		xloc=242;
		yloc=765;		
		break;	
	default:
		break;		
}
mun1=(str[4]-65)+(str[5]-65)*25;
//str[0]=sdatestructureget.Year+65;	
distemp=str[0]-65;
LCD_ShowxNum1(xloc,yloc,distemp,2,discolor);//年
//str[1]=sdatestructureget.Month+65;
distemp=str[1]-65;
LCD_ShowxNum1(xloc+22,yloc,distemp,2,discolor);//月

if(str[2]<64)
	distemp=str[2]-48;
else 
		distemp=str[2]-55;
LCD_ShowxNum1(xloc+44,yloc,distemp,2,discolor);//日
//BSP_LCD_DisplayStringAt(xloc+66,yloc,(uint8_t*)"-", LEFT_MODE);
//LCD_ShowxNum1(xloc+77,yloc,mun1,3,discolor);//序号
LCD_ShowxNum1(xloc+66,yloc,mun1,4,discolor);//序号
BSP_LCD_SetFont(&Font24_1);
BSP_LCD_SetTextColor(color);
}


void dispagehisname(uint16_t mun)
{
uint16_t i;
HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);

if(hisindex==1)//在历史界面
{
	if(ubNumberOfFiles>0)
	{
	if(((mun+1)*24)>=(ubNumberOfFiles-1))
	{
		for(i=mun*24;i<=(ubNumberOfFiles-1);i++)
			dishisname(i,0);
	}
	else
	{
		for(i=mun*24;i<mun*24+24;i++)
			dishisname(i,0);
	}
		
	}
}
else if(scanhisindex==1)//在扫描历史界面
{
	if(ubNumberOftxtFiles>0)
	{
	if(((mun+1)*24)>=(ubNumberOftxtFiles-1))
	{
		for(i=mun*24;i<=(ubNumberOftxtFiles-1);i++)
			dishisname(i,0);
	}
	else
	{
		for(i=mun*24;i<mun*24+15;i++)
			dishisname(i,0);
	}
	
	}
}


	 HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}
void delhisdata(void)
{
	HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
if(FATFS_LinkDriver(&SD_Driver, SD_Path) == 0)
{


	//sprintf ((char*)str, "%-11.11s", pDirectoryFiles[hispage*24+hismun]);//删除文件必须在读取文件之前，所以所有对文件的操作，最后都要读取一次文件，保证文件名是最新的
	sprintf ((char*)str, "%-11.11s", pDirectoryFiles[hismun]);//删除文件必须在读取文件之前，所以所有对文件的操作，最后都要读取一次文件，保证文件名是最新的
	f_unlink ((const char*)str);
	//ubNumberOfFiles = Storage_GetDirectoryBitmapFiles("/", pDirectoryFiles);
	ubNumberOfFiles = Storage_GetDirectoryBitmapFiles("", pDirectoryFiles);//2020-11-25
	}
if((hismun+1>ubNumberOfFiles)&&ubNumberOfFiles>0)
		hismun=ubNumberOfFiles-1;
		FATFS_UnLinkDriver(SD_Path);
     f_close(&MyFile);
			if(FATFS_LinkDriver(&SD_Driver, SD_Path) == 0)
		{
			if(f_mount(&SD_FatFs, (TCHAR const*)SD_Path, 0) == FR_OK)
		{
		if(f_open(&MyFile,(const char*)str, FA_READ) == FR_OK)
		{
			f_read(&MyFile, w_buf, sizeof(w_buf), (UINT*)&bytesread);
		}
		 f_close(&MyFile);
	}
	 }
		FATFS_UnLinkDriver(SD_Path);
		 f_close(&MyFile);
	 HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void delhistxtdata(void)
{
	HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
if(FATFS_LinkDriver(&SD_Driver, SD_Path) == 0)
{
	sprintf ((char*)str, "%-11.11s", pDirectoryFiles[hismun]);//删除文件必须在读取文件之前，所以所有对文件的操作，最后都要读取一次文件，保证文件名是最新的
		f_unlink ((const char*)str);
	ubNumberOftxtFiles = Storage_GetDirectoryBittxtFiles("", pDirectoryFiles);
	}
if((hismun+1>ubNumberOftxtFiles)&&ubNumberOftxtFiles>0)
		hismun=ubNumberOftxtFiles-1;
		FATFS_UnLinkDriver(SD_Path);
     f_close(&MyFile);
			if(FATFS_LinkDriver(&SD_Driver, SD_Path) == 0)
		{
			if(f_mount(&SD_FatFs, (TCHAR const*)SD_Path, 0) == FR_OK)
			{
		if(f_open(&MyFile,(const char*)str, FA_READ) == FR_OK)
		{
			f_read(&MyFile, SCAN_BUF, sizeof(SCAN_BUF), (UINT*)&bytesread);
		}
		 f_close(&MyFile);
	}
	 }
		FATFS_UnLinkDriver(SD_Path);
		 f_close(&MyFile);
	 HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void readhisdata(void)
{
	HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
	paramvaule[0][3]=0;
if(FATFS_LinkDriver(&SD_Driver, SD_Path) == 0)
{
	ubNumberOfFiles = Storage_GetDirectoryBitmapFiles("", pDirectoryFiles);
	}
sprintf ((char*)str, "%-11.11s", pDirectoryFiles[hismun]);
		FATFS_UnLinkDriver(SD_Path);
     f_close(&MyFile);
				if(FATFS_LinkDriver(&SD_Driver, SD_Path) == 0)
		{
			if(f_mount(&SD_FatFs, (TCHAR const*)SD_Path, 0) == FR_OK)
			{
		if(f_open(&MyFile,(const char*)str, FA_READ) == FR_OK)
		{
			f_read(&MyFile, CH1_BUF, sizeof(CH1_BUF), (UINT*)&bytesread);
		}
		 f_close(&MyFile);
	}
	 }
		FATFS_UnLinkDriver(SD_Path);
		 f_close(&MyFile);
	 	hisgain=CH1_BUF[1];//增益
	hisv=CH1_BUF[2];//声速
	hiszero=CH1_BUF[3];//零点
	hist=CH1_BUF[4];//温度
	histhick=CH1_BUF[5];
	histhick=(histhick&0x0000ffff) |(CH1_BUF[6]<<16);//厚度
	hiswavedis=CH1_BUF[7];//波形显示
 HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void readhistxtdata(void)
{
	HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
if(FATFS_LinkDriver(&SD_Driver, SD_Path) == 0)
{
	ubNumberOftxtFiles = Storage_GetDirectoryBittxtFiles("", pDirectoryFiles);
	}
sprintf ((char*)str, "%-11.11s", pDirectoryFiles[hismun]);
		FATFS_UnLinkDriver(SD_Path);
     f_close(&MyFile);
				if(FATFS_LinkDriver(&SD_Driver, SD_Path) == 0)
		{
			if(f_mount(&SD_FatFs, (TCHAR const*)SD_Path,0) == FR_OK)
			{
		if(f_open(&MyFile,(const char*)str, FA_READ) == FR_OK)
		{
			f_read(&MyFile, SCAN_BUF, sizeof(SCAN_BUF), (UINT*)&bytesread);
		}
		 f_close(&MyFile);
	}
	 }
		FATFS_UnLinkDriver(SD_Path);
		 f_close(&MyFile);
 HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void savehistxtdata(void)
	
{
uint8_t mun;
  RTC_DateTypeDef sdatestructureget;
  RTC_TimeTypeDef stimestructureget;
   HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
  /* Get the RTC current Time */
  HAL_RTC_GetTime(&RtcHandle, &stimestructureget, RTC_FORMAT_BIN);
  /* Get the RTC current Date */
  HAL_RTC_GetDate(&RtcHandle, &sdatestructureget, RTC_FORMAT_BIN);
getstoremun();

for(mun=0;mun<13;mun++)
	str[mun]=0;
//	str[6]=48;
str[0]=sdatestructureget.Year+65;	
str[1]=sdatestructureget.Month+65;
if(sdatestructureget.Date<10)
	str[2]=sdatestructureget.Date+48;
else
	str[2]=sdatestructureget.Date+65-10;
str[3]=stimestructureget.Hours+65;
str[4]=(bscanstoremun%25)+65;
str[5]=(bscanstoremun/25)+65;
str[6]=0x4d;
/*
str[3]=stimestructureget.Hours+65;

if(stimestructureget.Minutes<31)
{
if(stimestructureget.Minutes<10)
	str[4]=stimestructureget.Minutes+48;
	else if(stimestructureget.Minutes>9&&stimestructureget.Minutes<31)
				str[4]=stimestructureget.Minutes+65-10;
}
else
{
if(stimestructureget.Minutes<41)
	str[4]=stimestructureget.Minutes+48-31;
	else if(stimestructureget.Minutes>40&&stimestructureget.Minutes<61)
				str[4]=stimestructureget.Minutes+65-41;
str[6]=str[6]|0x02;
}

if(stimestructureget.Seconds<31)
{
if(stimestructureget.Seconds<10)
	str[5]=stimestructureget.Seconds+48;
	else if(stimestructureget.Seconds>9&&stimestructureget.Seconds<31)
				str[5]=stimestructureget.Seconds+65-10;
}
else
{
if(stimestructureget.Seconds<41)
	str[5]=stimestructureget.Seconds+48-31;
	else if(stimestructureget.Seconds>40&&stimestructureget.Seconds<61)
				str[5]=stimestructureget.Seconds+65-41;
str[6]=str[6]|0x01;
}
*/
str[7]=0x2e;
str[8]=0x54;
str[9]=0x58;	
str[10]=0x54;	
if(FATFS_LinkDriver(&SD_Driver, SD_Path) == 0)
{
	ubNumberOftxtFiles = Storage_GetDirectoryBittxtFiles("/", pDirectoryFiles);
	}

		FATFS_UnLinkDriver(SD_Path);
     f_close(&MyFile);	
		if(FATFS_LinkDriver(&SD_Driver, SD_Path) == 0)
		{
			if(f_mount(&SD_FatFs, (TCHAR const*)SD_Path, 0) == FR_OK)
			{
				if(f_open(&MyFile, (const char*)str, FA_CREATE_ALWAYS | FA_WRITE) == FR_OK)
					{
						f_write(&MyFile, SCAN_BUF, sizeof(SCAN_BUF), (void *)&byteswritten);
					}
					f_close(&MyFile);
				if(f_open(&MyFile, (const char*)str, FA_READ) == FR_OK)
					{
					f_read(&MyFile, SCAN_BUF, sizeof(SCAN_BUF), (UINT*)&bytesread);
					}
					f_close(&MyFile);
	   }
	 }
		FATFS_UnLinkDriver(SD_Path);
	 f_close(&MyFile);
	 
	 if(FATFS_LinkDriver(&SD_Driver, SD_Path) == 0)
{

ubNumberOftxtFiles = Storage_GetDirectoryBittxtFiles("/", pDirectoryFiles);
	}
//		BSP_SD_Init();
		FATFS_UnLinkDriver(SD_Path);
     f_close(&MyFile);
	sprintf ((char*)str, "%-11.11s", pDirectoryFiles[0]);
			if(FATFS_LinkDriver(&SD_Driver, SD_Path) == 0)
		{
			if(f_mount(&SD_FatFs, (TCHAR const*)SD_Path, 0) == FR_OK)
			{
		if(f_open(&MyFile,(const char*)str, FA_READ) == FR_OK)
		{
		//	f_read(&MyFile, SCAN_BUF, sizeof(SCAN_BUF), (UINT*)&bytesread);
		}
		 f_close(&MyFile);
	}
	 }
		FATFS_UnLinkDriver(SD_Path);
		 f_close(&MyFile);
 HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
	 saveindex=0;
	 dissaved(295,133);

}

uint16_t Calibration_GetX(uint16_t x)
{
  return (((A1 * x) + B1)/1000);
}


/**
  * @brief  Calibrate Y position
  * @param  y : Y position
  * @retval calibrated y
  */
uint16_t Calibration_GetY(uint16_t y)
{
  return 800-(((A2 * y) + B2)/1000);
}


void jisuanyingli(void)
{
uint16_t  x;
	yinglimax=yingli[2];
	yinglimin=yingli[2];
	for(x=2;x<yinglicount;x++)
		{
			if(yingli[x]>100)
			{
				if(yingli[x]>yinglimax)
					yinglimax=yingli[x];
				if(yingli[x]<yinglimin)
					yinglimin=yingli[x];
			
			}
		
		}
		if(yinglimin>0)
	yinglixishu=(float)(yinglimax-yinglimin)*2/(float)(yinglimax+yinglimin);
		yinglizhi=yinglixishu*100000;

}
void savehisdata(void)
{
uint8_t mun;
  RTC_DateTypeDef sdatestructureget;
  RTC_TimeTypeDef stimestructureget;
   HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
  /* Get the RTC current Time */
  HAL_RTC_GetTime(&RtcHandle, &stimestructureget, RTC_FORMAT_BIN);
  /* Get the RTC current Date */
  HAL_RTC_GetDate(&RtcHandle, &sdatestructureget, RTC_FORMAT_BIN);
	getstoremun();
for(mun=0;mun<13;mun++)
	str[mun]=0;
	//str[6]=48;
str[0]=sdatestructureget.Year+65;	
str[1]=sdatestructureget.Month+65;
if(sdatestructureget.Date<10)
	str[2]=sdatestructureget.Date+48;
else
	str[2]=sdatestructureget.Date+65-10;
str[3]=stimestructureget.Hours+65;
str[4]=(storemun%25)+65;
str[5]=(storemun/25)+65;
str[6]=0x4d;
str[7]=0x2e;
str[8]=0x42;
str[9]=0x4d;	
str[10]=0x50;	
CH1_BUF[1]=paramvaule[0][2];//增益
if(modelindex==0)		
	CH1_BUF[1]=agctemp;//增益
else
	CH1_BUF[1]=paramvaule[0][2];//增益
CH1_BUF[2]=usedv;//声速
CH1_BUF[3]=paramvaule[0][8];	//零点
CH1_BUF[4]=paramvaule[0][9];	//温度
CH1_BUF[5]=thickavg;	//厚度
CH1_BUF[6]=thickavg>>16;	//厚度
CH1_BUF[7]=paramvaule[0][3];	//波形
if(FATFS_LinkDriver(&SD_Driver, SD_Path) == 0)
{


//	sprintf ((char*)str, "%-11.11s", pDirectoryFiles[0]);
	//	f_unlink ((const char*)str);
	ubNumberOfFiles = Storage_GetDirectoryBitmapFiles("/", pDirectoryFiles);
	}

		FATFS_UnLinkDriver(SD_Path);
     f_close(&MyFile);

	
		if(FATFS_LinkDriver(&SD_Driver, SD_Path) == 0&&ubNumberOfFiles<MAX_BMP_FILES)
		{
			if(f_mount(&SD_FatFs, (TCHAR const*)SD_Path, 0) == FR_OK)
			{
				if(f_open(&MyFile, (const char*)str, FA_CREATE_ALWAYS | FA_WRITE) == FR_OK)
					{
						f_write(&MyFile, CH1_BUF, sizeof(CH1_BUF), (void *)&byteswritten);
					}
					f_close(&MyFile);
				if(f_open(&MyFile, (const char*)str, FA_READ) == FR_OK)
					{
					f_read(&MyFile, w_buf, sizeof(w_buf), (UINT*)&bytesread);
					}
					f_close(&MyFile);
	   }
	 }
		ubNumberOfFiles = Storage_GetDirectoryBitmapFiles("/", pDirectoryFiles);
		FATFS_UnLinkDriver(SD_Path);
	 f_close(&MyFile);
	/* 
	 if(FATFS_LinkDriver(&SD_Driver, SD_Path) == 0)
{

ubNumberOfFiles = Storage_GetDirectoryBitmapFiles("/", pDirectoryFiles);
	}
//		BSP_SD_Init();
		FATFS_UnLinkDriver(SD_Path);
     f_close(&MyFile);
	sprintf ((char*)str, "%-11.11s", pDirectoryFiles[0]);
			if(FATFS_LinkDriver(&SD_Driver, SD_Path) == 0)
		{
			if(f_mount(&SD_FatFs, (TCHAR const*)SD_Path, 0) == FR_OK)
			{
		if(f_open(&MyFile,(const char*)str, FA_READ) == FR_OK)
		{
			f_read(&MyFile, w_buf, sizeof(w_buf), (UINT*)&bytesread);
		}
		 f_close(&MyFile);
	}
	 }
		FATFS_UnLinkDriver(SD_Path);
		 f_close(&MyFile);
		 */
 HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
	 saveindex=0;
	 dissaved(295,133);

}
void clearfirstwaveindex(uint16_t coordinate)
{
uint32_t color;
	color=BSP_LCD_GetTextColor();
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	if(coordinate<479)
	{
	BSP_LCD_DrawLine(coordinate,114,coordinate,123);	
	BSP_LCD_DrawLine(coordinate+1,114,coordinate+1,123);			
	}
	BSP_LCD_SetTextColor(color);
}

u8 calfirstwaveindex(uint16_t coordinate)
{
u8 coordinatex;
		if(coordinate!=15001)
	{
	if(coordinate>(paramvaule[0][0]*100)&&coordinate<(paramvaule[0][1]*100))
	{
	coordinatex=(coordinate-paramvaule[0][0]*100)*240/(paramvaule[0][1]*100-paramvaule[0][0]*100)+2;
		if(coordinatex<2)
			coordinatex=2;
			if(coordinatex>238)	
					coordinatex=238;
			

	}
	else
	{
	coordinatex=250;
	}
}
	else
	coordinatex=250;
return	coordinatex;
}

u8 calsecondwaveindex(uint16_t coordinate)
{
u8  coordinatex;
if(coordinate!=15001)
{
	if((coordinate>(paramvaule[0][0]*100)&&coordinate<(paramvaule[0][1]*100)))
	{
	coordinatex=(coordinate-paramvaule[0][0]*100)*240/(paramvaule[0][1]*100-paramvaule[0][0]*100)+2;
		if(coordinatex<2)
			coordinatex=2;
			if(coordinatex>238)	
					coordinatex=238;
		
	}
	else
	{
	coordinatex=250;
	}
}
else
	coordinatex=250;
return coordinatex;
}
void disfirstwaveindex(uint16_t coordinate)
{
uint16_t  coordinatex;
uint32_t color;
	color=BSP_LCD_GetTextColor();
	BSP_LCD_SetTextColor(LCD_COLOR_RED);
	if(coordinate!=15001)
	{
	if(coordinate>(paramvaule[0][0]*100)&&coordinate<(paramvaule[0][1]*100))
	{
	coordinatex=(coordinate-paramvaule[0][0]*100)*480/(paramvaule[0][1]*100-paramvaule[0][0]*100)+2;
		if(coordinatex<2)
			coordinatex=2;
			if(coordinatex>478)	
					coordinatex=478;
	BSP_LCD_DrawLine(coordinatex,114,coordinatex,123);
	BSP_LCD_DrawLine(coordinatex+1,114,coordinatex+1,123);			
		prefirstcoordinate=coordinatex;
	}
	else
	{
	prefirstcoordinate=500;
	}
}
	else
	prefirstcoordinate=500;
BSP_LCD_SetTextColor(color);		
}
void dissecondwaveindex(uint16_t coordinate)
{
uint16_t  coordinatex;
uint32_t color;
	color=BSP_LCD_GetTextColor();
	BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
if(coordinate!=15001)
{
	if((coordinate>(paramvaule[0][0]*100)&&coordinate<(paramvaule[0][1]*100)))
	{
	coordinatex=(coordinate-paramvaule[0][0]*100)*480/(paramvaule[0][1]*100-paramvaule[0][0]*100)+2;
		if(coordinatex<2)
			coordinatex=2;
			if(coordinatex>478)	
					coordinatex=478;
	BSP_LCD_DrawLine(coordinatex,114,coordinatex,123);	
	BSP_LCD_DrawLine(coordinatex+1,114,coordinatex+1,123);			
		presecondcoordinate=coordinatex;
	}
	else
	{
	presecondcoordinate=500;
	}
}
else
	presecondcoordinate=500;
BSP_LCD_SetTextColor(color);
}
void clearsecondwaveindex(uint16_t coordinate)
{
uint32_t color;
	color=BSP_LCD_GetTextColor();
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	if(coordinate<479)
	{
	BSP_LCD_DrawLine(coordinate,114,coordinate,123);
	BSP_LCD_DrawLine(coordinate+1,114,coordinate+1,123);
	}
BSP_LCD_SetTextColor(color);
}
static void POWER_Config(void)
{
  GPIO_InitTypeDef   GPIO_InitStructure;

			  /* Enable GPIOB clock */
  __HAL_RCC_GPIOB_CLK_ENABLE();
	  /* Configure PB.5 pin as output */
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Pull = GPIO_PULLDOWN;
  GPIO_InitStructure.Pin = GPIO_PIN_5;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);		//开机拉高电平

	
//	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 10, 10);
//  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

	
}

 void GPIO_Config(void)
{
  GPIO_InitTypeDef   GPIO_InitStructure;

  /* Enable GPIOC clock */
  __HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
//	__HAL_RCC_GPIOD_CLK_ENABLE();
  /* Configure PA.6 pin as input floating */
  GPIO_InitStructure.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStructure.Pull = GPIO_PULLUP;
  GPIO_InitStructure.Pin = GPIO_PIN_6;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
	
  /* Configure PC.13 pin as input floating */
  GPIO_InitStructure.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Pin = GPIO_PIN_13;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* Enable and set EXTI lines 15 to 10 Interrupt to the lowest priority */
	//中断线13
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 15, 15);
  //HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
	HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
	  /* Configure PD.3 pin as input floating */
  GPIO_InitStructure.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStructure.Pull = GPIO_PULLUP;
  GPIO_InitStructure.Pin = GPIO_PIN_3;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);

  /* Enable and set EXTI lines 3 Interrupt to the lowest priority */ 
	//中断线3
  HAL_NVIC_SetPriority(EXTI3_IRQn, 10, 8);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);
				/* Enable GPIOB clock */
  __HAL_RCC_GPIOB_CLK_ENABLE();
	  /* Configure PB.13 pin as input floating */   //输出控制选择开关，使用WIFI，还是有线的开关0
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Pin = GPIO_PIN_13;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	
	
}



void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{ 
	switch(GPIO_Pin)
	{	case GPIO_PIN_13:
  //if (GPIO_Pin == GPIO_PIN_13)
				{  
          					
					if(lcd_flag==1)  //显示波形刷新
				     lcd_flag=0;
			    else
					   lcd_flag=lcd_flag+1;
		      if(lcd_flag1==40) //取按键值的次数
				     lcd_flag1=0;
			    else
		         lcd_flag1=lcd_flag1+1;		
					


		     		if(heartcount<1000)
							heartcount++;

			     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
			     BSP_SRAM_ReadData_DMA(SRAM_DEVICE_ADDR,w_buf,8000);
	

        }
      break;  
	   case GPIO_PIN_3:
	       {
	          if(scanhisindex==0&&scanindex==1&&pause==0)
	          {		
	          	if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6)==0)
	       	    		encoder--;
	       	    else
	       	    		encoder++;
	       	    if(encoder<0)
	       	    		encoder=0;
	       	    if(encoder>19999)
	       	    		encoder=19999;
							//scanthickdis=5000;
	       	    SCAN_BUF[encoder]=scanthickdis;
							SCAN_BUF[19999]=encoder;
	           }	
					 //encoder++;
	        }
	        break; 
	 }      
}

static void MX_ADC1_Init(void)
{
  /*##-1- Configure the ADC peripheral #######################################*/
  AdcHandle.Instance          = ADC3;
  
  AdcHandle.Init.ClockPrescaler        = ADC_CLOCKPRESCALER_PCLK_DIV4;
  AdcHandle.Init.Resolution            = ADC_RESOLUTION_12B;
  AdcHandle.Init.ScanConvMode          = DISABLE;                       /* Sequencer disabled (ADC conversion on only 1 channel: channel set on rank 1) */
  AdcHandle.Init.ContinuousConvMode    = DISABLE;                       /* Continuous mode disabled to have only 1 conversion at each conversion trig */
  AdcHandle.Init.DiscontinuousConvMode = DISABLE;                       /* Parameter discarded because sequencer is disabled */
  AdcHandle.Init.NbrOfDiscConversion   = 0;
  AdcHandle.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;        /* Conversion start trigged at each external event */
  AdcHandle.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_T1_CC1;
  AdcHandle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
  AdcHandle.Init.NbrOfConversion       = 1;
  AdcHandle.Init.DMAContinuousRequests = DISABLE;
  AdcHandle.Init.EOCSelection          = DISABLE;
  
  if (HAL_ADC_Init(&AdcHandle) != HAL_OK)
  {
    /* ADC initialization Error */
    Error_Handler();
  }
}

uint16_t Get_ADC(uint32_t ch)
{
	  ADC_ChannelConfTypeDef sConfig;
	
	  sConfig.Channel = ch;
		sConfig.Rank = 1;
		sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
		if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK)
		{
				Error_Handler();
		}
		
		if (HAL_ADC_Start(&AdcHandle) != HAL_OK)
		{
    /* Start Conversation Error */
			Error_Handler();
		}
		HAL_ADC_PollForConversion(&AdcHandle, 10);

		return(uint16_t)HAL_ADC_GetValue(&AdcHandle);	
}
//////////////////////////////////////////////////////////////////
void RTC_Init(void)
{

	RtcHandle.Instance = RTC;
  RTC_DateTypeDef  sdatestructure;
  RTC_TimeTypeDef  stimestructure;
  /* Configure RTC prescaler and RTC data registers */
  /* RTC configured as follows:
      - Hour Format    = Format 24
      - Asynch Prediv  = Value according to source clock
      - Synch Prediv   = Value according to source clock
      - OutPut         = Output Disable
      - OutPutPolarity = High Polarity
      - OutPutType     = Open Drain */
  RtcHandle.Init.HourFormat     = RTC_HOURFORMAT_24;
  RtcHandle.Init.AsynchPrediv   = RTC_ASYNCH_PREDIV;
  RtcHandle.Init.SynchPrediv    = RTC_SYNCH_PREDIV;
  RtcHandle.Init.OutPut         = RTC_OUTPUT_DISABLE;
  RtcHandle.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  RtcHandle.Init.OutPutType     = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&RtcHandle) != HAL_OK)
  {
    /* Initialization Error */
    BSP_LCD_DisplayStringAtLine(4, (uint8_t*)"    rtc OFF");
   // Error_Handler();
    
  }
  /*##-1- Configure the Date #################################################*/
  /* Set Date: Tuesday February 18th 2015 */
  sdatestructure.Year = 0x20;
  sdatestructure.Month = RTC_MONTH_OCTOBER;
  sdatestructure.Date = 0x14;
  sdatestructure.WeekDay = RTC_WEEKDAY_WEDNESDAY;
   /*##-2- Configure the Time #################################################*/
  /* Set Time: 02:20:00 */
  stimestructure.Hours = 0x14;
  stimestructure.Minutes = 0x55;
  stimestructure.Seconds = 0x50;
  stimestructure.TimeFormat = RTC_HOURFORMAT12_PM;
  stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE ;
  stimestructure.StoreOperation = RTC_STOREOPERATION_RESET; 
/*
	   if(HAL_RTC_SetTime(&RtcHandle,&stimestructure, RTC_FORMAT_BCD) != HAL_OK)//执行一次即可设定好
  {

   Error_Handler(); 
} 
  if(HAL_RTC_SetDate(&RtcHandle,&sdatestructure, RTC_FORMAT_BCD) != HAL_OK)//执行一次即可设定好
  {

   Error_Handler(); 
} 
*/
}

char transformHtoB(char data)
{char xd,yd;
						xd=((data)%100)/10;						
						xd=xd*0x10;
					yd=(data)%10+xd;	
return yd;
}

void RTC_set(char data)
{
		RtcHandle.Instance = RTC;
  RTC_DateTypeDef  sdatestructure;
  RTC_TimeTypeDef  stimestructure;
	  RTC_DateTypeDef sdatestructureget;
  RTC_TimeTypeDef stimestructureget;
	HAL_RTC_GetTime(&RtcHandle, &stimestructureget, RTC_FORMAT_BIN);
  HAL_RTC_GetDate(&RtcHandle, &sdatestructureget, RTC_FORMAT_BIN);
	sdatestructure.Year = transformHtoB(sdatestructureget.Year);
  sdatestructure.Month = transformHtoB(sdatestructureget.Month);
  sdatestructure.Date = transformHtoB(sdatestructureget.Date);
  sdatestructure.WeekDay = transformHtoB(sdatestructureget.WeekDay);
	stimestructure.Hours = transformHtoB(stimestructureget.Hours);
  stimestructure.Minutes = transformHtoB(stimestructureget.Minutes);
  stimestructure.Seconds = transformHtoB(stimestructureget.Seconds);
	if(timemodifything==1)	
  sdatestructure.Year = data;		
	else if(timemodifything==2)
  sdatestructure.Month = data;
	else if(timemodifything==3)
  sdatestructure.Date = data;
	else if(timemodifything==4)
  stimestructure.Hours = data;
	else if(timemodifything==5)
  stimestructure.Minutes = data;
	else if(timemodifything==6)
  stimestructure.Seconds = data;
HAL_Delay(100);
  RtcHandle.Init.HourFormat     = RTC_HOURFORMAT_24;
  RtcHandle.Init.AsynchPrediv   = RTC_ASYNCH_PREDIV;
  RtcHandle.Init.SynchPrediv    = RTC_SYNCH_PREDIV;
  RtcHandle.Init.OutPut         = RTC_OUTPUT_DISABLE;
  RtcHandle.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  RtcHandle.Init.OutPutType     = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&RtcHandle) != HAL_OK)
  {
    /* Initialization Error */
    BSP_LCD_DisplayStringAtLine(4, (uint8_t*)"    rtc OFF");
   // Error_Handler();
    
  }
	
	   if(HAL_RTC_SetTime(&RtcHandle,&stimestructure, RTC_FORMAT_BCD) != HAL_OK)//执行一次即可设定好
  {

   Error_Handler(); 
} 
  if(HAL_RTC_SetDate(&RtcHandle,&sdatestructure, RTC_FORMAT_BCD) != HAL_OK)//执行一次即可设定好
  {

   Error_Handler(); 
}  
	
}

void RTC_TimeShow(uint8_t* showtime)
{
  RTC_DateTypeDef sdatestructureget;
  RTC_TimeTypeDef stimestructureget;
  
  /* Get the RTC current Time */
  HAL_RTC_GetTime(&RtcHandle, &stimestructureget, RTC_FORMAT_BIN);
  /* Get the RTC current Date */
  HAL_RTC_GetDate(&RtcHandle, &sdatestructureget, RTC_FORMAT_BIN);
  
  BSP_LCD_DisplayStringAt(377,37,(uint8_t*)":", LEFT_MODE);//时间显示
  BSP_LCD_DisplayStringAt(415,37,(uint8_t*)":", LEFT_MODE);//
  
  BSP_LCD_DisplayStringAt(407,64,(uint8_t*)"-", LEFT_MODE);//
  BSP_LCD_DisplayStringAt(441,64,(uint8_t*)"-", LEFT_MODE);//
  /* Display time Format : hh:mm:ss */
//  sprintf((char*)showtime,"%02d:%02d:%02d",stimestructureget.Hours, stimestructureget.Minutes, stimestructureget.Seconds);	
	 LCD_ShowxNum(427,37,stimestructureget.Seconds,2,LCD_COLOR_WHITE);//LCD_COLOR_LIGHTMAGENTA	 
	 LCD_ShowxNum(391,37,stimestructureget.Minutes,2,LCD_COLOR_WHITE);
	 LCD_ShowxNum(355,37,stimestructureget.Hours,  2,LCD_COLOR_WHITE);	 
	 LCD_ShowxNum(355,64,20,2,LCD_COLOR_WHITE);      //2019年
	 LCD_ShowxNum(379,64,sdatestructureget.Year,2,LCD_COLOR_WHITE);	
	 LCD_ShowxNum(419,64,sdatestructureget.Month,2,LCD_COLOR_WHITE);
	 LCD_ShowxNum(455,64,sdatestructureget.Date,2,LCD_COLOR_WHITE); 
	 	if(timemodify==1)
		{
			if(timemodifything==1)//年
			{
			LCD_ShowxNum(379,64,sdatestructureget.Year,2,LCD_COLOR_GREEN);
			}
			else if(timemodifything==2)//月
			{
				LCD_ShowxNum(419,64,sdatestructureget.Month,2,LCD_COLOR_GREEN);
			}
			else if(timemodifything==3)//日
			{
				LCD_ShowxNum(455,64,sdatestructureget.Date,2,LCD_COLOR_GREEN); 			
			}
			else if(timemodifything==4)//时
			{
				LCD_ShowxNum(355,37,stimestructureget.Hours,  2,LCD_COLOR_GREEN);				
			}	
			else if(timemodifything==5)//分
			{
				 LCD_ShowxNum(391,37,stimestructureget.Minutes,2,LCD_COLOR_GREEN);
			}
			else if(timemodifything==6)//秒
			{
				LCD_ShowxNum(427,37,stimestructureget.Seconds,2,LCD_COLOR_GREEN);			
			}				
		}
	 
} 
void FPGA_PAR_EN_Init(void)
{
	GPIO_InitTypeDef  gpio_init_structure;
	GPIO_InitTypeDef  GPIO_InitStructure ;
	__HAL_RCC_GPIOB_CLK_ENABLE();
	gpio_init_structure.Pin = GPIO_PIN_0;
	gpio_init_structure.Mode = GPIO_MODE_OUTPUT_PP;
	gpio_init_structure.Pull = GPIO_PULLUP;
	gpio_init_structure.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(GPIOB, &gpio_init_structure);
	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
}
void wuxianshezhi(void)
{
	uint8_t *bufferData1;
uart_init(9600);//波特率固定位9600
HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_RESET);//无线电源关闭
delay_ms(100);
HAL_UART_Receive_IT(&UART3_Handler, (u8 *)aRxBuffer, 13);	
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_RESET);//SET置低
HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_SET);//无线电源打开
	delay_ms(200);	
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_SET);//SET置高
	bufferData1="DL-30 ";
	delay_ms(500);
	HAL_UART_Transmit(&UART3_Handler,bufferData1,6,1000);
   while (HAL_UART_GetState(&UART3_Handler) != HAL_UART_STATE_READY){}
		 if(paramvaule[2][9]==24)
			 bufferData1="0024 ";
		 else if(paramvaule[2][9]==48)
			 bufferData1="0048 ";
		 else if(paramvaule[2][9]==96)
			 bufferData1="0096 ";
		 		 else if(paramvaule[2][9]==1140)
			 bufferData1="1140 ";
				 		 else if(paramvaule[2][9]==1920)
			 bufferData1="1920 ";
						 		 else if(paramvaule[2][9]==3840)
			 bufferData1="3840 ";
								 		 else if(paramvaule[2][9]==5760)
			 bufferData1="5760 ";
										 		 else if(paramvaule[2][9]==1152)
			 bufferData1="1152 ";
												 else
													 bufferData1="1152 ";
	HAL_UART_Transmit(&UART3_Handler,bufferData1,5,1000);
   while (HAL_UART_GetState(&UART3_Handler) != HAL_UART_STATE_READY){}	
		 if(paramvaule[2][10]==0)
			 bufferData1="00 ";
		 else if(paramvaule[2][10]==1)
			 bufferData1="01 ";
		 else if(paramvaule[2][10]==2)
			 bufferData1="02 ";
		 		 else if(paramvaule[2][10]==3)
			 bufferData1="03 ";
				 else if(paramvaule[2][10]==4)
			 bufferData1="04 ";
				 else if(paramvaule[2][10]==5)
			 bufferData1="05 ";
				 else if(paramvaule[2][10]==6)
			 bufferData1="06 ";
				else if(paramvaule[2][10]==7)
			 bufferData1="07 ";
				 else if(paramvaule[2][10]==8)
			 bufferData1="08 ";
				 else if(paramvaule[2][10]==9)
			 bufferData1="09 ";
				 else if(paramvaule[2][10]==10)
			 bufferData1="10 ";
				 else if(paramvaule[2][10]==11)
			 bufferData1="11 ";
				 else if(paramvaule[2][10]==12)
			 bufferData1="12 ";
				 else if(paramvaule[2][10]==13)
			 bufferData1="13 ";
				 else if(paramvaule[2][10]==14)
			 bufferData1="14 ";
				 else if(paramvaule[2][10]==15)
			 bufferData1="15 ";
				else
			bufferData1="00 ";
	HAL_UART_Transmit(&UART3_Handler,bufferData1,3,1000);
   while (HAL_UART_GetState(&UART3_Handler) != HAL_UART_STATE_READY){}
		 if(paramvaule[2][11]==0)
			 bufferData1="A";
		 else if(paramvaule[2][11]==1)
			 bufferData1="B";
		 else if(paramvaule[2][11]==2)
			 bufferData1="O";
		 else
			 bufferData1="A";
	HAL_UART_Transmit(&UART3_Handler,bufferData1,1,1000);
   while (HAL_UART_GetState(&UART3_Handler) != HAL_UART_STATE_READY){}
		 delay_ms(500);
uart_init(115200);		 
}
uint8_t  GCMDFbuffer(uint8_t *pcmd, uint8_t *buffer)
{
uint16_t i,y,k, FrmLen;
	for(i=0;i<40;i++)
	{
		
		if(buffer[i]==0x66)
		{
			FrmLen=buffer[i+2]*256+buffer[i+3]+5;
			break;
		}	
buffer[i]=0;		
	}
	if((FrmLen+i)>39)
		return 0;
	else
	{
	for(y=0;y<=FrmLen;y++)
		pcmd[y]=buffer[i+y];
		k=0;
		for(y=i+FrmLen+1;y<40;y++)
		{
		buffer[k]=buffer[y];
		k++;
		}
		for(y=40-i-FrmLen;y<40;y++)		
		buffer[y]=0;	
		return 1;
	}

	


}
uint8_t FBufferHasExFrame(uint8_t *pcmd, uint8_t *buffer)
{
uint16_t i,y,k, FrmLen;
	i=0;
	for(y=0;y<40;y++)
	{
		if(buffer[y]==0&&buffer[y+1]==0&&buffer[y+2]==0&&buffer[y+3]==0)
		break;
		if(buffer[y]==0x66&&buffer[y+1]==0&&(buffer[y+4]==0x11||buffer[y+4]==0x22||buffer[y+4]==0x33||buffer[y+4]==0x41||buffer[y+4]==0x42||buffer[y+4]==0x44))//包头
		{
			FrmLen=(buffer[y+2]<<8)|buffer[y+3];
			if(y+FrmLen+5>40)
				break;
			if(buffer[y+FrmLen+5]==0x99)
			{
			for(k=y;k<y+FrmLen+6;k++)
				{
				pcmd[i]=buffer[k];
				i++;
				}
				y=y+FrmLen+5;
			}
		}
	}
}
uint16_t Cmd2Frm(uint8_t *pfrm, uint8_t *pcmd, uint16_t nByteLen)
{
    uint8_t nCheckRes = 0;
	uint16_t i, FrmLen;

	pfrm[0] = FRAME_HEAD;
	pfrm[1] = (uint8_t)(nByteLen>>8);
	pfrm[2] = (uint8_t)(nByteLen&0xFF);

	FrmLen = nByteLen + 3;
	nCheckRes = pfrm[0]^pfrm[1]^pfrm[2];
	for(i = 3; i < FrmLen; i++)
	{
		pfrm[i] = pcmd[i-3];
		nCheckRes ^= pfrm[i];
	}
	pfrm[FrmLen++] = nCheckRes;
	pfrm[FrmLen++] = FRAME_END;
	return FrmLen;
}
void ProcPrmFrame(uint8_t *pcmd)
{
	uint8_t frmType;
	frmType=pcmd[4];
			switch(frmType)
		{
		case 0x11:	ProcParaCmd(pcmd);	break;	//0x11	参数指令
		case 0x22:	ProcWaveCmd(pcmd);	break;	//0x22	波形指令

		default:
			break;
		}
}
uint8_t ProcParaCmd(uint8_t *pcmd)
{
  uint8_t nCheckRes = 0;
	uint16_t i, FrmLen,mun;
	FrmLen=pcmd[2]<<8|pcmd[3];
	mun=pcmd[6]<<8|pcmd[7];
	if(pcmd[0]==0x66&&pcmd[FrmLen+5]==0x99)
	{
		switch(pcmd[5])
		{
		case 0x55:	//	读参数
			wifisenddata[0]=0x66;
			wifisenddata[1]=0x00;
		if(pcmd[6]!=0xff)
		{
			wifisenddata[2]=6>>8;//数据长度
			wifisenddata[3]=6;
		}
		else
		{
			wifisenddata[2]=72>>8;//数据长度
			wifisenddata[3]=72;
		}			
			wifisenddata[4]=0x11;
			wifisenddata[5]=pcmd[5];
			wifisenddata[6]=pcmd[6];	//参数编号			
	
			if(pcmd[6]!=0xff)//读单个参数
			{
							switch(pcmd[6])								
								{
									case 0://铝合金版本增益调节上限
												wifisenddata[7]=paramvaule[3][6]>>8;
												wifisenddata[8]=paramvaule[3][6];																		
										  break;
									case 1://探头零点-包洛波
												wifisenddata[7]=paramvaule[0][15]>>8;
												wifisenddata[8]=paramvaule[0][15];									
										  break;
									case 2://声速	
												wifisenddata[7]=paramvaule[0][6]>>8;
												wifisenddata[8]=paramvaule[0][6];										

										  break;	
									case 3://探头零点-射频波		
												wifisenddata[7]=paramvaule[0][8]>>8;
												wifisenddata[8]=paramvaule[0][8];												
										  break;	
									case 4://钛合金版本增益调节上限			
												wifisenddata[7]=paramvaule[3][7]>>8;
												wifisenddata[8]=paramvaule[3][7];												
										  break;	
									case 5://闸门1开始
												wifisenddata[7]=paramvaule[0][4]>>8;
												wifisenddata[8]=paramvaule[0][4];												
										  break;
									case 6://闸门1结束	
												wifisenddata[7]=paramvaule[0][5]>>8;
												wifisenddata[8]=paramvaule[0][5];												
										  break;										
									case 7://标块厚度	
												wifisenddata[7]=paramvaule[0][7]>>8;
												wifisenddata[8]=paramvaule[0][7];												
										 break;
									case 8://温度	
												wifisenddata[7]=paramvaule[0][9]>>8;
												wifisenddata[8]=paramvaule[0][9];																						
										  break;																																			
									case 9://平滑次数				
												wifisenddata[7]=paramvaule[0][11]>>8;
												wifisenddata[8]=paramvaule[0][11];												;										
										  break;	
									case 10://增益	
												wifisenddata[7]=paramvaule[0][2]>>8;
												wifisenddata[8]=paramvaule[0][2];																					
										  break;	
									case 11://探头选择
												wifisenddata[7]=paramvaule[3][9]>>8;
												wifisenddata[8]=paramvaule[3][9];																						
										  break;	
									case 12:///波形选择1：原始波 0：包络波	
												wifisenddata[7]=paramvaule[0][3]>>8;
												wifisenddata[8]=paramvaule[0][3];																				
										  break;
									case 13://预设厚度
												wifisenddata[7]=paramvaule[2][8]>>8;
												wifisenddata[8]=paramvaule[2][8];																				
										  break;	
									case 14://补偿使能		
												wifisenddata[7]=paramvaule[3][8]>>8;
												wifisenddata[8]=paramvaule[3][8];																													
										  break;	
									case 15://脉冲个数 1-5	
												wifisenddata[7]=paramvaule[2][2]>>8;
												wifisenddata[8]=paramvaule[2][2];																														
										  break;	
									case 16://激励频率 100-1000 100=1M   1000=10M		
												wifisenddata[7]=paramvaule[3][2]>>8;
												wifisenddata[8]=paramvaule[3][2];																														
										  break;	
									case 17://测量模式		0固定PP  1自动切换		
												wifisenddata[7]=paramvaule[3][1]>>8;
												wifisenddata[8]=paramvaule[3][1];																																	
										  break;
									case 18://手动测量模式		0固定PP  1ZP	
												wifisenddata[7]=paramvaule[0][12]>>8;
												wifisenddata[8]=paramvaule[0][12];																																	
										  break;
									case 19://算法		0自乘 1无	
												wifisenddata[7]=paramvaule[3][3]>>8;
												wifisenddata[8]=paramvaule[3][3];																						
										  break;
									case 20://厚度平滑系数	
												wifisenddata[7]=paramvaule[3][4]>>8;
												wifisenddata[8]=paramvaule[3][4];																										
										  break;
									case 21://传感器到位置	
												wifisenddata[7]=inposition>>8;
												wifisenddata[8]=inposition;																			
										  break;									
									case 22://自动/手动增益选择1：自动 0：手动	
												wifisenddata[7]=modelindex>>8;
												wifisenddata[8]=modelindex;																							
										  break;
									case 64://无线下载打开	
												wifisenddata[7]=paramvaule[2][6]>>8;
												wifisenddata[8]=paramvaule[2][6];																										
										  break;									
									case 65://波特率	
												wifisenddata[7]=paramvaule[2][9]>>8;
												wifisenddata[8]=paramvaule[2][9];										
										  break;
									case 66://设置无线模块工作模式
												wifisenddata[7]=paramvaule[2][11]>>8;
												wifisenddata[8]=paramvaule[2][11];										
										  break;										
									case 67://设置无线模块
												wifisenddata[7]=shezhiwuxian>>8;
												wifisenddata[8]=shezhiwuxian;										
										  break;		
									case 68://信道
												wifisenddata[7]=paramvaule[2][10]>>8;
												wifisenddata[8]=paramvaule[2][10];																												
										  break;				
									case 80://显示开始		
												wifisenddata[7]=paramvaule[0][0]>>8;
												wifisenddata[8]=paramvaule[0][0];																														
										  break;
									case 81://显示结束		
												wifisenddata[7]=paramvaule[0][1]>>8;
												wifisenddata[8]=paramvaule[0][1];																													
										  break;	
									case 82://显示模式		0：显示厚度  1显示波形：	
												wifisenddata[7]=lcddisindex>>8;
												wifisenddata[8]=lcddisindex;																				
										  break;	
									case 83://版本选择	
												wifisenddata[7]=paramvaule[3][5]>>8;
												wifisenddata[8]=paramvaule[3][5];																							
										  break;	
									case 84://波形段选择	
												wifisenddata[7]=boxingduan>>8;
												wifisenddata[8]=boxingduan;																																					
										  break;									
									case 96://WIFI/有线信模式	
												wifisenddata[7]=paramvaule[3][0]>>8;
												wifisenddata[8]=paramvaule[3][0];																																
										  break;														
									default:
											break;										
									}		
										wifisenddata[9]=0xa5;
				nCheckRes = 0;
				for(i = 0; i <10; i++)
				{
					nCheckRes ^= wifisenddata[i];
				}	
			  wifisenddata[10]=	nCheckRes;
				wifisenddata[11]=	0x99;
				HAL_UART_Transmit(&UART3_Handler,wifisenddata,12,30000);
								}
			
			else//读全部参数
			{

												wifisenddata[7]=paramvaule[3][6]>>8;
												wifisenddata[8]=paramvaule[3][6];									
												wifisenddata[9]=paramvaule[0][15]>>8;
												wifisenddata[10]=paramvaule[0][15];									
												wifisenddata[11]=paramvaule[0][6]>>8;
												wifisenddata[12]=paramvaule[0][6];										
												wifisenddata[13]=paramvaule[0][8]>>8;
												wifisenddata[14]=paramvaule[0][8];														
												wifisenddata[15]=paramvaule[3][7]>>8;
												wifisenddata[16]=paramvaule[3][7];												
												wifisenddata[17]=paramvaule[0][4]>>8;
												wifisenddata[18]=paramvaule[0][4];												
												wifisenddata[19]=paramvaule[0][5]>>8;
												wifisenddata[20]=paramvaule[0][5];												
												wifisenddata[21]=paramvaule[0][7]>>8;
												wifisenddata[22]=paramvaule[0][7];													
												wifisenddata[23]=paramvaule[0][9]>>8;
												wifisenddata[24]=paramvaule[0][9];																						
												wifisenddata[25]=paramvaule[0][11]>>8;
												wifisenddata[26]=paramvaule[0][11];																											
												wifisenddata[27]=paramvaule[0][2]>>8;
												wifisenddata[28]=paramvaule[0][2];												;										
												wifisenddata[29]=paramvaule[3][9]>>8;
												wifisenddata[30]=paramvaule[3][9];																					
												wifisenddata[31]=paramvaule[0][3]>>8;
												wifisenddata[32]=paramvaule[0][3];																						
												wifisenddata[33]=paramvaule[2][8]>>8;
												wifisenddata[34]=paramvaule[2][8];												
												wifisenddata[35]=paramvaule[3][8]>>8;
												wifisenddata[36]=paramvaule[3][8];																							
												wifisenddata[37]=paramvaule[2][2]>>8;
												wifisenddata[38]=paramvaule[2][2];										
												wifisenddata[39]=paramvaule[3][2]>>8;
												wifisenddata[40]=paramvaule[3][2];										
												wifisenddata[41]=paramvaule[3][1]>>8;
												wifisenddata[42]=paramvaule[3][1];																				
												wifisenddata[43]=paramvaule[0][12]>>8;
												wifisenddata[44]=paramvaule[0][12];																												
												wifisenddata[45]=paramvaule[3][3]>>8;
												wifisenddata[46]=paramvaule[3][3];																													
												wifisenddata[47]=paramvaule[3][4]>>8;
												wifisenddata[48]=paramvaule[3][4];																									
												wifisenddata[49]=inposition>>8;
												wifisenddata[50]=inposition;																															
												wifisenddata[51]=modelindex>>8;
												wifisenddata[52]=modelindex;																														
												wifisenddata[53]=paramvaule[2][6]>>8;
												wifisenddata[54]=paramvaule[2][6];																															
												wifisenddata[55]=paramvaule[2][9]>>8;
												wifisenddata[56]=paramvaule[2][9];																													
												wifisenddata[57]=shezhiwuxian>>8;
												wifisenddata[58]=shezhiwuxian;																																	
												wifisenddata[59]=paramvaule[2][10]>>8;
												wifisenddata[60]=paramvaule[2][10];																																	
												wifisenddata[61]=paramvaule[2][11]>>8;
												wifisenddata[62]=paramvaule[2][11];																																	
												wifisenddata[63]=paramvaule[0][0]>>8;
												wifisenddata[64]=paramvaule[0][0];																				
												wifisenddata[65]=paramvaule[0][1]>>8;
												wifisenddata[66]=paramvaule[0][1];										
												wifisenddata[67]=lcddisindex>>8;
												wifisenddata[68]=lcddisindex;																										
												wifisenddata[69]=paramvaule[3][5]>>8;
												wifisenddata[70]=paramvaule[3][5];																							
												wifisenddata[71]=boxingduan>>8;
												wifisenddata[72]=boxingduan;																																					
												wifisenddata[73]=paramvaule[3][0]>>8;
												wifisenddata[74]=paramvaule[3][0];																												
											
										wifisenddata[75]=0xa5;
				nCheckRes = 0;
				for(i = 0; i <76; i++)
				{
					nCheckRes ^= wifisenddata[i];
				}	
			  wifisenddata[76]=	nCheckRes;
				wifisenddata[77]=	0x99;
				HAL_UART_Transmit(&UART3_Handler,wifisenddata,78,30000);			
			
			}
			break;	
		case 0xaa:	//	写参数
					
		
							paramentmun_r=pcmd[6];

							paramentdatatemp=pcmd[7];
							paramentdatatemp=(paramentdatatemp<<8)|pcmd[8];
							switch(paramentmun_r)								
								{
									case 0://铝合金版本增益调节上限
									if(paramentdatatemp<=paramvaulelimitup[3][6]&&paramentdatatemp>=200)
										paramvaule[3][6]=paramentdatatemp;									
										  break;
									case 1://探头零点-包洛波
										if(paramentdatatemp<=paramvaulelimitup[0][15]&&paramentdatatemp>=0)
										paramvaule[0][15]=paramentdatatemp;
										  break;
									case 2://声速								
										paramvaule[0][6]=paramentdatatemp;
										  break;	
									case 3://探头零点-射频波								
										if(paramentdatatemp<=paramvaulelimitup[0][8]&&paramentdatatemp>=0)
										paramvaule[0][8]=paramentdatatemp;
										  break;	
									case 4://钛合金版本增益调节上限						
										if(paramentdatatemp<=paramvaulelimitup[3][7]&&paramentdatatemp>=200)
										paramvaule[3][7]=paramentdatatemp;	
										  break;	
									case 5://闸门1开始							
										if(paramentdatatemp<=paramvaulelimitup[0][4]&&paramentdatatemp>=0)
										paramvaule[0][4]=paramentdatatemp;
										  break;
									case 6://闸门1结束							
										if(paramentdatatemp<=paramvaulelimitup[0][5]&&paramentdatatemp>=0)
										paramvaule[0][5]=paramentdatatemp;
										  break;										
									case 7://标块厚度							
										if(paramentdatatemp<=paramvaulelimitup[0][7]&&paramentdatatemp>=0)
										paramvaule[0][7]=paramentdatatemp;
										 break;
									case 8://温度						
										if(paramentdatatemp<=paramvaulelimitup[0][9]&&paramentdatatemp>=0)
										paramvaule[0][9]=paramentdatatemp;										
										  break;		
	
									case 9://平滑次数					
										if(paramentdatatemp<=paramvaulelimitup[0][11]&&paramentdatatemp>=0)
										paramvaule[0][11]=paramentdatatemp;										
										  break;	
									case 10://增益						
										if(paramentdatatemp<=paramvaulelimitup[0][2]&&paramentdatatemp>=0)
										paramvaule[0][2]=paramentdatatemp;										
										  break;	
									case 11://探头选择						
										if(paramentdatatemp>0)
										paramvaule[3][9]=1;	
											else	
										paramvaule[3][9]=0;													
										  break;										
									case 12:///波形选择1：原始波 0：包络波						
										if(paramentdatatemp<=paramvaulelimitup[0][3]&&paramentdatatemp>=0)
										paramvaule[0][3]=paramentdatatemp;										
										  break;	
									case 13://预设厚度
										if(paramentdatatemp<=paramvaulelimitup[2][8]&&paramentdatatemp>=0)
										paramvaule[2][8]=paramentdatatemp;	
										if(paramvaule[2][8]>100&&paramvaule[2][8]<200)
											taipinlv=500;
										else
											taipinlv=360;
										  break;	
									case 14://补偿使能				
									if(paramentdatatemp>0)
										paramvaule[3][8]=1;
									else	
										paramvaule[3][8]=0;										
										  break;
									case 15://脉冲个数 1-5						
										if(paramentdatatemp<=paramvaulelimitup[2][2]&&paramentdatatemp>=1)
										paramvaule[2][2]=paramentdatatemp;										
										  break;
									case 16://激励频率 100-1000 100=1M   1000=10M					
										if(paramentdatatemp<=paramvaulelimitup[3][2]&&paramentdatatemp>=100)
										paramvaule[3][2]=paramentdatatemp;										
										  break;	
									case 17://测量模式		0固定PP  1自动切换		
										if(paramentdatatemp<=paramvaulelimitup[3][1]&&paramentdatatemp>=0)
										paramvaule[3][1]=paramentdatatemp;	
													else	
										paramvaule[3][1]=paramvaulelimitup[3][1];														
										  break;
									case 18://手动测量模式		0固定PP  1ZP		
										if(paramentdatatemp<=paramvaulelimitup[0][12]&&paramentdatatemp>=0)
										paramvaule[0][12]=paramentdatatemp;	
													else	
										paramvaule[0][12]=paramvaulelimitup[0][12];														
										  break;
									case 19:///0自乘 1无							
										if(paramentdatatemp==0)
										  paramvaule[3][3]=0;		
											else		
											paramvaule[3][3]=1;		
										  break;
									case 20://厚度平滑系数	
										if(paramentdatatemp<=paramvaulelimitup[3][4]&&paramentdatatemp>=5)
										paramvaule[3][4]=paramentdatatemp;																		
										  break;												
									case 21://波传感器到位置	
										if(paramentdatatemp==0)									
													inposition=0;										
													else	
													inposition=1;	
								if(inposition==0)	
								{
										for(i=0;i<200;i++)
											thickavg1[i]=0;
								
								}									
										  break;										
									case 22://自动/手动增益选择1：自动 0：手动
										if(modelindex>=0&&modelindex<=4)
										modelindex=paramentdatatemp;		
												drawmainbroad();											
										  break;
									case 64://无线下载打开	
										if(paramentdatatemp==0)		
												paramvaule[2][6]=0;
										else
												paramvaule[2][6]=1;																									
										  break;									
									case 65://波特率
									if(paramentdatatemp<=paramvaulelimitup[2][9]&&paramentdatatemp>=0)										
										paramvaule[2][9]=paramentdatatemp;
										  break;	
									case 66://无线模块工作模式	
									if(paramentdatatemp<=paramvaulelimitup[2][11]&&paramentdatatemp>=0)										
													paramvaule[2][11]=paramentdatatemp;					
										  break;									
									case 67://设置无线模块
										shezhiwuxian=paramentdatatemp;
										  break;		
									case 68://信道
										if(paramentdatatemp<=paramvaulelimitup[2][10]&&paramentdatatemp>=0)
											paramvaule[2][10]=paramentdatatemp;										
										  break;				
									case 80://显示开始				
										if(paramentdatatemp<=paramvaulelimitup[0][0]&&paramentdatatemp>=0)
										paramvaule[0][0]=paramentdatatemp;	
											LCDxcoordinate();										
										  break;
									case 81://显示结束				
										if(paramentdatatemp<=paramvaulelimitup[0][1]&&paramentdatatemp>=0)
										paramvaule[0][1]=paramentdatatemp;	
													LCDxcoordinate();										
										  break;
									case 82://显示模式		0：显示厚度  1显示波形：		
									if(paramentdatatemp==0)		
											lcddisindex=0;	
										else
											lcddisindex=1;
										LCD_Fill(0,0,LCD_W,LCD_H,BLACK);
										if(lcddisindex==1)
											LCDxcoordinate();
										else
											LCDinformationunit();
										  break;
									case 83://版本选择	
										if(paramentdatatemp==0)
										{
										paramvaule[3][5]=0;	
											paramvaule[3][2]=500;																									
															paramvaule[3][4]=10;
										}
													else	
													{
										paramvaule[3][5]=1;	
														if(paramvaule[3][4]<70)
															paramvaule[3][4]=70;
														paramvaule[3][2]=360;
													}														
										  break;	
									case 84://波形段选择	
										if(paramentdatatemp==0)									
													boxingduan=0;										
													else	
													boxingduan=1;																												
										  break;													
									case 96:////WIFI/有线信模式					
										if(paramentdatatemp<=paramvaulelimitup[3][0]&&paramentdatatemp>=0)
										paramvaule[3][0]=paramentdatatemp;	
													else	
										paramvaule[3][0]=paramvaulelimitup[3][0];														
										  break;	
									default:
											break;										
									}
								 programparamentflash(); 
								wifisenddata[0]=0x66;
								wifisenddata[1]=0x00;
								wifisenddata[2]=6>>8;//数据长度
								wifisenddata[3]=6;
								wifisenddata[4]=0x11;
								wifisenddata[5]=pcmd[5];
								wifisenddata[6]=pcmd[6];	//参数编号		
								wifisenddata[7]=0x33;
								wifisenddata[8]=0xA5;	//总长度，2字节
								wifisenddata[9]=0xA5;										
					nCheckRes = 0;
				for(i = 0; i <10; i++)
				{
					nCheckRes ^= wifisenddata[i];
				}	
			  wifisenddata[10]=	nCheckRes;	
				wifisenddata[11]=	0x99;
				HAL_UART_Transmit(&UART3_Handler,wifisenddata,12,30000);	
							
			break;	
		case 0x5a:	//	恢复默认参数
				
			for(i = 0; i <4; i++)
		{ 	for(mun = 0; mun <16; mun++)
			paramvaule[i][mun]=paramvauledefault[i][mun];
		
		}
		 programparamentflash(); 
								wifisenddata[0]=0x66;
								wifisenddata[1]=0x00;
								wifisenddata[2]=6>>8;//数据长度
								wifisenddata[3]=6;
								wifisenddata[4]=0x11;
								wifisenddata[5]=pcmd[5];
								wifisenddata[6]=pcmd[6];	
								wifisenddata[7]=0x33;
								wifisenddata[8]=0xA5;	//总长度，2字节
								wifisenddata[9]=0xA5;	
							nCheckRes = 0;
				for(i = 0; i <10; i++)
				{
					nCheckRes ^= wifisenddata[i];
				}	
			  wifisenddata[10]=	nCheckRes;	
				wifisenddata[11]=	0x99;
				HAL_UART_Transmit(&UART3_Handler,wifisenddata,12,30000);	
					
			break;	
		//其他非法指令输出应答错误
		default:
			break;
		}	
		return 1;
	}
		else
			return 0;

}
uint8_t ProcWaveCmd(uint8_t *pcmd)
	{
	  uint8_t nCheckRes = 0,nCheckRes1;
	uint16_t i,k, FrmLen,mun;
			FrmLen=pcmd[2]<<8|pcmd[3];
		if(pcmd[0]==0x66&&pcmd[FrmLen+5]==0x99&&pcmd[4]==0x22)
		{
			//先发信息包
			wifisenddata[0]=0x66;
			wifisenddata[1]=0xa5;
			wifisenddata[2]=18>>8;//数据长度
			wifisenddata[3]=18;
			wifisenddata[4]=0x22;
			wifisenddata[5]=0x00;//包号
			wifisenddata[6]=32;	//总包数				
			wifisenddata[7]=0x0b;
			wifisenddata[8]=0xb8;	//总长度
				nCheckRes = 0;
				for(i = 0; i <1500; i++)
				{
					nCheckRes ^= CH1_BUF[i]>>8;
					nCheckRes ^= CH1_BUF[i];
				}
				nCheckRes1=nCheckRes;
			wifisenddata[9]=nCheckRes;
						wifisenddata[10]=thickavg>>8;
						wifisenddata[11]=thickavg;

						wifisenddata[12]=timeA1>>8;//一次回波
						wifisenddata[13]=timeA1;			
						wifisenddata[14]=timeA2>>8;//二次回波
						wifisenddata[15]=timeA2;		
		 	if(modelindex==0||modelindex==3||modelindex==4)//自动   增益
					{
						wifisenddata[16]=agctemp>>8;
						wifisenddata[17]=agctemp;						
					}
		else
					{
						wifisenddata[16]=paramvaule[0][2]>>8;
						wifisenddata[17]=paramvaule[0][2];	
					}
						wifisenddata[18]=paramvaule[3][2]>>8;//二次频率
						wifisenddata[19]=paramvaule[3][2];
						wifisenddata[20]=modelindex>>8;//测量模式
						wifisenddata[21]=modelindex;	
				nCheckRes = 0;
				for(i = 0; i <22; i++)
				{
					nCheckRes ^= wifisenddata[i];
				}
			wifisenddata[22]=nCheckRes;					
			wifisenddata[23]=0x99;	
			HAL_UART_Transmit(&UART3_Handler,wifisenddata,24,30000);	
	while (UART3_Handler.gState!= HAL_UART_STATE_READY)
																	{}	
HAL_Delay(10);																		
				//先把现在需要发送的波形存下来
				mun=1;
            for(wifisendcount=0;wifisendcount<1500;wifisendcount++)
                {
									if(boxingduan==0)
									{
									if(paramvaule[0][3]==1)
									{
                wavestrodata[wifisendcount*2]=(CH1_BUF[wifisendcount+4300])>>8;
                wavestrodata[1+wifisendcount*2]=CH1_BUF[wifisendcount+4300];
									}
									else
									{
                wavestrodata[wifisendcount*2]=CH1_BUF[wifisendcount+300]>>8;
                wavestrodata[1+wifisendcount*2]=CH1_BUF[wifisendcount+300];									
									
									}
								}else
									{
									if(paramvaule[0][3]==1)
									{
                wavestrodata[wifisendcount*2]=(CH1_BUF[wifisendcount+6300])>>8;
                wavestrodata[1+wifisendcount*2]=CH1_BUF[wifisendcount+6300];
									}
									else
									{
                wavestrodata[wifisendcount*2]=CH1_BUF[wifisendcount+2300]>>8;
                wavestrodata[1+wifisendcount*2]=CH1_BUF[wifisendcount+2300];																		
									}																		
									}								
                }

						 for(wifisendcount=0;wifisendcount<3000;)	
							{
							if(wifisendcount+96<=3000)
							{
								FrmLen=96;
								wifisenddata[2]=(FrmLen+4)>>8;//数据长度
								wifisenddata[3]=FrmLen+4;	
								wifisenddata[5]=mun;	
								wifisenddata[6]=wifisendcount>>8;	
								wifisenddata[7]=wifisendcount;	
								for(k=wifisendcount;k<wifisendcount+96;k++)
								wifisenddata[8+k-wifisendcount]=wavestrodata[k];
								wifisendcount=wifisendcount+96;
												nCheckRes = 0;
								for(i = 0; i <104; i++)
								{
									nCheckRes ^= wifisenddata[i];
								}
								wifisenddata[104]=nCheckRes;
								wifisenddata[105]=0x99;
							HAL_UART_Transmit(&UART3_Handler,wifisenddata,106,30000);
								  			while (HAL_UART_GetState(&UART3_Handler) != HAL_UART_STATE_READY)
																	{}
																		HAL_Delay(10);
								mun++;
							}
								else
								{
								FrmLen=3000-wifisendcount;
								wifisenddata[2]=(FrmLen+4)>>8;//数据长度
								wifisenddata[3]=FrmLen+4;	
								wifisenddata[5]=mun;	
								wifisenddata[6]=wifisendcount>>8;	
								wifisenddata[7]=wifisendcount;
								for(k=wifisendcount;k<3000;k++)
								wifisenddata[8+k-wifisendcount]=wavestrodata[k];		
												nCheckRes = 0;
								for(i = 0; i <(8+FrmLen); i++)
								{
									nCheckRes ^= wifisenddata[i];
								}	
								wifisenddata[8+FrmLen]=nCheckRes;
								wifisenddata[9+FrmLen]=0x99;	
							HAL_UART_Transmit(&UART3_Handler,wifisenddata,FrmLen+10,30000);		
									while (HAL_UART_GetState(&UART3_Handler) != HAL_UART_STATE_READY)
																	{}
																		HAL_Delay(10);
								break;
								}
								}	
	//确认帧
							wifisenddata[2]=6>>8;//数据长度
						  wifisenddata[3]=6;
							wifisenddata[5]=0xff;//帧序号		
							wifisenddata[6]=mun;//帧数量	
							wifisenddata[7]=0x0b;
							wifisenddata[8]=0xb8;	//总长度	
								
												nCheckRes = 0;
								for(i = 0; i <9; i++)
								{
									nCheckRes ^= wifisenddata[i];
								}
								nCheckRes^=nCheckRes1;
								wifisenddata[9]=nCheckRes;
								wifisenddata[10]=0x99;
									HAL_UART_Transmit(&UART3_Handler,wifisenddata,11,30000);
	while (HAL_UART_GetState(&UART3_Handler) != HAL_UART_STATE_READY)
																	{}								
											
		return 1;
		}
		else
	return 0;
	}
uint8_t ProcBanbenCmd(uint8_t *pcmd)
	{
	  uint8_t nCheckRes = 0;
	uint16_t i,k, FrmLen,mun;
			FrmLen=pcmd[2]<<8|pcmd[3];
		if(pcmd[0]==0x66&&pcmd[FrmLen+5]==0x99&&pcmd[4]==0x42&&pcmd[5]==1)	//软件版本
		{
			wifisenddata[0]=0x66;
			wifisenddata[1]=0x00;
			wifisenddata[2]=9>>8;//数据长度
			wifisenddata[3]=9;
			wifisenddata[4]=0x42;//命令号
			//版本类型
			wifisenddata[5]=1;
			wifisenddata[6]=0x53;
			wifisenddata[7]=0x54;	//ST
			//wifisenddata[6]=0x44;
			//wifisenddata[7]=0x56;	//DV
			//wifisenddata[6]=0x45;
			//wifisenddata[7]=0x58;	//EX	
//铝还是钛版本			
						if(paramvaule[3][5]==0)
							wifisenddata[8]=0x4c;//铝T
						else
							wifisenddata[8]=0x54;	//钛L	
//串口还是WIFI版本	
			wifisenddata[9]=0x53;	//S串口
		//	wifisenddata[9]=0x57;	//W  WIFI	
//年月日						
			wifisenddata[10]=25;
			wifisenddata[11]=8;
			wifisenddata[12]=6;
	//构建号				
			wifisenddata[13]=0x00;
			nCheckRes = 0;
			for(i = 0; i <13; i++)			
				nCheckRes ^= wifisenddata[i];
			wifisenddata[14]=nCheckRes;			
			wifisenddata[15]=0x99;
			if(HAL_UART_Transmit(&UART3_Handler,wifisenddata,16,30000)==0)
					return 1;
					else
					return 0;	
		}
		else
		if(pcmd[0]==0x66&&pcmd[FrmLen+5]==0x99&&pcmd[4]==0x42&&pcmd[5]==2)	//硬件版本
		{
			wifisenddata[0]=0x66;
			wifisenddata[1]=0x00;
			wifisenddata[2]=9>>8;//数据长度
			wifisenddata[3]=9;
			wifisenddata[4]=0x42;//命令号
			//版本类型
						wifisenddata[5]=2;
			for(i=0;i<8;i++)
			 wifisenddata[6+i]=datatemp[i];
			nCheckRes = 0;
			for(i = 0; i <13; i++)			
				nCheckRes ^= wifisenddata[i];
			wifisenddata[13]=nCheckRes;			
			wifisenddata[14]=0x99;
			if(HAL_UART_Transmit(&UART3_Handler,wifisenddata,16,30000)==0)
					return 1;
					else
					return 0;	
		}			
		else
		return 0;	
	}
	uint8_t ProcBatCmd(uint8_t *pcmd)
	{
	  uint8_t nCheckRes = 0;
	uint16_t i,k, FrmLen;
			FrmLen=pcmd[2]<<8|pcmd[3];
		if(pcmd[0]==0x66&&pcmd[FrmLen+5]==0x99&&pcmd[4]==0x44)	
		{
			wifisenddata[0]=0x66;
			wifisenddata[1]=0x00;
			wifisenddata[2]=6>>8;//数据长度
			wifisenddata[3]=6;
			wifisenddata[4]=0x44;//命令号		
			wifisenddata[5]=batpercent;//电池容量
			wifisenddata[6]=timeset>>8;//时间戳
			wifisenddata[7]=timeset;
			wifisenddata[8]=0xa5;
			wifisenddata[9]=0xa5;
			nCheckRes = 0;
			for(i = 0; i <10; i++)			
				nCheckRes ^= wifisenddata[i];
			wifisenddata[10]=nCheckRes;			
			wifisenddata[11]=0x99;
			if(HAL_UART_Transmit(&UART3_Handler,wifisenddata,12,30000)==0)
					return 1;
					else
					return 0;				
		}
		else
			return 0;
		}
uint8_t ProcSettimeCmd(uint8_t *pcmd)
{
	  uint8_t nCheckRes = 0;
	uint16_t i,k, FrmLen,mun;
			FrmLen=pcmd[2]<<8|pcmd[3];
	mun=pcmd[6]<<8|pcmd[7];//时间戳
	timeset=mun;
		if(pcmd[0]==0x66&&pcmd[FrmLen+5]==0x99&&pcmd[4]==0x41)	
		{
			wifisenddata[0]=0x66;
			wifisenddata[1]=0x00;
			wifisenddata[2]=6>>8;//数据长度
			wifisenddata[3]=6;
			wifisenddata[4]=0x41;//命令号	
			wifisenddata[5]=0xaa;//响应端
			wifisenddata[6]=mun>>8;//时间戳	
			wifisenddata[7]=mun;
			wifisenddata[8]=0x33;//成功
			wifisenddata[9]=0xa5;		
			nCheckRes = 0;
			for(i = 0; i <10; i++)			
				nCheckRes ^= wifisenddata[i];
			wifisenddata[10]=nCheckRes;			
			wifisenddata[11]=0x99;
			if(HAL_UART_Transmit(&UART3_Handler,wifisenddata,12,30000)==0)
					return 1;
					else
					return 0;				
			
		}
					else
			return 0;
}		
		
uint8_t ProchouduCmd(uint8_t *pcmd)	
{
	  uint8_t nCheckRes = 0;
	uint16_t i,k, FrmLen,mun;
			FrmLen=pcmd[2]<<8|pcmd[3];
		if(pcmd[0]==0x66&&pcmd[FrmLen+5]==0x99&&pcmd[4]==0x33)
		{
			switch(pcmd[5])	
			{
			
				case 0x11://标定
					break;
				case 0x55://开始发送
					sendthickindex=1;
			wifisenddata[0]=0x66;
			wifisenddata[1]=0x00;
			wifisenddata[2]=6>>8;//数据长度
			wifisenddata[3]=6;
			wifisenddata[4]=0x33;//命令号		
			wifisenddata[5]=0x55;		
			wifisenddata[6]=0x33;	
			wifisenddata[7]=timeset>>8;//时间戳	
			wifisenddata[8]=timeset;
			wifisenddata[9]=0xa5;	
			nCheckRes = 0;
			for(i = 0; i <10; i++)			
				nCheckRes ^= wifisenddata[i];
			wifisenddata[10]=nCheckRes;			
			wifisenddata[11]=0x99;
			if(HAL_UART_Transmit(&UART3_Handler,wifisenddata,12,30000)==0)
					return 1;
					else
					return 0;					
					break;
				case 0xaa://停止发送
					sendthickindex=0;
			wifisenddata[0]=0x66;
			wifisenddata[1]=0x00;
			wifisenddata[2]=6>>8;//数据长度
			wifisenddata[3]=6;
			wifisenddata[4]=0x33;//命令号		
			wifisenddata[5]=0xaa;		
			wifisenddata[6]=0x33;	
			wifisenddata[7]=timeset>>8;//时间戳	
			wifisenddata[8]=timeset;
			wifisenddata[9]=0xa5;	
			nCheckRes = 0;
			for(i = 0; i <10; i++)			
				nCheckRes ^= wifisenddata[i];
			wifisenddata[10]=nCheckRes;			
			wifisenddata[11]=0x99;
			if(HAL_UART_Transmit(&UART3_Handler,wifisenddata,12,30000)==0)
					return 1;
					else
					return 0;						
					break;	
				default:
					break;
			}
		
		
		}
		else
			return 0;		
}
void delay(void)
{
#ifndef USE_480x272
	unsigned int i,j;
	for(i = 0;i < 10;i++)
	{
		for(j = 0;j < 90000;j++);
	}
#endif
}
#ifdef  USE_FULLpage1_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/**
  * @}
  */ 
  
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
