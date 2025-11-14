#include "stm32f7xx_eval.h"
#include "stm32f7xx_hal.h"
#include "Tiky_LCD.h" 
#define LCD_SPI_CS(a)	\
						if (a)	\
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);\
						else		\
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
#define SPI_DCLK(a)	\
						if (a)	\
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);\
						else		\
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
#define SPI_SDA(a)	\
						if (a)	\
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);\
						else		\
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

//800*480
#define  HDP 479  //Hsync Display period   
#define  VDP 799 //Vertical Display period

#define  HT  580	  //Horizontal total period = (HT + 1) pixels
#define  HPS 20	  //Horizontal Sync Pulse Start Position = (HPS + 1) pixels	 		 
#define  LPS 1	  //Horizontal Display Period Start Position = LPS pixels		
#define  HPW 12   //Horizontal Sync Pulse Width = (HPW + 1) pixels	

#define  VT 850	 //Vertical Total = (VT + 1) lines
#define  VPS 25  //Vertical Sync Pulse Start Position = VPS lines					  
#define  FPS 1   //Vertical Display Period Start Position = FPS lines				  
#define  VPW 12  //Vertical Sync Pulse Width = (VPW + 1) lines
						
////240*320
//#define  HDP 239  //Hsync Display period 
//#define  HT  250   //Horizontal total period = (HT + 1) pixels
//#define  HPS 6   //Horizontal Sync Pulse Start Position = (HPS + 1) pixels	 		 
//#define  LPS 2	  //Horizontal Display Period Start Position = LPS pixels		
//#define  HPW 2   //Horizontal Sync Pulse Width = (HPW + 1) pixels	

//#define  VDP 319 //Vertical Display period						
//#define  VT 330	 //Vertical Total = (VT + 1) lines
//#define  VPS 6  //Vertical Sync Pulse Start Position = VPS lines					  
//#define  FPS 2   //Vertical Display Period Start Position = FPS lines				  
//#define  VPW 2  //Vertical Sync Pulse Width = (VPW + 1) lines
/*
 * 函数名：LCD_GPIO_Config
 * 描述  ：根据配置LCD的I/O
 * 输入  ：无
 * 输出  ：无
 * 调用  ：内部调用        
 */
static void LCD_GPIO_Config(void)
{
	  GPIO_InitTypeDef   GPIO_InitStructure;
	GPIO_InitTypeDef  gpio_init_structure;
   __HAL_RCC_GPIOB_CLK_ENABLE();//GPIO-B0
	gpio_init_structure.Pin = GPIO_PIN_0;
	gpio_init_structure.Mode = GPIO_MODE_OUTPUT_PP;
	gpio_init_structure.Pull = GPIO_PULLUP;
	gpio_init_structure.Speed = GPIO_SPEED_HIGH;

	HAL_GPIO_Init(GPIOB, &gpio_init_structure);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
	///////////////////////////////////////////////////////////
		__HAL_RCC_GPIOB_CLK_ENABLE(); //GPIO-B1
	gpio_init_structure.Pin = GPIO_PIN_1;
	gpio_init_structure.Mode = GPIO_MODE_OUTPUT_PP;
	gpio_init_structure.Pull = GPIO_PULLUP;
	gpio_init_structure.Speed = GPIO_SPEED_HIGH;

	HAL_GPIO_Init(GPIOB, &gpio_init_structure);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);	
	///////////////////////////////////////////////////////////
		__HAL_RCC_GPIOA_CLK_ENABLE(); //GPIO-A3
	gpio_init_structure.Pin = GPIO_PIN_3;
	gpio_init_structure.Mode = GPIO_MODE_OUTPUT_PP;
	gpio_init_structure.Pull = GPIO_PULLUP;
	gpio_init_structure.Speed = GPIO_SPEED_HIGH;

	HAL_GPIO_Init(GPIOA, &gpio_init_structure);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);	
	///////////////////////////////////////////////////////////
		__HAL_RCC_GPIOB_CLK_ENABLE(); //GPIO-b10
	gpio_init_structure.Pin = GPIO_PIN_10;
	gpio_init_structure.Mode = GPIO_MODE_OUTPUT_PP;
	gpio_init_structure.Pull = GPIO_PULLUP;
	gpio_init_structure.Speed = GPIO_SPEED_HIGH;

	HAL_GPIO_Init(GPIOB, &gpio_init_structure);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);	

}
////////////////////////////////////////////////
volatile void LCD_delay ( uint16_t j)
{
volatile uint16_t i;	
	while(j--)
for(i=7200;i>0;i--);
}
static void LCD_Rst(void)
{			
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);	
    LCD_delay(100);					   
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);		 	 
    LCD_delay(100);	
}
 void WriteComm(uint16_t CMD)
{			
	*(__IO uint16_t *) (Bank1_LCD_C) = CMD;
}
 void WriteData(uint16_t tem_data)
{			
	*(__IO uint16_t *) (Bank1_LCD_D) = tem_data;
}

void LCD_WriteByteSPI(unsigned char byte)
{
		unsigned char n;
   
   for(n=0; n<8; n++)			
   {  
	  if(byte&0x80) SPI_SDA(1)
      	else SPI_SDA(0)
      byte<<= 1;
		 
	  SPI_DCLK(0);
    SPI_DCLK(1);
   }
}

void SPI_WriteComm(uint16_t CMD)
{			
	LCD_SPI_CS(0);
//	SPI_SDA(0);
//	SPI_DCLK(0);
//	SPI_DCLK(1);
	LCD_WriteByteSPI(0x70);
	LCD_WriteByteSPI(CMD);
	
	LCD_SPI_CS(1);
}
void SPI_WriteData(uint16_t tem_data)
{			
	LCD_SPI_CS(0);
//	SPI_SDA(1);
//	SPI_DCLK(0);
//	SPI_DCLK(1);
	LCD_WriteByteSPI(0x72);
	LCD_WriteByteSPI(tem_data);
	LCD_SPI_CS(1);
}
void Lcd_Initialize(void)
{	
  LCD_GPIO_Config();
	LCD_SPI_CS(0);
  LCD_Rst();

		SPI_WriteComm(0x20);//exit_invert_mode
    SPI_WriteComm(0x29);//set_display_on
	  SPI_WriteComm(0x36);//Flip Horizontal
	  SPI_WriteData(0x01);//FH--FV
    SPI_WriteComm(0x3A);//set_pixel_format 
   SPI_WriteData(0x77);//70   0X60 26k
 
    SPI_WriteComm(0xB1);//RGB Interface Setting
   SPI_WriteData(0x00);
   SPI_WriteData(0x14);
   SPI_WriteData(0x06);
 
    SPI_WriteComm(0xB2);//Panel Characteristics Setting
	 //SPI_WriteData(0xC8);//800 pixels
   SPI_WriteData(0x10);//480 pixels
   SPI_WriteData(0xC8);//800 pixels
  
    SPI_WriteComm(0xB3);//Panel Drive Setting    Set the inversion mode
   
 
   SPI_WriteData(0x00);//1-dot inversion 0x01
 
    SPI_WriteComm(0xB4);//Display Mode Control
   SPI_WriteData(0x04);//Dither disable.
 
    SPI_WriteComm(0xB5);//Display Mode and Frame Memory Write Mode Setting
   SPI_WriteData(0x10);
   SPI_WriteData(0x30);
   SPI_WriteData(0x30);
   SPI_WriteData(0x00);
   SPI_WriteData(0x00);
 
    SPI_WriteComm(0xB6);//Display Control 2 ( GIP Specific )
   SPI_WriteData(0x01);
   SPI_WriteData(0x18);
   SPI_WriteData(0x02);
   SPI_WriteData(0x40);
   SPI_WriteData(0x10);
   SPI_WriteData(0x00);
 
 SPI_WriteComm(0xc0);
   SPI_WriteData(0x01);
   SPI_WriteData(0x18);
 
 
   SPI_WriteComm(0xC3); 
   SPI_WriteData(0x03);
   SPI_WriteData(0x04);
   SPI_WriteData(0x03);
   SPI_WriteData(0x03);
   SPI_WriteData(0x03);
   
 LCD_delay(10);
 
    SPI_WriteComm(0xC4);//VDD Regulator setting
   SPI_WriteData(0x02);
   SPI_WriteData(0x23);//GDC AP
   SPI_WriteData(0x11);//VRH1  Vreg1out=1.533xVCI(10)
   SPI_WriteData(0x12);//VRH2  Vreg2out=-1.533xVCI(10)
   SPI_WriteData(0x02);//BT 5 VGH/VGL  6/-4
   SPI_WriteData(0x77);//DDVDH 6C//0x56
 LCD_delay(10);
 
    SPI_WriteComm(0xC5);
   SPI_WriteData(0x73);
 LCD_delay(10);
 
    SPI_WriteComm(0xC6);
   SPI_WriteData(0x24);//VCI 23
   SPI_WriteData(0x60);//RESET RCO 53
   SPI_WriteData(0x00);//SBC GBC
 LCD_delay(10);
    //GAMMA SETTING
    SPI_WriteComm(0xD0);
   SPI_WriteData(0x14);
   SPI_WriteData(0x01);
   SPI_WriteData(0x53);
   SPI_WriteData(0x25);
   SPI_WriteData(0x02);
   SPI_WriteData(0x02);
   SPI_WriteData(0x66);
   SPI_WriteData(0x14);
   SPI_WriteData(0x03);
 
    SPI_WriteComm(0xD1);
   SPI_WriteData(0x14);
   SPI_WriteData(0x01);
   SPI_WriteData(0x53);
   SPI_WriteData(0x07);
   SPI_WriteData(0x02);
   SPI_WriteData(0x02);
   SPI_WriteData(0x66);
   SPI_WriteData(0x14);
   SPI_WriteData(0x03);
 
 
 
 SPI_WriteComm(0xD2);
   SPI_WriteData(0x14);
   SPI_WriteData(0x01);
   SPI_WriteData(0x53);
   SPI_WriteData(0x25);
   SPI_WriteData(0x02);
   SPI_WriteData(0x02);
   SPI_WriteData(0x66);
   SPI_WriteData(0x14);
   SPI_WriteData(0x03);
 
    SPI_WriteComm(0xD3);
   SPI_WriteData(0x14);
   SPI_WriteData(0x01);
   SPI_WriteData(0x53);
   SPI_WriteData(0x07);
   SPI_WriteData(0x02);
   SPI_WriteData(0x02);
   SPI_WriteData(0x66);
   SPI_WriteData(0x14);
   SPI_WriteData(0x03);
 
 
   SPI_WriteComm(0xD4);
   SPI_WriteData(0x14);
   SPI_WriteData(0x01);
   SPI_WriteData(0x53);
   SPI_WriteData(0x25);
   SPI_WriteData(0x02);
   SPI_WriteData(0x02);
   SPI_WriteData(0x66);
   SPI_WriteData(0x14);
   SPI_WriteData(0x03);
 
   SPI_WriteComm(0xD5);
   SPI_WriteData(0x14);
   SPI_WriteData(0x01);
   SPI_WriteData(0x53);
   SPI_WriteData(0x07);
   SPI_WriteData(0x02);
   SPI_WriteData(0x02);
   SPI_WriteData(0x66);
   SPI_WriteData(0x14);
   SPI_WriteData(0x03);
	//	Lcd_Light_ON;
		SPI_WriteComm(0x11);

		LCD_delay(10);
	//	Lcd_Light_ON;

		SPI_WriteComm(0x3A); 
		SPI_WriteData(0x55);
		SPI_WriteComm(0x36);
		SPI_WriteData(0x00);
	}


