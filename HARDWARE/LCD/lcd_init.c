#include "lcd_init.h"
#include "delay.h"
#include "gpio.h"
void LCD_GPIO_Init(void)
{
	  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /*Configure GPIO pin : PtPin */
  /*Configure GPIO pins : PAPin PAPin */
  GPIO_InitStruct.Pin = LCD_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(LCD_DC_GPIO_Port, &GPIO_InitStruct);
  /*Configure GPIO pins : PAPin PAPin */
  GPIO_InitStruct.Pin = LCD_SCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(LCD_SCK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = LCD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(LCD_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PCPin PCPin */
  GPIO_InitStruct.Pin = LCD_MOSI_Pin|LCD_RES_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}


/******************************************************************************
      函数说明：LCD串行数据写入函数
      入口数据：dat  要写入的串行数据
      返回值：  无
******************************************************************************/
void LCD_Writ_Bus(u8 dat) 
{	
	u8 i;
	LCD_CS_Clr();
//	LCD_CS_Clr();
	for(i=0;i<8;i++)
	{			  
		LCD_SCLK_Clr();
	//	LCD_SCLK_Clr();
		if(dat&0x80)
		{
		   LCD_MOSI_Set();
		//	LCD_MOSI_Set();
		}
		else
		{
		   LCD_MOSI_Clr();
			//LCD_MOSI_Clr();
		}
		LCD_SCLK_Set();
	//	LCD_SCLK_Set();
		dat<<=1;
	}	
  LCD_CS_Set();	
//	LCD_CS_Set();
}


/******************************************************************************
      函数说明：LCD写入数据
      入口数据：dat 写入的数据
      返回值：  无
******************************************************************************/
void LCD_WR_DATA8(u8 dat)
{
	LCD_Writ_Bus(dat);
}


/******************************************************************************
      函数说明：LCD写入数据
      入口数据：dat 写入的数据
      返回值：  无
******************************************************************************/
void LCD_WR_DATA(u16 dat)
{
	LCD_Writ_Bus(dat>>8);
	LCD_Writ_Bus(dat);
}


/******************************************************************************
      函数说明：LCD写入命令
      入口数据：dat 写入的命令
      返回值：  无
******************************************************************************/
void LCD_WR_REG(u8 dat)
{
	LCD_DC_Clr();//写命令
	LCD_Writ_Bus(dat);
	LCD_DC_Set();//写数据
}


/******************************************************************************
      函数说明：设置起始和结束地址
      入口数据：x1,x2 设置列的起始和结束地址
                y1,y2 设置行的起始和结束地址
      返回值：  无
******************************************************************************/
void LCD_Address_Set(u16 x1,u16 y1,u16 x2,u16 y2)
{
	if(USE_HORIZONTAL==0)
	{
		LCD_WR_REG(0x2a);//列地址设置
		LCD_WR_DATA(0x3c+x1);
		LCD_WR_DATA(0x3c+x2);
		LCD_WR_REG(0x2b);//行地址设置
		LCD_WR_DATA(y1);
		LCD_WR_DATA(y2);
		LCD_WR_REG(0x2c);//储存器写
	}
	else if(USE_HORIZONTAL==1)
	{
	
		LCD_WR_REG(0x2a);//列地址设置
		LCD_WR_DATA(0x3c+x1);
		LCD_WR_DATA(0x3c+x2);
		LCD_WR_REG(0x2b);//行地址设置
		LCD_WR_DATA(y1);
		LCD_WR_DATA(y2);
		LCD_WR_REG(0x2c);//储存器写
	}
	else if(USE_HORIZONTAL==2)
	{
		LCD_WR_REG(0x2a);//列地址设置
		LCD_WR_REG(0x2a);//列地址设置
		LCD_WR_DATA(x1);
		LCD_WR_DATA(x2);
		LCD_WR_REG(0x2b);//行地址设置
		LCD_WR_DATA(0x3c+y1);
		LCD_WR_DATA(0x3c+y2);
		LCD_WR_REG(0x2c);//储存器写
	}
	else
	{
		LCD_WR_REG(0x2a);//列地址设置
		LCD_WR_DATA(x1);
		LCD_WR_DATA(x2);
		LCD_WR_REG(0x2b);//行地址设置
		LCD_WR_DATA(0x3c+y1);
		LCD_WR_DATA(0x3c+y2);
		LCD_WR_REG(0x2c);//储存器写
	}
}
void LCD_Init(void)
{
		LCD_GPIO_Init();//初始化GPIO
	
	LCD_RES_Clr();//复位
	//delay_ms(100);
	//delay_us(200);   
	LCD_RES_Set();
	//delay_ms(100);
	//delay_us(200);
	//delay_us(200);
	

	
LCD_WR_REG(0x11);
delay_ms(120);
delay_us(200);
	
LCD_WR_REG(0xFE);			 
LCD_WR_REG(0xEF); 

LCD_WR_REG(0xEB);	
LCD_WR_DATA8(0x14); 

LCD_WR_REG(0x84);			
LCD_WR_DATA8(0x60); 

LCD_WR_REG(0x85);
LCD_WR_DATA8(0xF7);

LCD_WR_REG(0x86);            
LCD_WR_DATA8(0xFC); 

LCD_WR_REG(0x87);            
LCD_WR_DATA8(0x28);

LCD_WR_REG(0x8E);        
LCD_WR_DATA8(0x0F);

LCD_WR_REG(0x8F);     
LCD_WR_DATA8(0xFC); 

LCD_WR_REG(0x88);			
LCD_WR_DATA8(0x0A);

LCD_WR_REG(0x89);			
LCD_WR_DATA8(0x21); 

LCD_WR_REG(0x8A);			
LCD_WR_DATA8(0x00); 

LCD_WR_REG(0x8B);			
LCD_WR_DATA8(0x80); 

LCD_WR_REG(0x8C);			
LCD_WR_DATA8(0x01); 

LCD_WR_REG(0x8D);			
LCD_WR_DATA8(0x01); 

LCD_WR_REG(0xB6);			
LCD_WR_DATA8(0x00); 
LCD_WR_DATA8(0x20); 

LCD_WR_REG(0x36);			


	if(USE_HORIZONTAL==0)LCD_WR_DATA8(0x48);
	else if(USE_HORIZONTAL==1)LCD_WR_DATA8(0x18);
	else if(USE_HORIZONTAL==2)LCD_WR_DATA8(0x38);
	else LCD_WR_DATA8(0x28);  

LCD_WR_REG(0x3A);			
LCD_WR_DATA8(0x05); 

LCD_WR_REG(0x90);			
LCD_WR_DATA8(0x08);
LCD_WR_DATA8(0x08);
LCD_WR_DATA8(0x08);
LCD_WR_DATA8(0x08); 

LCD_WR_REG(0xBD);			
LCD_WR_DATA8(0x06);

LCD_WR_REG(0xBC);			
LCD_WR_DATA8(0x00);	

LCD_WR_REG(0xFF);			
LCD_WR_DATA8(0x60);
LCD_WR_DATA8(0x01);
LCD_WR_DATA8(0x04);

LCD_WR_REG(0xC3);			
LCD_WR_DATA8(0x25);//48
LCD_WR_REG(0xC4);			
LCD_WR_DATA8(0x25);

LCD_WR_REG(0xC9);			
LCD_WR_DATA8(0x25);

LCD_WR_REG(0xBE);			
LCD_WR_DATA8(0x11);

LCD_WR_REG(0xE1);
LCD_WR_DATA8(0x10);
LCD_WR_DATA8(0x0E);

LCD_WR_REG(0xDF);			
LCD_WR_DATA8(0x21);
LCD_WR_DATA8(0x10);
LCD_WR_DATA8(0x02);

LCD_WR_REG(0xf0);
LCD_WR_DATA8(0x4b);
LCD_WR_DATA8(0x0f);
LCD_WR_DATA8(0x0A);
LCD_WR_DATA8(0x0B);
LCD_WR_DATA8(0x15);
LCD_WR_DATA8(0x30);

LCD_WR_REG(0xf1);
LCD_WR_DATA8(0x43);
LCD_WR_DATA8(0x70);
LCD_WR_DATA8(0x72);
LCD_WR_DATA8(0x36);
LCD_WR_DATA8(0x37);
LCD_WR_DATA8(0x6f); 

LCD_WR_REG(0xf2);
LCD_WR_DATA8(0x4b);
LCD_WR_DATA8(0x0f);
LCD_WR_DATA8(0x0A);
LCD_WR_DATA8(0x0B);
LCD_WR_DATA8(0x15);
LCD_WR_DATA8(0x30);

LCD_WR_REG(0xf3);
LCD_WR_DATA8(0x43);
LCD_WR_DATA8(0x70);
LCD_WR_DATA8(0x72);
LCD_WR_DATA8(0x36);
LCD_WR_DATA8(0x37);
LCD_WR_DATA8(0x6f);

LCD_WR_REG(0xED);	
LCD_WR_DATA8(0x1B); 
LCD_WR_DATA8(0x0B); 

LCD_WR_REG(0xAC);			
LCD_WR_DATA8(0x47);

LCD_WR_REG(0xAE);			
LCD_WR_DATA8(0x77);

LCD_WR_REG(0xCD);			
LCD_WR_DATA8(0x63);		

LCD_WR_REG(0x70);			
LCD_WR_DATA8(0x07);
LCD_WR_DATA8(0x09);
LCD_WR_DATA8(0x04);
LCD_WR_DATA8(0x0C);
LCD_WR_DATA8(0x0D); 
LCD_WR_DATA8(0x09);
LCD_WR_DATA8(0x07);
LCD_WR_DATA8(0x08);
LCD_WR_DATA8(0x03);

LCD_WR_REG(0xE8);			
LCD_WR_DATA8(0x14);

/////////////////////////////////////////////
LCD_WR_REG(0x60);		
LCD_WR_DATA8(0x38);
LCD_WR_DATA8(0x0B);
LCD_WR_DATA8(0x76);
LCD_WR_DATA8(0x62);

LCD_WR_DATA8(0x39);
LCD_WR_DATA8(0xF0);
LCD_WR_DATA8(0x76);
LCD_WR_DATA8(0x62);

LCD_WR_REG(0x61);
LCD_WR_DATA8(0x38);
LCD_WR_DATA8(0xF6);
LCD_WR_DATA8(0x76);
LCD_WR_DATA8(0x62);

LCD_WR_DATA8(0x38);
LCD_WR_DATA8(0xF7);
LCD_WR_DATA8(0x76);
LCD_WR_DATA8(0x62);
/////////////////////////////////////
LCD_WR_REG(0x62);
LCD_WR_DATA8(0x38);
LCD_WR_DATA8(0x0D);
LCD_WR_DATA8(0x71);
LCD_WR_DATA8(0xED);
LCD_WR_DATA8(0x76);
LCD_WR_DATA8(0x62);
LCD_WR_DATA8(0x38);
LCD_WR_DATA8(0x0F);
LCD_WR_DATA8(0x71);
LCD_WR_DATA8(0xEF);
LCD_WR_DATA8(0x76); 
LCD_WR_DATA8(0x62);

LCD_WR_REG(0x63);			
LCD_WR_DATA8(0x38);
LCD_WR_DATA8(0x11);
LCD_WR_DATA8(0x71);
LCD_WR_DATA8(0xF1);
LCD_WR_DATA8(0x76);
LCD_WR_DATA8(0x62);
LCD_WR_DATA8(0x38);
LCD_WR_DATA8(0x13);
LCD_WR_DATA8(0x71);
LCD_WR_DATA8(0xF3);
LCD_WR_DATA8(0x76); 
LCD_WR_DATA8(0x62);

///////////////////////////////////////////////////////
LCD_WR_REG(0x64);			
LCD_WR_DATA8(0x3b);
LCD_WR_DATA8(0x29);
LCD_WR_DATA8(0xF1);
LCD_WR_DATA8(0x01);
LCD_WR_DATA8(0xF1);
LCD_WR_DATA8(0x00);
LCD_WR_DATA8(0x0a);

LCD_WR_REG(0x66);			
LCD_WR_DATA8(0x3C);
LCD_WR_DATA8(0x00);
LCD_WR_DATA8(0xCD);
LCD_WR_DATA8(0x67);
LCD_WR_DATA8(0x45);
LCD_WR_DATA8(0x45);
LCD_WR_DATA8(0x10);
LCD_WR_DATA8(0x00);
LCD_WR_DATA8(0x00);
LCD_WR_DATA8(0x00);

LCD_WR_REG(0x67);			
LCD_WR_DATA8(0x00);
LCD_WR_DATA8(0x3C);
LCD_WR_DATA8(0x00);
LCD_WR_DATA8(0x00);
LCD_WR_DATA8(0x00);
LCD_WR_DATA8(0x01);
LCD_WR_DATA8(0x54);
LCD_WR_DATA8(0x10);
LCD_WR_DATA8(0x32);
LCD_WR_DATA8(0x98);

LCD_WR_REG(0xB5);			
LCD_WR_DATA8(0x08);
LCD_WR_DATA8(0x09);
LCD_WR_DATA8(0x14);
LCD_WR_DATA8(0x08);
					
LCD_WR_REG(0x74);			
LCD_WR_DATA8(0x10);	
LCD_WR_DATA8(0x85);	
LCD_WR_DATA8(0x80);
LCD_WR_DATA8(0x00); 
LCD_WR_DATA8(0x00); 
LCD_WR_DATA8(0x4E);
LCD_WR_DATA8(0x00);					

LCD_WR_REG(0x98);			
LCD_WR_DATA8(0x3e);
LCD_WR_DATA8(0x07);

LCD_WR_REG(0x35);	
LCD_WR_DATA8(0x00); 

LCD_WR_REG(0x21);

LCD_WR_REG(0x11);
delay_ms(120);
delay_us(200);
LCD_WR_REG(0x29);
delay_ms(100);
delay_us(200);



	LCD_WR_REG(0x2A);       
	LCD_WR_DATA8(0x00);
	LCD_WR_DATA8(0x3C);
	LCD_WR_DATA8(0x00);
	LCD_WR_DATA8(0xb3);

	LCD_WR_REG(0x2B);       
	LCD_WR_DATA8(0x00);
	LCD_WR_DATA8(0x00);  
	LCD_WR_DATA8(0x00);
	LCD_WR_DATA8(0xEF); 

	LCD_WR_REG(0x2C);
}









