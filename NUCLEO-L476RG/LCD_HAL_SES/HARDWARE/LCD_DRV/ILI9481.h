//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//测试硬件：单片机STM32F429IGT6,正点原子Apollo STM32F4/F7开发板,主频180MHZ，晶振12MHZ
//QDtech-TFT液晶驱动 for STM32 IO模拟
//xiao冯@ShenZhen QDtech co.,LTD
//公司网站:www.qdtft.com
//淘宝网站：http://qdtech.taobao.com
//wiki技术网站：http://www.lcdwiki.com
//我司提供技术支持，任何技术问题欢迎随时交流学习
//固话(传真) :+86 0755-23594567 
//手机:15989313508（冯工） 
//邮箱:lcdwiki01@gmail.com    support@lcdwiki.com    goodtft@163.com 
//技术支持QQ:3002773612  3002778157
//技术交流QQ群:324828016
//创建日期:2018/08/09
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 深圳市全动电子技术有限公司 2018-2028
//All rights reserved
/****************************************************************************************************
//=========================================电源接线================================================//
//     LCD模块                STM32单片机
//      VCC          接        DC5V/3.3V      //电源
//      GND          接          GND          //电源地
//=======================================液晶屏数据线接线==========================================//
//本模块默认数据总线类型为SPI总线
//     LCD模块                STM32单片机    
//    SDI(MOSI)      接          PF9          //液晶屏SPI总线数据写信号
//    SDO(MISO)      接          PF8          //液晶屏SPI总线数据读信号，如果不需要读，可以不接线
//=======================================液晶屏控制线接线==========================================//
//     LCD模块 					      STM32单片机 
//       LED         接          PD6          //液晶屏背光控制信号，如果不需要控制，接5V或3.3V
//       SCK         接          PF7          //液晶屏SPI总线时钟信号
//      DC/RS        接          PD5          //液晶屏数据/命令控制信号
//       RST         接          PD12         //液晶屏复位控制信号
//       CS          接          PD11         //液晶屏片选控制信号
//=========================================触摸屏触接线=========================================//
//如果模块不带触摸功能或者带有触摸功能，但是不需要触摸功能，则不需要进行触摸屏接线
//	   LCD模块                STM32单片机 
//      T_IRQ        接          PH11         //触摸屏触摸中断信号
//      T_DO         接          PG3          //触摸屏SPI总线读信号
//      T_DIN        接          PI3          //触摸屏SPI总线写信号
//      T_CS         接          PI8          //触摸屏片选控制信号
//      T_CLK        接          PH6          //触摸屏SPI总线时钟信号
**************************************************************************************************/			
 /* @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, QD electronic SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
**************************************************************************************************/	
#ifndef __ILI9481_H
#define __ILI9481_H

		
//#include "sys.h"	 
#include "stdlib.h"
#include "Global.h"

#define	delay_ms(x)		HAL_Delay(x)

//LCD重要参数集
typedef struct  
{										    
	U16 width;			//LCD 宽度
	U16 height;			//LCD 高度
	U16 id;				//LCD ID
	U8  dir;			//横屏还是竖屏控制：0，竖屏；1，横屏。	
	U16	 wramcmd;		//开始写gram指令
	U16  setxcmd;		//设置x坐标指令
	U16  setycmd;		//设置y坐标指令	 
}_lcd_dev; 	

//LCD参数
extern _lcd_dev lcddev;	//管理LCD重要参数
/////////////////////////////////////用户配置区///////////////////////////////////	 
#define DIR_HORIZONTAL  	 	0	//定义液晶屏顺时针旋转方向 	0-0度旋转，1-90度旋转，2-180度旋转，3-270度旋转
#define DIR_HORIZONTAL_2  	1
#define DIR_VERTICAL  	 		2
#define DIR_VERTICAL_2  	 	3

//////////////////////////////////////////////////////////////////////////////////	  
//定义LCD的尺寸
//#define LCD_W 								320
//#define LCD_H 								480
//#define BYTES_PER_PIXEL				3			//RGB - 18bit; 6bits per colour
//#if (LCD_W > LCD_H)
//	#define LCD_LINEBUFFER_MAX	(LCD_W * BYTES_PER_PIXEL)
//#else
//	#define LCD_LINEBUFFER_MAX	(LCD_H * BYTES_PER_PIXEL)
//#endif

//TFTLCD部分外要调用的函数		   
extern U16  POINT_COLOR;//默认红色    
extern U16  BACK_COLOR; //背景颜色.默认为白色

////////////////////////////////////////////////////////////////////
//-----------------LCD端口定义---------------- 

#define ILI9328_ID    0x9328

#define LED   9        //背光控制引脚
#define CS    11       //片选引脚
#define RS    10       //寄存器/数据选择引脚  
#define RST   12       //复位引脚

#define LCD_LED_PIN                				GPIO_PIN_9
#define LCD_CS_PIN                  			GPIO_PIN_11
#define LCD_RS_PIN                  			GPIO_PIN_10
#define LCD_RST_PIN                  			GPIO_PIN_12

#define GPIO_PORTB		                     GPIOB
#define LCD_GPIO_CLK_ENABLE()              __HAL_RCC_GPIOB_CLK_ENABLE()

//QDtech全系列模块采用了三极管控制背光亮灭，用户也可以接PWM调节背光亮度
//#define	LCD_LED PBout(LED) //LCD背光    		 
//#define LCD_CS  PBout(CS) 
//#define LCD_RS  PBout(RS) 
//#define LCD_RST PBout(RST)

//如果使用官方库函数定义下列底层，速度将会下降到14帧每秒，建议采用我司推荐方法
//以下IO定义直接操作寄存器，快速IO操作，刷屏速率可以达到28帧每秒！ 

//#define	LCD_CS_SET  LCD_CS=1//GPIO_TYPE->BSRR=1<<LCD_CS    //片选端口  	PB11
//#define	LCD_RS_SET	LCD_RS=1//GPIO_TYPE->BSRR=1<<LCD_RS    //数据/命令  PB10	  
//#define	LCD_RST_SET	LCD_RST=1//GPIO_TYPE->BSRR=1<<LCD_RST    //复位			PB12

#define	GPIOB_SET_PIN(PIN)		HAL_GPIO_WritePin(GPIO_PORTB, PIN, GPIO_PIN_SET)
#define	GPIOB_CLR_PIN(PIN)		HAL_GPIO_WritePin(GPIO_PORTB, PIN, GPIO_PIN_RESET)

#define	LCD_LED_SET  	GPIOB_SET_PIN(LCD_LED_PIN)
#define	LCD_CS_SET  	GPIOB_SET_PIN(LCD_CS_PIN)
#define	LCD_RS_SET		GPIOB_SET_PIN(LCD_RS_PIN)	  
#define	LCD_RST_SET		GPIOB_SET_PIN(LCD_RST_PIN)
 							    
//#define	LCD_CS_CLR  LCD_CS=0//GPIO_TYPE->BRR=1<<LCD_CS     //片选端口  	PB11
//#define	LCD_RS_CLR	LCD_RS=0//GPIO_TYPE->BRR=1<<LCD_RS     //数据/命令  PB10	 
//#define	LCD_RST_CLR	LCD_RST=0//GPIO_TYPE->BRR=1<<LCD_RST    //复位			PB12

#define	LCD_LED_CLR  	GPIOB_CLR_PIN(LCD_LED_PIN)
#define	LCD_CS_CLR  	GPIOB_CLR_PIN(LCD_CS_PIN)
#define	LCD_RS_CLR		GPIOB_CLR_PIN(LCD_RS_PIN)	  
#define	LCD_RST_CLR		GPIOB_CLR_PIN(LCD_RST_PIN)			


//RGB565 colours
#define WHITE       0xFFFF
#define BLACK      	0x0000	  
#define BLUE       	0x001F  
#define BRED        0XF81F
#define GRED 			 	0XFFE0
#define GBLUE			 	0X07FF
#define RED         0xF800
#define MAGENTA     0xF81F
#define INDIGO			0x4810
#define VIOLET			0XEC1D
#define GREEN       0x07E0
#define CYAN        0x7FFF
#define YELLOW      0xFFE0
#define ORANGE			0xFC60
#define BROWN 			0XBC40 //棕色
#define BRRED 			0XFC07 //棕红色
#define GRAY  			0X8430 //灰色
//GUI颜色

#define DARKBLUE      	 0X01CF	//深蓝色
#define LIGHTBLUE      	 0X7D7C	//浅蓝色  
#define GRAYBLUE       	 0X5458 //灰蓝色
//以上三色为PANEL的颜色 
 
#define LIGHTGREEN     	0X841F //浅绿色
#define LIGHTGRAY     0XEF5B //浅灰色(PANNEL)
#define LGRAY 			 		0XC618 //浅灰色(PANNEL),窗体背景色

#define LGRAYBLUE      	0XA651 //浅灰蓝色(中间层颜色)
#define LBBLUE          0X2B12 //浅棕蓝色(选择条目的反色)
	    	
//RGB565 -> RGB888 colour mapping
#define RGB565_RGB888_RED(x)		( (U8) ((x&0xF800)>>8))
#define RGB565_RGB888_GREEN(x)	( (U8) ((x&0x07E0)>>3))
#define RGB565_RGB888_BLUE(x)		( (U8) ((x&0x001F)<<3))

void LCD_Init(void);
U16 LCD_GetDeviceID(void);
void LCD_WR_REG(U8 data);
void LCD_WR_DATA(U8 data);
void LCD_WR_MULTIBYTE_DATA(U8 *data, U16 nbytes);
void LCD_RD_DATA(U8 *data);
void LCD_RD_MULTIBYTE_DATA(U8 *data, U16 nbytes);	


void LCD_WriteReg(U8 LCD_Reg, U8 LCD_RegValue);
U16 LCD_ReadReg(U8 LCD_Reg);
void LCD_WriteRAM_Prepare(void);
void LCD_WriteRAM(U16 RGB_Code);
U16 LCD_ReadRAM(void);		   
U16 LCD_BGR2RGB(U16 c);
void LCD_SetParam(void);
void Lcd_WriteData_16Bit(U16 Data);
void LCD_Write_PixelData(U8 *data, U16 npixels);
void LCD_direction(U8 direction );

//如果仍然觉得速度不够快，可以使用下面的宏定义,提高速度.
//注意要去掉lcd.c中void LCD_WR_DATA(U16 data)函数定义哦
/*
#if LCD_USE8BIT_MODEL==1//使用8位并行数据总线模式
	#define LCD_WR_DATA(data){\
	LCD_RS_SET;\
	LCD_CS_CLR;\
	DATAOUT(data);\
	LCD_WR_CLR;\
	LCD_WR_SET;\
	DATAOUT(data<<8);\
	LCD_WR_CLR;\
	LCD_WR_SET;\
	LCD_CS_SET;\
	}
	#else//使用16位并行数据总线模式
	#define LCD_WR_DATA(data){\
	LCD_RS_SET;\
	LCD_CS_CLR;\
	DATAOUT(data);\
	LCD_WR_CLR;\
	LCD_WR_SET;\
	LCD_CS_SET;\
	} 	
#endif
*/
				  		 
#endif  
	 
	 



