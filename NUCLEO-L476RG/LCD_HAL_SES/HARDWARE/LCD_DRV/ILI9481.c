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
//#include "stm32l4xx.h"
//#include "delay.h"
#include "ILI9481.h"
#include "SPI.h"
#include "stdlib.h"
#include "string.h"
	   
//管理LCD重要参数
//默认为竖屏
_lcd_dev lcddev;

//画笔颜色,背景颜色
U16 POINT_COLOR = 0x0000,BACK_COLOR = 0xFFFF;  
U16 DeviceCode;	 
//U8 LineBuffer[LCD_LINEBUFFER_MAX];

/*****************************************************************************
 * @name       :void LCD_WR_REG(U8 data)
 * @date       :2018-08-09 
 * @function   :Write an 8-bit command to the LCD screen
 * @parameters :data:Command value to be written
 * @retvalue   :None
******************************************************************************/
void LCD_WR_REG(U8 data)
{ 
    LCD_CS_CLR;     
    LCD_RS_CLR;	  
    SPI_WriteByte(data);
    LCD_CS_SET;	
}

/*****************************************************************************
 * @name       :void LCD_WR_DATA(U8 data)
 * @date       :2018-08-09 
 * @function   :Write an 8-bit data to the LCD screen
 * @parameters :data:data value to be written
 * @retvalue   :None
******************************************************************************/
void LCD_WR_DATA(U8 data)
{
    LCD_CS_CLR;
    LCD_RS_SET;
    SPI_WriteByte(data);
    LCD_CS_SET;
}

/*****************************************************************************
 * @name       :void LCD_WriteReg(U8 LCD_Reg, U16 LCD_RegValue)
 * @date       :2018-08-09 
 * @function   :Write data into registers
 * @parameters :LCD_Reg:Register address
                LCD_RegValue:Data to be written
 * @retvalue   :None
******************************************************************************/
//void LCD_WriteReg(U8 LCD_Reg, U8 LCD_RegValue)
//{	
//	LCD_WR_REG(LCD_Reg);  
//	LCD_WR_DATA(LCD_RegValue);	    		 
//}	

void LCD_WR_MULTIBYTE_DATA(U8 *data, U16 nbytes)
{
    LCD_CS_CLR;
    LCD_RS_SET;
    SPI_WriteMultiByte(data,nbytes);
    LCD_CS_SET;
}

void LCD_RD_DATA(U8 *data)
{
    LCD_CS_CLR;
    LCD_RS_SET;
    SPI_ReadMultiByte(data,1);
    LCD_CS_SET;
}

void LCD_RD_MULTIBYTE_DATA(U8 *data, U16 nbytes)
{
    LCD_CS_CLR;
    LCD_RS_SET;
    SPI_ReadMultiByte(data,nbytes);
    LCD_CS_SET;
}

void LCD_RD_REG_DATA(U8 *RegCmd, U8 *RegValue, U16 nbytes)
{	
    LCD_CS_CLR;     
    LCD_RS_CLR;
	SPI_TransmitReceiveMultiByte(RegCmd,RegValue,nbytes);
    LCD_RS_SET;
    LCD_CS_SET;    		 
}	

//void LCD_Write_PixelData(U8 *data, U16 npixels)
//{
//    LCD_CS_CLR;
//    LCD_RS_SET;
//    SPI_WriteMultiByte(data,3*npixels);
//    LCD_CS_SET;
//}

   

/*****************************************************************************
 * @name       :void LCD_WriteRAM_Prepare(void)
 * @date       :2018-08-09 
 * @function   :Write GRAM
 * @parameters :None
 * @retvalue   :None
******************************************************************************/	 
//void LCD_WriteRAM_Prepare(void)
//{
//	LCD_WR_REG(lcddev.wramcmd);
//}	 

/*****************************************************************************
 * @name       :void Lcd_WriteData_16Bit(U16 Data)
 * @date       :2018-08-09 
 * @function   :Write an 16-bit command to the LCD screen
 * @parameters :Data:Data to be written
 * @retvalue   :None
******************************************************************************/	 
//void Lcd_WriteData_16Bit(U16 Data)
//{	
//  //16bit RGB565 -> 24Bit	BGR 888 format
//	LCD_WR_DATA((Data>>8)&0xF8);//RED
//	LCD_WR_DATA((Data>>3)&0xFC);//GREEN
//	LCD_WR_DATA(Data<<3);//BLUE
//}

//void Lcd_WriteData_16Bit(U16 Data)
//{	
//  //16bit RGB565 -> 24Bit	BGR 888 format
//	U8 Pixel[3];
//	Pixel[0] = (U8) ((Data&0xF800)>>11)<<2;			//RED
//	Pixel[1] = (U8) ((Data&0x07E0)>>5)<<2;			//GREEN
//	Pixel[2] = (U8) ((Data&0x001F)>>0)<<2;			//BLUE
////	Pixel[0] = 0xFF;
////	Pixel[1] = 0xFF;
////	Pixel[2] = 0xFF;
//	LCD_Write_PixelData(Pixel,1);
//}



/*****************************************************************************
 * @name       :void LCD_Clear(U16 Color)
 * @date       :2018-08-09 
 * @function   :Initialization LCD screen GPIO
 * @parameters :None
 * @retvalue   :None
******************************************************************************/	
void LCD_GPIOInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
    
    __HAL_RCC_GPIOB_CLK_ENABLE();           //使能GPIOF时钟
    
    //LCD_LED,LCD_CS,LCD_D/C,LCD_RST
    GPIO_InitStructure.Pin=GPIO_PIN_9| GPIO_PIN_10|GPIO_PIN_11| GPIO_PIN_12;
    GPIO_InitStructure.Mode=GPIO_MODE_OUTPUT_PP;  //推挽输出
    GPIO_InitStructure.Pull=GPIO_PULLUP;          //上拉
    GPIO_InitStructure.Speed=GPIO_SPEED_FREQ_HIGH;     //快速         
    HAL_GPIO_Init(GPIOB,&GPIO_InitStructure);     //初始化


}

/*****************************************************************************
 * @name       :void LCD_RESET(void)
 * @date       :2018-08-09 
 * @function   :Reset LCD screen
 * @parameters :None
 * @retvalue   :None
******************************************************************************/	
void LCD_RESET(void)
{
	LCD_RST_CLR;
	delay_ms(100);	
	LCD_RST_SET;
	delay_ms(50);
}

U16 LCD_GetDeviceID(void)
{  

    //typedef struct
    //{
    //    U8 Entry_Code;
    //    U16 MIPI_Code;
    //    U16 Device_ID;
    //    U8 Exit_Code;
        
    //} stData;

    U8 Data[6] = {0x00};
    U8 Cmd = {0xBF};

    LCD_RD_REG_DATA(&Cmd, Data, 6);

    //LCD_WR_REG(0xBF);
    //LCD_RD_MULTIBYTE_DATA(Data,6);

    //LCD_RD_DATA(&Data[0]);
    //LCD_RD_DATA(&Data[1]);
    //LCD_RD_DATA(&Data[2]);
    //LCD_RD_DATA(&Data[3]);
    //LCD_RD_DATA(&Data[4]);
    //LCD_RD_DATA(&Data[5]);
    //LCD_RD_REG_DATA(0xBF, Data, 6);
    return(((U16) Data[3])<<8 | ((U16) Data[4]));
}

/*****************************************************************************
 * @name       :void LCD_Init(void)
 * @date       :2018-08-09 
 * @function   :Initialization LCD screen
 * @parameters :None
 * @retvalue   :None
******************************************************************************/	 	 
void LCD_Init(void)
{  
	U16 Device_ID;

    SPI2_Init();		   			        //初始化SPI
    //	SPI_SetSpeed(SPI_BAUDRATEPRESCALER_4); //设置为45M时钟,高速模式	//	original: <commented>
	LCD_GPIOInit();//LCD GPIO初始化										 
 	LCD_RESET(); //LCD 复位
	
    Device_ID = LCD_GetDeviceID();

    //3.5IPS ILI9481+CMI	
	LCD_WR_REG(0x01); //soft_reset
	delay_ms(220);

	LCD_WR_REG(0x11); //exit_sleep_mode
	delay_ms(280);

	LCD_WR_REG(0xd0); //Power Setting
	LCD_WR_DATA(0x07);//07  VC[2:0] Sets the ratio factor of Vci to generate the reference voltages Vci1
	LCD_WR_DATA(0x44);//41  BT[2:0] Sets the Step up factor and output voltage level from the reference voltages Vci1
	LCD_WR_DATA(0x1E);//1f  17   1C  VRH[3:0]: Sets the factor to generate VREG1OUT from VCILVL
	delay_ms(220);

	LCD_WR_REG(0xd1); //VCOM Control
	LCD_WR_DATA(0x00);//00
	LCD_WR_DATA(0x0C);//1A   VCM [6:0] is used to set factor to generate VCOMH voltage from the reference voltage VREG1OUT  15    09
	LCD_WR_DATA(0x1A);//1F   VDV[4:0] is used to set the VCOM alternating amplitude in the range of VREG1OUT x 0.70 to VREG1OUT   1F   18

	LCD_WR_REG(0xC5);  //Frame rate and Inversion Control
	LCD_WR_DATA(0x03); //original: 03 - 72Hz

	LCD_WR_REG(0xd2);  //Power Setting for Normal Mode
	LCD_WR_DATA(0x01);  //01
	LCD_WR_DATA(0x11);  //11

	LCD_WR_REG(0xE4);  //?
	LCD_WR_DATA(0xa0);
	LCD_WR_REG(0xf3);	//?
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x2a);

	//1  OK
	LCD_WR_REG(0xc8);	//Gamma Setting
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x26);
	LCD_WR_DATA(0x21);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x1f);
	LCD_WR_DATA(0x65);
	LCD_WR_DATA(0x23);
	LCD_WR_DATA(0x77);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x0f);
	LCD_WR_DATA(0x00);
	//GAMMA SETTING

	LCD_WR_REG(0xC0);	//Panel Driving Setting
	LCD_WR_DATA(0x00); //1//00  REV  SM  GS
	LCD_WR_DATA(0x3B); //2//NL[5:0]: Sets the number of lines to drive the LCD at an interval of 8 lines. 
	LCD_WR_DATA(0x00); //3//SCN[6:0]
	LCD_WR_DATA(0x02); //4//PTV: Sets the Vcom output in non-display area drive period
	LCD_WR_DATA(0x11); //5//NDL: Sets the source output level in non-display area.  PTG: Sets the scan mode in non-display area.

	LCD_WR_REG(0xC6); //Interface Control
	LCD_WR_DATA(0x00); //SDA_EN(D7) = 0, EPL(D1) = 0, DPL(D0) = 0

    //DPL: Sets the signal polarity of the PCLK pin.
    //DPL = ?0? The data is input on the rising edge of PCLK.
    //DPL = ?1? The data is input on the falling edge of PCLK.
    //EPL: Sets the signal polarity of the ENABLE pin.
    //EPL = ?0? The data DB[17:0] is written when ENABLE = ?0?.
    //EPL = ?1? The data DB[17:0] is written when ENABLE = ?1?.
    //SDA_EN: DBI type C interface selection
    //SDA_EN = ?0?, DIN and DOUT pins are used for DBI type C interface mode.
    //SDA_EN = ?1?, DIN/SDA pin is used for DBI type C interface mode and DOUT pin is not used.

	LCD_WR_REG(0xf0); //?
	LCD_WR_DATA(0x01);

	LCD_WR_REG(0xE4);	//?
	LCD_WR_DATA(0xa0);

	LCD_WR_REG(0x3a);	//set_pixel_format
	LCD_WR_DATA(0x66); //18bit/pixel

	LCD_WR_REG(0xb4);	//Display Mode and Frame Memory Write Mode setting
	LCD_WR_DATA(0x02);
	LCD_WR_DATA(0x00); //?
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x01);

	delay_ms(280);

	LCD_WR_REG(0x2a);	//Set_column_address
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x01);
	LCD_WR_DATA(0x3F); //3F

	LCD_WR_REG(0x2b);	//Set_page_address
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x01);
	LCD_WR_DATA(0xDf); //DF

	//LCD_WR_REG(0x21);
	LCD_WR_REG(0x29);	
	LCD_WR_REG(0x2c);	//Write_memory_start
	
    //LCD_direction(DIR_HORIZONTAL_2);//设置LCD显示方向
//	LCD_LED=1;//点亮背光	 
	LCD_LED_SET;
	//LCD_Clear(BLACK);//清全屏白色
}

