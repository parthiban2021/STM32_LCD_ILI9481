//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//����Ӳ������Ƭ��STM32F429IGT6,����ԭ��Apollo STM32F4/F7������,��Ƶ180MHZ������12MHZ
//QDtech-TFTҺ������ for STM32 IOģ��
//xiao��@ShenZhen QDtech co.,LTD
//��˾��վ:www.qdtft.com
//�Ա���վ��http://qdtech.taobao.com
//wiki������վ��http://www.lcdwiki.com
//��˾�ṩ����֧�֣��κμ������⻶ӭ��ʱ����ѧϰ
//�̻�(����) :+86 0755-23594567 
//�ֻ�:15989313508���빤�� 
//����:lcdwiki01@gmail.com    support@lcdwiki.com    goodtft@163.com 
//����֧��QQ:3002773612  3002778157
//��������QQȺ:324828016
//��������:2018/08/09
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������ȫ�����Ӽ������޹�˾ 2018-2028
//All rights reserved
/****************************************************************************************************
//=========================================��Դ����================================================//
//     LCDģ��                STM32��Ƭ��
//      VCC          ��        DC5V/3.3V      //��Դ
//      GND          ��          GND          //��Դ��
//=======================================Һ���������߽���==========================================//
//��ģ��Ĭ��������������ΪSPI����
//     LCDģ��                STM32��Ƭ��    
//    SDI(MOSI)      ��          PF9          //Һ����SPI��������д�ź�
//    SDO(MISO)      ��          PF8          //Һ����SPI�������ݶ��źţ��������Ҫ�������Բ�����
//=======================================Һ���������߽���==========================================//
//     LCDģ�� 					      STM32��Ƭ�� 
//       LED         ��          PD6          //Һ������������źţ��������Ҫ���ƣ���5V��3.3V
//       SCK         ��          PF7          //Һ����SPI����ʱ���ź�
//      DC/RS        ��          PD5          //Һ��������/��������ź�
//       RST         ��          PD12         //Һ������λ�����ź�
//       CS          ��          PD11         //Һ����Ƭѡ�����ź�
//=========================================������������=========================================//
//���ģ�鲻���������ܻ��ߴ��д������ܣ����ǲ���Ҫ�������ܣ�����Ҫ���д���������
//	   LCDģ��                STM32��Ƭ�� 
//      T_IRQ        ��          PH11         //�����������ж��ź�
//      T_DO         ��          PG3          //������SPI���߶��ź�
//      T_DIN        ��          PI3          //������SPI����д�ź�
//      T_CS         ��          PI8          //������Ƭѡ�����ź�
//      T_CLK        ��          PH6          //������SPI����ʱ���ź�
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

//LCD��Ҫ������
typedef struct  
{										    
	U16 width;			//LCD ���
	U16 height;			//LCD �߶�
	U16 id;				//LCD ID
	U8  dir;			//���������������ƣ�0��������1��������	
	U16	 wramcmd;		//��ʼдgramָ��
	U16  setxcmd;		//����x����ָ��
	U16  setycmd;		//����y����ָ��	 
}_lcd_dev; 	

//LCD����
extern _lcd_dev lcddev;	//����LCD��Ҫ����
/////////////////////////////////////�û�������///////////////////////////////////	 
#define DIR_HORIZONTAL  	 	0	//����Һ����˳ʱ����ת���� 	0-0����ת��1-90����ת��2-180����ת��3-270����ת
#define DIR_HORIZONTAL_2  	1
#define DIR_VERTICAL  	 		2
#define DIR_VERTICAL_2  	 	3

//////////////////////////////////////////////////////////////////////////////////	  
//����LCD�ĳߴ�
//#define LCD_W 								320
//#define LCD_H 								480
//#define BYTES_PER_PIXEL				3			//RGB - 18bit; 6bits per colour
//#if (LCD_W > LCD_H)
//	#define LCD_LINEBUFFER_MAX	(LCD_W * BYTES_PER_PIXEL)
//#else
//	#define LCD_LINEBUFFER_MAX	(LCD_H * BYTES_PER_PIXEL)
//#endif

//TFTLCD������Ҫ���õĺ���		   
extern U16  POINT_COLOR;//Ĭ�Ϻ�ɫ    
extern U16  BACK_COLOR; //������ɫ.Ĭ��Ϊ��ɫ

////////////////////////////////////////////////////////////////////
//-----------------LCD�˿ڶ���---------------- 

#define ILI9328_ID    0x9328

#define LED   9        //�����������
#define CS    11       //Ƭѡ����
#define RS    10       //�Ĵ���/����ѡ������  
#define RST   12       //��λ����

#define LCD_LED_PIN                				GPIO_PIN_9
#define LCD_CS_PIN                  			GPIO_PIN_11
#define LCD_RS_PIN                  			GPIO_PIN_10
#define LCD_RST_PIN                  			GPIO_PIN_12

#define GPIO_PORTB		                     GPIOB
#define LCD_GPIO_CLK_ENABLE()              __HAL_RCC_GPIOB_CLK_ENABLE()

//QDtechȫϵ��ģ������������ܿ��Ʊ��������û�Ҳ���Խ�PWM���ڱ�������
//#define	LCD_LED PBout(LED) //LCD����    		 
//#define LCD_CS  PBout(CS) 
//#define LCD_RS  PBout(RS) 
//#define LCD_RST PBout(RST)

//���ʹ�ùٷ��⺯���������еײ㣬�ٶȽ����½���14֡ÿ�룬���������˾�Ƽ�����
//����IO����ֱ�Ӳ����Ĵ���������IO������ˢ�����ʿ��Դﵽ28֡ÿ�룡 

//#define	LCD_CS_SET  LCD_CS=1//GPIO_TYPE->BSRR=1<<LCD_CS    //Ƭѡ�˿�  	PB11
//#define	LCD_RS_SET	LCD_RS=1//GPIO_TYPE->BSRR=1<<LCD_RS    //����/����  PB10	  
//#define	LCD_RST_SET	LCD_RST=1//GPIO_TYPE->BSRR=1<<LCD_RST    //��λ			PB12

#define	GPIOB_SET_PIN(PIN)		HAL_GPIO_WritePin(GPIO_PORTB, PIN, GPIO_PIN_SET)
#define	GPIOB_CLR_PIN(PIN)		HAL_GPIO_WritePin(GPIO_PORTB, PIN, GPIO_PIN_RESET)

#define	LCD_LED_SET  	GPIOB_SET_PIN(LCD_LED_PIN)
#define	LCD_CS_SET  	GPIOB_SET_PIN(LCD_CS_PIN)
#define	LCD_RS_SET		GPIOB_SET_PIN(LCD_RS_PIN)	  
#define	LCD_RST_SET		GPIOB_SET_PIN(LCD_RST_PIN)
 							    
//#define	LCD_CS_CLR  LCD_CS=0//GPIO_TYPE->BRR=1<<LCD_CS     //Ƭѡ�˿�  	PB11
//#define	LCD_RS_CLR	LCD_RS=0//GPIO_TYPE->BRR=1<<LCD_RS     //����/����  PB10	 
//#define	LCD_RST_CLR	LCD_RST=0//GPIO_TYPE->BRR=1<<LCD_RST    //��λ			PB12

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
#define BROWN 			0XBC40 //��ɫ
#define BRRED 			0XFC07 //�غ�ɫ
#define GRAY  			0X8430 //��ɫ
//GUI��ɫ

#define DARKBLUE      	 0X01CF	//����ɫ
#define LIGHTBLUE      	 0X7D7C	//ǳ��ɫ  
#define GRAYBLUE       	 0X5458 //����ɫ
//������ɫΪPANEL����ɫ 
 
#define LIGHTGREEN     	0X841F //ǳ��ɫ
#define LIGHTGRAY     0XEF5B //ǳ��ɫ(PANNEL)
#define LGRAY 			 		0XC618 //ǳ��ɫ(PANNEL),���屳��ɫ

#define LGRAYBLUE      	0XA651 //ǳ����ɫ(�м����ɫ)
#define LBBLUE          0X2B12 //ǳ����ɫ(ѡ����Ŀ�ķ�ɫ)
	    	
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

//�����Ȼ�����ٶȲ����죬����ʹ������ĺ궨��,����ٶ�.
//ע��Ҫȥ��lcd.c��void LCD_WR_DATA(U16 data)��������Ŷ
/*
#if LCD_USE8BIT_MODEL==1//ʹ��8λ������������ģʽ
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
	#else//ʹ��16λ������������ģʽ
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
	 
	 



