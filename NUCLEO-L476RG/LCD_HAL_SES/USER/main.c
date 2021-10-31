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
//#include "delay.h"
//#include "sys.h"
//#include "lcd.h"
//#include "touch.h"
//#include "gui.h"
//#include "test.h"

//#include "stm32l476xx.h"
//#include "stm32l4xx_hal.h"
//#include "stm32l4xx_nucleo.h"
#include "main.h"
//#include "lcd.h"
#include "GUI.h"
#include "WM.h"

#define LED2_PIN                           GPIO_PIN_5
#define LED2_GPIO_PORT                     GPIOA
#define LED2_GPIO_CLK_ENABLE()             __HAL_RCC_GPIOA_CLK_ENABLE()
#define LCD_GPIO_CLK_ENABLE()              __HAL_RCC_GPIOB_CLK_ENABLE()

extern volatile GUI_TIMER_TIME OS_TimeMS;   //used by GUI_X_Delay() to update OS_TimeMS every ms

void LED2_Init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	LED2_GPIO_CLK_ENABLE();

    /* -2- Configure IOs in output push-pull mode to drive external LEDs */
    GPIO_InitStructure.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Pull  = GPIO_PULLUP;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStructure.Pin = LED2_PIN;
    HAL_GPIO_Init(LED2_GPIO_PORT, &GPIO_InitStructure);
	
}

void LCD_BackLight_Init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	LCD_GPIO_CLK_ENABLE();

    /* -2- Configure IOs in output push-pull mode to drive external LEDs */
    GPIO_InitStructure.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Pull  = GPIO_PULLUP;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStructure.Pin = GPIO_PIN_9;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
	
}

void SystemClock_Config(void)
{
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};

    /* MSI is enabled after System reset, activate PLL with MSI as source */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
    RCC_OscInitStruct.MSIState = RCC_MSI_ON;
    RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
    RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
    RCC_OscInitStruct.PLL.PLLM = 1;
    RCC_OscInitStruct.PLL.PLLN = 40;
    RCC_OscInitStruct.PLL.PLLR = 2;
    RCC_OscInitStruct.PLL.PLLP = 7;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        /* Initialization Error */
        while(1);
    }
    /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
    RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;  //APB1 -> SPI2; 80 MHz
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;  //80 MHz
    if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
    {
        /* Initialization Error */
        while(1);
    }
}

void Default_MainTask(void) 
{
    U16 Device_ID;

    /* Init the STemWin GUI Library */
    GUI_Init();

    while(1)
    {
        Device_ID = LCD_GetDeviceID();
        GUI_SetBkColor(GUI_WHITE);
        GUI_Clear();
        delay_ms(1000);
        GUI_SetBkColor(GUI_RED);
        GUI_Clear();
        delay_ms(1000);
        GUI_SetBkColor(GUI_GREEN);
        GUI_Clear();
        delay_ms(1000);
        GUI_SetBkColor(GUI_BLUE);
        GUI_Clear();
        delay_ms(1000);
        GUI_SetBkColor(GUI_YELLOW);
        GUI_Clear();
        delay_ms(1000);
        GUI_SetFont(&GUI_Font32_1);
        GUI_SetColor(GUI_RED);
        GUI_FillCircle(LCD_GetXSize()/2,LCD_GetYSize()/2,50);
        GUI_SetColor(GUI_BLACK);
        GUI_DispStringAt("Hello world!", (LCD_GetXSize()-100)/2, (LCD_GetYSize()-20)/2);
        HAL_GPIO_TogglePin(LED2_GPIO_PORT, LED2_PIN);
        delay_ms(1000);
    }
}

void HAL_SYSTICK_Callback(void)
{
    if (OS_TimeMS < UINT32_MAX)
        OS_TimeMS++;
    else
        OS_TimeMS = 0U;
}

int main(void)
{	
    HAL_Init();
	SystemClock_Config();
    delay_ms(180);
	LED2_Init();

    /* Enable the CRC Module */
    __HAL_RCC_CRC_CLK_ENABLE();

    //Default_MainTask();
    MainTask();
}

