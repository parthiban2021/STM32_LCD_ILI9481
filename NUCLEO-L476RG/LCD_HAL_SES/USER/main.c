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

