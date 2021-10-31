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
#include "SPI.h"

SPI_HandleTypeDef SPI2_Handler;  //SPI句柄

/*****************************************************************************
 * @name       :U8 SPI_WriteByte(SPI_TypeDef* SPIx,u8 Byte)
 * @date       :2018-08-09 
 * @function   :Write a byte of data using STM32's hardware SPI
 * @parameters :TxData:Data to be written
 * @retvalue   :Data received by the bus
******************************************************************************/
void SPI_WriteByte(U8 TxData)
{
  //original: U8 Rxdata;
	HAL_SPI_Transmit(&SPI2_Handler,&TxData,1,0); //original: HAL_SPI_TransmitReceive(&SPI2_Handler,&TxData,&Rxdata,1, 1);       
 	//original: return Rxdata;           	     //返回收到的数据			
} 

void SPI_WriteMultiByte(U8 *TxData, U16 nbytes)
{
	//Timeout(ms): 1ms x nbytes
	HAL_SPI_Transmit(&SPI2_Handler,TxData,nbytes,nbytes);
} 

void SPI_ReadMultiByte(U8 *RxData, U16 nbytes)
{
	//Timeout(ms): 1ms x nbytes
	HAL_SPI_Receive(&SPI2_Handler,RxData,nbytes,nbytes);
}

void SPI_TransmitReceiveMultiByte(U8 *TxData, U8 *RxData, U16 nbytes)
{
    //Timeout(ms): 1ms x nbytes
    HAL_SPI_TransmitReceive(&SPI2_Handler,TxData,RxData,nbytes,nbytes);
}

/*****************************************************************************
 * @name       :void SPI_SetSpeed(SPI_TypeDef* SPIx,U8 SpeedSet)
 * @date       :2018-08-09 
 * @function   :Set hardware SPI Speed
 * @parameters :SPI_BaudRatePrescaler:SPI speed
 * @retvalue   :None
******************************************************************************/
void SPI_SetSpeed(U8 SPI_BaudRatePrescaler)
{
    assert_param(IS_SPI_BAUDRATE_PRESCALER(SPI_BaudRatePrescaler));//判断有效性
    __HAL_SPI_DISABLE(&SPI2_Handler);            //关闭SPI
    SPI2_Handler.Instance->CR1&=0XFFC7;          //位3-5清零，用来设置波特率
    SPI2_Handler.Instance->CR1|=SPI_BaudRatePrescaler;//设置SPI速度
    __HAL_SPI_ENABLE(&SPI2_Handler);   
} 


/*****************************************************************************
 * @name       :void SPI2_Init(void)	
 * @date       :2018-08-09 
 * @function   :Initialize the STM32 hardware SPI2
 * @parameters :None
 * @retvalue   :None
******************************************************************************/
void SPI2_Init(void)
{
    SPI2_Handler.Instance=SPI2;                         
    SPI2_Handler.Init.Mode=SPI_MODE_MASTER;             
    SPI2_Handler.Init.Direction=SPI_DIRECTION_2LINES;   
    SPI2_Handler.Init.DataSize=SPI_DATASIZE_8BIT;       
	SPI2_Handler.Init.CLKPolarity=SPI_POLARITY_HIGH;                //SPI_CPOL = SPI_POLARITY_HIGH
	SPI2_Handler.Init.CLKPhase=SPI_PHASE_2EDGE;                     //SPI_CPHA = SPI_PHASE_2EDGE
    SPI2_Handler.Init.NSS=SPI_NSS_SOFT;                 
	SPI2_Handler.Init.BaudRatePrescaler=SPI_BAUDRATEPRESCALER_8;	//APB1(80 MHz) -> SPI2; DIV8: 10MHz; DIV16: 5MHz; ILI9481 supports max 10MHz
    SPI2_Handler.Init.FirstBit=SPI_FIRSTBIT_MSB;        
    SPI2_Handler.Init.TIMode=SPI_TIMODE_DISABLE;        
    SPI2_Handler.Init.CRCCalculation=SPI_CRCCALCULATION_DISABLE;
    SPI2_Handler.Init.CRCPolynomial=7;                  
    HAL_SPI_Init(&SPI2_Handler);
    
    __HAL_SPI_ENABLE(&SPI2_Handler);
}

/*****************************************************************************
 * @name       :void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)	
 * @date       :2018-08-09 
 * @function   :Configuring SPI pins and clocks,
                it will be called by HAL_SPI_Init
 * @parameters :hspi:SPI handle
 * @retvalue   :None
******************************************************************************/
void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Enable SPI clock  */
    __HAL_RCC_SPI2_CLK_ENABLE();        //SPI2

    /* enable SPI gpio clock */
	__HAL_RCC_GPIOB_CLK_ENABLE();       //GPIO Port B
    
    //Alternate function: AF5
    //SPI2_SCK: PB13 
    //SPI2_MISO: PB14
    //SPI2_MOSI: PB15
	GPIO_InitStructure.Pin=GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
    GPIO_InitStructure.Mode=GPIO_MODE_AF_PP;
    //GPIO_InitStructure.Pull=GPIO_NOPULL;
    GPIO_InitStructure.Speed=GPIO_SPEED_FAST;          
    GPIO_InitStructure.Alternate=GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB,&GPIO_InitStructure);
}
