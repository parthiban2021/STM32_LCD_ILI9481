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
#include "SPI.h"

SPI_HandleTypeDef SPI2_Handler;  //SPI���

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
 	//original: return Rxdata;           	     //�����յ�������			
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
    assert_param(IS_SPI_BAUDRATE_PRESCALER(SPI_BaudRatePrescaler));//�ж���Ч��
    __HAL_SPI_DISABLE(&SPI2_Handler);            //�ر�SPI
    SPI2_Handler.Instance->CR1&=0XFFC7;          //λ3-5���㣬�������ò�����
    SPI2_Handler.Instance->CR1|=SPI_BaudRatePrescaler;//����SPI�ٶ�
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
