#ifndef __CH_LIB_INTERRUPT_H__
#define __CH_LIB_INTERRUPT_H__

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stddef.h>
//*************************************************************************************************
//*************************************************************************************************
#ifdef MK10D5
#include "MK10D5.h"
#elif MK10D7
#include "MK10D7.h"					//�ⲿ������common.h  ,�����������ʲô�õģ�����ȥ����һ�ѱ���	
#elif MK20D5
#include "MK20D5.h"
#elif MK20D7
#include "MK20D7.h"
#elif MK10D10
#include "MK10D10.h"
#elif MK20D10
#include "MK20D10.h"
#elif MK40D10
#include "MK40D10.h"
#elif MK60D10
#include "MK60D10.h"
#elif MK60F15
#include "MK60F15.h"
#elif MK60DZ10
#include "MK60DZ10.h"
#elif MK70F12
#include "MK70F12.h"
#elif MK70F15
#include "MK70F15.h"
#elif MK21D5
#include "MK21D5.h"
#elif MK64F12
#include "MK64F12.h"
#elif MK22F12
#include "MK22F12.h"
#elif MK22F25612
#include "MK22F25612.h"
#elif MK22F51212
#include "MK22F51212.h"
#elif MK22F12810
#include "MK22F12810.h"
#else
#error "No CPU defined! please define CPU Type in Preprocessor Symbols, eg: MK60D10"
#endif
                            
/*
**enum  FunctionalState
**brief disable or enable
*/
typedef enum 
{
    DISABLE = 0,        // ��ʹ��
    ENABLE = !DISABLE,  // ʹ�� 
}FunctionalState;

#if !defined(ARRAY_SIZE)
    #define ARRAY_SIZE(x)	(sizeof(x) / sizeof((x)[0]))
#endif

/*
** enum  Clock_t
** brief ʱ��Դ����
*/
typedef enum
{
    kCoreClock,     /**< coreʱ��Ƶ�� */
    kBusClock,      /**< ����ʱ��Ƶ�� */
    kFlexBusClock,  /**< Flex����ʱ��Ƶ�� */
    kFlashClock,    /**< flashʱ��Ƶ�� */
    kMCGOutClock,   /**< MCGģ�����ʱ��Ƶ�� */
}Clock_t; 

//*********************************************************************************************************
//*********************************************************************************************************


static uint32_t fac_us = 0;     //!< usDelay Mut
static uint32_t fac_ms = 0;     //!< msDelay Mut
static GPIO_Type * const GPIO_InstanceTable[] = GPIO_BASES;
static PORT_Type * const PORT_InstanceTable[] = PORT_BASES;
//SIMģ�飬���ڿ���ģ��ʱ��
static const uint32_t SIM_GPIOClockGateTable[] =
{
    SIM_SCGC5_PORTA_MASK,
    SIM_SCGC5_PORTB_MASK,
    SIM_SCGC5_PORTC_MASK,
    SIM_SCGC5_PORTD_MASK,
    SIM_SCGC5_PORTE_MASK,
};

//********************************************************************************************
//GPIO����

//GPIO����ģʽ
typedef enum
{
    kGPIO_Mode_IFT = 0x00,       ///**< ��������
    kGPIO_Mode_IPD = 0x01,       ///**< �������� 
    kGPIO_Mode_IPU = 0x02,       ///**< �������� 
    kGPIO_Mode_OOD = 0x03,       ///**< ��©��� 
    kGPIO_Mode_OPP = 0x04,       ///**< ������� 
}GPIO_Mode_Type;

typedef struct
{
    uint8_t                instance;    ///<���Ŷ˿�HW_GPIOA~HW_GPIOF
    GPIO_Mode_Type         mode;        ///<����ģʽ
    uint32_t               pinx;        ///<���ź�0~31
}GPIO_InitTypeDef;

#define HW_GPIOA  (0x00U) ///*GPIOģ��A,�������� 
#define HW_GPIOB  (0x01U)
#define HW_GPIOC  (0x02U)
#define HW_GPIOD  (0x03U)
#define HW_GPIOE  (0x04U)
#define HW_GPIOF  (0x05U)

typedef enum
{
    kPinAlt0,  /**<0���ܸ���*/
    kPinAlt1,  /**<1���ܸ���*/
    kPinAlt2,  /**<2���ܸ���*/
    kPinAlt3,  /**<3���ܸ���*/
    kPinAlt4,  /**<4���ܸ���*/
    kPinAlt5,  /**<5���ܸ���*/
    kPinAlt6,  /**<6���ܸ���*/
    kPinAlt7,  /**<7���ܸ���*/
}PORT_PinMux_Type;

typedef enum
{
    kPullDisabled,  /**<�ر����������蹦��*/
    kPullUp,        /**<�����������蹦��*/
    kPullDown,      /**<�����������蹦��*/
}PORT_Pull_Type;

typedef enum
{
    kInput,                  /**< ��������ģʽ */
    kOutput,                 /**< �������ģʽ */
}GPIO_PinConfig_Type;
//*****************************************************************************************************************

//**************************************************************************************************************
//�жϲ���

//�˿��жϻص��������� 
typedef void (*GPIO_CallBackType)(uint32_t pinxArray);

static GPIO_CallBackType GPIO_CallBackTable[ARRAY_SIZE(PORT_InstanceTable)] = {NULL};

typedef enum
{
    kGPIO_DMA_RisingEdge,	      ///**<�����ش���DMA
    kGPIO_DMA_FallingEdge,        ///**<�½��ش���DMA
    kGPIO_DMA_RisingFallingEdge,  ////**<�����غ��½��ش���DMA
    kGPIO_IT_Low,                 ///**<�͵�ƽ�����ж�
    kGPIO_IT_RisingEdge,          ///**<�����ش����ж�
    kGPIO_IT_FallingEdge,         ///**<�½��ش����ж�
    kGPIO_IT_RisingFallingEdge,   ///**<�����غ��½��ش����ж�
    kGPIO_IT_High,                ///**<�ߵ�ƽ�����ж�
}GPIO_ITDMAConfig_Type;

static const IRQn_Type GPIO_IRQnTable[] = 
{
    PORTA_IRQn,
    PORTB_IRQn,
    PORTC_IRQn,
    PORTD_IRQn,
    PORTE_IRQn,
};
//***********************************************************************************************************

//**********************************************************************************************************************
#define MCGOUT_TO_CORE_DIVIDER           (((SIM->CLKDIV1 & SIM_CLKDIV1_OUTDIV1_MASK)>>SIM_CLKDIV1_OUTDIV1_SHIFT) + 1)
#define MCGOUT_TO_SYSTEM_DIVIDER         (((SIM->CLKDIV1 & SIM_CLKDIV1_OUTDIV1_MASK)>>SIM_CLKDIV1_OUTDIV1_SHIFT) + 1)
#define MCGOUT_TO_BUS_DIVIDER            (((SIM->CLKDIV1 & SIM_CLKDIV1_OUTDIV2_MASK)>>SIM_CLKDIV1_OUTDIV2_SHIFT) + 1)
#define MCGOUT_TO_PERIPHERAL_DIVIDER     (((SIM->CLKDIV1 & SIM_CLKDIV1_OUTDIV2_MASK)>>SIM_CLKDIV1_OUTDIV2_SHIFT) + 1)
#define MCGOUT_TO_FLEXBUS_DIVIDER        (((SIM->CLKDIV1 & SIM_CLKDIV1_OUTDIV3_MASK)>>SIM_CLKDIV1_OUTDIV3_SHIFT) + 1)
#define MCGOUT_TO_FLASH_DIVIDER          (((SIM->CLKDIV1 & SIM_CLKDIV1_OUTDIV4_MASK)>>SIM_CLKDIV1_OUTDIV4_SHIFT) + 1)
//*********************************************************************************************************************
uint32_t GetClock(Clock_t clockName);
void SYSTICK_Init(uint32_t timeInUs);
void SYSTICK_Cmd(bool NewState);
void SYSTICK_ITConfig(bool NewState);
void SYSTICK_DelayInit(void);
void SYSTICK_DelayUs(uint32_t us);
void SYSTICK_DelayMs(uint32_t ms);
void PORT_PinMuxConfig(uint32_t instance, uint8_t pin, PORT_PinMux_Type pinMux);
void PORT_PinPullConfig(uint32_t instance, uint8_t pin, PORT_Pull_Type pull);
void PORT_PinOpenDrainConfig(uint32_t instance, uint8_t pin, bool status);
void GPIO_PinConfig(uint32_t instance, uint8_t pin, GPIO_PinConfig_Type mode);
void GPIO_Init(GPIO_InitTypeDef * GPIO_InitStruct);
uint8_t GPIO_QuickInit(uint32_t instance, uint32_t pinx, GPIO_Mode_Type mode);
void GPIO_WriteBit(uint32_t instance, uint8_t pin, uint8_t data);
void GPIO_ToggleBit(uint32_t instance, uint8_t pin);
void GPIO_CallbackInstall(uint32_t instance, GPIO_CallBackType AppCBFun);
void GPIO_ITDMAConfig(uint32_t instance, uint8_t pin, GPIO_ITDMAConfig_Type config, bool status);


#ifdef __cplusplus
}
#endif

#endif


