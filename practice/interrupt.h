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
#include "MK10D7.h"					//这部分来自common.h  ,不清楚具体做什么用的，但是去掉有一堆报错	
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
    DISABLE = 0,        // 不使能
    ENABLE = !DISABLE,  // 使能 
}FunctionalState;

#if !defined(ARRAY_SIZE)
    #define ARRAY_SIZE(x)	(sizeof(x) / sizeof((x)[0]))
#endif

/*
** enum  Clock_t
** brief 时钟源定义
*/
typedef enum
{
    kCoreClock,     /**< core时钟频率 */
    kBusClock,      /**< 总线时钟频率 */
    kFlexBusClock,  /**< Flex总线时钟频率 */
    kFlashClock,    /**< flash时钟频率 */
    kMCGOutClock,   /**< MCG模块输出时钟频率 */
}Clock_t; 

//*********************************************************************************************************
//*********************************************************************************************************


static uint32_t fac_us = 0;     //!< usDelay Mut
static uint32_t fac_ms = 0;     //!< msDelay Mut
static GPIO_Type * const GPIO_InstanceTable[] = GPIO_BASES;
static PORT_Type * const PORT_InstanceTable[] = PORT_BASES;
//SIM模块，用于开各模块时钟
static const uint32_t SIM_GPIOClockGateTable[] =
{
    SIM_SCGC5_PORTA_MASK,
    SIM_SCGC5_PORTB_MASK,
    SIM_SCGC5_PORTC_MASK,
    SIM_SCGC5_PORTD_MASK,
    SIM_SCGC5_PORTE_MASK,
};

//********************************************************************************************
//GPIO部分

//GPIO工作模式
typedef enum
{
    kGPIO_Mode_IFT = 0x00,       ///**< 浮空输入
    kGPIO_Mode_IPD = 0x01,       ///**< 下拉输入 
    kGPIO_Mode_IPU = 0x02,       ///**< 上拉输入 
    kGPIO_Mode_OOD = 0x03,       ///**< 开漏输出 
    kGPIO_Mode_OPP = 0x04,       ///**< 推挽输出 
}GPIO_Mode_Type;

typedef struct
{
    uint8_t                instance;    ///<引脚端口HW_GPIOA~HW_GPIOF
    GPIO_Mode_Type         mode;        ///<工作模式
    uint32_t               pinx;        ///<引脚号0~31
}GPIO_InitTypeDef;

#define HW_GPIOA  (0x00U) ///*GPIO模块A,依次类推 
#define HW_GPIOB  (0x01U)
#define HW_GPIOC  (0x02U)
#define HW_GPIOD  (0x03U)
#define HW_GPIOE  (0x04U)
#define HW_GPIOF  (0x05U)

typedef enum
{
    kPinAlt0,  /**<0功能复用*/
    kPinAlt1,  /**<1功能复用*/
    kPinAlt2,  /**<2功能复用*/
    kPinAlt3,  /**<3功能复用*/
    kPinAlt4,  /**<4功能复用*/
    kPinAlt5,  /**<5功能复用*/
    kPinAlt6,  /**<6功能复用*/
    kPinAlt7,  /**<7功能复用*/
}PORT_PinMux_Type;

typedef enum
{
    kPullDisabled,  /**<关闭上下拉电阻功能*/
    kPullUp,        /**<开启上拉电阻功能*/
    kPullDown,      /**<开启下拉电阻功能*/
}PORT_Pull_Type;

typedef enum
{
    kInput,                  /**< 引脚输入模式 */
    kOutput,                 /**< 引脚输出模式 */
}GPIO_PinConfig_Type;
//*****************************************************************************************************************

//**************************************************************************************************************
//中断部分

//端口中断回调函数定义 
typedef void (*GPIO_CallBackType)(uint32_t pinxArray);

static GPIO_CallBackType GPIO_CallBackTable[ARRAY_SIZE(PORT_InstanceTable)] = {NULL};

typedef enum
{
    kGPIO_DMA_RisingEdge,	      ///**<上升沿触发DMA
    kGPIO_DMA_FallingEdge,        ///**<下降沿触发DMA
    kGPIO_DMA_RisingFallingEdge,  ////**<上升沿和下降沿触发DMA
    kGPIO_IT_Low,                 ///**<低电平出发中断
    kGPIO_IT_RisingEdge,          ///**<上升沿触发中断
    kGPIO_IT_FallingEdge,         ///**<下降沿触发中断
    kGPIO_IT_RisingFallingEdge,   ///**<上升沿和下降沿触发中断
    kGPIO_IT_High,                ///**<高电平触发中断
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


