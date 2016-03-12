#include <interrupt.h>

//获得各种时钟
uint32_t GetClock(Clock_t clockName)
{
    uint32_t clock = 0;
    /* calualte MCGOutClock system_MKxxx.c must not modified */
    SystemCoreClockUpdate();
    clock = SystemCoreClock * MCGOUT_TO_CORE_DIVIDER;
    switch (clockName)
    {
        case kCoreClock:
            clock = clock / MCGOUT_TO_CORE_DIVIDER;
            break;
        case kBusClock:
            clock = clock / MCGOUT_TO_BUS_DIVIDER;
            break;
        case kFlashClock:
            clock = clock / MCGOUT_TO_FLASH_DIVIDER;
            break;
        case kMCGOutClock:
            break;
        default:
            clock = 0;
    }
    return clock;
}

//**********************************************************************************************************************
//利用SysTick的延时

//初始化SysTick时钟
void SYSTICK_Init(uint32_t timeInUs)
{
    ///* Set clock source = core clock 
    SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk; 
    fac_us = GetClock(kCoreClock);
    fac_us /= 1000000;
    fac_ms = fac_us * 1000;
    
    SysTick->LOAD = fac_us * timeInUs;
}

//开启或者停止SysTick时钟      true 使能    false  停止
void SYSTICK_Cmd(bool NewState)
{
    (true == NewState)?(SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk):(SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk);
}

//开启或关闭SysTick中断     true 使能   false  关闭
void SYSTICK_ITConfig(bool NewState)
{
    (true == NewState)?(SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk):(SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk);
}

//利用Systick 的延时函数的初始化
void SYSTICK_DelayInit(void)
{
    SYSTICK_Init(1000);         //初始化SysTick                
    SYSTICK_Cmd(true);			//开启SysTick时钟
    SYSTICK_ITConfig(false);	//关闭SysTick中断
}

//微妙级延时
void SYSTICK_DelayUs(uint32_t us)
{
    uint32_t temp;
    SysTick->LOAD = us * fac_us;
    SysTick->VAL = 0;
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
    do
    {
        temp = SysTick->CTRL;
    }
    while((temp & SysTick_CTRL_ENABLE_Msk) && !(temp & SysTick_CTRL_COUNTFLAG_Msk));
}

//毫秒级延时
void SYSTICK_DelayMs(uint32_t ms)
{
    uint32_t temp;
    uint32_t i;
    SysTick->LOAD = fac_ms;
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
    for(i = 0; i < ms; i++)
	{
		SysTick->VAL = 0;
		do
		{
			temp = SysTick->CTRL;
		}
        while((temp & SysTick_CTRL_ENABLE_Msk) && !(temp & SysTick_CTRL_COUNTFLAG_Msk));
	}
}
//*******************************************************************************************************************

//*******************************************************************************************************************
//GPIO相关函数    这里只写了部分的操作GPIO的函数
//设置GPIO引脚复用
void PORT_PinMuxConfig(uint32_t instance, uint8_t pin, PORT_PinMux_Type pinMux)
{
    SIM->SCGC5 |= SIM_GPIOClockGateTable[instance];
    PORT_InstanceTable[instance]->PCR[pin] &= ~(PORT_PCR_MUX_MASK);
    PORT_InstanceTable[instance]->PCR[pin] |=  PORT_PCR_MUX(pinMux);
}

//配置GPIO的上下拉选项
void PORT_PinPullConfig(uint32_t instance, uint8_t pin, PORT_Pull_Type pull)
{
    SIM->SCGC5 |= SIM_GPIOClockGateTable[instance];
    switch(pull)
    {
        case kPullDisabled:
            PORT_InstanceTable[instance]->PCR[pin] &= ~PORT_PCR_PE_MASK;
            break;
        case kPullUp:
            PORT_InstanceTable[instance]->PCR[pin] |= PORT_PCR_PE_MASK;
            PORT_InstanceTable[instance]->PCR[pin] |= PORT_PCR_PS_MASK;
            break;
        case kPullDown:
            PORT_InstanceTable[instance]->PCR[pin] |= PORT_PCR_PE_MASK;
            PORT_InstanceTable[instance]->PCR[pin] &= ~PORT_PCR_PS_MASK;
            break;
        default:
            break;
    }
}

//端口引脚的开漏状态配置 用户一般不必调用	ENABLE   开启功能	DISABLE  关闭功能
void PORT_PinOpenDrainConfig(uint32_t instance, uint8_t pin, bool status)
{
    SIM->SCGC5 |= SIM_GPIOClockGateTable[instance];
    (status) ? (PORT_InstanceTable[instance]->PCR[pin] |= PORT_PCR_ODE_MASK):(PORT_InstanceTable[instance]->PCR[pin] &= ~PORT_PCR_ODE_MASK);
}

//设置引脚为输入还是输出功能  用户一般不必调用		kInpput 输入功能选择	kOutput 输出功能选择
void GPIO_PinConfig(uint32_t instance, uint8_t pin, GPIO_PinConfig_Type mode)
{
    SIM->SCGC5 |= SIM_GPIOClockGateTable[instance];
    (mode == kOutput) ? (GPIO_InstanceTable[instance]->PDDR |= (1 << pin)):(GPIO_InstanceTable[instance]->PDDR &= ~(1 << pin));
}

/*
** @brief  GPIO初始化配置
** @code
**    //初始化PTB_10为推挽输出
**    GPIO_InitTypeDef GPIO_InitStruct1;      //申请一个结构体变量
**    GPIO_InitStruct1.instance = HW_GPIOB;   //选择PORTB端口
**    GPIO_InitStruct1.mode = kGPIO_Mode_OPP; //推挽输出
**    GPIO_InitStruct1.pinx = 10;             //选择10引脚
**    //然后调用初始化GPIO函数 
**    GPIO_Init(&GPIO_InitStruct1);
** @endcode
** instance:端口号  HW_GPIOA---HW_GPIOE    mode:工作方式  见前面的定义
** 
**/
void GPIO_Init(GPIO_InitTypeDef * GPIO_InitStruct)
{
    switch(GPIO_InitStruct->mode)
    {
        case kGPIO_Mode_IFT:
            PORT_PinPullConfig(GPIO_InitStruct->instance, GPIO_InitStruct->pinx, kPullDisabled);
            PORT_PinOpenDrainConfig(GPIO_InitStruct->instance, GPIO_InitStruct->pinx, DISABLE);
            GPIO_PinConfig(GPIO_InitStruct->instance, GPIO_InitStruct->pinx, kInput);
            break;
        case kGPIO_Mode_IPD:
            PORT_PinPullConfig(GPIO_InitStruct->instance, GPIO_InitStruct->pinx, kPullDown);
            PORT_PinOpenDrainConfig(GPIO_InitStruct->instance, GPIO_InitStruct->pinx, DISABLE);
            GPIO_PinConfig(GPIO_InitStruct->instance, GPIO_InitStruct->pinx, kInput);
            break;
        case kGPIO_Mode_IPU:
            PORT_PinPullConfig(GPIO_InitStruct->instance, GPIO_InitStruct->pinx, kPullUp);
            PORT_PinOpenDrainConfig(GPIO_InitStruct->instance, GPIO_InitStruct->pinx, DISABLE);
            GPIO_PinConfig(GPIO_InitStruct->instance, GPIO_InitStruct->pinx, kInput);
            break;
        case kGPIO_Mode_OOD:
            PORT_PinPullConfig(GPIO_InitStruct->instance, GPIO_InitStruct->pinx, kPullUp);
            PORT_PinOpenDrainConfig(GPIO_InitStruct->instance, GPIO_InitStruct->pinx, ENABLE);
            GPIO_PinConfig(GPIO_InitStruct->instance, GPIO_InitStruct->pinx, kOutput);
            break;
        case kGPIO_Mode_OPP:
            PORT_PinPullConfig(GPIO_InitStruct->instance, GPIO_InitStruct->pinx, kPullDisabled);
            PORT_PinOpenDrainConfig(GPIO_InitStruct->instance, GPIO_InitStruct->pinx, DISABLE);
            GPIO_PinConfig(GPIO_InitStruct->instance, GPIO_InitStruct->pinx, kOutput);
            break;
        default:
            break;					
    }
    
    PORT_PinMuxConfig(GPIO_InitStruct->instance, GPIO_InitStruct->pinx, kPinAlt1);
}

//GPIO快速初始化
//需要给instance:端口号  pinx:引脚号  mode:工作方式
uint8_t GPIO_QuickInit(uint32_t instance, uint32_t pinx, GPIO_Mode_Type mode)
{
    GPIO_InitTypeDef GPIO_InitStruct1;
    GPIO_InitStruct1.instance = instance;
    GPIO_InitStruct1.mode = mode;
    GPIO_InitStruct1.pinx = pinx;
    GPIO_Init(&GPIO_InitStruct1);
    return  instance;
}

//设置指定引脚输出高电平或者低电平  0 低    1高   首先要设置他为输出脚
void GPIO_WriteBit(uint32_t instance, uint8_t pin, uint8_t data)
{
    SIM->SCGC5 |= SIM_GPIOClockGateTable[instance];
    (data) ? (GPIO_InstanceTable[instance]->PSOR |= (1 << pin)):(GPIO_InstanceTable[instance]->PCOR |= (1 << pin));
}

//翻转指定引脚电平状态      首先要设置他为输出脚                    
void GPIO_ToggleBit(uint32_t instance, uint8_t pin)
{
    GPIO_InstanceTable[instance]->PTOR |= (1 << pin);
}
//*******************************************************************************************************************

//*******************************************************************************************************************
//中断相关函数
/*
** @brief  注册中断回调函数
** @param  instance: GPIO模块中断入口号
**         @arg HW_GPIOA :A端口中断入口
**         @arg HW_GPIOB :B端口中断入口
**         @arg HW_GPIOC :C端口中断入口
**         @arg HW_GPIOD :D端口中断入口
**         @arg HW_GPIOE :E端口中断入口
** @param AppCBFun: 回调函数指针入口
** @retval None
*/
void GPIO_CallbackInstall(uint32_t instance, GPIO_CallBackType AppCBFun)
{
    
    SIM->SCGC5 |= SIM_GPIOClockGateTable[instance];
    if(AppCBFun != NULL)
    {
        GPIO_CallBackTable[instance] = AppCBFun;
    }
}

/*
** @brief  设置GPIO引脚中断类型或者DMA功能
** @code
**      //设置PORTB端口的10引脚为下降沿触发中断
**      GPIO_ITDMAConfig(HW_GPIOB, 10, kGPIO_IT_FallingEdge, true); 
** @endcode
** @param[in]  instance GPIO模块号
**              @arg HW_GPIOA 芯片的PORTA端口
**              @arg HW_GPIOB 芯片的PORTB端口
**              @arg HW_GPIOC 芯片的PORTC端口
**              @arg HW_GPIOD 芯片的PORTD端口
**              @arg HW_GPIOE 芯片的PORTE端口
** @param[in]  pin 端口上的引脚号 0~31
** @param[in]  config 配置模式
**              @arg kGPIO_DMA_RisingEdge DMA上升沿触发
**              @arg kGPIO_DMA_FallingEdge DMA下降沿触发
**              @arg kGPIO_DMA_RisingFallingEdge DMA上升和下降沿都触发
**              @arg kGPIO_IT_Low 低电平触发中断
**              @arg kGPIO_IT_RisingEdge 上升沿触发中断
**              @arg kGPIO_IT_FallingEdge 下降沿触发中断
**              @arg kGPIO_IT_RisingFallingEdge 上升和下降沿都触发中断
**              @arg kGPIO_IT_High 高电平触发中断
** @param[in]  status 是否使能
**              @arg 0 disable
**              @arg 1 enable
** @retval None
*/
void GPIO_ITDMAConfig(uint32_t instance, uint8_t pin, GPIO_ITDMAConfig_Type config, bool status)
{
    //* init moudle 
    SIM->SCGC5 |= SIM_GPIOClockGateTable[instance];
    PORT_InstanceTable[instance]->PCR[pin] &= ~PORT_PCR_IRQC_MASK;
    
    if(!status)
    {
        NVIC_DisableIRQ(GPIO_IRQnTable[instance]);
        return;
    }
    
    switch(config)
    {
        case kGPIO_DMA_RisingEdge:
            PORT_InstanceTable[instance]->PCR[pin] |= PORT_PCR_IRQC(1);
            break;
        case kGPIO_DMA_FallingEdge:
            PORT_InstanceTable[instance]->PCR[pin] |= PORT_PCR_IRQC(2);
            break;
        case kGPIO_DMA_RisingFallingEdge:
            PORT_InstanceTable[instance]->PCR[pin] |= PORT_PCR_IRQC(3);
            break;
        case kGPIO_IT_Low:
            PORT_InstanceTable[instance]->PCR[pin] |= PORT_PCR_IRQC(8);
            NVIC_EnableIRQ(GPIO_IRQnTable[instance]);
            break;
        case kGPIO_IT_RisingEdge:
            PORT_InstanceTable[instance]->PCR[pin] |= PORT_PCR_IRQC(9);
            NVIC_EnableIRQ(GPIO_IRQnTable[instance]);
            break;
        case kGPIO_IT_FallingEdge:
            PORT_InstanceTable[instance]->PCR[pin] |= PORT_PCR_IRQC(10);
            NVIC_EnableIRQ(GPIO_IRQnTable[instance]);
            break;
        case kGPIO_IT_RisingFallingEdge:
            PORT_InstanceTable[instance]->PCR[pin] |= PORT_PCR_IRQC(11);
            NVIC_EnableIRQ(GPIO_IRQnTable[instance]);
            break;
        case kGPIO_IT_High:
            PORT_InstanceTable[instance]->PCR[pin] |= PORT_PCR_IRQC(12);
            NVIC_EnableIRQ(GPIO_IRQnTable[instance]);
            break;
        default:
            break;
    }
}
/*
** @brief  GPIO中断总处理函数，用户一般无需使用和修改
** @param[in]  instance GPIO模块中断入口号
**              @arg HW_GPIOA 芯片的PORTA端口中断入口
**              @arg HW_GPIOB 芯片的PORTB端口中断入口
**              @arg HW_GPIOC 芯片的PORTC端口中断入口
**              @arg HW_GPIOD 芯片的PORTD端口中断入口
**              @arg HW_GPIOE 芯片的PORTE端口中断入口
*/
static void PORT_IRQHandler(uint32_t instance)
{
    uint32_t ISFR;
    // safe copy 
    ISFR = PORT_InstanceTable[instance]->ISFR;
    // clear IT pending bit 
    PORT_InstanceTable[instance]->ISFR = 0xFFFFFFFF;
    if(GPIO_CallBackTable[instance])
    {
        GPIO_CallBackTable[instance](ISFR);
    }
}

//系统GPIO中断函数，用户无需使用
void PORTA_IRQHandler(void)
{
    PORT_IRQHandler(HW_GPIOA);
}

//系统GPIO中断函数，用户无需使用
void PORTB_IRQHandler(void)
{
    PORT_IRQHandler(HW_GPIOB);
}

//系统GPIO中断函数，用户无需使用
void PORTC_IRQHandler(void)
{
    PORT_IRQHandler(HW_GPIOC);
}

//系统GPIO中断函数，用户无需使用
void PORTD_IRQHandler(void)
{
    PORT_IRQHandler(HW_GPIOD);
}

//系统GPIO中断函数，用户无需使用
void PORTE_IRQHandler(void)
{
    PORT_IRQHandler(HW_GPIOE);
}

#if (defined(MK70F12))
//系统GPIO中断函数，用户无需使用
void PORTF_IRQHandler(void)
{
    PORT_IRQHandler(HW_GPIOF);
}
#endif
//*************************************************************************************************************

