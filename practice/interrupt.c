#include <interrupt.h>

//��ø���ʱ��
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
//����SysTick����ʱ

//��ʼ��SysTickʱ��
void SYSTICK_Init(uint32_t timeInUs)
{
    ///* Set clock source = core clock 
    SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk; 
    fac_us = GetClock(kCoreClock);
    fac_us /= 1000000;
    fac_ms = fac_us * 1000;
    
    SysTick->LOAD = fac_us * timeInUs;
}

//��������ֹͣSysTickʱ��      true ʹ��    false  ֹͣ
void SYSTICK_Cmd(bool NewState)
{
    (true == NewState)?(SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk):(SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk);
}

//������ر�SysTick�ж�     true ʹ��   false  �ر�
void SYSTICK_ITConfig(bool NewState)
{
    (true == NewState)?(SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk):(SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk);
}

//����Systick ����ʱ�����ĳ�ʼ��
void SYSTICK_DelayInit(void)
{
    SYSTICK_Init(1000);         //��ʼ��SysTick                
    SYSTICK_Cmd(true);			//����SysTickʱ��
    SYSTICK_ITConfig(false);	//�ر�SysTick�ж�
}

//΢���ʱ
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

//���뼶��ʱ
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
//GPIO��غ���    ����ֻд�˲��ֵĲ���GPIO�ĺ���
//����GPIO���Ÿ���
void PORT_PinMuxConfig(uint32_t instance, uint8_t pin, PORT_PinMux_Type pinMux)
{
    SIM->SCGC5 |= SIM_GPIOClockGateTable[instance];
    PORT_InstanceTable[instance]->PCR[pin] &= ~(PORT_PCR_MUX_MASK);
    PORT_InstanceTable[instance]->PCR[pin] |=  PORT_PCR_MUX(pinMux);
}

//����GPIO��������ѡ��
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

//�˿����ŵĿ�©״̬���� �û�һ�㲻�ص���	ENABLE   ��������	DISABLE  �رչ���
void PORT_PinOpenDrainConfig(uint32_t instance, uint8_t pin, bool status)
{
    SIM->SCGC5 |= SIM_GPIOClockGateTable[instance];
    (status) ? (PORT_InstanceTable[instance]->PCR[pin] |= PORT_PCR_ODE_MASK):(PORT_InstanceTable[instance]->PCR[pin] &= ~PORT_PCR_ODE_MASK);
}

//��������Ϊ���뻹���������  �û�һ�㲻�ص���		kInpput ���빦��ѡ��	kOutput �������ѡ��
void GPIO_PinConfig(uint32_t instance, uint8_t pin, GPIO_PinConfig_Type mode)
{
    SIM->SCGC5 |= SIM_GPIOClockGateTable[instance];
    (mode == kOutput) ? (GPIO_InstanceTable[instance]->PDDR |= (1 << pin)):(GPIO_InstanceTable[instance]->PDDR &= ~(1 << pin));
}

/*
** @brief  GPIO��ʼ������
** @code
**    //��ʼ��PTB_10Ϊ�������
**    GPIO_InitTypeDef GPIO_InitStruct1;      //����һ���ṹ�����
**    GPIO_InitStruct1.instance = HW_GPIOB;   //ѡ��PORTB�˿�
**    GPIO_InitStruct1.mode = kGPIO_Mode_OPP; //�������
**    GPIO_InitStruct1.pinx = 10;             //ѡ��10����
**    //Ȼ����ó�ʼ��GPIO���� 
**    GPIO_Init(&GPIO_InitStruct1);
** @endcode
** instance:�˿ں�  HW_GPIOA---HW_GPIOE    mode:������ʽ  ��ǰ��Ķ���
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

//GPIO���ٳ�ʼ��
//��Ҫ��instance:�˿ں�  pinx:���ź�  mode:������ʽ
uint8_t GPIO_QuickInit(uint32_t instance, uint32_t pinx, GPIO_Mode_Type mode)
{
    GPIO_InitTypeDef GPIO_InitStruct1;
    GPIO_InitStruct1.instance = instance;
    GPIO_InitStruct1.mode = mode;
    GPIO_InitStruct1.pinx = pinx;
    GPIO_Init(&GPIO_InitStruct1);
    return  instance;
}

//����ָ����������ߵ�ƽ���ߵ͵�ƽ  0 ��    1��   ����Ҫ������Ϊ�����
void GPIO_WriteBit(uint32_t instance, uint8_t pin, uint8_t data)
{
    SIM->SCGC5 |= SIM_GPIOClockGateTable[instance];
    (data) ? (GPIO_InstanceTable[instance]->PSOR |= (1 << pin)):(GPIO_InstanceTable[instance]->PCOR |= (1 << pin));
}

//��תָ�����ŵ�ƽ״̬      ����Ҫ������Ϊ�����                    
void GPIO_ToggleBit(uint32_t instance, uint8_t pin)
{
    GPIO_InstanceTable[instance]->PTOR |= (1 << pin);
}
//*******************************************************************************************************************

//*******************************************************************************************************************
//�ж���غ���
/*
** @brief  ע���жϻص�����
** @param  instance: GPIOģ���ж���ں�
**         @arg HW_GPIOA :A�˿��ж����
**         @arg HW_GPIOB :B�˿��ж����
**         @arg HW_GPIOC :C�˿��ж����
**         @arg HW_GPIOD :D�˿��ж����
**         @arg HW_GPIOE :E�˿��ж����
** @param AppCBFun: �ص�����ָ�����
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
** @brief  ����GPIO�����ж����ͻ���DMA����
** @code
**      //����PORTB�˿ڵ�10����Ϊ�½��ش����ж�
**      GPIO_ITDMAConfig(HW_GPIOB, 10, kGPIO_IT_FallingEdge, true); 
** @endcode
** @param[in]  instance GPIOģ���
**              @arg HW_GPIOA оƬ��PORTA�˿�
**              @arg HW_GPIOB оƬ��PORTB�˿�
**              @arg HW_GPIOC оƬ��PORTC�˿�
**              @arg HW_GPIOD оƬ��PORTD�˿�
**              @arg HW_GPIOE оƬ��PORTE�˿�
** @param[in]  pin �˿��ϵ����ź� 0~31
** @param[in]  config ����ģʽ
**              @arg kGPIO_DMA_RisingEdge DMA�����ش���
**              @arg kGPIO_DMA_FallingEdge DMA�½��ش���
**              @arg kGPIO_DMA_RisingFallingEdge DMA�������½��ض�����
**              @arg kGPIO_IT_Low �͵�ƽ�����ж�
**              @arg kGPIO_IT_RisingEdge �����ش����ж�
**              @arg kGPIO_IT_FallingEdge �½��ش����ж�
**              @arg kGPIO_IT_RisingFallingEdge �������½��ض������ж�
**              @arg kGPIO_IT_High �ߵ�ƽ�����ж�
** @param[in]  status �Ƿ�ʹ��
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
** @brief  GPIO�ж��ܴ��������û�һ������ʹ�ú��޸�
** @param[in]  instance GPIOģ���ж���ں�
**              @arg HW_GPIOA оƬ��PORTA�˿��ж����
**              @arg HW_GPIOB оƬ��PORTB�˿��ж����
**              @arg HW_GPIOC оƬ��PORTC�˿��ж����
**              @arg HW_GPIOD оƬ��PORTD�˿��ж����
**              @arg HW_GPIOE оƬ��PORTE�˿��ж����
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

//ϵͳGPIO�жϺ������û�����ʹ��
void PORTA_IRQHandler(void)
{
    PORT_IRQHandler(HW_GPIOA);
}

//ϵͳGPIO�жϺ������û�����ʹ��
void PORTB_IRQHandler(void)
{
    PORT_IRQHandler(HW_GPIOB);
}

//ϵͳGPIO�жϺ������û�����ʹ��
void PORTC_IRQHandler(void)
{
    PORT_IRQHandler(HW_GPIOC);
}

//ϵͳGPIO�жϺ������û�����ʹ��
void PORTD_IRQHandler(void)
{
    PORT_IRQHandler(HW_GPIOD);
}

//ϵͳGPIO�жϺ������û�����ʹ��
void PORTE_IRQHandler(void)
{
    PORT_IRQHandler(HW_GPIOE);
}

#if (defined(MK70F12))
//ϵͳGPIO�жϺ������û�����ʹ��
void PORTF_IRQHandler(void)
{
    PORT_IRQHandler(HW_GPIOF);
}
#endif
//*************************************************************************************************************

