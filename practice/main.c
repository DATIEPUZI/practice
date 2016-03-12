#include <interrupt.h>

void PTE_6_ISR(uint32_t pinxArray)
{
	GPIO_ITDMAConfig(HW_GPIOE, 6, kGPIO_IT_RisingEdge, false);		
//	printf("KEY:0x%x\r\n",pinxArray);
	if(pinxArray&(1<<6))
	{		
		GPIO_ToggleBit(HW_GPIOA, 14);								//翻转P4亮灭状态，指示中断
	}
	GPIO_ITDMAConfig(HW_GPIOE, 6, kGPIO_IT_RisingEdge, true);
}

int main()
{
	SYSTICK_DelayInit();
	
//引脚初始化
	GPIO_QuickInit(HW_GPIOA, 14, kGPIO_Mode_OPP);
	GPIO_QuickInit(HW_GPIOA, 17, kGPIO_Mode_OPP);
	GPIO_QuickInit(HW_GPIOE, 6, kGPIO_Mode_IPU);
	
//注册中断回调函数
	GPIO_CallbackInstall(HW_GPIOE, PTE_6_ISR);
//配置中断
	GPIO_ITDMAConfig(HW_GPIOE, 6, kGPIO_IT_RisingEdge, true);
												
	GPIO_WriteBit(HW_GPIOA, 14, 1);									//开始时关闭小灯P2
	
	while(1)
	{
		GPIO_ToggleBit(HW_GPIOA, 17);								//小灯P1重复闪烁，指示系统运行
		SYSTICK_DelayMs(500);
	}
}



