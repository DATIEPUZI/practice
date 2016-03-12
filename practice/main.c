#include <interrupt.h>

void PTE_6_ISR(uint32_t pinxArray)
{
	GPIO_ITDMAConfig(HW_GPIOE, 6, kGPIO_IT_RisingEdge, false);		
//	printf("KEY:0x%x\r\n",pinxArray);
	if(pinxArray&(1<<6))
	{		
		GPIO_ToggleBit(HW_GPIOA, 14);								//��תP4����״̬��ָʾ�ж�
	}
	GPIO_ITDMAConfig(HW_GPIOE, 6, kGPIO_IT_RisingEdge, true);
}

int main()
{
	SYSTICK_DelayInit();
	
//���ų�ʼ��
	GPIO_QuickInit(HW_GPIOA, 14, kGPIO_Mode_OPP);
	GPIO_QuickInit(HW_GPIOA, 17, kGPIO_Mode_OPP);
	GPIO_QuickInit(HW_GPIOE, 6, kGPIO_Mode_IPU);
	
//ע���жϻص�����
	GPIO_CallbackInstall(HW_GPIOE, PTE_6_ISR);
//�����ж�
	GPIO_ITDMAConfig(HW_GPIOE, 6, kGPIO_IT_RisingEdge, true);
												
	GPIO_WriteBit(HW_GPIOA, 14, 1);									//��ʼʱ�ر�С��P2
	
	while(1)
	{
		GPIO_ToggleBit(HW_GPIOA, 17);								//С��P1�ظ���˸��ָʾϵͳ����
		SYSTICK_DelayMs(500);
	}
}



