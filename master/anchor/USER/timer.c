#include "stm32f0xx.h"

/**************************************
* ������   : timer_init
* ����     : ��ʱ��500ms�жϳ�ʼ��
* ������� : ��
* ����ֵ   : ��
**************************************/
void timer_init(void)
{   
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;		//���嶨ʱ���Ľṹ�����
	NVIC_InitTypeDef  NVIC_InitStructure;//����Ƕ���жϽṹ�����
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);//ʹ��TIM3ʱ��
		
	TIM_TimeBaseStructure.TIM_Prescaler=48000-1;//Ԥ��Ƶ;��ʱ��3����Ƶ��=48000000/48000=1000Hz
	TIM_TimeBaseStructure.TIM_Period=5200;//�������ڣ�������=10000����10s�ж�һ�Σ�
	
	TIM_TimeBaseStructure.TIM_ClockDivision=0;//������ʱ�ӷ�Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;//���ϼ���
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);//��ʼ����ʱ��3
	
	TIM_ARRPreloadConfig(TIM3, ENABLE);/*ʹ��Ԥװ��*/
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update); /*Ԥ������ж�λ*/
	
	TIM_ITConfig(TIM3,TIM_IT_CC3,ENABLE);//����ʱ��3�ж�
	TIM_Cmd(TIM3,ENABLE);//ʹ�ܶ�ʱ��3
	
	//ʹ�ܶ�ʱ��3�ж�
	NVIC_InitStructure.NVIC_IRQChannel=TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority=0;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}