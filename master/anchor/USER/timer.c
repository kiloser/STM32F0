#include "stm32f0xx.h"

/**************************************
* 函数名   : timer_init
* 描述     : 定时器500ms中断初始化
* 输入参数 : 无
* 返回值   : 无
**************************************/
void timer_init(void)
{   
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;		//定义定时器的结构体变量
	NVIC_InitTypeDef  NVIC_InitStructure;//定义嵌套中断结构体变量
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);//使能TIM3时钟
		
	TIM_TimeBaseStructure.TIM_Prescaler=48000-1;//预分频;定时器3计数频率=48000000/48000=1000Hz
	TIM_TimeBaseStructure.TIM_Period=5200;//计数周期（次数）=10000（即10s中断一次）
	
	TIM_TimeBaseStructure.TIM_ClockDivision=0;//不进行时钟分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;//向上计数
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);//初始化定时器3
	
	TIM_ARRPreloadConfig(TIM3, ENABLE);/*使能预装载*/
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update); /*预先清除中断位*/
	
	TIM_ITConfig(TIM3,TIM_IT_CC3,ENABLE);//开定时器3中断
	TIM_Cmd(TIM3,ENABLE);//使能定时器3
	
	//使能定时器3中断
	NVIC_InitStructure.NVIC_IRQChannel=TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority=0;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}