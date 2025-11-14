#include "stm32f7xx_hal.h"
#include "stm32f7xx_eval.h"			 
#include "TIM7.h" 
#include "string.h"	 
#include "math.h"
//uint8_t aRxBuffer;//HAL库使用的串口接收缓冲

///////////////////////////////////////////
//通用定时器7中断初始化
//arr：自动重装值。
//psc：时钟预分频数
//定时器溢出时间计算方法:Tout=((arr+1)*(psc+1))/Ft us.
//Ft=定时器工作频率,单位:Mhz
//这里使用的是定时器7!(定时器7挂在APB1上，时钟为HCLK/2)
void TIM7_Init(uint16_t arr,uint16_t psc)
{  
    __HAL_RCC_TIM7_CLK_ENABLE();                        //使能TIM7时钟
    
    TIM7_Handler.Instance=TIM7;                          //通用定时器7
    TIM7_Handler.Init.Prescaler=psc;                     //分频系数
    TIM7_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;    //向上计数器
    TIM7_Handler.Init.Period=arr;                        //自动装载值
    TIM7_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;//时钟分频因子
    HAL_TIM_Base_Init(&TIM7_Handler);
    HAL_TIM_Base_Start_IT(&TIM7_Handler);   //使能定时器7和定时器7更新中断：TIM_IT_UPDATE   
    
    HAL_NVIC_SetPriority(TIM7_IRQn,1,1);    //设置中断优先级，抢占优先级1，子优先级3
    HAL_NVIC_EnableIRQ(TIM7_IRQn);          //开启ITM7中断  
}
/***************************************************************************************/

//定时器7中断服务函数
void TIM7_IRQHandler(void)
{
    if(__HAL_TIM_GET_FLAG(&TIM7_Handler,TIM_FLAG_UPDATE)!=RESET)    //更新中断
    {
        __HAL_TIM_CLEAR_IT(&TIM7_Handler,TIM_IT_UPDATE);            //清除中断
        USART6_RX_STA|=1<<15;	                                    //标记接收完成
        __HAL_TIM_DISABLE(&TIM7_Handler);                           //关闭定时器7 
    }
}
/***************************************************************************************/











