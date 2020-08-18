/**
  ******************************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2016-xx-xx
  * @brief   GPIO输入--按键
  ******************************************************************************
  * @attention
  *
  * 实验平台:野火 STM32F767 开发板 
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :http://firestm32.taobao.com
  *
  ******************************************************************************
  */
  
#include "main.h"
#include "stm32f7xx.h"
#include "./tim/bsp_motor_tim.h"
#include "./led/bsp_led.h"
#include "./key/bsp_key.h" 
#include "./motor_control/bsp_motor_control.h"
#include "./usart/bsp_debug_usart.h"
#include "./Encoder/bsp_encoder.h"

/* 电机旋转方向 */
__IO int8_t Motor_Direction = 0;
/* 当前时刻总计数值 */
__IO int32_t Capture_Count = 0;
/* 上一时刻总计数值 */
__IO int32_t Last_Count = 0;
/* 电机转轴转速 */
__IO float Shaft_Speed = 0.0f;
  
/**
  * @brief  主函数
  * @param  无
  * @retval 无
  */
int main(void) 
{
  __IO uint16_t ChannelPulse = PWM_MAX_PERIOD_COUNT/2;
  uint8_t i = 0;
  
  /* HAL库初始化*/
  HAL_Init();
	/* 初始化系统时钟为168MHz */
	SystemClock_Config();
  /* 配置1ms时基为SysTick */
  HAL_InitTick(5);
	/* 初始化按键GPIO */
	Key_GPIO_Config();
  /* 初始化USART */
  DEBUG_USART_Config();
  
  printf("\r\n——————————野火减速电机编码器测速演示程序——————————\r\n");
  
  /* 电机初始化 */
  motor_init();
  
  /* 设置速度 */
	set_motor_speed(ChannelPulse);
  
  /* 编码器接口初始化 */
	Encoder_Init();
  
	while(1)
	{ 
    /* 扫描KEY1 */
    if( Key_Scan(KEY1_GPIO_PORT, KEY1_PIN) == KEY_ON)
    {
      /* 增大占空比 */
      set_motor_enable();
			while(1)
			{
				/* 扫描KEY1 */
				if( Key_Scan(KEY1_GPIO_PORT, KEY1_PIN) == KEY_ON)
				{
					/* 增大占空比 */
					ChannelPulse += PWM_MAX_PERIOD_COUNT/10;
					
					if(ChannelPulse > PWM_MAX_PERIOD_COUNT)
						ChannelPulse = PWM_MAX_PERIOD_COUNT;
					
					set_motor_speed(ChannelPulse);
				}
				
				/* 扫描KEY2 */
				if( Key_Scan(KEY2_GPIO_PORT, KEY2_PIN) == KEY_ON)
				{
					if(ChannelPulse < PWM_MAX_PERIOD_COUNT/10)
						ChannelPulse = 0;
					else
						ChannelPulse -= PWM_MAX_PERIOD_COUNT/10;
					
					set_motor_speed(ChannelPulse);
				}
			}
    }
    

#if 0//按键数量少,换向\禁用电机功能暂时不用
    /* 扫描KEY2 */
    if( Key_Scan(KEY2_GPIO_PORT, KEY2_PIN) == KEY_ON)
    {
      set_motor_disable();
    }
	
    
    /* 扫描KEY5 */
    if( Key_Scan(KEY5_GPIO_PORT, KEY5_PIN) == KEY_ON)
    {
      /* 转换方向 */
      set_motor_direction( (++i % 2) ? MOTOR_FWD : MOTOR_REV);
    }
#endif
	}
}

/**
  * @brief  SysTick中断回调函数
  * @param  无
  * @retval 无
  */
void HAL_SYSTICK_Callback(void)
{
  static uint16_t i = 0;

  i++;
  if(i == 100)/* 100ms计算一次 */
  {
    /* 电机旋转方向 = 计数器计数方向 */
    Motor_Direction = __HAL_TIM_IS_TIM_COUNTING_DOWN(&TIM_EncoderHandle);
    
    /* 当前时刻总计数值 = 计数器值 + 计数溢出次数 * ENCODER_TIM_PERIOD  */
    Capture_Count =__HAL_TIM_GET_COUNTER(&TIM_EncoderHandle) + (Encoder_Overflow_Count * ENCODER_TIM_PERIOD);
    
    /* 转轴转速 = 单位时间内的计数值 / 编码器总分辨率 * 时间系数  */
    Shaft_Speed = (float)(Capture_Count - Last_Count) / ENCODER_TOTAL_RESOLUTION * 10 ;

    printf("电机方向：%d\r\n", Motor_Direction);
    printf("单位时间内有效计数值：%d\r\n", Capture_Count - Last_Count);/* 单位时间计数值 = 当前时刻总计数值 - 上一时刻总计数值 */
    printf("电机转轴处转速：%.2f 转/秒 \r\n", Shaft_Speed);
    printf("电机输出轴转速：%.2f 转/秒 \r\n", Shaft_Speed/REDUCTION_RATIO);/* 输出轴转速 = 转轴转速 / 减速比 */
    
    /* 记录当前总计数值，供下一时刻计算使用 */
    Last_Count = Capture_Count;
    i = 0;
  }
}



/**
  * @brief  System Clock 配置
  *         system Clock 配置如下 : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 216000000
  *            HCLK(Hz)                       = 216000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 25000000
  *            PLL_M                          = 25
  *            PLL_N                          = 432
  *            PLL_P                          = 2
  *            PLL_Q                          = 9
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 7
  * @param  无
  * @retval 无
  */
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  HAL_StatusTypeDef ret = HAL_OK;

  /* 使能HSE，配置HSE为PLL的时钟源，配置PLL的各种分频因子M N P Q 
	 * PLLCLK = HSE/M*N/P = 25M / 25 *432 / 2 = 216M
	 */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 432;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  
  ret = HAL_RCC_OscConfig(&RCC_OscInitStruct);
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }
  
  /* 激活 OverDrive 模式以达到216M频率  */  
  ret = HAL_PWREx_EnableOverDrive();
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }
  
  /* 选择PLLCLK作为SYSCLK，并配置 HCLK, PCLK1 and PCLK2 的时钟分频因子 
	 * SYSCLK = PLLCLK     = 216M
	 * HCLK   = SYSCLK / 1 = 216M
	 * PCLK2  = SYSCLK / 2 = 108M
	 * PCLK1  = SYSCLK / 4 = 54M
	 */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2; 
  
  ret = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7);
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }  
}

/*********************************************END OF FILE**********************/

