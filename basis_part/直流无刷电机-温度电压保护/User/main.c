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
#include "./led/bsp_led.h"
#include ".\key\bsp_key.h" 
#include ".\bldcm_control\bsp_bldcm_control.h"
#include "./usart/bsp_debug_usart.h"
#include "./adc/bsp_adc.h"

#define TEMP_MAX    80    // 温度最大值
#define TEMP_MIN    10    // 温度最小值

/**
  * @brief  主函数
  * @param  无
  * @retval 无
  */
int main(void) 
{
  __IO uint16_t ChannelPulse = PWM_MAX_PERIOD_COUNT/10;
  uint8_t i = 0;
  uint8_t flag = 0;
  uint8_t t_max_count = 0;
  
	/* 初始化系统时钟为216MHz */
	SystemClock_Config();
  
  /* HAL 初始化 */
  HAL_Init();
  
	/* 初始化按键GPIO */
	Key_GPIO_Config();
  
  /* LED 灯初始化 */
  LED_GPIO_Config();
  
  /* 调试串口初始化 */
  DEBUG_USART_Config();
  
  /* ADC 初始化 */
  ADC_Init();
  
  printf("野火直流无刷电机按键控制例程\r\n");

  /* 电机初始化 */
  bldcm_init();
	
	while(1)
	{
    /* 扫描KEY1 */
    if( Key_Scan(KEY1_GPIO_PORT, KEY1_PIN) == KEY_ON)
    {
      /* 使能电机 */
      set_bldcm_speed(ChannelPulse);
      set_bldcm_enable();
			
			while(1)
			{
					/* 扫描KEY3 */
					if( Key_Scan(KEY1_GPIO_PORT, KEY1_PIN) == KEY_ON)
					{
						/* 增大占空比 */
						ChannelPulse += PWM_MAX_PERIOD_COUNT/10;
						
						if(ChannelPulse > PWM_MAX_PERIOD_COUNT)
							ChannelPulse = PWM_MAX_PERIOD_COUNT;
						
						set_bldcm_speed(ChannelPulse);
					}
					
					/* 扫描KEY4 */
					if( Key_Scan(KEY2_GPIO_PORT, KEY2_PIN) == KEY_ON)
					{
						if(ChannelPulse < PWM_MAX_PERIOD_COUNT/10)
							ChannelPulse = 0;
						else
							ChannelPulse -= PWM_MAX_PERIOD_COUNT/10;

						set_bldcm_speed(ChannelPulse);
					}
					
					if (HAL_GetTick()%50 == 0 && flag == 0)    // 每50毫秒读取一次温度、电压
					{
						flag = 1;
						float temp = 0;
						temp = get_ntc_t_val();

						printf("电源电压=%0.1fV, NTC=%0.0fΩ, T=%0.1f℃.\r\n", 
									 get_vbus_val(), get_ntc_r_val(), temp);
						
						if (temp < TEMP_MIN || temp > TEMP_MAX)    // 判断是不是超过限定的值
						{
							if (t_max_count++ > 5)    // 连续5次超过
							{
								LED2_ON;
								set_bldcm_disable();
								t_max_count = 0;
								printf("温度超过限制！请检查原因，复位开发板在试！\r\n");
								while(1);
							}
						}
					}
					else if (HAL_GetTick()%50 != 0 && flag == 1)
					{
						flag = 0;
					}
					
			}
    }
    

    
    

    
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

