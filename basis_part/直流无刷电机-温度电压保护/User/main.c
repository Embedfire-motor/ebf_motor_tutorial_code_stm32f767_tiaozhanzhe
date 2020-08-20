/**
  ******************************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2016-xx-xx
  * @brief   GPIO����--����
  ******************************************************************************
  * @attention
  *
  * ʵ��ƽ̨:Ұ�� STM32F767 ������ 
  * ��̳    :http://www.firebbs.cn
  * �Ա�    :http://firestm32.taobao.com
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

#define TEMP_MAX    80    // �¶����ֵ
#define TEMP_MIN    10    // �¶���Сֵ

/**
  * @brief  ������
  * @param  ��
  * @retval ��
  */
int main(void) 
{
  __IO uint16_t ChannelPulse = PWM_MAX_PERIOD_COUNT/10;
  uint8_t i = 0;
  uint8_t flag = 0;
  uint8_t t_max_count = 0;
  
	/* ��ʼ��ϵͳʱ��Ϊ216MHz */
	SystemClock_Config();
  
  /* HAL ��ʼ�� */
  HAL_Init();
  
	/* ��ʼ������GPIO */
	Key_GPIO_Config();
  
  /* LED �Ƴ�ʼ�� */
  LED_GPIO_Config();
  
  /* ���Դ��ڳ�ʼ�� */
  DEBUG_USART_Config();
  
  /* ADC ��ʼ�� */
  ADC_Init();
  
  printf("Ұ��ֱ����ˢ���������������\r\n");

  /* �����ʼ�� */
  bldcm_init();
	
	while(1)
	{
    /* ɨ��KEY1 */
    if( Key_Scan(KEY1_GPIO_PORT, KEY1_PIN) == KEY_ON)
    {
      /* ʹ�ܵ�� */
      set_bldcm_speed(ChannelPulse);
      set_bldcm_enable();
			
			while(1)
			{
					/* ɨ��KEY3 */
					if( Key_Scan(KEY1_GPIO_PORT, KEY1_PIN) == KEY_ON)
					{
						/* ����ռ�ձ� */
						ChannelPulse += PWM_MAX_PERIOD_COUNT/10;
						
						if(ChannelPulse > PWM_MAX_PERIOD_COUNT)
							ChannelPulse = PWM_MAX_PERIOD_COUNT;
						
						set_bldcm_speed(ChannelPulse);
					}
					
					/* ɨ��KEY4 */
					if( Key_Scan(KEY2_GPIO_PORT, KEY2_PIN) == KEY_ON)
					{
						if(ChannelPulse < PWM_MAX_PERIOD_COUNT/10)
							ChannelPulse = 0;
						else
							ChannelPulse -= PWM_MAX_PERIOD_COUNT/10;

						set_bldcm_speed(ChannelPulse);
					}
					
					if (HAL_GetTick()%50 == 0 && flag == 0)    // ÿ50�����ȡһ���¶ȡ���ѹ
					{
						flag = 1;
						float temp = 0;
						temp = get_ntc_t_val();

						printf("��Դ��ѹ=%0.1fV, NTC=%0.0f��, T=%0.1f��.\r\n", 
									 get_vbus_val(), get_ntc_r_val(), temp);
						
						if (temp < TEMP_MIN || temp > TEMP_MAX)    // �ж��ǲ��ǳ����޶���ֵ
						{
							if (t_max_count++ > 5)    // ����5�γ���
							{
								LED2_ON;
								set_bldcm_disable();
								t_max_count = 0;
								printf("�¶ȳ������ƣ�����ԭ�򣬸�λ���������ԣ�\r\n");
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
  * @brief  System Clock ����
  *         system Clock �������� : 
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
  * @param  ��
  * @retval ��
  */
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  HAL_StatusTypeDef ret = HAL_OK;

  /* ʹ��HSE������HSEΪPLL��ʱ��Դ������PLL�ĸ��ַ�Ƶ����M N P Q 
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
  
  /* ���� OverDrive ģʽ�Դﵽ216MƵ��  */  
  ret = HAL_PWREx_EnableOverDrive();
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }
  
  /* ѡ��PLLCLK��ΪSYSCLK�������� HCLK, PCLK1 and PCLK2 ��ʱ�ӷ�Ƶ���� 
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

