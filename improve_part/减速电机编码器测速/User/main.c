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
#include "./tim/bsp_motor_tim.h"
#include "./led/bsp_led.h"
#include "./key/bsp_key.h" 
#include "./motor_control/bsp_motor_control.h"
#include "./usart/bsp_debug_usart.h"
#include "./Encoder/bsp_encoder.h"

/* �����ת���� */
__IO int8_t Motor_Direction = 0;
/* ��ǰʱ���ܼ���ֵ */
__IO int32_t Capture_Count = 0;
/* ��һʱ���ܼ���ֵ */
__IO int32_t Last_Count = 0;
/* ���ת��ת�� */
__IO float Shaft_Speed = 0.0f;
  
/**
  * @brief  ������
  * @param  ��
  * @retval ��
  */
int main(void) 
{
  __IO uint16_t ChannelPulse = PWM_MAX_PERIOD_COUNT/2;
  uint8_t i = 0;
  
  /* HAL���ʼ��*/
  HAL_Init();
	/* ��ʼ��ϵͳʱ��Ϊ168MHz */
	SystemClock_Config();
  /* ����1msʱ��ΪSysTick */
  HAL_InitTick(5);
	/* ��ʼ������GPIO */
	Key_GPIO_Config();
  /* ��ʼ��USART */
  DEBUG_USART_Config();
  
  printf("\r\n��������������������Ұ����ٵ��������������ʾ���򡪡�����������������\r\n");
  
  /* �����ʼ�� */
  motor_init();
  
  /* �����ٶ� */
	set_motor_speed(ChannelPulse);
  
  /* �������ӿڳ�ʼ�� */
	Encoder_Init();
  
	while(1)
	{ 
    /* ɨ��KEY1 */
    if( Key_Scan(KEY1_GPIO_PORT, KEY1_PIN) == KEY_ON)
    {
      /* ����ռ�ձ� */
      set_motor_enable();
			while(1)
			{
				/* ɨ��KEY1 */
				if( Key_Scan(KEY1_GPIO_PORT, KEY1_PIN) == KEY_ON)
				{
					/* ����ռ�ձ� */
					ChannelPulse += PWM_MAX_PERIOD_COUNT/10;
					
					if(ChannelPulse > PWM_MAX_PERIOD_COUNT)
						ChannelPulse = PWM_MAX_PERIOD_COUNT;
					
					set_motor_speed(ChannelPulse);
				}
				
				/* ɨ��KEY2 */
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
    

#if 0//����������,����\���õ��������ʱ����
    /* ɨ��KEY2 */
    if( Key_Scan(KEY2_GPIO_PORT, KEY2_PIN) == KEY_ON)
    {
      set_motor_disable();
    }
	
    
    /* ɨ��KEY5 */
    if( Key_Scan(KEY5_GPIO_PORT, KEY5_PIN) == KEY_ON)
    {
      /* ת������ */
      set_motor_direction( (++i % 2) ? MOTOR_FWD : MOTOR_REV);
    }
#endif
	}
}

/**
  * @brief  SysTick�жϻص�����
  * @param  ��
  * @retval ��
  */
void HAL_SYSTICK_Callback(void)
{
  static uint16_t i = 0;

  i++;
  if(i == 100)/* 100ms����һ�� */
  {
    /* �����ת���� = �������������� */
    Motor_Direction = __HAL_TIM_IS_TIM_COUNTING_DOWN(&TIM_EncoderHandle);
    
    /* ��ǰʱ���ܼ���ֵ = ������ֵ + ����������� * ENCODER_TIM_PERIOD  */
    Capture_Count =__HAL_TIM_GET_COUNTER(&TIM_EncoderHandle) + (Encoder_Overflow_Count * ENCODER_TIM_PERIOD);
    
    /* ת��ת�� = ��λʱ���ڵļ���ֵ / �������ֱܷ��� * ʱ��ϵ��  */
    Shaft_Speed = (float)(Capture_Count - Last_Count) / ENCODER_TOTAL_RESOLUTION * 10 ;

    printf("�������%d\r\n", Motor_Direction);
    printf("��λʱ������Ч����ֵ��%d\r\n", Capture_Count - Last_Count);/* ��λʱ�����ֵ = ��ǰʱ���ܼ���ֵ - ��һʱ���ܼ���ֵ */
    printf("���ת�ᴦת�٣�%.2f ת/�� \r\n", Shaft_Speed);
    printf("��������ת�٣�%.2f ת/�� \r\n", Shaft_Speed/REDUCTION_RATIO);/* �����ת�� = ת��ת�� / ���ٱ� */
    
    /* ��¼��ǰ�ܼ���ֵ������һʱ�̼���ʹ�� */
    Last_Count = Capture_Count;
    i = 0;
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

