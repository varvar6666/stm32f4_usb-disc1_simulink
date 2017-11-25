/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

/* USER CODE BEGIN Includes */
#define TX_BUFF_NUM 11


/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint16_t TX_Buff[TX_BUFF_NUM];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void DMA2_Stream0_IRQHandler(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{
  /* USER CODE BEGIN 1 */
    TX_Buff[0] = 0x4637;// "7F"
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
    MX_USB_DEVICE_Init();

  /* USER CODE BEGIN 2 */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN |
                    RCC_AHB1ENR_GPIOBEN |
                    RCC_AHB1ENR_GPIOCEN |
                    RCC_AHB1ENR_GPIODEN |
                    RCC_AHB1ENR_GPIOEEN;
    
    //GPIOD PIN12-15 leds
    GPIOD->MODER |= GPIO_MODE_OUTPUT_PP << PIN12*2 |
                    GPIO_MODE_OUTPUT_PP << PIN13*2 |
                    GPIO_MODE_OUTPUT_PP << PIN14*2 |
                    GPIO_MODE_OUTPUT_PP << PIN15*2;
                    
    //GPIOC PIN0 USB PWR_ON
    GPIOC->MODER |= GPIO_MODE_OUTPUT_PP << PIN0*2;
    
    GPIOA->MODER |= GPIO_MODE_AF_PP << PIN11*2 |
                    GPIO_MODE_AF_PP << PIN12*2;
    GPIOA->PUPDR |= GPIO_NOPULL << PIN11*2 |
                    GPIO_NOPULL << PIN12*2;
    GPIOA->OSPEEDR |= GPIO_SPEED_FREQ_VERY_HIGH << PIN11*2 |
                      GPIO_SPEED_FREQ_VERY_HIGH << PIN12*2;
    GPIOA->AFR[1] |= GPIO_AF10_OTG_FS << (PIN11*4-32) |
                     GPIO_AF10_OTG_FS << (PIN12*4-32);
    
    //Set GPIOA PIN1-3, B0-1, C0-1 as analog in
    GPIOA->MODER |= GPIO_MODE_ANALOG << PIN1*2 |
                    GPIO_MODE_ANALOG << PIN2*2 |
                    GPIO_MODE_ANALOG << PIN3*2;
    GPIOB->MODER |= GPIO_MODE_ANALOG << PIN0*2 |
                    GPIO_MODE_ANALOG << PIN1*2;
    GPIOC->MODER |= GPIO_MODE_ANALOG << PIN1*2;    
    GPIOA->PUPDR |= GPIO_NOPULL <<PIN1*2 |
                    GPIO_NOPULL <<PIN2*2 |
                    GPIO_NOPULL <<PIN3*2;
    GPIOB->PUPDR |= GPIO_NOPULL <<PIN0*2 |
                    GPIO_NOPULL <<PIN1*2;
    GPIOC->PUPDR |= GPIO_NOPULL <<PIN1*2;
    GPIOA->OSPEEDR |= GPIO_SPEED_FREQ_HIGH << PIN1*2 |
                      GPIO_SPEED_FREQ_HIGH << PIN2*2 |
                      GPIO_SPEED_FREQ_HIGH << PIN3*2;
    GPIOB->OSPEEDR |= GPIO_SPEED_FREQ_HIGH << PIN0*2 |
                      GPIO_SPEED_FREQ_HIGH << PIN1*2;
    GPIOC->OSPEEDR |= GPIO_SPEED_FREQ_HIGH << PIN1*2;
    
    //Set GPIOE PIN9,11,13 as TIM1 PWM out
    GPIOE->MODER |= GPIO_MODE_AF_PP << PIN9*2  |
                    GPIO_MODE_AF_PP << PIN11*2 |
                    GPIO_MODE_AF_PP << PIN13*2;
    GPIOE->PUPDR |= GPIO_NOPULL << PIN9*2  |
                    GPIO_NOPULL << PIN11*2 |
                    GPIO_NOPULL << PIN13*2;
    GPIOE->OSPEEDR |= GPIO_SPEED_FREQ_MEDIUM << PIN9*2  |
                      GPIO_SPEED_FREQ_MEDIUM << PIN11*2 |
                      GPIO_SPEED_FREQ_MEDIUM << PIN13*2;
    GPIOE->AFR[1] |= GPIO_AF1_TIM1 << (PIN9*4  - 32) |
                     GPIO_AF1_TIM1 << (PIN11*4 - 32) |
                     GPIO_AF1_TIM1 << (PIN13*4 - 32);   
    
    //GPIOC PIN6 for capture
    GPIOC->MODER |= GPIO_MODE_AF_PP << PIN6*2;
    GPIOA->MODER |= GPIO_MODE_AF_PP << PIN15*2 |
                    GPIO_MODE_AF_PP << PIN0*2;
    GPIOC->PUPDR |= GPIO_NOPULL << PIN6*2;
    GPIOA->PUPDR |= GPIO_NOPULL << PIN15*2 |
                    GPIO_NOPULL << PIN0*2;
    GPIOC->OSPEEDR |= GPIO_SPEED_FREQ_VERY_HIGH << PIN6*2;
    GPIOA->OSPEEDR |= GPIO_SPEED_FREQ_VERY_HIGH << PIN15*2 |
                      GPIO_SPEED_FREQ_VERY_HIGH << PIN0*2;
    GPIOC->AFR[0] |= GPIO_AF3_TIM8 << PIN6*4;
    GPIOA->AFR[0] |= GPIO_AF2_TIM5 << PIN0*4;
    GPIOA->AFR[1] |= GPIO_AF1_TIM2 << (PIN15*4-32);
    
    
    //ADC1 Init
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; //APB2 clk = 84MHz
    ADC1->CR2 = ADC_CR2_ADON | 
                ADC_CR2_DMA |
                ADC_CR2_DDS |
                ADC_CR2_EXTSEL_3 | 
                ADC_CR2_EXTEN_0;// подаем питание на АЦП
    ADC1->CR1 = ADC_CR1_SCAN;// разрешаем прерывания по окончанию преобразования
    ADC1->SMPR2 = ADC_SMPR2_SMP1_0 | //; для всех каналов 3 семпла
                  ADC_SMPR2_SMP2_0 |
                  ADC_SMPR2_SMP3_0 |
                  ADC_SMPR2_SMP8_0 |
                  ADC_SMPR2_SMP9_0;
    ADC1->SMPR1 = ADC_SMPR1_SMP11_0;
    ADC1->SQR1 =  5 << ADC_SQR1_L_Pos; // 6 преобразований (5 + 1)
    ADC1->SQR3 = (1 << ADC_SQR3_SQ1_Pos) | //порядок каналов
                 (2 << ADC_SQR3_SQ2_Pos) |
                 (3 << ADC_SQR3_SQ3_Pos) |
                 (8 << ADC_SQR3_SQ4_Pos) |
                 (9 << ADC_SQR3_SQ5_Pos) |
                (11 << ADC_SQR3_SQ6_Pos);
                   
    //TIM3 for ADC1 Init
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;//84 Mhz
    TIM3->PSC = APB1_TIM/10000-1;//10kHz
    TIM3->ARR = 100;
    TIM3->CR2 = TIM_CR2_MMS_1;
    TIM3->CR1 = TIM_CR1_CEN;

    //DMA Init
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
    DMA2_Stream0->CR = DMA_SxCR_CIRC |
                       DMA_SxCR_MINC | 
                       DMA_SxCR_MSIZE_0 | 
                       DMA_SxCR_PSIZE_0 |
                       DMA_SxCR_TCIE;
    DMA2_Stream0->PAR = (uint32_t) &ADC1->DR;
    DMA2_Stream0->M0AR = (uint32_t) &TX_Buff[2];
    DMA2_Stream0->NDTR = 6;
    DMA2_Stream0->CR |= DMA_SxCR_EN;
    
    //TIM1 PWM mode
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;//APB2_TIM clk=168MHz
    TIM1->PSC = APB2_TIM/10000-1;//10kHz
    TIM1->ARR = 100;//100Hz
    TIM1->CCER = TIM_CCER_CC1E |
                 TIM_CCER_CC2E |
                 TIM_CCER_CC3E;
    TIM1->CCMR1 = TIM_CCMR1_OC1M_1 |
                  TIM_CCMR1_OC1M_2 |
                  TIM_CCMR1_OC2M_1 |
                  TIM_CCMR1_OC2M_2;
    TIM1->CCMR2 = TIM_CCMR2_OC3M_1 |
                  TIM_CCMR2_OC3M_2;
    TIM1->BDTR = TIM_BDTR_MOE;
    TIM1->CCR1 = 20;
    TIM1->CCR2 = 40;
    TIM1->CCR3 = 60;
    TIM1->CR1 = TIM_CR1_CEN;


    //TIM2,5,8 for input capture 1
    RCC->APB2ENR |= RCC_APB2ENR_TIM8EN;//APB2_TIM clk = 100MHz
    TIM8->CCMR1 = TIM_CCMR1_CC1S_0;
    TIM8->CCER = TIM_CCER_CC1P |
                 TIM_CCER_CC1E;
    TIM8->SMCR = TIM_SMCR_SMS_2 |
                 TIM_SMCR_TS_2;
    TIM8->PSC = APB2_TIM/1000000-1;//1Mhz
    TIM8->CR1 = TIM_CR1_CEN;
    
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2->CCMR1 = TIM_CCMR1_CC1S_0;
    TIM2->CCER = TIM_CCER_CC1P |
                 TIM_CCER_CC1E;
    TIM2->SMCR = TIM_SMCR_SMS_2 |
                 TIM_SMCR_TS_2;
    TIM2->PSC = APB1_TIM/1000000-1;//1Mhz
    TIM2->CR1 = TIM_CR1_CEN;

    RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
    TIM5->CCMR1 = TIM_CCMR1_CC1S_0;
    TIM5->CCER = TIM_CCER_CC1P |
                 TIM_CCER_CC1E;
    TIM5->SMCR = TIM_SMCR_SMS_2 |
                 TIM_SMCR_TS_2;
    TIM5->PSC = APB1_TIM/1000000-1;//1Mhz
    TIM5->CR1 = TIM_CR1_CEN;
    
    GPIOD->BSRR |= GPIO_BSRR_BS12;
    GPIOC->BSRR |= GPIO_BSRR_BR0;
    
    HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
    
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */
        
  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/

/* USER CODE BEGIN 4 */
void DMA2_Stream0_IRQHandler(void)
{
    static uint8_t t = 0;
    t++;
    if (t==100) t = 0;
    DMA2->LIFCR = DMA_LIFCR_CHTIF0 |
                  DMA_LIFCR_CTCIF0;
    
    TX_Buff[1]=t;
    TX_Buff[8]  = TIM2->CCR1;
    TX_Buff[9]  = TIM5->CCR1;
    TX_Buff[10] = TIM8->CCR1;
    CDC_Transmit_FS((uint8_t*) TX_Buff,sizeof(TX_Buff));
    
}

void CDC_recive_buff(uint8_t* Buffer)
{
//    static uint16_t i, a;
    
//    a = Buffer[0] | Buffer[1] << 8;
//    i = Buffer[2] | Buffer[3] << 8;
    
    TIM1->CCR1 = Buffer[4] | Buffer[5] << 8; 
    TIM1->CCR2 = Buffer[6] | Buffer[7] << 8; 
    TIM1->CCR3 = Buffer[8] | Buffer[9] << 8; 

}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
