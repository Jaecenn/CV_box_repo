/**
  ******************************************************************************
  * @file    UART/UART_HyperTerminal_DMA/Src/main.c
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    01-July-2015
  * @brief   This sample code shows how to use UART HAL API to transmit
  *          and receive a data buffer with a communication process based on
  *          DMA transfer.
  *          The communication is done with the Hyperterminal PC application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#define ADCCONVERTEDVALUES_BUFFER_SIZE ((uint32_t) 5)     /* Size of array containing ADC converted values */


#define SAMPLE_RATE 1700
#define RXBUFFERSIZE 4
#define RXBUFFERSIZEADC 5

/** @addtogroup STM32F4xx_HAL_Examples
  * @{
  */

/** @addtogroup UART_Hyperterminal_DMA
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* UART handler declaration */
UART_HandleTypeDef UartHandle;

/* Buffer used for transmission */
uint8_t aTxStartMessage[] = "\n\r ****UART-Hyperterminal communication based on DMA****\n\r Enter 10 characters using keyboard :\n\r";
uint8_t aTxEndMessage[] = "\n\r Example Finished\n\r";

/* Buffer used for reception */
uint8_t aRxBuffer[RXBUFFERSIZE];
uint8_t aTxMessageSign[] = "\n\r";
uint8_t aTxMessageComma[] = ",";
uint8_t aTxMessageTest[] = "OK__OK__";
uint8_t aRxBufferADC[RXBUFFERSIZEADC];
uint32_t waitIter = 0;
uint32_t nuSampleDebug = 0;
uint16_t debugArray[756];


uint32_t freqCheck;
uint32_t sampleRate;
uint8_t onADCs;
uint8_t numOfADC;

ADC_HandleTypeDef    AdcHandle;
ADC_HandleTypeDef    AdcHandle2;
ADC_HandleTypeDef    AdcHandle3;
TIM_HandleTypeDef htim1;
/* Variable used to get converted value */
uint16_t uhADCxConvertedValue = 0;
uint16_t   aADCxConvertedValues[ADCCONVERTEDVALUES_BUFFER_SIZE];

//__IO uint16_t   aTESTValues[ADCCONVERTEDVALUES_BUFFER_SIZE] = {513, 1027};

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Error_Handler(void);
void LED_init(void);
void TIM_init(void);
void LED_blickXtimes(uint8_t Xtimes, uint32_t interval);
void ADC1_init(void);
void ADC2_init(void);
void ADC3_init(void);
void UART_sendOneSample(uint8_t nu_sample);
void StartOfMeasurement(void);
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /* STM32F4xx HAL library initialization:
       - Configure the Flash prefetch, instruction and Data caches
       - Systick timer is configured by default as source of time base, but user 
         can eventually implement his proper time base source (a general purpose 
         timer for example or other time source), keeping in mind that Time base 
         duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
         handled in milliseconds basis.
       - Set NVIC Group Priority to 4
       - Low Level Initialization: global MSP (MCU Support Package) initialization
     */
  HAL_Init();
  
  /* Configure the system clock to 180 MHz */
  SystemClock_Config();



  /* Configure LED1 and LED3 */
  LED_init();
  
  /*##-1- Configure the UART peripheral ######################################*/
  /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
  /* UART configured as follows:
      - Word Length = 8 Bits (7 data bit + 1 parity bit) : 
	                  BE CAREFUL : Program 7 data bits + 1 parity bit in PC HyperTerminal
      - Stop Bit    = One Stop bit
      - Parity      = ODD parity
      - BaudRate    = 9600 baud
      - Hardware flow control disabled (RTS and CTS signals) */
  UartHandle.Instance          = USARTx;
  
  UartHandle.Init.BaudRate     = 256000;
  UartHandle.Init.WordLength   = UART_WORDLENGTH_8B;
  UartHandle.Init.StopBits     = UART_STOPBITS_1;
  UartHandle.Init.Parity       = UART_PARITY_NONE;
  UartHandle.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  UartHandle.Init.Mode         = UART_MODE_TX_RX;
  UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;

  if (HAL_UART_Init(&UartHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
  

 // freqCheck = HAL_RCC_GetPCLK2Freq();


  /*##-2- Start the transmission process #####################################*/
  /* User start transmission data through "TxBuffer" buffer */
 /* if(HAL_UART_Transmit_DMA(&UartHandle, (uint8_t*)aTxStartMessage, TXSTARTMESSAGESIZE)!= HAL_OK)
  {
    Error_Handler();
  }*/

  LED_blickXtimes(1,1000);
  /*##-4- Wait for the end of the transfer ###################################*/
  /*  Before starting a new communication transfer, you need to check the current
      state of the peripheral; if it’s busy you need to wait for the end of current
      transfer before starting a new one.
      For simplicity reasons, this example is just waiting till the end of the
      transfer, but application may perform other tasks while transfer operation
      is ongoing. */
  /*while (HAL_UART_GetState(&UartHandle) != HAL_UART_STATE_READY)
  {
  }*/

 /* TIM_init();
  ADC1_init();*/




 /* ##-3- Put UART peripheral in reception process ###########################
  if (HAL_UART_Receive_DMA(&UartHandle, (uint8_t *)aRxBuffer, RXBUFFERSIZE) != HAL_OK)
  {
    Error_Handler();
  }

  while (HAL_UART_GetState(&UartHandle) != HAL_UART_STATE_READY)
  {
  }*/

  StartOfMeasurement();
  /* Infinite loop */
  while (1)
  {
  }
}

void StartOfMeasurement(void)
{
	uint8_t var;
	if(HAL_UART_Receive_DMA(&UartHandle, &aRxBufferADC, RXBUFFERSIZEADC)!= HAL_OK)
	  {
		  Error_Handler();
	  }

	  while (HAL_UART_GetState(&UartHandle) != HAL_UART_STATE_READY)
		  {
		  if (waitIter == 500000){
		  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_10);
		  waitIter = 0;
		  }
		  waitIter++;
		  }

	  onADCs = 0;
	  if(aRxBufferADC[0]==0x31) onADCs |= 1;
	  if(aRxBufferADC[1]==0x31) onADCs |= (1 << 1);
	  if(aRxBufferADC[2]==0x31) onADCs |= (1 << 2);
	  if(aRxBufferADC[3]==0x31) onADCs |= (1 << 3);
	  if(aRxBufferADC[4]==0x31) onADCs |= (1 << 4);


	  numOfADC = 0;
	  for (var = 0; var < 5; ++var) {
		  if(var == 0)
		  {
			  if ((onADCs & 1) == 1) numOfADC++;
		  }
		  else{
			  if (((onADCs >> var) & 1) == 1) numOfADC++;
		  }
	  }


	  if(HAL_UART_Transmit(&UartHandle, &aTxMessageTest, 8,0xFFFF)!= HAL_OK)
	  	  			  {
	  	  				  Error_Handler();
	  	  			  }

	  	  while (HAL_UART_GetState(&UartHandle) != HAL_UART_STATE_READY)
	  		  {
	  		  }


	if(HAL_UART_Receive_DMA(&UartHandle, &aRxBuffer, RXBUFFERSIZE)!= HAL_OK)
	  {
		  Error_Handler();
	  }
	  //if(HAL_UART_Receive(&UartHandle, &aRxBuffer, 2,0xFFFF)!= HAL_OK)




	  while (HAL_UART_GetState(&UartHandle) != HAL_UART_STATE_READY)
		  {
		  }

	  while(strcmp(aRxBuffer, "0000") == 0);

	  if (strcmp(aRxBuffer, "1000") == 0) sampleRate = 1000;
	  else if (strcmp(aRxBuffer, "1500") == 0) sampleRate = 1500;
	  else if (strcmp(aRxBuffer, "1900") == 0) sampleRate = 1900;
	  else if (strcmp(aRxBuffer, "2000") == 0) sampleRate = 2000;
	  else if (strcmp(aRxBuffer, "2400") == 0) sampleRate = 2400;
	  else if (strcmp(aRxBuffer, "3100") == 0) sampleRate = 3100;
	  else if (strcmp(aRxBuffer, "3500") == 0) sampleRate = 3500;
	  else if (strcmp(aRxBuffer, "4000") == 0) sampleRate = 4000;
	  else if (strcmp(aRxBuffer, "4400") == 0) sampleRate = 4400;
	  else if (strcmp(aRxBuffer, "7000") == 0) sampleRate = 7000;
	  else{
	  sampleRate = 0;
	  }

	  if (sampleRate > 0)
	  {
	  if(HAL_UART_Transmit(&UartHandle, &aTxMessageTest, 8,0xFFFF)!= HAL_OK)
	  			  {
	  				  Error_Handler();
	  			  }

	  while (HAL_UART_GetState(&UartHandle) != HAL_UART_STATE_READY)
		  {
		  }
	  }else{
		 // sampleRate = 1700;

		  while(1)
			  {LED_blickXtimes(1,300);}
	  }

	  TIM_init();
	  ADC1_init();

	  //ADC2_init();
	  //ADC3_init();



	  if(HAL_TIM_Base_Start_IT(&htim1) != HAL_OK)
	  {
	    /* Counter Enable Error */
	    Error_Handler();
	  }

	  /*##-3- Start the conversion process #######################################*/
	/*  if(HAL_ADC_Start_DMA(&AdcHandle, (uint32_t*)&uhADCxConvertedValue, 1) != HAL_OK)
	  {
	    Error_Handler();
	  }*/


	  /* Start ADC conversion on regular group with transfer by DMA */
/*	  if (HAL_ADC_Start_DMA(&AdcHandle,
	                        (uint32_t *)aADCxConvertedValues,
	                        ADCCONVERTEDVALUES_BUFFER_SIZE
	                       ) != HAL_OK)
	  {
	    Error_Handler();
	  }*/
	  nuSampleDebug = 0;
	  	  if (HAL_ADC_Start_DMA(&AdcHandle,
	  	                        (uint32_t *)aADCxConvertedValues,
	  	                        (uint32_t) numOfADC
	  	                       ) != HAL_OK)
	  	  {
	  	    Error_Handler();
	  	  }

}


/*
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{

}*/

/**
  * @brief  Conversion complete callback in non blocking mode
  * @param  AdcHandle : AdcHandle handle
  * @note   This example shows a simple way to report end of conversion, and
  *         you can add your own implementation.
  * @retval None
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* AdcHandle)
{

	/* DEBUG
	if (nuSampleDebug<512)
	{*/



/*	uint8_t high_byte = (uint8_t) (aTESTValues[1] >> 8);
	uint8_t low_byte = (uint8_t) aTESTValues[1] & 0xFF; */

	/*uint8_t high_byte = (uint8_t) (aADCxConvertedValues[0] >> 8);
	uint8_t low_byte = (uint8_t) aADCxConvertedValues[0] & 0xFF;*/
	uint16_t var;
	for (var = 0; var < numOfADC; ++var)
	{
		UART_sendOneSample(var);

		  while (HAL_UART_GetState(&UartHandle) != HAL_UART_STATE_READY)
		  {
		  }

		  if (var<numOfADC-1)
		  {
		  /*if(HAL_UART_Transmit_DMA(&UartHandle, (uint8_t*)aTxMessageComma, 1)!= HAL_OK)
		  {
		    Error_Handler();
		  }*/
			  if(HAL_UART_Transmit(&UartHandle, &aTxMessageComma, 1,0xFFFF)!= HAL_OK)
			  {
				  Error_Handler();
			  }
		  }
	}

	  while (HAL_UART_GetState(&UartHandle) != HAL_UART_STATE_READY)
		  {
		  }


	  if(HAL_UART_Transmit(&UartHandle, &aTxMessageSign, 2,0xFFFF)!= HAL_OK)
	  {
		  Error_Handler();
	  }

	  while (HAL_UART_GetState(&UartHandle) != HAL_UART_STATE_READY)
		  {
		  }


	/*	if(HAL_UART_Receive_IT(&UartHandle, &aRxBuffer, RXBUFFERSIZE)!= HAL_OK)
		  {
			  Error_Handler();
		  }
		if (strcmp(aRxBuffer, "STOP") == 0)
				  {


					  if (HAL_ADC_Stop_DMA(&AdcHandle)!= HAL_OK)
					  {
					    Error_Handler();
					  }

					  StartOfMeasurement();
		}*/


/*	  while (HAL_UART_GetState(&UartHandle) != HAL_UART_STATE_READY)
	  {
	  }

	  if(HAL_UART_Transmit_DMA(&UartHandle, (uint8_t*)aTxMessageSign, sizeof(aTxMessageSign) - 1)!= HAL_OK)
	  {

	    Error_Handler();
	  }

	  while (HAL_UART_GetState(&UartHandle) != HAL_UART_STATE_READY)
	  {
	  }


	  if(HAL_UART_Transmit_DMA(&UartHandle, (uint8_t*)aADCxConvertedValues, sizeof(aADCxConvertedValues) - 1)!= HAL_OK)
	  {
	    Error_Handler();
	  }*/

	  //HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_10);

/* DEBUG*/
/*
	  debugArray[nuSampleDebug] = aADCxConvertedValues[0];
	  nuSampleDebug++;
	}
	else{
		nuSampleDebug = 513;
	}*/
}


void UART_sendOneSample(uint8_t nu_sample)
{
	uint8_t high_byte = (uint8_t) (aADCxConvertedValues[nu_sample] >> 8);
	uint8_t low_byte = (uint8_t) aADCxConvertedValues[nu_sample] & 0xFF;



	while (HAL_UART_GetState(&UartHandle) != HAL_UART_STATE_READY)
	  {
	  }


	  if(HAL_UART_Transmit(&UartHandle, &high_byte, 1,0xFFFF)!= HAL_OK)
	  {
		  Error_Handler();
	  }

	  while (HAL_UART_GetState(&UartHandle) != HAL_UART_STATE_READY)
	  {
	  }


	  if(HAL_UART_Transmit(&UartHandle, &low_byte, 1,0xFFFF)!= HAL_OK)
	  {
		  Error_Handler();
	  }


}


void ADC1_init(void)
{
	ADC_ChannelConfTypeDef sConfig;

	/*##-1- Configure the ADC peripheral #######################################*/
	  AdcHandle.Instance          = ADC1;


	  AdcHandle.Init.ClockPrescaler        = ADC_CLOCKPRESCALER_PCLK_DIV8;
	  AdcHandle.Init.Resolution            = ADC_RESOLUTION12b;
	  AdcHandle.Init.ScanConvMode          = ENABLE;                       /* Sequencer disabled (ADC conversion on only 1 channel: channel set on rank 1) */
	  AdcHandle.Init.ContinuousConvMode    = DISABLE;                        /* Continuous mode disabled to have only 1 conversion at each conversion trig */
	  AdcHandle.Init.DiscontinuousConvMode = ENABLE;                       /* Parameter discarded because sequencer is disabled */
	  AdcHandle.Init.NbrOfDiscConversion   = numOfADC;
	  AdcHandle.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_RISING;        /* Conversion start trigged at each external event */
	  AdcHandle.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_T2_TRGO;
	  AdcHandle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
	  AdcHandle.Init.NbrOfConversion       = numOfADC;
	  AdcHandle.Init.DMAContinuousRequests = ENABLE;
	  AdcHandle.Init.EOCSelection          = ADC_EOC_SEQ_CONV;

	  if (HAL_ADC_Init(&AdcHandle) != HAL_OK)
	  {
	    /* ADC initialization Error */
	    Error_Handler();
	  }


	  if ((onADCs & 1) == 1)
	  {
	  /*##-2- Configure ADC regular channel ######################################*/
	  sConfig.Channel      = ADC_CHANNEL_0;
	  sConfig.Rank         = 1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
	  sConfig.Offset       = 0;


	  if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK)
	  {
	    /* Channel Configuration Error */
	    Error_Handler();
	  }

	  }


	  if (((onADCs >> 1) & 1) == 1)
	  {
	  /*##-2- Configure ADC regular channel ######################################*/
	  sConfig.Channel      = ADC_CHANNEL_1;
	  sConfig.Rank         = 2;
	  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
	  sConfig.Offset       = 0;

	  if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK)
	  {
	    /* Channel Configuration Error */
	    Error_Handler();
	  }

	  }




	  if (((onADCs >> 2) & 1) == 1)
	  {
	  /*##-2- Configure ADC regular channel ######################################*/
	  sConfig.Channel      = ADC_CHANNEL_2;
	  sConfig.Rank         = 3;
	  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
	  sConfig.Offset       = 0;

	  if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK)
	  {
	    /* Channel Configuration Error */
	    Error_Handler();
	  }

	  }


	  if (((onADCs >> 3) & 1) == 1)
	  {
	  /*##-2- Configure ADC regular channel ######################################*/
	  sConfig.Channel      = ADC_CHANNEL_3;
	  sConfig.Rank         = 4;
	  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
	  sConfig.Offset       = 0;

	  if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK)
	  {
	    /* Channel Configuration Error */
	    Error_Handler();
	  }

	  }


	  if (((onADCs >> 4) & 1) == 1)
	  {
	  /*##-2- Configure ADC regular channel ######################################*/
	  sConfig.Channel      = ADC_CHANNEL_4;
	  sConfig.Rank         = 5;
	  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
	  sConfig.Offset       = 0;

	  if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK)
	  {
	    /* Channel Configuration Error */
	    Error_Handler();
	  }

	  }

	  /*##-3- Start the conversion process #######################################*/
	/*  if(HAL_ADC_Start_DMA(&AdcHandle, (uint32_t*)&uhADCxConvertedValue, 1) != HAL_OK)
	  {
	    Error_Handler();
	  }*/
}

void LED_init(void)
{
	 GPIO_InitTypeDef  GPIO_InitStruct;
	/*##-1- Enable GPIOA Clock (to be able to program the configuration registers) */
	  __HAL_RCC_GPIOB_CLK_ENABLE();

	  /*##-2- Configure PA05 IO in output push-pull mode to drive external LED ###*/
	  GPIO_InitStruct.Pin = GPIO_PIN_10;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_PULLUP;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
	  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void LED_blickXtimes(uint8_t Xtimes, uint32_t interval)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
	uint8_t var;
	for (var = 0; var < Xtimes; ++var)
	{
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_10);
		HAL_Delay(interval);
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_10);
		HAL_Delay(interval);
	}
}

void TIM_init(void)
{

	  TIM_ClockConfigTypeDef sClockSourceConfig;
	  TIM_MasterConfigTypeDef sMasterConfig;

	  htim1.Instance = TIM2;
	  htim1.Init.Prescaler = 0;
	  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	  htim1.Init.Period = ((SystemCoreClock/2)/sampleRate)-1;
	  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	  HAL_TIM_Base_Init(&htim1);

	  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	  HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig);

	  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	  HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig);

}
/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 180000000
  *            HCLK(Hz)                       = 180000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 8
  *            PLL_N                          = 360
  *            PLL_P                          = 2
  *            PLL_Q                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 5
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  HAL_StatusTypeDef ret = HAL_OK;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();

  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  RCC_OscInitStruct.PLL.PLLR = 2;
  
  ret = HAL_RCC_OscConfig(&RCC_OscInitStruct);
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }
  
  /* Activate the OverDrive to reach the 180 MHz Frequency */  
  ret = HAL_PWREx_EnableOverDrive();
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  
  
  ret = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* Toggle LED3 for error */
  while(1)
  {
//    BSP_LED_Toggle(LED3);
    HAL_Delay(1000);
  }
}

/**
  * @brief  Tx Transfer completed callback
  * @param  huart: UART handle.
  * @note   This example shows a simple way to report end of DMA Tx transfer, and
  *         you can add your own implementation.
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{

}

/**
  * @brief  Rx Transfer completed callback
  * @param  huart: UART handle
  * @note   This example shows a simple way to report end of DMA Rx transfer, and
  *         you can add your own implementation.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Turn LED3 on: Transfer in reception process is correct */
//  BSP_LED_On(LED3);
}

/**
  * @brief  UART error callbacks
  * @param  huart: UART handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  /* Turn LED3 off: Transfer error in reception/transmission process */
//  BSP_LED_Off(LED3);
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
