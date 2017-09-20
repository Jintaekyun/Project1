/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
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
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include "stdlib.h"
#include "stdio.h"
#include "string.h"

#define JIN 1
#define J 1
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);                                    
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void EN_Straight(void);
void EN_Right(void);
void EN_Left(void);
void EN_Back(void);
void EN_Wait(void);
void EN_Drop(void);
void EN_Callback(void);   
uint8_t FLAG_RxCplt=1;
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	char msg[100] = "AT\r\n", g_msg[100] = "G";
	char recv[100] = "0";
	//int i=0;
	//char test_1[100] = "WFWRWLWBWDWQ";
	//char comm;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
	//HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,GPIO_PIN_SET);\
	
	#if J
		HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),1000);
		HAL_Delay(1000);
		
		memset(msg,0,sizeof(msg));
		strcpy(msg,"AT+RST\r\n"); //모드 선택
		HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),1000);
		HAL_Delay(1000);
		
		memset(msg,0,sizeof(msg));
		strcpy(msg,"AT+CWMODE=1\r\n"); //모드 선택
		HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),1000);
		HAL_Delay(1000);
		
		memset(msg,0,sizeof(msg));
		strcpy(msg,"AT+CWJAP=\"wifi_k\",\"01010101\"\r\n"); // 공유기 연결
		//strcpy(msg,"AT+CIPSTA?\r\n");
		HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),1000);
		HAL_Delay(1000);

		memset(msg,0,sizeof(msg));
		strcpy(msg,"AT+CWDHCP=1,1\r\n"); //동적할당을 해제하고 고정으로 사용
		HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),1000);
		HAL_Delay(1000);
		

/*
		memset(msg,0,sizeof(msg));
		strcpy(msg,"AT+CIFSR\r\n"); //동적할당을 해제하고 고정으로 사용
		HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),1000);
		HAL_Delay(2000);
*/		
		memset(msg,0,sizeof(msg));
		strcpy(msg,"AT+CIPSTA=\"192.168.0.105\"\r\n"); // 고정 IP 설정
		HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),1000);
		HAL_Delay(1000);
		
		memset(msg,0,sizeof(msg));
		strcpy(msg,"AT+CIPSTART=?\r\n"); //현재 IP상태 확인 빼도 무관
		HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),1000);
		HAL_Delay(1000);
		
		for(;;){
	 	memset(msg,0,sizeof(msg));
		strcpy(msg,"AT+CIPSTART=\"TCP\",\"192.168.0.103\",9000\r\n"); // 이걸로 연결하겠다.
		HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),1000);
		HAL_Delay(1000);
/*		
		memset(msg,0,sizeof(msg));
		strcpy(msg,"AT+CIPSTATUS\r\n"); //상태확인
		HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),1000);
		HAL_Delay(2000);
	*/
		memset(recv,0,sizeof(recv));
		HAL_UART_Receive(&huart2,(uint8_t*)recv,1,5000); //받는다
		HAL_Delay(1000);
		
			if(recv[0] != 0){
				break;
			}
		}
		
		memset(msg,0,sizeof(msg));
		strcpy(msg,"AT+CIPSEND=8\r\n"); //보내는 것
		HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),1000); //몇개까지
		HAL_Delay(1000);
		
		memset(msg,0,sizeof(msg));
		strcpy(msg,"hiserver"); //보내는 메세지
		HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),1000);
		HAL_Delay(3000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
		
		memset(msg,0,sizeof(msg));
		strcpy(msg,"AT+CIPSEND=1\r\n"); //다시 보낸다.
		HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),1000);
		HAL_Delay(2000);
		
		strcpy(recv,"G");
		HAL_UART_Transmit(&huart2,(uint8_t*)recv,strlen(recv),1000);
		HAL_Delay(2000);
	#endif
  while (1)
  {
	
		memset(recv,0,sizeof(recv));	
		HAL_UART_Receive(&huart2,(uint8_t*)recv,11,4000); //받는다
		HAL_Delay(2000);
		
		memset(msg,0,sizeof(msg));
		strcpy(msg,"AT+CIPSEND=1\r\n"); //다시 보낸다.
		HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),1000);
		HAL_Delay(1000);	
		
		memset(g_msg,0,sizeof(g_msg));
		strcpy(g_msg,&recv[10]);		
		HAL_UART_Transmit(&huart2,(uint8_t*)g_msg,strlen(g_msg),1000);
		HAL_Delay(3000);
		

		//memset(msg,0,sizeof(msg));
		//strcpy(msg,recv);
		//strcpy(msg,&msg[10]); //받아온거에서 자리 맞춰줌
		//HAL_UART_Transmit(&huart2,(uint8_t*)msg,1,1000);
		//HAL_Delay(1000);

		//comm = msg[0];
		
		//strcpy(&g_msg[0],&recv[i++%13]);
		
		#ifdef JIN	
		switch(g_msg[0]){
		//switch(test_1[i++]){
			case 'F' : EN_Straight();   //로봇 직진
								 strcpy(g_msg,"G");
								 //HAL_Delay(2000);
								 break;
			
			case 'R' : EN_Right();			//로봇 오른쪽
								 strcpy(g_msg,"G");
							   //HAL_Delay(2000);
								 break;
			
			case 'L' : EN_Left();				//로봇 왼쪽
								 strcpy(g_msg,"G");
								 //HAL_Delay(2000);
								 break;
			
			case 'B' : EN_Back();				//로봇 뒤로가기
								 strcpy(g_msg,"G");
								 //HAL_Delay(2000);
								 break;
			
			case 'W' : EN_Wait();				//로봇 정지
								 strcpy(g_msg,"G");
								 //HAL_Delay(2000);
								 break;
			
			case 'D' : EN_Drop();				//로봇 짐 내리기
								 strcpy(g_msg,"G");
								 //HAL_Delay(2000);
								 break;
			
			case 'C' : EN_Callback();		//로봇 호출
								 strcpy(g_msg,"G");
								 break;
			
			default :  //i = 0;  //그외
								 strcpy(g_msg,"E");
								 break;
		}
		#endif
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

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
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

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 80;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 300;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim1);

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 80;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 100;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.Pulse = 0;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim2);

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
        * Free pins are configured automatically as Analog (this feature is enabled through 
        * the Code Generation settings)
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 PA6 PA7 
                           PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7 
                          |GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10 
                           PB12 PB13 PB14 PB15 
                           PB7 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10 
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15 
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB5 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */


void EN_Straight(void){
	int bu1=0, pinread1 = 0;
	int Encoder_count=800;
	TIM1->CCR1 = 700; //PA 08
	TIM1->CCR2 = 1000; //PA 09
	TIM1->CCR3 = 700; //PA 10
	TIM1->CCR4 = 1000; //PA 11
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);  
	
	for(;;){
		bu1 = pinread1;
		pinread1 = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_5);
		
		if(pinread1==1 && bu1==0){
			Encoder_count--;
		}
		if(Encoder_count == 0)
		{
			TIM1->CCR1 = 0; //PA 08
			TIM1->CCR2 = 0; //PA 09
			TIM1->CCR3 = 0; //PA 10
			TIM1->CCR4 = 0; //PA 11
			HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1);
			HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_2);
			HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_3);
			HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_4);
			break;
		}
	}	
}

void EN_Right(void){
	int bu1=0, pinread1 = 0;
	int Encoder_count = 400;  //history 390 down 
	TIM1->CCR1 = 1000; //PA 08
	TIM1->CCR2 = 700; //PA 09
	TIM1->CCR3 = 700; //PA 10
	TIM1->CCR4 = 1000; //PA 11
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);  
	
	for(;;){
		bu1 = pinread1;
		pinread1 = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_5);
		
		if(pinread1==1 && bu1==0){
			Encoder_count--;
		}
		if(Encoder_count == 0)
		{
			TIM1->CCR1 = 0; //PA 08
			TIM1->CCR2 = 0; //PA 09
			TIM1->CCR3 = 0; //PA 10
			TIM1->CCR4 = 0; //PA 11
			HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1);
			HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_2);
			HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_3);
			HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_4);
			break;
		}
	}	
}


void EN_Left(void){
	int bu1=0, pinread1 = 0;
	int Encoder_count=390;
	TIM1->CCR1 = 700; //PA 08
	TIM1->CCR2 = 1000; //PA 09
	TIM1->CCR3 = 1000; //PA 10
	TIM1->CCR4 = 700; //PA 11
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);  
	
	for(;;){
		bu1 = pinread1;
		pinread1 = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_5);
		
		if(pinread1==1 && bu1==0){
			Encoder_count--;
		}
		if(Encoder_count == 0)
		{
			TIM1->CCR1 = 0; //PA 08
			TIM1->CCR2 = 0; //PA 09
			TIM1->CCR3 = 0; //PA 10
			TIM1->CCR4 = 0; //PA 11
			HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1);
			HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_2);
			HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_3);
			HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_4);
			break;
		}
	}	
}

void EN_Back(void){
		int bu1=0, pinread1 = 0;
	int Encoder_count=850;  //history 750
	TIM1->CCR1 = 1000;
	TIM1->CCR2 = 700;
	TIM1->CCR3 = 1000;
	TIM1->CCR4 = 700;
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);  
	
	for(;;){
		bu1 = pinread1;
		pinread1 = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_5);
		
		if(pinread1==1 && bu1==0){
			Encoder_count--;
		}
		if(Encoder_count == 0)
		{
			TIM1->CCR1 = 0; //PA 08
			TIM1->CCR2 = 0; //PA 09
			TIM1->CCR3 = 0; //PA 10
			TIM1->CCR4 = 0; //PA 11
			HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1);
			HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_2);
			HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_3);
			HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_4);
			break;
		}
	}	
}

void EN_Wait(void){
			HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1);
			HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_2);
			HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_3);
			HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_4);
}


void EN_Drop(void){
		TIM2->CCR1 = 850;
		TIM2->CCR2 = 1000;
		HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
		HAL_Delay(230);
		TIM2->CCR1 = 1000;
		HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_1);
		HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_2);
		
		HAL_Delay(1000);
		TIM2->CCR1 = 1000;
		TIM2->CCR2 = 850;
  	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
		HAL_Delay(230);
		TIM2->CCR2 = 1000;
		HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_1);
		HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_2);
}

void EN_Callback(void){
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
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
