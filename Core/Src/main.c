/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "FreeRTOSConfig.h"
#include "semphr.h"
#include "queue.h"
#include "i2c-lcd.h"
#include <stdio.h>
#include <string.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

//osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void taskreadbutton(void *pvParameters);
static void off_moto1(void *pvParameters);
static void tang_moto1(void *pvParameters);
static void giam_moto1(void *pvParameters);
static void off_moto2(void *pvParameters);
static void tang_moto2(void *pvParameters);
static void giam_moto2(void *pvParameters);
static xSemaphoreHandle xButton1Semphore=NULL;
static xSemaphoreHandle xButton2Semphore=NULL;
static xSemaphoreHandle xButton3Semphore=NULL;
static xSemaphoreHandle xButton4Semphore=NULL;
static xSemaphoreHandle xButton5Semphore=NULL;
static xSemaphoreHandle xButton6Semphore=NULL;
int nhannut1(void);
int nhannut2(void);
int nhannut3(void);
int nhannut4(void);
int nhannut5(void);
int nhannut6(void);
int duty1=0;
int duty2=0;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_I2C1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  lcd_init();
	lcd_put_cur(0,1);
  lcd_send_string("Hello Everyone");
  HAL_Delay(2000);
  lcd_clear ();
	lcd_put_cur(0,5);
  lcd_send_string("DH-BK");
	lcd_put_cur(1,2);
lcd_send_string("Demo - Nhom 4");
  HAL_Delay(2000);
  lcd_clear ();
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_SET);
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,0);
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,0);
  xTaskCreate(taskreadbutton,"readbutton", 256, NULL, 0, NULL );
  xTaskCreate(off_moto1,"Led1", 128, NULL, 0, NULL );
  xTaskCreate(tang_moto1,"Led2", 128, NULL, 0, NULL );
  xTaskCreate(giam_moto1,"Led2", 128, NULL, 0, NULL );
  xTaskCreate(off_moto2,"Led3", 128, NULL, 0, NULL );
  xTaskCreate(tang_moto2,"Led4", 128, NULL, 0, NULL );
  xTaskCreate(giam_moto2,"Led4", 128, NULL, 0, NULL );
  vTaskStartScheduler();
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
//  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
//  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
//  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 5000;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 99;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
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
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA5 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD8 PD9 PD10 PD11 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PD1 PD3 PD5 PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
int nhannut1(){
    if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14)==1) {
      HAL_Delay(20);        //delay chong rung phim
      if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14)==1){
        while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14)==1);//cho phim nha
        return 1;//nut START
        }
     }
     return 0;
}
int nhannut2(){
	if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15)==1) {
	      HAL_Delay(20);        //delay chong rung phim
	      if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15)==1){
	        while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15)==1);//cho phim nha
	        return 1;//nut START
        }
     }
     return 0;
}
int nhannut3(){
    if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_8)==1) {
      HAL_Delay(20);        //delay chong rung phim
      if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_8)==1){
        while (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_8)==1);//cho phim nha
        return 1;//nut START
        }
     }
     return 0;
}
int nhannut4(){
    if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_9)==1) {
      HAL_Delay(20);        //delay chong rung phim
      if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_9)==1){
        while (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_9)==1);//cho phim nha
        return 1;//nut START
        }
     }
     return 0;
}
int nhannut5(){
    if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_10)==1) {
      HAL_Delay(20);        //delay chong rung phim
      if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_10)==1){
        while (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_10)==1);//cho phim nha
        return 1;//nut START
        }
     }
     return 0;
}
int nhannut6(){
    if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_11)==1) {
      HAL_Delay(20);        //delay chong rung phim
      if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_11)==1){
        while (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_11)==1);//cho phim nha
        return 1;//nut START
        }
     }
     return 0;
}
static void taskreadbutton(void *pvParameters)
{
const unsigned char Frequency = 40; //20ms
vSemaphoreCreateBinary(xButton1Semphore);
if(xButton1Semphore!= NULL)
{
 xSemaphoreTake(xButton1Semphore,(portTickType)1);
}
vSemaphoreCreateBinary(xButton2Semphore);
if(xButton2Semphore!= NULL)
{
 xSemaphoreTake(xButton2Semphore,(portTickType)1);
}
vSemaphoreCreateBinary(xButton3Semphore);
if(xButton3Semphore!= NULL)
{
 xSemaphoreTake(xButton3Semphore,(portTickType)1);
}

vSemaphoreCreateBinary(xButton4Semphore);
if(xButton4Semphore!= NULL)
{
 xSemaphoreTake(xButton4Semphore,(portTickType)1);
}

vSemaphoreCreateBinary(xButton5Semphore);
if(xButton5Semphore!= NULL)
{
 xSemaphoreTake(xButton5Semphore,(portTickType)1);
}

vSemaphoreCreateBinary(xButton6Semphore);
if(xButton6Semphore!= NULL)
{
 xSemaphoreTake(xButton6Semphore,(portTickType)1);
}
while(1)
{
	if(nhannut1()==1){
		xSemaphoreGive(xButton1Semphore);
	}
	if(nhannut2()==1){
		xSemaphoreGive(xButton2Semphore);
	}
	if(nhannut3()==1){
		xSemaphoreGive(xButton3Semphore);
	}
	if(nhannut4()==1){
		xSemaphoreGive(xButton4Semphore);
	}
	if(nhannut5()==1){
		xSemaphoreGive(xButton5Semphore);
	}
	if(nhannut6()==1){
		xSemaphoreGive(xButton6Semphore);
	}
 vTaskDelay(Frequency);
 }
}
static void off_moto1(void *pvParameters){
while(1){
		if(xButton2Semphore != NULL){
		if(xSemaphoreTake(xButton2Semphore,((portTickType)1)==pdTRUE)){
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_1,0);
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_3,1);
			duty1 =0;
			lcd_put_cur(0,0);
			lcd_send_string("moto1 : dang off");
			__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,0);
			}
		}
		vTaskDelay(20);
	}
}
static void tang_moto1(void *pvParameters){

	char buffer[20];
while(1){
		if(xButton1Semphore != NULL){
			if(xSemaphoreTake(xButton1Semphore,((portTickType)1)==pdTRUE)){
				HAL_GPIO_WritePin(GPIOD,GPIO_PIN_1,1);
				HAL_GPIO_WritePin(GPIOD,GPIO_PIN_3,0);
			duty1 += 33;
//			lcd_clear();
			lcd_put_cur(0,0);
			if (duty1>99) duty1=99;
			sprintf(buffer,"speed moto1: %d%%",duty1);
			lcd_send_string(buffer);
				__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,duty1);
			}
		}
		vTaskDelay(20);
	}
}
static void giam_moto1(void *pvParameters){
	char buffer[20];
while(1){
		if(xButton3Semphore != NULL){
			if(xSemaphoreTake(xButton3Semphore,((portTickType)1)==pdTRUE)){
			duty1 -= 33;
			lcd_put_cur(0,0);
			if (duty1<=0)
			{	duty1=0;
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_1,0);
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_3,1);
				__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,duty1);
				lcd_send_string("moto1 : dang off");
			}
			else
			{
				sprintf(buffer,"speed moto1: %d%%",duty1);
				lcd_send_string(buffer);
				__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,duty1);
				HAL_GPIO_WritePin(GPIOD,GPIO_PIN_1,1);
				HAL_GPIO_WritePin(GPIOD,GPIO_PIN_3,0);
			}
			}
		}
		vTaskDelay(20);
	}
}
static void off_moto2(void *pvParameters){
while(1){
		if(xButton5Semphore != NULL){
		if(xSemaphoreTake(xButton5Semphore,((portTickType)1)==pdTRUE)){
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_7,1);
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_5,0);
			duty2 =0;
			lcd_put_cur(1,0);
			lcd_send_string("moto2 : dang off");
			__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,0);
			}
		}
		vTaskDelay(20);
	}
}
static void tang_moto2(void *pvParameters){

	char buffer[20];
while(1){
		if(xButton4Semphore != NULL){
			if(xSemaphoreTake(xButton4Semphore,((portTickType)1)==pdTRUE)){
				HAL_GPIO_WritePin(GPIOD,GPIO_PIN_5,1);
				HAL_GPIO_WritePin(GPIOD,GPIO_PIN_7,0);
			duty2 += 33;
			lcd_put_cur(1,0);
			if (duty2>99) duty2=99;
			sprintf(buffer,"speed moto2: %d%%",duty2);
			lcd_send_string(buffer);
				__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,duty2);
			}
		}
		vTaskDelay(20);
	}
}
static void giam_moto2(void *pvParameters){
	char buffer[20];
while(1){
		if(xButton6Semphore != NULL){
			if(xSemaphoreTake(xButton6Semphore,((portTickType)1)==pdTRUE)){

			duty2 -= 33;
			lcd_put_cur(1,0);
			if (duty2<=0)
			{	duty2=0;
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_7,1);
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_5,0);
				__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,duty2);
				lcd_send_string("moto2 : dang off");
			}
			else
			{
				HAL_GPIO_WritePin(GPIOD,GPIO_PIN_5,1);
				HAL_GPIO_WritePin(GPIOD,GPIO_PIN_7,0);
				lcd_put_cur(1,0);
				sprintf(buffer,"speed moto2: %d%%",duty2);
				lcd_send_string(buffer);
				__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,duty2);
			}
			}
		}
		vTaskDelay(20);
	}
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
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
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
