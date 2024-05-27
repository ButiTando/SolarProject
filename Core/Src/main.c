/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "proj_utils.h"
#include <string.h>
#include <stdlib.h>
#include "lcd.h"
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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
DMA_HandleTypeDef hdma_tim2_ch1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint16_t LED2 = GPIO_PIN_14;
uint16_t LED3 = GPIO_PIN_13;
uint16_t LED4 = GPIO_PIN_12;
uint16_t LED5 = GPIO_PIN_10;

uint16_t btnTop = GPIO_PIN_8;
uint16_t btnLeft = GPIO_PIN_6; //This is not set yet
uint16_t btnCenter = GPIO_PIN_7;
uint16_t btnRight = GPIO_PIN_8; //This is not set yet
uint16_t btnBottom = GPIO_PIN_9;

char txBuffer[20];
volatile uint16_t txCounter = 0;
char testMsg[16];
uint8_t studentNumber[] = "&_23540850_*\n";

bounceState btn1bounce;
bounceState btn2bounce;
bounceState btn3bounce;
bounceState btn4bounce;
bounceState btn5bounce;
//progStates state;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

	state = (progStates* )calloc(1,sizeof(progStates));
	tempSense = (tempSensor* )calloc(1,sizeof(tempSensor));
	screen = (lcdScreen* )calloc(1,sizeof(lcdScreen));
	powerMeasure = (powerSensor* )calloc(1,sizeof(powerSensor));
	sDateTime = (dateTime*)calloc(1,sizeof(dateTime));
	calibVals = (calibValue*)calloc(1,sizeof(calibValue));
	tMenu = (timeMenu*)calloc(1, sizeof(timeMenu));

//	Initialising the debounce button states. Assuming that all btns are pulled put this can be checked though ioc generated file.
	btn1bounce.level = 1;
	btn2bounce.level = 1;
	btn3bounce.level = 1;
	btn4bounce.level = 1;
	btn5bounce.level = 1;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  progInit(state);
  tempSensorInit(tempSense);
  screenInit(screen, tempSense, powerMeasure, sDateTime);
  calibIinit(calibVals);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_RTC_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  Lcd_PortType LCD_pin_ports[] = { GPIOB, GPIOC, GPIOA, GPIOA};
  Lcd_PinType LCD_pins[] ={GPIO_PIN_2, GPIO_PIN_8, GPIO_PIN_11, GPIO_PIN_12};

  Lcd_HandleTypeDef lcd;

  lcd = Lcd_create(LCD_pin_ports, LCD_pins, GPIOC, GPIO_PIN_4, GPIOB, GPIO_PIN_1, LCD_4_BIT_MODE);
  HAL_GPIO_WritePin(GPIOB, D4_Pin, GPIO_PIN_SET);

  while(HAL_GetTick() <=100){
	  __NOP();
  }

  HAL_UART_Transmit_IT(&huart2, studentNumber, sizeof(studentNumber)-1);
  int timeCapture = 0;
  int timeCaptured = 0;

#if 0 // This is for demo 3
  displayMode1(&lcd, screen);
  screen->currentDisplay = 1;
#endif

  HAL_RTC_GetTime(&hrtc, &sDateTime->Time, RTC_FORMAT_BIN);
  HAL_RTC_GetDate(&hrtc, &sDateTime->Date, RTC_FORMAT_BIN);
  sDateTime->year = 2024;
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

  setDisplayMode3(screen, sDateTime);
  displayMode1(&lcd, screen);
#if 1
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

//	  Checking for button presses

	  if (state->activeBtn != 0){

		  switch (state->activeBtn){

		  case S1_Pin:

			  if(state->mode == SMODE_DATETIME){
				  tMenu->topButtonPressed = 1;
			  }
			  else{
				  EN_Measure(state, tempSense, screen);
			  }
			  state->activeBtn = 0;
			  break;

		  case S5_Pin:

			  if(state->mode == SMODE_DATETIME){
				  tMenu->bottomButtonPressed = 1;
			  }
			  else{
			  SP_Measure(state, powerMeasure, calibVals, screen);
//			  screen->updateScreen = 0;

			  }
			  state->activeBtn = 0;
			  break;

		  case S2_Pin:

			  state->leftPress ++;

			  switch(state->leftPress){
			  case 3:
				  screen->screenRotate = 1;
				  screen->rortateTick = HAL_GetTick();
				  switchScreen(screen);
				  break;

			  case 4:
				  state->leftPress = 1;
				  screen->screenRotate = 0;
				  switchScreen(screen);
				  break;

			  default:
				  switchScreen(screen);
				  break;
			  }

			  state->activeBtn = 0;
			  break;

		  case S3_Pin:

			  if(state->mode == SMODE_DATETIME){
				  tMenu->itemIndex ++;
			  }
			  else{
				  displayMode3(&lcd, screen);
				  state->mode = SMODE_DATETIME;
				  tMenu->itemIndex ++;
			  }

			  state->activeBtn = 0;
			  break;

		  case S4_Pin:
			  state->mode = SMODE_CALB;
			  state->modeState = MEASURE;
			  state->activeBtn = 0;
			  break;

		  default:
			  state->activeBtn = 0;
			  break;
		  }

	  }

	  else{
		  __NOP();
	  }

//	  Checking for command
	  if (state->commandReady){
		  int isCommandVal = isCommandValid(state);

#if 1
		  switch(isCommandVal){

		  case 1:
			  uart_command_set(state);
			  switch(state->cmdCall){

			  case SMODE_EN:
				  EN_Measure(state,tempSense, screen);
//				  screen->updateScreen = 1;
//				  screen->currentDisplay = 2;
				  break;

			  case SMODE_SP:
				  SP_Measure(state, powerMeasure, calibVals,screen);
//				  screen->updateScreen = 1;
//				  screen->currentDisplay = 1;
				  break;

			  case SMODE_CALB:
				  state->mode = SMODE_CALB;
				  state->modeState = MEASURE;
				  break;

			  default:
				  state->commandReady = 0;
				  resetStr(state->commandMsg,sizeof(state->commandMsg));
				  break;
			  }

		  default:
			  state->commandReady = 0;
			  resetStr(state->commandMsg,sizeof(state->commandMsg));
			  break;
		 }

#endif

#if 0
		  if(isCommandValid(state)){
			  uart_command_set(state);
			  state->commandReady = 0;
			  if(state->mode == SMODE_EN){
				  EN_Measure(state,tempSense, screen);
			  }

			  else if(state->mode == SMODE_SP){
				  __NOP();
			  }

			  else if(state->mode == SMODE_CALB){
				  __NOP();
			  }

			  else{
				  state->commandReady = 0;
				  resetStr(state->commandMsg,sizeof(state->commandMsg));
			  }
		  }

		  else{
			  state->commandReady = 0;
			  resetStr(state->commandMsg,sizeof(state->commandMsg));
		  }

	  }
#endif
	  }else{
		  __NOP();
	  }
//	----End of command check----

//	  Power Measurement

	  if(state->mode == SMODE_SP && state->modeState == MEASURE){
		  if(powerMeasure->mppDutyCycle == 0){
			  powerMeasure->mppDutyCycle = 39;
		  }

		  else{
			  powerMeasure->mppDutyCycle = powerMeasure->mppDutyCycle - 1;
		  }

		  TIM2->CCR1 = powerMeasure->mppDutyCycle;

		  setAdcChannel(ADC_CHANNEL_10, &hadc1);
		  HAL_ADC_Start(&hadc1);
		  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
		  powerMeasure->voltageRaw = HAL_ADC_GetValue(&hadc1);
		  HAL_ADC_Stop(&hadc1);

		  setAdcChannel(ADC_CHANNEL_4, &hadc1);
		  HAL_ADC_Start(&hadc1);
		  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
		  powerMeasure->currentVoltageRaw = HAL_ADC_GetValue(&hadc1);
		  HAL_ADC_Stop(&hadc1);

		  toPVMeasures(powerMeasure, tempSense);
		  screen->updateScreen = 1;
		  setDisplayMode1(screen, powerMeasure);

	  }
	  else{
		  __NOP();
	  }

//	  End of power measurement

//	  Start of Calibration measure

	  if(state->mode == SMODE_CALB && state->modeState == MEASURE){
		  CA_Measure(calibVals, state, powerMeasure, tempSense, screen, &hadc1, &huart2);
	  }
	  else{
		  __NOP();
	  }

//	  End of Calibration measure


//	  Check for response
	  if (state->responseReady){
		  HAL_UART_Transmit_IT(&huart2,(uint8_t*) state->response, state->responseSize);
	  }

	  else{
		  __NOP();
	  }

//	  ----End of response check

//	  LED state
	  if (state->ledFlash == 1){
	  	if(timeCaptured){
	  		if((HAL_GetTick() - timeCapture) >= state->tickLedMax){
	  			  	  HAL_GPIO_TogglePin(GPIOB, state->targetLed);
	  			  	  timeCapture = HAL_GetTick();
	  			  	  state->ticks = 0;
	  		}
	  		else{
	  			  	  state->ticks = state->ticks + 1;
	  		}
	  	}

	  	else{
	  		timeCapture = HAL_GetTick();
	  		timeCaptured = 1;
	  	}

	  }

	  else{
		  HAL_GPIO_WritePin(GPIOB, state->targetLed, GPIO_PIN_SET);
	  	  timeCaptured = 0;
	  	}
//	  ----End of LED status----

//	  ----Begin temp sensor----
	  if(tempSense->digitalBegin){
		  if( (HAL_GetTick()-tempSense->startTime) >= 55){
			  if(tempSense->sampleIndex < 10){
				  setAdcChannel(ADC_CHANNEL_11, &hadc1);
				  HAL_ADC_Start(&hadc1);
				  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
				  tempSense->taRaw = HAL_ADC_GetValue(&hadc1);
				  HAL_ADC_Stop(&hadc1);

//				  Getting lux value
				  setAdcChannel(ADC_CHANNEL_15, &hadc1);
				  HAL_ADC_Start(&hadc1);
				  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
				  tempSense->luxRaw = HAL_ADC_GetValue(&hadc1);
				  HAL_ADC_Stop(&hadc1);


//				  Load sample
				  tempSense->digitalSamples[tempSense->sampleIndex] = tempSense->tspRaw;
				  tempSense->analogSamples[tempSense->sampleIndex] = tempSense->taRaw;
				  tempSense->luxSamples[tempSense->sampleIndex] = tempSense->luxRaw;

//				  Reset
				  tempSense->sampleIndex = tempSense->sampleIndex + 1;
				  tempSense->tspRaw = 0;
				  tempSense->taRaw = 0;
				  tempSense->luxRaw = 0;
				  tempSense->digitalBegin = 0;
				  tempSense->startTime = 0;
			  }
			  else{
				  tempSense->sampleIndex = 0;
				  setAdcChannel(ADC_CHANNEL_11, &hadc1);
				  HAL_ADC_Start(&hadc1);
				  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
				  tempSense->taRaw = HAL_ADC_GetValue(&hadc1);
				  HAL_ADC_Stop(&hadc1);

				  setAdcChannel(ADC_CHANNEL_15, &hadc1);
				  HAL_ADC_Start(&hadc1);
				  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
				  tempSense->luxRaw = HAL_ADC_GetValue(&hadc1);
				  HAL_ADC_Stop(&hadc1);

#if 0 //This is for debugging purposes.
				  taToTemp(tempSense);
				  tspToTemp(tempSense);
#endif
				  //Load sample
				  tempSense->digitalSamples[tempSense->sampleIndex] = tempSense->tspRaw;
				  tempSense->analogSamples[tempSense->sampleIndex] = tempSense->taRaw;
				  setAdcChannel(ADC_CHANNEL_11, &hadc1);
				  HAL_ADC_Start(&hadc1);
				  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
				  tempSense->taRaw = HAL_ADC_GetValue(&hadc1);
				  HAL_ADC_Stop(&hadc1);

				  tempSense->tspRaw = 0;
				  tempSense->taRaw = 0;
				  tempSense->luxRaw = 0;
				  tempSense->digitalBegin = 0;
				  tempSense->startTime = 0;
			  }
		  }
	  }
//	  ----End of temp sensor----

//	  ----Start of screen----

	  if(screen->screenRotate == 1){
		  if((HAL_GetTick()-screen->rortateTick) >= 2000){
		  		  switchScreen(screen);
		  		  screen->rortateTick = HAL_GetTick();
		  }
		  else{
			  __NOP();
		  }
	  }

	  else{
		  __NOP();
	  }


	  if(screen->updateScreen == 1 && screen->screenClear == 0){

		switch(screen->currentDisplay){
			case 1:
				displayMode1(&lcd, screen);
				screen->updateScreen = 0;
				break;

			case 2:
				displayMode2(&lcd, screen);
				screen->updateScreen = 0;
				break;

			case 3:
				displayMode3(&lcd, screen);
				screen->updateScreen = 0;
				break;

			default:
				__NOP();
				break;

		  }
#if 0
		  if(screen->currentDisplay == 1){
			  displayMode1(&lcd, screen);
		  }

		  else if(screen->currentDisplay == 2){
			  displayMode2(&lcd, screen);
		  }

		  else{
			  __NOP();
		  }
		  screen->updateScreen = 0;
#endif


	  }

	  else if(screen->screenClear == 1){
		  Lcd_clear(&lcd);
		  screen->screenClear = 0;
	  }

	  else{
		  __NOP();
	  }


//	  ----Start of screen----

//	  ----Start of Date Time----
	  if(state->mode == SMODE_DATETIME){

		  RTC_DateTypeDef sDate = sDateTime->Date;
		  RTC_TimeTypeDef sTime = sDateTime->Time;

		  switch(tMenu->itemIndex){

		  case DAY:

			  if(tMenu->topButtonPressed == 1){
				  sDate.Date = sDate.Date + 1;
				  sDateTime->Date = sDate;

			  }

			  else if(tMenu->bottomButtonPressed == 1){
				  sDate.Date = sDate.Date - 1;
				  sDateTime->Date = sDate;
			  }
			  HAL_RTC_SetDate(&hrtc, &sDateTime->Date, RTC_FORMAT_BIN);
			  break;

		  case MONTH:

			  if(tMenu->topButtonPressed == 1){
				  sDate.Month = sDate.Month + 1;
				  sDateTime->Date = sDate;
			  }

			  else if(tMenu->bottomButtonPressed == 1){
				  sDate.Month = sDate.Month - 1;
				  sDateTime->Date = sDate;
			  }
			  HAL_RTC_SetDate(&hrtc, &sDateTime->Date, RTC_FORMAT_BIN);
			  break;

		  case YEAR:

			  if(tMenu->topButtonPressed == 1){
				  sDateTime->year ++;
			  }

			  else if(tMenu->bottomButtonPressed == 1){
				  sDateTime->year --;
			  }
			  HAL_RTC_SetDate(&hrtc, &sDateTime->Date, RTC_FORMAT_BIN);
			  break;

		  case HOUR:
			  if(tMenu->topButtonPressed == 1){
					  sTime.Hours ++;
					  sDateTime->Time = sTime;
			  }

			  else if(tMenu->bottomButtonPressed == 1){
				  	  sTime.Hours --;
					  sDateTime->Time = sTime;
			  }
			  HAL_RTC_SetTime(&hrtc, &sDateTime->Time, RTC_FORMAT_BIN);
			  break;

		  case MIN:
			  if(tMenu->topButtonPressed == 1){
					  sTime.Minutes ++;
					  sDateTime->Time = sTime;
			  }

			  else if(tMenu->bottomButtonPressed == 1){
				  	  sTime.Minutes --;
					  sDateTime->Time = sTime;
			  }
			  HAL_RTC_SetTime(&hrtc, &sDateTime->Time, RTC_FORMAT_BIN);
			  break;

		  case SEC:
			  if(tMenu->topButtonPressed == 1){
					  sTime.Seconds ++;
					  sDateTime->Time = sTime;
			  }

			  else if(tMenu->bottomButtonPressed == 1){
				  	  sTime.Seconds --;
					  sDateTime->Time = sTime;
			  }
			  HAL_RTC_SetTime(&hrtc, &sDateTime->Time, RTC_FORMAT_BIN);
			  break;

		  case SET_DATE_TIME:

			  tMenu->timeSet = 1;
			  tMenu->itemIndex = 1;
			  break;

		  default:
			  __NOP();

		  }
#if 0
		  HAL_RTC_SetTime(&hrtc, &sDateTime->Time, RTC_FORMAT_BIN);
		  HAL_RTC_SetDate(&hrtc, &sDateTime->Date, RTC_FORMAT_BIN);
#endif
		  if(tMenu->timeSet == 1 ){
			  HAL_RTC_GetTime(&hrtc, &sDateTime->Time, RTC_FORMAT_BIN);
			  HAL_RTC_GetDate(&hrtc, &sDateTime->Date, RTC_FORMAT_BIN);
			  setDisplayMode3(screen, sDateTime);
			  tMenu->topButtonPressed = 0;
			  tMenu->bottomButtonPressed = 0;
			  screen->updateScreen = 1;
		  }

		  else{
			  tMenu->topButtonPressed = 0;
			  tMenu->bottomButtonPressed = 0;
			  HAL_RTC_GetTime(&hrtc, &sDateTime->Time, RTC_FORMAT_BIN);
			  HAL_RTC_GetDate(&hrtc, &sDateTime->Date, RTC_FORMAT_BIN);
			  screen->updateScreen = 1;
			  setDisplayMode3(screen, sDateTime);
		  }
	  }

	  else{
		  __NOP();
	  }

	  if(tMenu->timeSet == 1){
		  HAL_RTC_GetTime(&hrtc, &sDateTime->Time, RTC_FORMAT_BIN);
		  HAL_RTC_GetDate(&hrtc, &sDateTime->Date, RTC_FORMAT_BIN);
		  setDisplayMode3(screen, sDateTime);
		  screen->updateScreen = 1;
	  }
	  else{
		  __NOP();
	  }


//	  ----End of screen----


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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x16;
  sTime.Minutes = 0x20;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_THURSDAY;
  sDate.Month = RTC_MONTH_SEPTEMBER;
  sDate.Date = 0x19;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
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
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 83;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 39;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 42-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 25999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_9B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_ODD;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|Data6_Pin|Data7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, RS_Pin|Data5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, EN_Pin|Data4_Pin|D5_Pin|D4_Pin
                          |D3_Pin|D2_Pin|RNW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : S3_Pin S1_Pin S5_Pin S4_Pin */
  GPIO_InitStruct.Pin = S3_Pin|S1_Pin|S5_Pin|S4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : RS_Pin Data5_Pin */
  GPIO_InitStruct.Pin = RS_Pin|Data5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : EN_Pin Data4_Pin RNW_Pin */
  GPIO_InitStruct.Pin = EN_Pin|Data4_Pin|RNW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : D5_Pin D4_Pin D3_Pin D2_Pin */
  GPIO_InitStruct.Pin = D5_Pin|D4_Pin|D3_Pin|D2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Data6_Pin Data7_Pin */
  GPIO_InitStruct.Pin = Data6_Pin|Data7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : S2_Pin */
  GPIO_InitStruct.Pin = S2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(S2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){

#if 1
	switch (GPIO_Pin){

	case S1_Pin:
		if(btn1bounce.level == 1){
			btn1bounce.goLowTime = HAL_GetTick();
			btn1bounce.level = 0;
		}
		else {
			btn1bounce.goHighTime = HAL_GetTick();
			btn1bounce.level = 1;
			if( (btn1bounce.goHighTime - btn1bounce.goLowTime) > 25){
				if(state->buttonEnabled){
					state->uartEnabled = !state->uartEnabled;

#if 1
					state->activeBtn = S1_Pin;
#endif

#if 0
					EN_Measure(state, tempSense, screen);
#endif

				}

				else{
					__NOP();
				}
			}
			else{
				__NOP();
			}
		}

		break;

	case S5_Pin:

		if(btn1bounce.level == 1){
					btn1bounce.goLowTime = HAL_GetTick();
					btn1bounce.level = 0;
				}
				else {
					btn1bounce.goHighTime = HAL_GetTick();
					btn1bounce.level = 1;
					if( (btn1bounce.goHighTime - btn1bounce.goLowTime) > 25){
						if(state->buttonEnabled){
							state->uartEnabled = !state->uartEnabled;
							state->activeBtn = S5_Pin;
						}

						else{
							__NOP();
						}
					}
					else{
						__NOP();
					}
				}
		  break;

	case S3_Pin:

		if(btn1bounce.level == 1){
					btn1bounce.goLowTime = HAL_GetTick();
					btn1bounce.level = 0;
				}
				else {
					btn1bounce.goHighTime = HAL_GetTick();
					btn1bounce.level = 1;
					if( (btn1bounce.goHighTime - btn1bounce.goLowTime) > 25){
						if(state->buttonEnabled){
							state->uartEnabled = !state->uartEnabled;
							state->activeBtn = S3_Pin;
						}

						else{
							__NOP();
						}
					}
					else{
						__NOP();
					}
				}
		  break;

	case S4_Pin:

		if(btn1bounce.level == 1){
					btn1bounce.goLowTime = HAL_GetTick();
					btn1bounce.level = 0;
				}
				else {
					btn1bounce.goHighTime = HAL_GetTick();
					btn1bounce.level = 1;
					if( (btn1bounce.goHighTime - btn1bounce.goLowTime) > 25){
						if(state->buttonEnabled){
							state->uartEnabled = !state->uartEnabled;
							state->activeBtn = S4_Pin;
						}

						else{
							__NOP();
						}
					}
					else{
						__NOP();
					}
				}
		  break;

	case S2_Pin:

		if(btn1bounce.level == 1){
					btn1bounce.goLowTime = HAL_GetTick();
					btn1bounce.level = 0;
				}
				else {
					btn1bounce.goHighTime = HAL_GetTick();
					btn1bounce.level = 1;
					if( (btn1bounce.goHighTime - btn1bounce.goLowTime) > 25){
#if 1
						state->activeBtn = S2_Pin;
#endif

#if 0 // This is if we can not control the other button when measuring
						if(state->buttonEnabled){
							state->activeBtn = S2_Pin;
						}

						else{
							__NOP();
						}
#endif
					}
					else{
						__NOP();
					}
				}
		  break;
#if 0
	case DigitalTempSensor_Pin:
		if(tempSense->digitalBegin == 0){
			tempSense->digitalBegin = 1;
			tempSense->startTime = HAL_GetTick();
			tempSense->tspRaw ++;
		}

		else{
			tempSense->tspRaw ++;
		}
		break;
#endif

	default:
		break;

	}

#endif

#if 0

	  if (GPIO_Pin == topCenter){
		  if(btn1bounce.level == 1){
		  			btn1bounce.goLowTime = HAL_GetTick();
		  			btn1bounce.level = 0;
		  		}

		  		else {
		  			btn1bounce.goHighTime = HAL_GetTick();
		  			btn1bounce.level = 1;
		  			if( (btn1bounce.goHighTime - btn1bounce.goLowTime) > 25){
		  				if(state->buttonEnabled){
		  					state->uartEnabled = !state->uartEnabled;
		  					EN_Measure(state, tempSense, screen);
		  				}

		  				else{
		  					__NOP();
		  				}
		  			}
		  			else{
		  				__NOP();
		  			}
		  		}
	  }

	  else if(GPIO_Pin == btnCenter){
		  HAL_GPIO_WritePin(GPIOB, state->targetLed, 0);
		  state->ledFlash = !state->ledFlash;
		  state->targetLed = LED2;
		  state->tickLedMax = D2_TICKS;
		  state->mode = 100;
		  state->ticks = 0;
	  }

	  else if (GPIO_Pin == GPIO_PIN_6){
		  if(tempSense->digitalBegin == 0){
			  tempSense->digitalBegin = 1;
			  tempSense->startTime = HAL_GetTick();
			  tempSense->tspRaw ++;
		  }

		  else{
			  tempSense->tspRaw ++;
		  }
	  }

	  else{
		  __NOP();
	  }

#endif

}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart2){
//    	Should make a cleanup function for response
    	state->responseReady = 0;
    	resetStr(state->commandMsg,sizeof(state->commandMsg));
    	state->responseSize = 0;

//    	Should make a cleanup function for command
    	state->commandReady = 0;
    	resetStr(state->response,sizeof(state->response));
    	state->commandSize = 0;
    	HAL_UART_Receive_IT(&huart2, (uint8_t*)state->commandByte, 1);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){

	if (state->commandByte[0] == '\n'){
		if(state->uartEnabled){
			state->commandMsg[state->commandSize] = state->commandByte[0];
			state->commandSize = 0;
			state->commandReady = 1;
			state->buttonEnabled = !state->buttonEnabled;
			HAL_UART_Receive_IT(&huart2, (uint8_t*)state->commandByte, 1);
		}

		else{
			state->commandSize = 0;
			HAL_UART_Receive_IT(&huart2, (uint8_t*)state->commandByte, 1);
		}

//		HAL_UART_Transmit_IT(&huart2, (uint8_t*)state->commandMsg, state->commandSize);
	}
	else{
		state->commandMsg[state->commandSize] = state->commandByte[0];
		state->commandSize = state->commandSize + 1;
//		HAL_UART_Transmit_IT(&huart2, (uint8_t*)state->commandMsg, state->commandSize);
		HAL_UART_Receive_IT(&huart2, (uint8_t*)state->commandByte, 1);
	}
}
#if 1
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
	switch(htim->Channel){
	case HAL_TIM_ACTIVE_CHANNEL_1:

		if((HAL_GetTick()-tempSense->startTime) >= 55){
			if(tempSense->digitalBegin == 0){
				tempSense->digitalBegin = 1;
				tempSense->startTime = HAL_GetTick();
				tempSense->tspRaw ++;
			}
		}

		else{
			if(tempSense->digitalBegin == 0){
				tempSense->digitalBegin = 1;
				tempSense->startTime = HAL_GetTick();
				tempSense->tspRaw ++;
			}

			else{
				tempSense->tspRaw ++;
			}
		}


		break;
	default:
		__NOP();
		break;
	}
}
#endif

#if 1
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM3){
		if((HAL_GetTick()-tempSense->startTime) >= 55){
			tempSense->digitalBegin = 0;
			tempSense->tspRaw = 0;
		}
		else{
			__NOP();
		}
	}
}
#endif
/* USER CODE END 4 */

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
