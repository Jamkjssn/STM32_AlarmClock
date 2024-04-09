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
#include <stdio.h>
#include <stdbool.h>
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define I2C_ADDR 0x27 // I2C address of the PCF8574
#define RS_BIT 0 // Register select bit
#define EN_BIT 2 // Enable bit
#define BL_BIT 3 // Backlight bit
#define D4_BIT 4 // Data 4 bit
#define D5_BIT 5 // Data 5 bit
#define D6_BIT 6 // Data 6 bit
#define D7_BIT 7 // Data 7 bit
#define LCD_ROWS 2 // Number of rows on the LCD
#define LCD_COLS 16 // Number of columns on the LCD


// Uart Macros
#define UART_DELAY 100 // wait max of 100 ms between frames in message
#define MAX_MESSAGE_SIZE 100 // 100 characters maximum message size
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

// Private variable for LCD
uint8_t backlight_state = 1;

// Private variables for UART
uint8_t message[MAX_MESSAGE_SIZE] = {0}; // char array to store message received
uint8_t response[MAX_MESSAGE_SIZE] = {0}; // char array to store response message
uint8_t uart2_byte; // byte received from UART2
uint8_t buffer_position = 0; // how many bytes received so far

// Private variables for UART Logic
bool setting_time = false;
bool setting_alarm = false;

// Time, Date, and Alarm variable initializations
RTC_TimeTypeDef tempTime;
RTC_DateTypeDef tempDate;
RTC_AlarmTypeDef sAlarm;
RTC_TimeTypeDef sTime;
RTC_DateTypeDef sDate;
bool alarm = false;		// Is an alarm currently going off
bool alarm_cleared = false;		// Was an alarm cleared more recenly than it was set("Get Alarm" cant see cleared status)
int seconds=0;
int minutes=0;
int hour=0;
int day=0;
int month=0;
int year=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void GetAlarmStatus(char*);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Functions for interfacing with the LCD
void lcd_write_nibble(uint8_t nibble, uint8_t rs) {
 uint8_t data = nibble << D4_BIT;
 data |= rs << RS_BIT;
 data |= backlight_state << BL_BIT; // Include backlight state in data
 data |= 1 << EN_BIT;
 HAL_I2C_Master_Transmit(&hi2c1, I2C_ADDR << 1, &data, 1, 100);
 HAL_Delay(1);
 data &= ~(1 << EN_BIT);
 HAL_I2C_Master_Transmit(&hi2c1, I2C_ADDR << 1, &data, 1, 100);
}
void lcd_send_cmd(uint8_t cmd) {
 uint8_t upper_nibble = cmd >> 4;
 uint8_t lower_nibble = cmd & 0x0F;
 lcd_write_nibble(upper_nibble, 0);
 lcd_write_nibble(lower_nibble, 0);
 if (cmd == 0x01 || cmd == 0x02) {
 HAL_Delay(2);
 }
}
void lcd_send_data(uint8_t data) {
 uint8_t upper_nibble = data >> 4;
 uint8_t lower_nibble = data & 0x0F;
 lcd_write_nibble(upper_nibble, 1);
 lcd_write_nibble(lower_nibble, 1);
}

void lcd_init() {
 HAL_Delay(50);
 lcd_write_nibble(0x03, 0);
 HAL_Delay(5);
 lcd_write_nibble(0x03, 0);
 HAL_Delay(1);
 lcd_write_nibble(0x03, 0);
 HAL_Delay(1);
 lcd_write_nibble(0x02, 0);
 lcd_send_cmd(0x28);
 lcd_send_cmd(0x0C);
 lcd_send_cmd(0x06);
 lcd_send_cmd(0x01);
 HAL_Delay(2);
}
void lcd_write_string(char *str) {
 while (*str) {
 lcd_send_data(*str++);
 }
}
void lcd_set_cursor(uint8_t row, uint8_t column) {
 uint8_t address;
 switch (row) {
 case 0:
 address = 0x00;
 break;
 case 1:
 address = 0x40;
 break;
 default:
 address = 0x00;
 }
 address += column;
 lcd_send_cmd(0x80 | address);
}
void lcd_clear(void) {
lcd_send_cmd(0x01);
 HAL_Delay(2);
}
void lcd_backlight(uint8_t state) {
 if (state) {
 backlight_state = 1;
 } else {
 backlight_state = 0;
 }
}
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
  MX_USART2_UART_Init();
  MX_RTC_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  // Start UART2 to receive interrupts
  HAL_UART_Receive_IT(&huart2, &uart2_byte, 1); // put byte from UART2 in "uart2_byte"

  // Set up I2C pull-up resistors
  GPIOB->PUPDR |= 0b01 << (8*2);
  GPIOB->PUPDR |= 0b01 << (9*2);

  // Initialize the LCD and turn on backlight
  lcd_init();
  lcd_backlight(1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  // Re-initialize these here to minimize distance between the time retrieval and display.
	  char time_str[9];
	  char date_str[16];

	  // Retrieve time and date from RTC
	  HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	  HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

	  // Assign time and date to initialized variables
	  seconds = sTime.Seconds;
	  minutes = sTime.Minutes;
	  hour = sTime.Hours;
	  day = sDate.Date;
	  month = sDate.Month;
	  year = sDate.Year;

	  //Format our time and date into a printable string
	  snprintf(time_str, sizeof(time_str), "%02d:%02d:%02d", hour, minutes, seconds);
	  snprintf(date_str, sizeof(date_str), "%02d/%02d/20%02d", month, day, year);

	  // Clear the LCD and then write the updated time and date to it
	  lcd_clear();
	  lcd_write_string(time_str);
	  lcd_set_cursor(1, 0);
	  lcd_write_string(date_str);

	  // Delay for one second between writing
	  HAL_Delay(1000);
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
  hi2c1.Init.Timing = 0x10909CEC;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  RTC_AlarmTypeDef sAlarm = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_12;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
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
  sTime.Hours = 0;
  sTime.Minutes = 0;
  sTime.Seconds = 0;
  sTime.TimeFormat = RTC_HOURFORMAT12_AM;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_SUNDAY;
  sDate.Month = RTC_MONTH_APRIL;
  sDate.Date = 7;
  sDate.Year = 24;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the Alarm A
  */
  sAlarm.AlarmTime.Hours = 1;
  sAlarm.AlarmTime.Minutes = 0;
  sAlarm.AlarmTime.Seconds = 0;
  sAlarm.AlarmTime.SubSeconds = 0;
  sAlarm.AlarmTime.TimeFormat = RTC_HOURFORMAT12_PM;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay = 1;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  HAL_GPIO_WritePin(Alarm_GPIO_Port, Alarm_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Alarm_Pin */
  GPIO_InitStruct.Pin = Alarm_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Alarm_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc) {

	// When the alarm triggers, Turn on the LED
	HAL_GPIO_WritePin(Alarm_GPIO_Port, Alarm_Pin, GPIO_PIN_SET);
	alarm = true;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){

	// Check if interrupt was on line 13
	if(GPIO_Pin == B1_Pin){

		// Toggle the LED and turn off the Alarm
		HAL_GPIO_TogglePin(Alarm_GPIO_Port, Alarm_Pin);

		if(alarm){
			// If the button is pressed after an alarm has gone off reset it and clear the alarm variable
			HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_A);
			HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN);
			alarm = false;
		}

		// Debounce the switch with a delay
		for(int i=0; i<=100; i++);
	}

}

// UART communication section
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
 // Check if byte received was on UART2 (from laptop)
 if (huart == &huart2){
     // If uart2_byte isn't \r, \n, or \0, it means the message isn't over yet
     if ((uart2_byte != '\r') && (uart2_byte != '\n') && (uart2_byte != '\0')){

   	  // Add uart2_byte to the message
         message[buffer_position] = uart2_byte;
         buffer_position++;

     } else {
    	 // Code for interpreting commands

    	 // First check for special setting cases
    	 if(setting_time && (sizeof(message) > 15)){
    		 // Input should be Current Time

    		 // Initialize temporary variables
    		 int year;
    		 int month;
    		 int day;
    		 int hour;
    		 int minute;
    		 int second;

    		 // Cast data from "message" onto our variables
			 sscanf(message, "20%d-%d-%d %d:%d:%d", &year, &month, &day, &hour, &minute, &second);

			 // Account for AM/PM hours
			 if (hour >= 12){
				 tempTime.Hours = (hour-12);
				 tempTime.TimeFormat = 1;
			 }
			 else{
				 tempTime.Hours = hour;
				 tempTime.TimeFormat = 0;
			 }


			 // Finish setting Time and Date variables into their struct equivalent
			 tempDate.Year = year;
			 tempDate.Month = month;
			 tempDate.Date = day;
			 tempTime.Minutes = minute;
			 tempTime.Seconds = second;

			 // Write to the RTC to change time/date
			 HAL_RTC_SetTime(&hrtc, &tempTime, RTC_FORMAT_BIN);
			 HAL_RTC_SetDate(&hrtc, &tempDate, RTC_FORMAT_BIN);

			 // Set response messge
			 strncpy((char*)response, "The time was successfully set ", MAX_MESSAGE_SIZE);

			 // Clear time setting special case
			 setting_time = false;
    	 }
    	 else if (setting_alarm && (sizeof(message) > 10)){
			 // Input is Alarm time

    		 // Initialize temporary variables
    		 int hour;
    		 int minute;
    		 int second;

    		 // Cast message data to the variables
			 sscanf(message, "%d:%d:%d", &hour, &minute, &second);

			 // Adjust for AM/PM
			 if (hour >= 12){
				 tempTime.Hours = (hour-12);
				 tempTime.TimeFormat = 1;
			 }
			 else{
				 tempTime.Hours = hour;
				 tempTime.TimeFormat = 0;
			 }

			 // Set remaining variables to the temporary time struct
			 tempTime.Minutes = minute;
			 tempTime.Seconds = second;

			 //Set the time and mask of the alarm
			 sAlarm.AlarmTime = tempTime;
			 sAlarm.AlarmMask = RTC_ALRMAR_MSK4;
			 sAlarm.Alarm = RTC_ALARM_A;

			 // Activate the created alarm with interrupt enabled
			 HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN);

			 // Set the respond message to be returned
			 strncpy((char*)response, "The alarm was successfully set ", MAX_MESSAGE_SIZE);

			 // Clear the setting_alarm special case as well as cleared_alarm case
			 setting_alarm = false;
			 alarm_cleared = false;
    	 }

    	 else{
    		 // No data message means check for command

			 // Check message
			 if(strcmp((char*)message, "Set Time") == 0) {
				 // Set the special time setting case to true
				 setting_time = true;

				 // Request time in specific format
				 strncpy((char*)response, "Ready to set time. Input the current time with the following format: \"YYYY-MM-DD HH:MM:SS\" ", MAX_MESSAGE_SIZE);
			 }
			 else if(strcmp((char*)message, "Set Alarm") == 0) {
				 // Set the special alarm setting case to true
				 setting_alarm = true;

				 // Request time in specific format
				 strncpy((char*)response, "Ready to set an Alarm. Input the alarm time with the following format: \"HH:MM:SS\" ", MAX_MESSAGE_SIZE);
			 }
			 else if(strcmp((char*)message, "Get Alarm") == 0) {
				 // Retrieve current alarm status
				 char alarm_status[50];
				 GetAlarmStatus(alarm_status);

				 // Send current alarm status as response
				 strncpy((char*)response, alarm_status, MAX_MESSAGE_SIZE);
			 }
			 else if(strcmp((char*)message, "Clear Alarm") == 0) {
				 // Clear the currently set alarm
				 HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_A);

				 // Set program local alarm_cleared variable so that "Get Alarm" command will respond correctly
				 alarm_cleared = true;

				 // Send clear success response
				 strncpy((char*)response, "Your alarm has been cleared " , MAX_MESSAGE_SIZE);
			 }
			 else{
				 // Send Unrecognized command for anything else
				 strncpy((char*)response, "Unrecognized command ", MAX_MESSAGE_SIZE);
			 }
    	 }

		 // Transmit response to Laptop
		 HAL_UART_Transmit(&huart2, (uint8_t*)response, strlen(response), UART_DELAY);

		 // Zero out message array and response array
         memset(message,  0, sizeof(message));
         memset(response, 0, sizeof(response));
         buffer_position = 0;
     }

	 // Restart UART2's receive interrupt to wait for next byte from laptop
	 HAL_UART_Receive_IT(&huart2, &uart2_byte, 1); //start next byte receive interrupt
 }
}


void GetAlarmStatus(char* alarm_status){
	// Create the string version of alarm time
	int hour = sAlarm.AlarmTime.Hours;
	int minutes = sAlarm.AlarmTime.Minutes;
	int seconds = sAlarm.AlarmTime.Seconds;
	char next_alarm[50];
	snprintf(next_alarm, sizeof(next_alarm), "Your next alarm is at %02d:%02d:%02d ", hour, minutes, seconds);

	// Check if the alarm is in the future
    if ((sAlarm.AlarmTime.Hours < sTime.Hours) ||
        (sAlarm.AlarmTime.Hours == sTime.Hours && sAlarm.AlarmTime.Minutes < sTime.Minutes) ||
        (sAlarm.AlarmTime.Hours == sTime.Hours && sAlarm.AlarmTime.Minutes == sTime.Minutes && sAlarm.AlarmTime.Seconds < sTime.Seconds)||(alarm_cleared)) {
    	// It is not in the future, send no future alarm message
		strcpy(alarm_status, "You don't have a future alarm set for today. ");
    }
	else{
		// Alarm is set for a future time, send according message
    	strcpy(alarm_status, next_alarm);
	}
}
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
