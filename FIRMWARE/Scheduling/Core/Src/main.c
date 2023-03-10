/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <lcd.h>
#include "ads1115.h"
#include <stdlib.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
	double dbSensorStm;
	double dbSensorAds;
	uint8_t ucSensorStm[10];
	uint8_t ucSensorAds[10];
} sensorDisHandle_t;

typedef struct {
	uint8_t ucCursor;
	bool bCfg;
	double dbVal[2];
	uint8_t ucVal[2][10];
} cfgDisHandle_t;

typedef struct {
	uint8_t ucPage;
	sensorDisHandle_t sensorDis;
	cfgDisHandle_t cfgDis;
} lcdDisHandle_t;

typedef struct {
	uint8_t ucType;
	uint8_t ucVal[5];
} uartCfgHandle_t;

typedef enum {
	BUT0_BIT,
	BUT1_BIT,
	BUT2_BIT,
	BUT3_BIT,
	NONE
} push_button_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define START_BYTE 1
#define END_BYTE 2
#define ERROR_FRAME 3
#define UART_BUFFER_LEN 20
#define LED_ON 0
#define LED_OFF 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static const char *TAG = "FREERTOS.c";

lcdDisHandle_t lcdDis = {
	.ucPage = 0,
	.sensorDis = {0},
	.cfgDis = {
		.ucCursor = 0,
		.bCfg = false,
		.dbVal[0] = 1000,
		.dbVal[1] = 25.0,
	},
};

push_button_t push_button = NONE;
char cVal[2][20];
uint8_t ucDataAds[3] = {0};

uint8_t n = 1;
uint8_t max_n = 20;

unsigned long previous_time_sensor_task;
unsigned long previous_time_lcd_task;
unsigned long previous_time_general_task;
unsigned long previous_time_count;
unsigned long current_time_count;
unsigned long i_log_time;
char log_time[10] = {0};

uartCfgHandle_t uartCfg = {0};
uint8_t ucRxData;
uint8_t ucRxBuffer[UART_BUFFER_LEN] = {0};
uint8_t ucRxFlag = START_BYTE;
uint8_t ucRxCnt = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void reverse(char *str, int len)
{
    int i = 0, j = len - 1, temp;
    while (i < j)
    {
        temp = str[i];
        str[i] = str[j];
        str[j] = temp;
        i++;
        j--;
    }
}

int intToStr(int x, char str[], int d)
{
    int i = 0;
    if(x == 0)
        str[i++] = '0';

    while (x)
    {
        str[i++] = (x % 10) + '0';
        x = x / 10;
    }

    while (i < d)
        str[i++] = '0';

    reverse(str, i);
    str[i] = '\0';
    return i;
}

void ftoa(double n, char* res, int afterpoint)
{
    int ipart = (int)n;
    double fpart = n - (double)ipart;
    int i = intToStr(ipart, res, 0);
    if (afterpoint != 0)
    {
        res[i] = '.';
        fpart = fpart * pow(10, afterpoint);
        intToStr((int)fpart, res + i + 1, afterpoint);
    }
}

void LOG(const char *TAG, char *data)
{
	char data_log[100] = {0};
	sprintf(data_log, "\r\n%s: %s\r\n", TAG, data);
	HAL_UART_Transmit(&huart2, (uint8_t *)data_log, strlen(data_log), 1000);
}

void lcd_task(void);
void sensor_task(void);
void general_task(void);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
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
	void (*stt_task_point_array[]) (void) = {sensor_task, general_task, lcd_task};
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
  MX_ADC1_Init();
  MX_I2C2_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  /* Setup Sensor Task */
  ads_init();

  /* Setup LCD Task */
  lcd_init();
  lcd_clear();
  ftoa(lcdDis.cfgDis.dbVal[0] / 1000, (char *)lcdDis.cfgDis.ucVal[0], 1);
  ftoa(lcdDis.cfgDis.dbVal[1], (char *)lcdDis.cfgDis.ucVal[1], 1);

  /* Setup General Task */
  HAL_UART_Receive_IT(&huart2, &ucRxData, 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  previous_time_lcd_task = HAL_GetTick();
//  previous_time_sensor_task = HAL_GetTick();
  previous_time_general_task = HAL_GetTick();
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	if (HAL_GetTick() - (lcdDis.cfgDis.dbVal[0] + previous_time_sensor_task) >= 0 || (previous_time_sensor_task == 0))
//	{
//		previous_time_sensor_task = HAL_GetTick();
//		sensor_task();
//	}
//
//	if (HAL_GetTick() - previous_time_general_task >= 50)
//	{
//		previous_time_general_task = HAL_GetTick();
//		general_task();
//	}
//
//	if ((HAL_GetTick() - previous_time_lcd_task >= 500))
//	{
//		previous_time_lcd_task = HAL_GetTick();
//		lcd_task();
//	}

	if ((uwTick % 50) == 0)
	{
		itoa(uwTick, log_time, 10);
		LOG("GeneralTask: ", log_time);
		stt_task_point_array[1]();
	}

	if ((uwTick % (int)lcdDis.cfgDis.dbVal[0]) == (n * 50 + 10))
	{
		itoa(uwTick, log_time, 10);
		LOG("LCDTask: ", log_time);
		stt_task_point_array[2]();
		n++;
		if (n >= max_n)
		{
			n = 0;
		}
	}

	if ((uwTick % (int)lcdDis.cfgDis.dbVal[0]) == 80)
	{
		itoa(uwTick, log_time, 10);
		LOG("SensorTask: ", log_time);
		stt_task_point_array[0]();
	}
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void lcd_task(void)
{
	lcd_clear();
	if (lcdDis.ucPage == 0)
	{
		for(int i = 0; i < 2; i++)
		{
			memset(cVal[i], 0, strlen(cVal[i]));
			if (i == 0)
			{
				sprintf(cVal[i], "STM: %soC", (char *)lcdDis.sensorDis.ucSensorStm);
			}
			else
				sprintf(cVal[i], "ADS: %soC", (char *)lcdDis.sensorDis.ucSensorAds);
		}
		lcd_send_cmd(0x80 | 0x00);
		lcd_send_string(cVal[0]);
		lcd_send_cmd(0x80 | 0x40);
		lcd_send_string(cVal[1]);
	}
	else if (lcdDis.ucPage == 1)
	{
		lcd_send_cmd(0x80 | 0x00);
		lcd_send_string((char *)"Chu ky:");
		lcd_send_cmd(0x80 | 0x08);
		lcd_send_string((char *)lcdDis.cfgDis.ucVal[0]);
		lcd_send_cmd(0x80 | 0x40);
		lcd_send_string((char *)"Nguong:");
		lcd_send_cmd(0x80 | 0x48);
		lcd_send_string((char *)lcdDis.cfgDis.ucVal[1]);
		if (lcdDis.cfgDis.ucCursor == 0)
		{
			if (lcdDis.cfgDis.bCfg == true)
			{
				lcd_send_cmd(0x80 | 0x0F);
				lcd_send_string((char *)"<");
			}
			else
			{
				lcd_send_cmd(0x80 | 0x0E);
				lcd_send_string((char *)"<-");
			}
		}
		else if (lcdDis.cfgDis.ucCursor == 1)
		{
			if (lcdDis.cfgDis.bCfg == true)
			{
				lcd_send_cmd(0x80 | 0x4F);
				lcd_send_string((char *)"<");
			}
			else
			{
				lcd_send_cmd(0x80 | 0x4E);
				lcd_send_string((char *)"<-");
			}
		}
	}
}

void sensor_task(void)
{
	HAL_ADC_Start_IT(&hadc1);
	lcdDis.sensorDis.dbSensorStm = (double)HAL_ADC_GetValue(&hadc1) * 100 * 3.3 / 4095;
	ftoa(lcdDis.sensorDis.dbSensorStm, (char *)lcdDis.sensorDis.ucSensorStm, 2);
	memset((char *)ucDataAds, '\0', strlen((char *)ucDataAds));
	ads_read(ADS1115_CONVERSION_REG, ucDataAds);
	lcdDis.sensorDis.dbSensorAds = ((double)((((uint16_t)ucDataAds[0] << 8) & 0xFF00) | ((uint16_t)ucDataAds[1] & 0x00FF)) * 1.024 / 32767) * 100;
	ftoa(lcdDis.sensorDis.dbSensorAds, (char *)lcdDis.sensorDis.ucSensorAds, 2);
	if (lcdDis.sensorDis.dbSensorStm > lcdDis.cfgDis.dbVal[1])
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, LED_ON);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, LED_ON);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, LED_OFF);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, LED_OFF);
	}
	if (lcdDis.sensorDis.dbSensorAds > lcdDis.cfgDis.dbVal[1])
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, LED_ON);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, LED_ON);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, LED_OFF);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, LED_OFF);
	}
}

void general_task(void)
{
	if (push_button == BUT0_BIT)
	{
			lcdDis.cfgDis.bCfg = false;
			lcdDis.cfgDis.ucCursor = 0;
			lcdDis.ucPage = 1 - lcdDis.ucPage;
	}
	else if (push_button == BUT1_BIT)
	{
		if (lcdDis.ucPage == 1 && lcdDis.cfgDis.bCfg == false)
		{
			lcdDis.cfgDis.ucCursor = 1 - lcdDis.cfgDis.ucCursor;
		}
		else if (lcdDis.ucPage == 1 && lcdDis.cfgDis.bCfg == true)
		{
			if (lcdDis.cfgDis.ucCursor == 0)
			{
				if(lcdDis.cfgDis.dbVal[lcdDis.cfgDis.ucCursor] == 9500)
				{
					lcdDis.cfgDis.dbVal[lcdDis.cfgDis.ucCursor] = 500;
				}
				else
					lcdDis.cfgDis.dbVal[lcdDis.cfgDis.ucCursor] += 500;
			}
			else if(lcdDis.cfgDis.ucCursor == 1)
			{
				if(lcdDis.cfgDis.dbVal[lcdDis.cfgDis.ucCursor] == 99)
				{
					lcdDis.cfgDis.dbVal[lcdDis.cfgDis.ucCursor] = 1;
				}
				else
					lcdDis.cfgDis.dbVal[lcdDis.cfgDis.ucCursor] += 1;
			}
			ftoa(lcdDis.cfgDis.dbVal[0] / 1000, (char *)lcdDis.cfgDis.ucVal[0], 1);
			ftoa(lcdDis.cfgDis.dbVal[1], (char *)lcdDis.cfgDis.ucVal[1], 1);
		}
	}
	else if (push_button == BUT2_BIT)
	{
		if (lcdDis.ucPage == 1 && lcdDis.cfgDis.bCfg == false)
		{
			lcdDis.cfgDis.ucCursor = 1 - lcdDis.cfgDis.ucCursor;
		}
		else if (lcdDis.ucPage == 1 && lcdDis.cfgDis.bCfg == true)
		{
			if (lcdDis.cfgDis.ucCursor == 0)
			{
				if(lcdDis.cfgDis.dbVal[lcdDis.cfgDis.ucCursor] == 500)
				{
					lcdDis.cfgDis.dbVal[lcdDis.cfgDis.ucCursor] = 9500;
				}
				else
					lcdDis.cfgDis.dbVal[lcdDis.cfgDis.ucCursor] -= 500;
			}
			else if(lcdDis.cfgDis.ucCursor == 1)
			{
				if(lcdDis.cfgDis.dbVal[lcdDis.cfgDis.ucCursor] == 1)
				{
					lcdDis.cfgDis.dbVal[lcdDis.cfgDis.ucCursor] = 99;
				}
				else
					lcdDis.cfgDis.dbVal[lcdDis.cfgDis.ucCursor] -= 1;
			}
			ftoa(lcdDis.cfgDis.dbVal[0] / 1000, (char *)lcdDis.cfgDis.ucVal[0], 1);
			ftoa(lcdDis.cfgDis.dbVal[1], (char *)lcdDis.cfgDis.ucVal[1], 1);
		}
	}
	else if (push_button == BUT3_BIT)
	{
		if (lcdDis.ucPage == 1)
		{
			lcdDis.cfgDis.bCfg = !lcdDis.cfgDis.bCfg;
		}
	}

	if (ucRxFlag == END_BYTE)
	{
		LOG(TAG, (char *)ucRxBuffer);
		sscanf((char *)ucRxBuffer, "$,%c,%s,*", (char *)&uartCfg.ucType, (char *)uartCfg.ucVal);
		if (uartCfg.ucType == 'P')
		{
			lcdDis.cfgDis.dbVal[0] = atof((char *)uartCfg.ucVal) * 1000;
			if (lcdDis.cfgDis.dbVal[0] > 9500)
				lcdDis.cfgDis.dbVal[0] = 9500;
			else if (lcdDis.cfgDis.dbVal[0] < 500)
				lcdDis.cfgDis.dbVal[0] = 500;
			ftoa(lcdDis.cfgDis.dbVal[0] / 1000, (char *)lcdDis.cfgDis.ucVal[0], 1);
		}
		else if (uartCfg.ucType == 'T')
		{
			lcdDis.cfgDis.dbVal[1] = atof((char *)uartCfg.ucVal);
			if (lcdDis.cfgDis.dbVal[1] > 99)
				lcdDis.cfgDis.dbVal[1] = 99;
			else if (lcdDis.cfgDis.dbVal[1] < 1)
				lcdDis.cfgDis.dbVal[1] = 1;
			ftoa(lcdDis.cfgDis.dbVal[1], (char *)lcdDis.cfgDis.ucVal[1], 1);
		}
		ucRxCnt = 0;
		ucRxFlag = START_BYTE;
		HAL_UART_Receive_IT(&huart2, &ucRxData, 1);
	}

	max_n = lcdDis.cfgDis.dbVal[0] / 50;
	push_button = NONE;

}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == GPIO_PIN_12)
	{
		push_button = BUT0_BIT;
	}
	else if (GPIO_Pin == GPIO_PIN_13)
	{
		push_button = BUT1_BIT;
	}
	else if (GPIO_Pin == GPIO_PIN_14)
	{
		push_button = BUT2_BIT;
	}
	else if (GPIO_Pin == GPIO_PIN_15)
	{
		push_button = BUT3_BIT;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(ucRxData == '$')
	{
		memset((char *)ucRxBuffer, 0, strlen((char *)ucRxBuffer));
		ucRxBuffer[0] = ucRxData;
		ucRxFlag = START_BYTE;
		HAL_UART_Receive_IT(&huart2, &ucRxData, 1);
	}
	else if (ucRxFlag == START_BYTE && ucRxCnt < 20)
	{
		ucRxBuffer[++ucRxCnt] = ucRxData;
		if (ucRxData == '*')
			ucRxFlag = END_BYTE;
		else
			HAL_UART_Receive_IT(&huart2, &ucRxData, 1);
	}
	else if (ucRxCnt == UART_BUFFER_LEN)
	{
		ucRxFlag = ERROR_FRAME;
		memset((char *)ucRxBuffer, 0, strlen((char *)ucRxBuffer));
		ucRxCnt = 0;
		HAL_UART_Receive_IT(&huart2, &ucRxData, 1);
	}
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
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
