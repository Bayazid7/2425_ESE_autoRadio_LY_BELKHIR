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
#include "cmsis_os.h"
#include "dma.h"
#include "i2c.h"
#include "sai.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "shell.h"
#include "drv_uart1.h"
#include "IOExpander.h"
#include "sgtl5000.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LONG_TIME 0xffff
#define SHELL_TASK_SIZE 256
#define SHELL_PRIORITY 2
#define CHENILLART_TASK_SIZE 128
#define CHENILLART_PRIORITY 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
h_shell_t h_shell;
uint16_t idvalue;
uint8_t rCodec[50];
uint8_t tCodec[50] = {0xff};
const uint16_t sample_rate = 48000;
const float period_ms = 1.0;
uint16_t period_samples = (uint16_t)(sample_rate * (period_ms / 1000.0));
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
void MX_FREERTOS_Init(void);

/* USER CODE BEGIN PFP */
int __io_putchar(int chr);
int fonction(h_shell_t *h_shell, int argc, char **argv);
int led(h_shell_t *h_shell, int argc, char **argv) ;
void vTaskShell(void *param);
void vTaskChenillard(void *pvParameters);
void Triangle(uint8_t *buffer, uint16_t sample_rate, float period_ms);
	/* USER CODE END PFP */

	/* Private user code ---------------------------------------------------------*/
	/* USER CODE BEGIN 0 */

	/* USER CODE END 0 */

	/**
	 * @brief  The application entry point.
	 * @retval int
	 */
	int
	main(void)
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

	/* Configure the peripherals common clocks */
	PeriphCommonClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_USART2_UART_Init();
	MX_I2C2_Init();
	MX_SAI2_Init();
	MX_SPI3_Init();
	/* USER CODE BEGIN 2 */
	printf("============== AUTO RADIO ============\n");
	h_shell.drv.receive = drv_uart1_receive;
	h_shell.drv.transmit = drv_uart1_transmit;
	IOExpanderInit();
	__HAL_SAI_ENABLE(&hsai_BlockA2);
	// for(int i = 0 ; i>=50 ; i++)
	//{
	// tCodec[i]=0xA0;
	// } ;
	sgtl5000_Init();
	/* TASK CREATION */
	xTaskCreate(vTaskShell, "TaskShell", SHELL_TASK_SIZE, NULL, 2, NULL);
	xTaskCreate(vTaskChenillard, "ChenillardTask", configMINIMAL_STACK_SIZE, NULL, 1, NULL);

	/* USER CODE END 2 */

	/* Call init function for freertos objects (in cmsis_os2.c) */
	MX_FREERTOS_Init();
	Triangle(tCodec, 50, period_samples);
	HAL_SAI_Receive_DMA(&hsai_BlockB2, rCodec, 50);
	HAL_SAI_Transmit_DMA(&hsai_BlockA2, tCodec, 50);

	/* Start scheduler */
	osKernelStart();

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
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
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
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
 * @brief Peripherals Common Clock Configuration
 * @retval None
 */
void PeriphCommonClock_Config(void)
{
	RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

	/** Initializes the peripherals clock
	 */
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_SAI2;
	PeriphClkInit.Sai2ClockSelection = RCC_SAI2CLKSOURCE_PLLSAI1;
	PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSI;
	PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
	PeriphClkInit.PLLSAI1.PLLSAI1N = 13;
	PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV17;
	PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
	PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
	PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_SAI1CLK;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */
int __io_putchar(int chr)
{
	HAL_UART_Transmit(&huart2, (uint8_t *)&chr, 1, HAL_MAX_DELAY);
	return chr;
}

int fonction(h_shell_t *h_shell, int argc, char **argv)
{
	int size = snprintf(h_shell->print_buffer, BUFFER_SIZE, "Je suis une fonction bidon\r\n");
	h_shell->drv.transmit(h_shell->print_buffer, size);

	return 0;
}

void vTaskShell(void *param)
{
	shell_init(&h_shell);
	shell_add(&h_shell, 'f', fonction, "Une fonction inutile");
	shell_add(&h_shell,l,led,"Une fonction pour allumer une led");
	shell_run(&h_shell);
}

void vTaskChenillard(void *pvParameters)
{
	const uint32_t delayMs = 200;
	const uint8_t maxPins = 8;

	while (1)
	{
		IOExpanderWrite(0x14, 0xff);
		IOExpanderWrite(0x13, 0xff);
		for (uint8_t pin = 0; pin < maxPins; pin++)
		{
			IOExpanderGPIO_WritePin(PortA, pin, 0);
			vTaskDelay(pdMS_TO_TICKS(delayMs));
			IOExpanderGPIO_WritePin(PortA, pin, 1);
		}

		for (int8_t pin = maxPins; pin >= 0; pin--)
		{
			IOExpanderGPIO_WritePin(PortB, pin, 0);
			vTaskDelay(pdMS_TO_TICKS(delayMs));
			IOExpanderGPIO_WritePin(PortB, pin, 1);
		}
	}
}

int led(h_shell_t *h_shell, int argc, char **argv)
{
	if (argc == 3)
	{
		uint8_t pin = (uint8_t)atoi(argv[1]);
		char port = argv[2][0];
		if (pin <= Pin7 && (port == 'A' || port == 'B'))
		{
			if (port == 'A')
			{
				IOExpanderGPIO_WritePin(PortA, pin, 1);
			}
			else
			{
				IOExpanderGPIO_WritePin(PortA, pin, 1);
			}
		}
	}

	return 0;
}

void Triangle(uint8_t *buffer, uint16_t sample_rate, float period_ms)
{
	uint16_t period_samples = (uint16_t)(sample_rate * (period_ms / 1000.0));
	uint16_t half_period = period_samples / 2;

	for (uint16_t i = 0; i < period_samples; i++)
	{
		if (i < half_period)
		{
			buffer[i] = (uint8_t)(255.0 * ((float)i / half_period)); // Montée
		}
		else
		{
			buffer[i] = (uint8_t)(255.0 * (1.0 - ((float)(i - half_period) / half_period))); // Descente
		}
	}
}

/* USER CODE END 4 */

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM6 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM6)
	{
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

#ifdef USE_FULL_ASSERT
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
