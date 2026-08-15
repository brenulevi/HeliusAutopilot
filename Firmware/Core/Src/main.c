/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
#include "i2c_protocol.h"
#include "mpu9250.h"
#include "helius/ahrs.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define GYRO_RATE_HZ 1000U
#define ACCEL_RATE_HZ 100U
#define LOG_RATE_HZ 20U
#define RAD_TO_DEG (180.0f / 3.14159265358979323846f)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
int _write(int file, char *ptr, int len)
{
	uint32_t timeout = HAL_GetTick() + 100;

	while (CDC_Transmit_FS((uint8_t *)ptr, len) == USBD_BUSY)
	{
		if (HAL_GetTick() >= timeout)
			return 0;
	}

	return len;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void DWT_CycleCounter_Init(void)
{
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0U;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static uint32_t DWT_CyclesSince(uint32_t start)
{
	return DWT->CYCCNT - start;
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
	MX_I2C1_Init();
	MX_USB_DEVICE_Init();
	/* USER CODE BEGIN 2 */
	HAL_Delay(1000); // Wait for USB enumeration to complete

	I2C_Protocol i2c_protocol;
	if (i2c_protocol_init(&i2c_protocol, &hi2c1, 1000) != I2C_PROTOCOL_OK)
	{
		printf("I2C protocol initialization failed!\n");
		return -1;
	}

	printf("I2C protocol initialized successfully.\n");

	MPU9250_Driver mpu9250;
	if (mpu9250_init(&mpu9250, &i2c_protocol, 0x68 << 1) != MPU9250_STATUS_OK)
	{
		printf("MPU9250 initialization failed!\n");
		return -1;
	}

	printf("MPU9250 initialized successfully.\n");

	ahrs_t ahrs;
	ahrs_config_t ahrs_config = {
		.initial_orientation_std = 0.2f,
		.initial_gyro_bias_std = 0.1f,
		.gyro_noise_density = 0.00017f,
		.gyro_bias_random_walk = 2.0e-5f,
		.accel_noise_density = 0.002942f};

	if (ahrs_init(&ahrs, &ahrs_config) != AHRS_OK)
	{
		printf("AHRS initialization failed!\n");
		return -1;
	}

	DWT_CycleCounter_Init();
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	const uint32_t gyro_period_cycles = SystemCoreClock / GYRO_RATE_HZ;
	const uint32_t accel_divider = GYRO_RATE_HZ / ACCEL_RATE_HZ;
	const uint32_t log_divider = GYRO_RATE_HZ / LOG_RATE_HZ;
	uint32_t last_gyro_cycle = DWT->CYCCNT;
	uint32_t next_gyro_cycle = last_gyro_cycle + gyro_period_cycles;
	uint32_t gyro_count = 0U;

	while (1)
	{
		/* Signed subtraction keeps this comparison valid across CYCCNT wrap. */
		if ((int32_t)(DWT->CYCCNT - next_gyro_cycle) < 0)
		{
			continue;
		}

		uint32_t sample_cycle = DWT->CYCCNT;
		float gyro_dt = (float)DWT_CyclesSince(last_gyro_cycle) /
						(float)SystemCoreClock;
		last_gyro_cycle = sample_cycle;

		/*
		 * Advance from the previous deadline instead of from "now", avoiding
		 * accumulated drift. Skip expired slots if one iteration overruns.
		 */
		do
		{
			next_gyro_cycle += gyro_period_cycles;
		} while ((int32_t)(sample_cycle - next_gyro_cycle) >= 0);

		if (mpu9250_read_data(&mpu9250) == MPU9250_STATUS_OK)
		{
			vec3_t gyro_measurement = {
				.x = mpu9250.data.gyro_rps.x,
				.y = mpu9250.data.gyro_rps.y,
				.z = mpu9250.data.gyro_rps.z};
			(void)ahrs_predict(&ahrs, &gyro_measurement, gyro_dt);

			gyro_count++;
			if ((gyro_count % accel_divider) == 0U)
			{
				vec3_t accel_measurement = {
					.x = mpu9250.data.accel_mps2.x,
					.y = mpu9250.data.accel_mps2.y,
					.z = mpu9250.data.accel_mps2.z};
				(void)ahrs_update_accel(&ahrs, &accel_measurement,
										1.0f / (float)ACCEL_RATE_HZ);
			}

			if ((gyro_count % log_divider) == 0U)
			{
				vec3_t euler_angles;
				quat_to_euler(&ahrs.orientation, &euler_angles);

				char msg[96];
				int len = snprintf(msg, sizeof(msg), "%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\r\n",
								   euler_angles.x,
								   euler_angles.y,
								   euler_angles.z,
								   ahrs.gyro_bias.x,
								   ahrs.gyro_bias.y,
								   ahrs.gyro_bias.z);

				if (len > 0)
				{
					(void)CDC_Transmit_FS((uint8_t *)msg, (uint16_t)len);
				}
			}
		}
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
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 25;
	RCC_OscInitStruct.PLL.PLLN = 192;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : LED_Pin */
	GPIO_InitStruct.Pin = LED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */

	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
