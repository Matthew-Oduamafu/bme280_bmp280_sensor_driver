/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "bme280_sensor_driver.h"
#include "lcd_i2c_driver.h"
#include <stdio.h>
#include <string.h>
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

/* USER CODE BEGIN PV */
BME280_Handle bme280;
LCD_Handle lcd;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
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
  /* USER CODE BEGIN 2 */

  // Initialize drivers
  LCD_Init(&lcd, &hi2c1);

  LCD_Clear(&lcd);
  LCD_PrintAt(&lcd, 0, 0, "BME280 Sensor Test");
  LCD_DrawSeparator(&lcd, 1);
  LCD_PrintAt(&lcd, 2, 0, "Initializing...");

  // Initialize BME280
  if (BME280_Init(&bme280, &hi2c1) != HAL_OK) {
    LCD_ClearRow(&lcd, 2);
    LCD_PrintAt(&lcd, 2, 0, "Sensor Init Failed!");
    LCD_PrintAt(&lcd, 3, 0, "Check I2C Address");
    while(1) {
      HAL_Delay(1000);
    }
  }

  // Display sensor type
  char buffer[21];
  if (bme280.is_bme280) {
    snprintf(buffer, sizeof(buffer), "BME280 Detected!");
  } else {
    snprintf(buffer, sizeof(buffer), "BMP280 Detected!");
  }
  LCD_ClearRow(&lcd, 2);
  LCD_PrintAt(&lcd, 2, 0, buffer);

  snprintf(buffer, sizeof(buffer), "Chip ID: 0x%02X", bme280.chip_id);
  LCD_PrintAt(&lcd, 3, 0, buffer);
  HAL_Delay(2000);

  BME280_Data sensor_data;
  uint32_t last_update = 0;
  uint16_t sample_count = 0;

  LCD_Clear(&lcd);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // Update every second
        if (HAL_GetTick() - last_update >= 1000) {
            last_update = HAL_GetTick();
            sample_count++;

            // Read all sensor data
            if (BME280_ReadSensorData(&bme280, &sensor_data) == HAL_OK) {
                // Line 0: Title with sample count
                snprintf(buffer, sizeof(buffer), "Environmental #%d", sample_count);
                LCD_PrintAt(&lcd, 0, 0, "                    ");
                LCD_PrintAt(&lcd, 0, 0, buffer);

                LCD_DrawSeparator(&lcd, 1);

                // Line 2: Temperature
                snprintf(buffer, sizeof(buffer), "Temp: %.2f", sensor_data.temperature);
                LCD_PrintAt(&lcd, 2, 0, buffer);
                LCD_PrintCustomChar(&lcd, CHAR_DEGREE);
                LCD_Print(&lcd, "C   ");

                // Line 2 (right): Pressure
                snprintf(buffer, sizeof(buffer), "%.1f", sensor_data.pressure);
                LCD_PrintAt(&lcd, 2, 20 - strlen(buffer) - 3, buffer);
                LCD_Print(&lcd, "hPa");

                // Line 3: Humidity (if BME280) or Altitude
                if (bme280.is_bme280) {
                    snprintf(buffer, sizeof(buffer), "Humidity: %.1f%%", sensor_data.humidity);
                    LCD_PrintAt(&lcd, 3, 0, buffer);
                } else {
                    snprintf(buffer, sizeof(buffer), "Alt: %.1fm", sensor_data.altitude);
                    LCD_PrintAt(&lcd, 3, 0, buffer);
                }

                // Visual indicator based on temperature
                if (sensor_data.temperature < 20.0f) {
                    LCD_PrintAt(&lcd, 3, 18, "[C]");  // Cold
                } else if (sensor_data.temperature < 25.0f) {
                    LCD_PrintAt(&lcd, 3, 18, "[O]");  // OK
                } else if (sensor_data.temperature < 30.0f) {
                    LCD_PrintAt(&lcd, 3, 18, "[W]");  // Warm
                } else {
                    LCD_PrintAt(&lcd, 3, 18, "[H]");  // Hot
                }

            } else {
                LCD_PrintAt(&lcd, 2, 0, "Read Error!         ");
            }
        }

        HAL_Delay(10);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  hi2c1.Init.Timing = 0x00201D2B;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();

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
