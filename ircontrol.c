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
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "lcd.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


/* --- Forward declarations --- */

/* Queue handle for IR codes */

QueueHandle_t irQueue;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
typedef enum {
    MODE_FAN = 0,
    MODE_LIGHT
} DeviceMode;

static bool       systemOn    = false;
static DeviceMode currentMode = MODE_FAN;   // default when turning ON
static bool       fanOn       = false;
static bool       lightOn     = false;
static int        fanPct      = 50;         // 30/50/80
static int        lightPct    = 50;         // 30/50/80

/* IR codes you provided */
#define IR_POWER        0x01FE48B7u
#define IR_MODE         0x01FE58A7u
#define IR_PLAY_PAUSE   0x01FE807Fu
#define IR_1            0x01FE50AFu
#define IR_2            0x01FED827u
#define IR_3            0x01FEF807u
#define AIN1_PIN        12   // PA12 - Motor AIN1
#define AIN2_PIN        4    // PA4  - Motor AIN2
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//void vIRTask(void *argument)
//
//{
//
//    uint32_t code;
//
//    char buf[17];
//
//    LcdInit();
//
//    lprint(0x80, "IR Remote Demo");
//
//    for (;;)
//
//    {
//
//        if (HX1838_GetCode(&code, 100) == pdPASS)   // timeout 100ms
//
//        {
//
//            snprintf(buf, sizeof(buf), "%08lX", code);
//
//            lprint(0xC0, buf);  // show on 2nd line
//
//        }
//
//        vTaskDelay(pdMS_TO_TICKS(10));
//
//    }
//
//}
static void SetFanPWM(int percent)
{
    if (percent < 0)   percent = 0;
    if (percent > 100) percent = 100;

    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim4);  // from TIM4 init
    uint32_t ccr = (arr + 1) * percent / 100;
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, ccr);
}

static void SetLightPWM(int percent)
{
    if (percent < 0)   percent = 0;
    if (percent > 100) percent = 100;

    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim1);  // period value
    uint32_t ccr = (arr + 1) * percent / 100;         // duty cycle
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ccr);
}

static void ShowTopLine(void)
{
    if (!systemOn) {
        lprint(0x80, "Press POWER    ");
        return;
    }
    if (currentMode == MODE_FAN) {
        lprint(0x80, "Fan Mode       ");
    } else {
        lprint(0x80, "Light Mode     ");
    }
}

static void ShowBottomLine_Status(void)
{
    char line[20];
    if (!systemOn) {
        lprint(0xC0, "                ");
        return;
    }

    if (currentMode == MODE_FAN) {
        if (fanOn) {
            snprintf(line, sizeof(line), "Fan %d%%       ", fanPct);
        } else {
            snprintf(line, sizeof(line), "Fan OFF        ");
        }
    } else {
        if (lightOn) {
            snprintf(line, sizeof(line), "Light %d%%     ", lightPct);
        } else {
            snprintf(line, sizeof(line), "Light OFF      ");
        }
    }
    lprint(0xC0, line);
}
static inline void motor_on(void) {
    GPIOA->ODR |=  (1u << AIN1_PIN);
    GPIOA->ODR &= ~(1u << AIN2_PIN);
}
static inline void motor_off(void) {
    GPIOA->ODR &= ~(1u << AIN1_PIN);
    GPIOA->ODR &= ~(1u << AIN2_PIN);
}

void vIRTask(void *argument)
{
    uint32_t code;

    LcdInit();
    lprint(0x80, "Press POWER    ");
    lprint(0xC0, "                ");

    for (;;)
    {
        if (HX1838_GetCode(&code, 500) == pdPASS)
        {
            switch (code)
            {
            case IR_POWER:
                systemOn = !systemOn;
                if (systemOn) {
                    currentMode = MODE_FAN;
                    fanOn   = true;  fanPct   = 50;
                    lightOn = true;  lightPct = 50;

                    SetFanPWM(fanPct);       // start fan at 50%
                    motor_on();              // enable motor driver pins
                    SetLightPWM(lightPct);   // start light at 50%

                    lprint(0x80, "System ON      ");
                    lprint(0xC0, "Press MODE     ");
                } else {
                    fanOn = lightOn = false;
                    SetFanPWM(0);            // stop fan
                    motor_off();             // disable driver
                    SetLightPWM(0);          // turn light OFF

                    lprint(0x80, "System OFF     ");
                    lprint(0xC0, "Press POWER    ");
                }
                break;

            case IR_MODE:
                if (systemOn) {
                    currentMode = (currentMode == MODE_FAN) ? MODE_LIGHT : MODE_FAN;
                    ShowTopLine();
                    ShowBottomLine_Status();
                }
                break;

            case IR_PLAY_PAUSE:
                if (systemOn) {
                    if (currentMode == MODE_FAN) {
                        fanOn = !fanOn;
                        if (fanOn) {
                            SetFanPWM(fanPct);   // resume PWM
                            motor_on();
                        } else {
                            SetFanPWM(0);        // duty 0
                            motor_off();
                        }
                    } else {
                        lightOn = !lightOn;
                        if (lightOn) {
                            SetLightPWM(lightPct);
                        } else {
                            SetLightPWM(0);
                        }
                    }
                    ShowTopLine();
                    ShowBottomLine_Status();
                }
                break;

            case IR_1:
                if (systemOn) {
                    if (currentMode == MODE_FAN) {
                        fanPct = 30;
                        if (fanOn) SetFanPWM(fanPct);
                    } else {
                        lightPct = 30;
                        if (lightOn) SetLightPWM(lightPct);
                    }
                    ShowTopLine();
                    ShowBottomLine_Status();
                }
                break;

            case IR_2:
                if (systemOn) {
                    if (currentMode == MODE_FAN) {
                        fanPct = 50;
                        if (fanOn) SetFanPWM(fanPct);
                    } else {
                        lightPct = 50;
                        if (lightOn) SetLightPWM(lightPct);
                    }
                    ShowTopLine();
                    ShowBottomLine_Status();
                }
                break;

            case IR_3:
                if (systemOn) {
                    if (currentMode == MODE_FAN) {
                        fanPct = 80;
                        if (fanOn) SetFanPWM(fanPct);
                    } else {
                        lightPct = 80;
                        if (lightOn) SetLightPWM(lightPct);
                    }
                    ShowTopLine();
                    ShowBottomLine_Status();
                }
                break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
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
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  HX1838_Init(&htim2, GPIO_PIN_5);   // only pin, no port
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  xTaskCreate(vIRTask, "IR Task", 256, NULL, tskIDLE_PRIORITY + 1, NULL);

  vTaskStartScheduler();

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
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 192;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM3 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM3) {
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
