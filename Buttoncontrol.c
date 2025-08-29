
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
#include "gpio.h"
#include "stm32f405xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdint.h>
// -------- Pin definitions --------
#define AIN1_PIN        12   // PA12 - Motor AIN1
#define AIN2_PIN        4    // PA4  - Motor AIN2
#define FAN_BTN_PIN     15   // PA15 - Fan pushbutton (active low)
#define LIGHT_BTN_PIN   7    // PB7  - Light pushbutton (active low)
#define RELAY_PIN       0    // PC0  - Relay IN (LOW-trigger)
// Globals
static int fan_state   = 0; // 0=OFF, 1=ON
static int light_state = 0; // 0=OFF, 1=ON
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
/* USER CODE BEGIN PV */
// Small delay (for debounce)
static inline void delayms(uint32_t ms) {
    for (uint32_t i = 0; i < ms * 16000; i++) __asm__("nop");
}
// -------- Motor (fan) control --------
static inline void motor_on(void) {
    GPIOA->ODR |=  (1u << AIN1_PIN);
    GPIOA->ODR &= ~(1u << AIN2_PIN);
}
static inline void motor_off(void) {
    GPIOA->ODR &= ~(1u << AIN1_PIN);
    GPIOA->ODR &= ~(1u << AIN2_PIN);
}
// -------- Relay (light) control --------
// Low-trigger relay: IN=0 -> ON, IN=1 -> OFF
static inline void relay_write(int on) {
    if (on)
        GPIOC->ODR &= ~(1u << RELAY_PIN); // drive low -> ON
    else
        GPIOC->ODR |=  (1u << RELAY_PIN); // drive high -> OFF
}
// -------- GPIO Init --------
void gpio_init(void) {
    // Enable GPIOA, GPIOB, GPIOC clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN |
                    RCC_AHB1ENR_GPIOBEN |
                    RCC_AHB1ENR_GPIOCEN;
    // --- Fan outputs: PA12, PA4 ---
    GPIOA->MODER &= ~((3u << (AIN1_PIN * 2)) | (3u << (AIN2_PIN * 2)));
    GPIOA->MODER |=  (1u << (AIN1_PIN * 2)) | (1u << (AIN2_PIN * 2));
    motor_off(); // default OFF
    // --- Fan button: PA15 (input pull-up) ---
    GPIOA->MODER &= ~(3u << (FAN_BTN_PIN * 2));
    GPIOA->PUPDR &= ~(3u << (FAN_BTN_PIN * 2));
    GPIOA->PUPDR |=  (1u << (FAN_BTN_PIN * 2)); // pull-up
    // --- Light button: PB7 (input pull-up) ---
    GPIOB->MODER &= ~(3u << (LIGHT_BTN_PIN * 2));
    GPIOB->PUPDR &= ~(3u << (LIGHT_BTN_PIN * 2));
    GPIOB->PUPDR |=  (1u << (LIGHT_BTN_PIN * 2)); // pull-up
    // --- Relay output: PC0 ---
    GPIOC->MODER &= ~(3u << (RELAY_PIN * 2));
    GPIOC->MODER |=  (1u << (RELAY_PIN * 2)); // output
    relay_write(0); // default OFF
}
// -------- FreeRTOS Tasks --------
// Fan control task
void Task_FanControl(void *pv) {
    for (;;) {
        if (!(GPIOA->IDR & (1u << FAN_BTN_PIN))) {   // Button pressed?
            delayms(50);                             // Debounce
            if (!(GPIOA->IDR & (1u << FAN_BTN_PIN))) {
                fan_state ^= 1;
                if (fan_state) motor_on();
                else motor_off();
                // Wait until button released
                while (!(GPIOA->IDR & (1u << FAN_BTN_PIN)));
                delayms(10);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50)); // Run every 50ms
    }
}
// Light control task
void Task_LightControl(void *pv) {
    for (;;) {
        if (!(GPIOB->IDR & (1u << LIGHT_BTN_PIN))) { // Button pressed?
            delayms(50);                             // Debounce
            if (!(GPIOB->IDR & (1u << LIGHT_BTN_PIN))) {
                light_state ^= 1;
                relay_write(light_state);
                // Wait until button released
                while (!(GPIOB->IDR & (1u << LIGHT_BTN_PIN)));
                delayms(10);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50)); // Run every 50ms
    }
}
/* USER CODE END PV */
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  /* USER CODE BEGIN 2 */
  SystemInit();   // From CMSIS (clock setup)
      gpio_init();    // Init GPIOs
      // Create tasks
      xTaskCreate(Task_FanControl, "Fan",   128, NULL, 2, NULL);
      xTaskCreate(Task_LightControl, "Light", 128, NULL, 2, NULL);
      // Start FreeRTOS scheduler
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
