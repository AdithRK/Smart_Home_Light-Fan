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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f405xx.h"
#include "lcd.h"
#include "FreeRTOS.h"
#include "string.h"
#include "task.h"
#include "spi.h"
#include <stdio.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define I2C_DELAY()  for(volatile int i = 0; i < 30; i++) __NOP(); // ~5-10us delay
// ========== KEYPAD ==========
static const char keypad_map[4][4] = {
    {'1','2','3','A'},
    {'4','5','6','B'},
    {'7','8','9','C'},
    {'*','0','#','D'}
};

static inline void delay_ms_busy(uint32_t ms){
for (volatile uint32_t i=0; i<ms*8000; ++i) __ASM volatile("nop");
}

void delay_ms(uint32_t ms) {
    for (volatile uint32_t i = 0; i < ms * 8000; i++) {
        __asm__("nop");
    }
}





/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// ===== EEPROM layout (per-user 16-byte block) =====
#define EE_USER_BASE 0x0000u
#define EE_USER_STRIDE 16u
#define EE_MAGIC 0x5Au
#define USER_MIN 1u
#define USER_MAX 5u
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define LCD_TOP(s) lprint(0x80, (s))
#define LCD_BOT(s) lprint(0xC0, (s))
#define LCD_CLR() lprint(0x01, "")
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

static void keypad_init(void);
static char keypad_getkey_blocking(void);
static bool keypad_trygetkey(char *out);


static void eeprom_init_defaults_if_needed(void);
static bool eeprom_load_pin(uint8_t user, char out_pin4[4]);
static void eeprom_store_pin(uint8_t user, const char pin4[4]);


static uint8_t ui_get_userid_1to5_hash(void);
static void ui_get_pin_4digits_hash(char out4[4]);


static void vAccessControlTask(void *params);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// ===== Keypad: Rows PB2..PB5 -> outputs; Cols PB6..PB9 -> inputs (PU) =====
static void keypad_init(void) {
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
// Rows as push‑pull outputs, default HIGH
	for (int i = 2; i <= 5; i++) {
	GPIOB->MODER &= ~(3u << (2*i));
	GPIOB->MODER |= (1u << (2*i));
	GPIOB->OTYPER &= ~(1u << i);
	GPIOB->OSPEEDR |= (3u << (2*i));
	GPIOB->ODR |= (1u << i); // HIGH
}
// Cols as inputs with pull‑ups
for (int i = 6; i <= 9; i++) {
	GPIOB->MODER &= ~(3u << (2*i));
	GPIOB->PUPDR &= ~(3u << (2*i));
	GPIOB->PUPDR |= (1u << (2*i)); // PU
}
}

static bool keypad_trygetkey(char *out){
	for (int row = 0; row < 4; row++) {
		GPIOB->ODR |= (0xF << 2); // set all rows HIGH
		GPIOB->ODR &= ~(1u << (2 + row)); // drive one row LOW
	for(volatile int d=0; d<300; ++d) __ASM volatile("nop");
	uint32_t idr = GPIOB->IDR;
	for (int col = 0; col < 4; col++) {
		if (!(idr & (1u << (6 + col)))) { // active low
// wait for release (debounce)
			while (!(GPIOB->IDR & (1u << (6 + col)))) delay_ms_busy(2);
			*out = keypad_map[row][col];
			return true;
}
}
}
return false;
}

static char keypad_getkey_blocking(void){
	char k;
	for(;;){ if(keypad_trygetkey(&k)) return k; vTaskDelay(pdMS_TO_TICKS(2)); }
}


// ===== EEPROM helpers =====
static inline uint32_t ee_addr_for_user(uint8_t user){
	return EE_USER_BASE + (uint32_t)(user-1u)*EE_USER_STRIDE;
}


static bool eeprom_load_pin(uint8_t user, char out_pin4[4]){
	uint32_t a = ee_addr_for_user(user);
	uint8_t magic = EepromReadByte(a+0);
	if (magic != EE_MAGIC) return false;
	for (int i=0;i<4;i++) out_pin4[i] = (char)EepromReadByte(a+1+i);
	return true;
}

static void eeprom_store_pin(uint8_t user, const char pin4[4]){
	uint32_t a = ee_addr_for_user(user);
	EepromWriteEnable(); EepromWriteByte(a+0, EE_MAGIC);
for (int i=0;i<4;i++){ EepromWriteEnable(); EepromWriteByte(a+1+i, (uint8_t)pin4[i]); }
}


static void eeprom_init_defaults_if_needed(void){
	bool need = false;
	for (uint8_t u=USER_MIN; u<=USER_MAX; ++u){ char p[4]; if(!eeprom_load_pin(u,p)){ need=true; break; } }
	if (need){ const char def[4]={'1','2','3','4'}; for(uint8_t u=USER_MIN; u<=USER_MAX; ++u) eeprom_store_pin(u, def); }
}


// ===== UI input helpers =====
static uint8_t ui_get_userid_1to5_hash(void){
	for(;;){
		char k = keypad_getkey_blocking();
		if (k>='1' && k<='5'){
			char conf = keypad_getkey_blocking();
			if (conf=='#') return (uint8_t)(k-'0');
}
// ignore others
}
}

static void ui_get_pin_4digits_hash(char out4[4]){
	uint8_t i=0;
	while(i<4){
		char k = keypad_getkey_blocking();
		if (k>='0' && k<='9'){
			out4[i++] = k; LcdFxn(1,'*');
} else if (k=='*') { // clear
	while(i){ --i; /* no backspace on 16x2, just keep */ }
	LCD_BOT(" "); LCD_TOP("Security Code:");
} else {
// ignore A,B,C,D until we get 4 digits
}
}
// require '#'
while (keypad_getkey_blocking() != '#') { /* wait until user confirms */ }
}

// ===== Access Control Task (FreeRTOS) =====
static void vAccessControlTask(void *params){
(void)params;


LcdInit();
keypad_init();
SpiEepromInit();


eeprom_init_defaults_if_needed();


for(;;){
// Welcome / idle
LCD_CLR();
LCD_TOP("Welcome");
LCD_BOT("* Login B Pwd");


// Wait for action
char k=0;
do { k = keypad_getkey_blocking(); } while(k!='*' && k!='B');


if (k=='B'){
// Change password: need UserID + old PIN check (optional).
LCD_CLR(); LCD_TOP("User ID (1-5)#");
uint8_t user = ui_get_userid_1to5_hash();


// verify old code
LCD_CLR(); LCD_TOP("Old Code:");
char oldp[4]; ui_get_pin_4digits_hash(oldp);
char stored[4]; bool ok = eeprom_load_pin(user, stored) && (memcmp(oldp,stored,4)==0);
if(!ok){ LCD_BOT("Access Denied"); vTaskDelay(pdMS_TO_TICKS(900)); continue; }


// new code
LCD_CLR(); LCD_TOP("New Code:");
char np[4]; ui_get_pin_4digits_hash(np);
eeprom_store_pin(user, np);
LCD_BOT("Password Changed!");
vTaskDelay(pdMS_TO_TICKS(1000));
continue; // back to welcome
}


// Login flow
LCD_CLR(); LCD_TOP("User ID (1-5)#");
uint8_t user = ui_get_userid_1to5_hash();


LCD_CLR(); LCD_TOP("Security Code:");
char pin[4]; ui_get_pin_4digits_hash(pin);


char stored[4]; bool ok = eeprom_load_pin(user, stored) && (memcmp(pin,stored,4)==0);
if (ok){
LCD_BOT("Access Granted");
vTaskDelay(pdMS_TO_TICKS(600));
// Menu after success
bool inMenu = true;
while(inMenu){
LCD_CLR(); LCD_TOP("Menu"); LCD_BOT("1)Pwd 2)Enter");
char sel = keypad_getkey_blocking();
if (sel=='1'){
LCD_CLR(); LCD_TOP("New Code:");
char np[4]; ui_get_pin_4digits_hash(np);
eeprom_store_pin(user, np);
LCD_BOT("Password Changed!");
vTaskDelay(pdMS_TO_TICKS(900));
inMenu=false; // return to login per spec
} else if (sel=='2'){
LCD_CLR(); LCD_TOP("Entering...");
vTaskDelay(pdMS_TO_TICKS(700));
// TODO: signal other tasks that security passed
// vTaskDelete(NULL); // if you want this task to end here
inMenu=false; // fall back to Welcome for next session
} else {
// ignore others
}
}
} else {
LCD_BOT("Access Denied");
vTaskDelay(pdMS_TO_TICKS(1200));
}
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
  /* USER CODE BEGIN 2 */
  LcdInit();
  keypad_init();
  SpiEepromInit();

  xTaskCreate(vAccessControlTask, "Access", 256, NULL, 2, NULL);
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
