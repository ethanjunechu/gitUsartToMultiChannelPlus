/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 ** This notice applies to any and all portions of this file
 * that are not between comment pairs USER CODE BEGIN and
 * USER CODE END. Other portions of this file, whether
 * inserted by the user or by software development tools
 * are owned by their respective copyright owners.
 *
 * COPYRIGHT(c) 2018 STMicroelectronics
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include <string.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
/* 定义6路GPIO模拟PWM输出 */
/* 顺序对应STM32_IO板QB1-QB18 */
#define PWM1_DIR(x)  (x == 1 ? HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET))
#define PWM1_EN(x)  (x == 1 ? HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET))
#define PWM1_OUT(x)  (x == 1 ? HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET) : HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET))

#define PWM2_DIR(x)  (x == 1 ? HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET))
#define PWM2_EN(x)  (x == 1 ? HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET))
#define PWM2_OUT(x)  (x == 1 ? HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET) : HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET))

#define PWM3_DIR(x)  (x == 1 ? HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET))
#define PWM3_EN(x)  (x == 1 ? HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_RESET))
#define PWM3_OUT(x)  (x == 1 ? HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_RESET) : HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_SET))

#define PWM4_DIR(x)  (x == 1 ? HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET))
#define PWM4_EN(x)  (x == 1 ? HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_RESET))
#define PWM4_OUT(x)  (x == 1 ? HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET) : HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET))

#define PWM5_DIR(x)  (x == 1 ? HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_RESET))
#define PWM5_EN(x)  (x == 1 ? HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET))
#define PWM5_OUT(x)  (x == 1 ? HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET) : HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET))

#define PWM6_DIR(x)  (x == 1 ? HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET))
#define PWM6_EN(x)  (x == 1 ? HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_RESET))
#define PWM6_OUT(x)  (x == 1 ? HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET) : HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET))

#define TransferLED1_OUT(x)  (x == 1 ? HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET))
#define TransferLED2_OUT(x)  (x == 1 ? HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET))
#define TransferLED3_OUT(x)  (x == 1 ? HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET))
#define TransferLED4_OUT(x)  (x == 1 ? HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_RESET))
#define TransferLED5_OUT(x)  (x == 1 ? HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET))
#define TransferLED6_OUT(x)  (x == 1 ? HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET))
#define TransferRELAY1_OUT(x)  (x == 1 ? HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET))
/* 最大命令数 */
#define MAX_CMD_SIZE 1000
/* 串口接收缓存 */
uint8_t RxBuffer[8];
/* 命令缓存 */
uint8_t CMDBuffer[MAX_CMD_SIZE][4];
/* 已接收的命令数 */
uint32_t CMDRevNum = 0;
/* 当前命令位置 */
uint32_t CMDRunNum = 0;
/* 6路PWM倍数 */
uint8_t PWM1Step = 10, PWM2Step = 10, PWM3Step = 10, PWM4Step = 10, PWM5Step =
		10, PWM6Step = 10;
/* 6路PWM方向 */
uint8_t Dir1 = 0, Dir2 = 0, Dir3 = 0, Dir4 = 0, Dir5 = 0, Dir6 = 0;
/* 6路PWM脉宽时间 */
uint32_t PWMHoldTime1 = 10000, PWMHoldTime2 = 7000, PWMHoldTime3 = 7000,
		PWMHoldTime4 = 7000, PWMHoldTime5 = 7000, PWMHoldTime6 = 7000;
/* 脉宽时间临时计数 */
uint32_t iTime;

// CRC 高位位字节值表
static uint8_t auchCRCHi[] = { 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
		0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
		0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
		0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
		0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
		0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
		0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
		0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
		0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
		0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
		0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
		0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
		0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
		0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
		0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
		0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
		0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
		0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
		0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
		0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
		0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
		0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40 };
// CRC 低位字节值表
static uint8_t auchCRCLo[] = { 0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2,
		0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD,
		0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8,
		0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F,
		0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6,
		0xD2, 0x12, 0x13, 0xD3, 0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1,
		0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
		0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB,
		0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA,
		0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5,
		0x27, 0xE7, 0xE6, 0x26, 0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0,
		0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67,
		0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE,
		0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79,
		0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C,
		0xB4, 0x74, 0x75, 0xB5, 0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73,
		0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92,
		0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C, 0x5D, 0x9D,
		0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98,
		0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F,
		0x8D, 0x4D, 0x4C, 0x8C, 0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86,
		0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80, 0x40 };
//步进电机S曲线加速查表法
static uint32_t period[202] = { 168750, 167622, 167564, 167504, 167441, 167374,
		167304, 167231, 167153, 167072, 166987, 166898, 166804, 166705, 166602,
		166493, 166379, 166259, 166134, 166002, 165863, 165718, 165565, 165405,
		165237, 165061, 164876, 164683, 164479, 164266, 164043, 163809, 163563,
		163306, 163036, 162753, 162457, 162147, 161823, 161483, 161127, 160755,
		160365, 159958, 159532, 159086, 158621, 158134, 157625, 157095, 156540,
		155962, 155358, 154729, 154072, 153388, 152676, 151934, 151162, 150358,
		149523, 148655, 147753, 146816, 145844, 144837, 143792, 142710, 141590,
		140431, 139234, 137996, 136719, 135402, 134044, 132646, 131207, 129728,
		128208, 126649, 125050, 123412, 121735, 120022, 118271, 116486, 114666,
		112813, 110928, 109014, 107072, 105104, 103111, 101096, 99061, 97009,
		94941, 92860, 90769, 88670, 86566, 84459, 82353, 80248, 78149, 76058,
		73978, 71910, 69857, 67822, 65808, 63815, 61847, 59904, 57990, 56106,
		54253, 52433, 50648, 48897, 47184, 45507, 43869, 42270, 40711, 39191,
		37712, 36273, 34875, 33517, 32199, 30922, 29685, 28487, 27329, 26209,
		25127, 24082, 23074, 22103, 21166, 20264, 19396, 18560, 17757, 16985,
		16243, 15530, 14846, 14190, 13561, 12957, 12378, 11824, 11293, 10785,
		10298, 9833, 9387, 8961, 8553, 8164, 7792, 7436, 7096, 6771, 6461, 6165,
		5883, 5613, 5356, 5110, 4876, 4653, 4439, 4236, 4042, 3858, 3681, 3513,
		3353, 3201, 3056, 2917, 2785, 2659, 2540, 2426, 2317, 2213, 2115, 2021,
		1932, 1846, 1765, 1688, 1615, 1545, 1478, 1415, 1354, 1297 };

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void
SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
unsigned short
CRC16(uint8_t *puchMsg, unsigned short usDataLen);
void
CMDProcess(void);
void
HAL_UART_RxCpltCallback(UART_HandleTypeDef* uartHandle);
void
OutPWM1(uint32_t Step, uint8_t Dir);
void
OutPWM2(uint32_t Step, uint8_t Dir);
void
OutPWM3(uint32_t Step, uint8_t Dir);
void
OutPWM4(uint32_t Step, uint8_t Dir);
void
OutPWM5(uint32_t Step, uint8_t Dir);
void
OutPWM6(uint32_t Step, uint8_t Dir);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 *
 * @retval None
 */
int main(void) {
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------------------------------*/

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
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	/* USER CODE BEGIN 2 */
	HAL_Delay(1000);
	PWM1_OUT(0);
	PWM2_OUT(0);
	PWM3_OUT(0);
	PWM4_OUT(0);
	PWM5_OUT(0);
	PWM6_OUT(0);
	/* 获取内存步距细分 */
	PWM1Step = 10;
	PWM2Step = 10;
	PWM3Step = 10;
	PWM4Step = 10;
	PWM5Step = 10;
	PWM6Step = 10;
	TransferLED1_OUT(0);
	TransferLED2_OUT(0);
	TransferLED3_OUT(0);
	TransferLED4_OUT(0);
	TransferLED5_OUT(0);
	TransferLED6_OUT(0);
	/* 初始化串口接收 */
	HAL_UART_Receive_IT(&huart2, RxBuffer, 8);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		/* 处理命令 */
		CMDProcess();
	}
	/* USER CODE END 3 */

}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure the Systick interrupt time
	 */
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	/**Configure the Systick
	 */
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
/* 6路PWM输出函数 */
void OutPWM1(uint32_t Step, uint8_t Dir) {
	uint32_t PWMHoldTime = period[0] / 50;
	uint32_t OldStep = Step;
	PWM1_DIR(Dir);
	PWM1_EN(0);
	for (; Step > 0; Step--) {
		//S加速
		if ((OldStep - Step) < 200 && (Step > (OldStep / 2))) {
			PWMHoldTime = period[OldStep - Step] / 50;
		}
		//S减速
		if (Step <= 200 && (Step < (OldStep / 2))) {
			PWMHoldTime = period[Step] / 50;
		}
		PWM1_OUT(1);
		TransferLED1_OUT(1);
		for (iTime = 0; iTime < 4; iTime++)
			;
		PWM1_OUT(0);
		TransferLED1_OUT(0);
		for (iTime = 0; iTime < PWMHoldTime; iTime++)
			;
	}
	PWM1_EN(1);
}
void OutPWM2(uint32_t Step, uint8_t Dir) {
	uint32_t PWMHoldTime = period[0] / 50;
	uint32_t OldStep = Step;
	PWM2_DIR(Dir);
	PWM2_EN(0);
	for (; Step > 0; Step--) {
		//S加速
		if ((OldStep - Step) < 200 && (Step > (OldStep / 2))) {
			PWMHoldTime = period[OldStep - Step] / 50;
		}
		//S减速
		if (Step <= 200 && (Step < (OldStep / 2))) {
			PWMHoldTime = period[Step] / 50;
		}
		PWM2_OUT(1);
		TransferLED2_OUT(1);
		for (iTime = 0; iTime < 4; iTime++)
			;
		PWM2_OUT(0);
		TransferLED2_OUT(0);
		for (iTime = 0; iTime < PWMHoldTime; iTime++)
			;
	}
	PWM2_EN(1);
}
void OutPWM3(uint32_t Step, uint8_t Dir) {
	uint32_t PWMHoldTime = period[0] / 50;
	uint32_t OldStep = Step;
	PWM3_DIR(Dir);
	PWM3_EN(0);
	for (; Step > 0; Step--) {
		//S加速
		if ((OldStep - Step) < 200 && (Step > (OldStep / 2))) {
			PWMHoldTime = period[OldStep - Step] / 50;
		}
		//S减速
		if (Step <= 200 && (Step < (OldStep / 2))) {
			PWMHoldTime = period[Step] / 50;
		}
		PWM3_OUT(1);
		TransferLED3_OUT(1);
		for (iTime = 0; iTime < 4; iTime++)
			;
		PWM3_OUT(0);
		TransferLED3_OUT(0);
		for (iTime = 0; iTime < PWMHoldTime; iTime++)
			;
	}
	PWM3_EN(1);
}
void OutPWM4(uint32_t Step, uint8_t Dir) {
	uint32_t PWMHoldTime = period[0] / 50;
	uint32_t OldStep = Step;
	PWM4_DIR(Dir);
	PWM4_EN(0);
	for (; Step > 0; Step--) {
		//S加速
		if ((OldStep - Step) < 200 && (Step > (OldStep / 2))) {
			PWMHoldTime = period[OldStep - Step] / 50;
		}
		//S减速
		if (Step <= 200 && (Step < (OldStep / 2))) {
			PWMHoldTime = period[Step] / 50;
		}
		PWM4_OUT(1);
		TransferLED4_OUT(1);
		for (iTime = 0; iTime < 4; iTime++)
			;
		PWM4_OUT(0);
		TransferLED4_OUT(0);
		for (iTime = 0; iTime < PWMHoldTime; iTime++)
			;
	}
	PWM4_EN(1);
}
void OutPWM5(uint32_t Step, uint8_t Dir) {
	uint32_t PWMHoldTime = period[0] / 50;
	uint32_t OldStep = Step;
	PWM5_DIR(Dir);
	PWM5_EN(0);
	for (; Step > 0; Step--) {
		//S加速
		if ((OldStep - Step) < 200 && (Step > (OldStep / 2))) {
			PWMHoldTime = period[OldStep - Step] / 50;
		}
		//S减速
		if (Step <= 200 && (Step < (OldStep / 2))) {
			PWMHoldTime = period[Step] / 50;
		}
		PWM5_OUT(1);
		TransferLED5_OUT(1);
		for (iTime = 0; iTime < 4; iTime++)
			;
		PWM5_OUT(0);
		TransferLED5_OUT(0);
		for (iTime = 0; iTime < PWMHoldTime; iTime++)
			;
	}
	PWM5_EN(1);
}
void OutPWM6(uint32_t Step, uint8_t Dir) {
	uint32_t PWMHoldTime = period[0] / 50;
	uint32_t OldStep = Step;
	PWM6_DIR(Dir);
	PWM6_EN(0);
	for (; Step > 0; Step--) {
		//S加速
		if ((OldStep - Step) < 200 && (Step > (OldStep / 2))) {
			PWMHoldTime = period[OldStep - Step] / 50;
		}
		//S减速
		if (Step <= 200 && (Step < (OldStep / 2))) {
			PWMHoldTime = period[Step] / 50;
		}
		PWM6_OUT(1);
		TransferLED6_OUT(1);
		for (iTime = 0; iTime < 4; iTime++)
			;
		PWM6_OUT(0);
		TransferLED6_OUT(0);
		for (iTime = 0; iTime < PWMHoldTime; iTime++)
			;
	}
	PWM6_EN(1);
}
/**
 * 函数功能: 命令处理函数
 * 输入参数: 无
 * 返 回 值: 无
 * 说    明: 无
 */
void CMDProcess(void) {
	if (CMDBuffer[CMDRunNum][0] != 0) {
		switch (CMDBuffer[CMDRunNum][0]) {
		case 1:
			OutPWM1((CMDBuffer[CMDRunNum][1] << 8) + CMDBuffer[CMDRunNum][2],
					CMDBuffer[CMDRunNum][3]);
			break;
		case 2:
			OutPWM2((CMDBuffer[CMDRunNum][1] << 8) + CMDBuffer[CMDRunNum][2],
					CMDBuffer[CMDRunNum][3]);
			break;
		case 3:
			OutPWM3((CMDBuffer[CMDRunNum][1] << 8) + CMDBuffer[CMDRunNum][2],
					CMDBuffer[CMDRunNum][3]);
			break;
		case 4:
			OutPWM4((CMDBuffer[CMDRunNum][1] << 8) + CMDBuffer[CMDRunNum][2],
					CMDBuffer[CMDRunNum][3]);
			break;
		case 5:
			OutPWM5((CMDBuffer[CMDRunNum][1] << 8) + CMDBuffer[CMDRunNum][2],
					CMDBuffer[CMDRunNum][3]);
			break;
		case 6:
			OutPWM6((CMDBuffer[CMDRunNum][1] << 8) + CMDBuffer[CMDRunNum][2],
					CMDBuffer[CMDRunNum][3]);
			break;
		}
		CMDBuffer[CMDRunNum][0] = 0;
		CMDBuffer[CMDRunNum][1] = 0;
		CMDBuffer[CMDRunNum][2] = 0;
		CMDBuffer[CMDRunNum][3] = 0;
		CMDRunNum++;
		if (CMDRunNum == MAX_CMD_SIZE)
			CMDRunNum = 0;
	}
}
/**
 * 函数功能: Modbus CRC16 校验
 * 输入参数: *puchMsg: 需校验字符串起始地址 | usDataLen: 需校验位数
 * 返 回 值: 0xFFFF: 同时返回高低位校验码
 * 说    明: 无
 */
unsigned short CRC16(uint8_t *puchMsg, unsigned short usDataLen) {
	uint8_t uchCRCHi = 0xFF; /* 高CRC字节初始化 */
	uint8_t uchCRCLo = 0xFF; /* 低CRC 字节初始化 */
	unsigned uIndex; /* CRC循环中的索引 */
	while (usDataLen--) /* 传输消息缓冲区 */
	{
		uIndex = uchCRCHi ^ *puchMsg++; /* 计算CRC */
		uchCRCHi = uchCRCLo ^ auchCRCHi[uIndex];
		uchCRCLo = auchCRCLo[uIndex];
	}
	return (uchCRCHi << 8 | uchCRCLo);
}
/**
 * 函数功能: 串口中断回调函数
 * 输入参数: UartHandle：串口外设设备句柄
 * 返 回 值: 无
 * 说    明: 无
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef* uartHandle) {
	/* 判断起始符&结束符 */
	if (RxBuffer[0] == 0xFF && RxBuffer[7] == 0xEE) {
		unsigned short CRCCal = CRC16(&RxBuffer[1], 4);
		/* 判断CRC校验 */
		if ((RxBuffer[5] == ((CRCCal & 0xFF00) >> 8))
				&& (RxBuffer[6] == (CRCCal & 0xFF))) {
			/* 命令缓存达到最大值, 重新循环 */
			if (CMDRevNum > MAX_CMD_SIZE)
				CMDRevNum = 0;
			/* 缓存命令 */
			CMDBuffer[CMDRevNum][0] = RxBuffer[1];
			CMDBuffer[CMDRevNum][1] = RxBuffer[2];
			CMDBuffer[CMDRevNum][2] = RxBuffer[3];
			CMDBuffer[CMDRevNum][3] = RxBuffer[4];
			CMDRevNum++;
			/* 处理特殊命令: 暂未定义 */
			if (RxBuffer[1] == 99) {
				memset(CMDBuffer, 0, sizeof(CMDBuffer));
				CMDRevNum = 0;
				CMDRunNum = 0;
			}

			HAL_UART_Transmit(&huart2, RxBuffer, 8, 0xFFFF);

			/* 清除接收缓存 */
			memset(RxBuffer, 0, 8);
		}
	}
	/* 重启串口接收缓存 */
	HAL_UART_Receive_IT(&huart2, RxBuffer, 8);
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  file: The file name as string.
 * @param  line: The line in file as a number.
 * @retval None
 */
void _Error_Handler(char *file, int line) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while (1) {
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
void assert_failed(uint8_t* file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
