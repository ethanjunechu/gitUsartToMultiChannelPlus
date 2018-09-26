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
static uint32_t period[202] = { 8438, 8390, 8388, 8386, 8383, 8380, 8377, 8374,
		8371, 8368, 8364, 8360, 8356, 8352, 8348, 8343, 8339, 8334, 8328, 8323,
		8317, 8311, 8305, 8298, 8291, 8284, 8276, 8268, 8259, 8250, 8241, 8231,
		8221, 8210, 8199, 8187, 8175, 8162, 8149, 8134, 8120, 8104, 8088, 8071,
		8053, 8034, 8015, 7995, 7974, 7951, 7928, 7904, 7879, 7853, 7825, 7797,
		7767, 7736, 7704, 7670, 7636, 7599, 7562, 7523, 7482, 7440, 7397, 7351,
		7305, 7256, 7206, 7155, 7102, 7047, 6990, 6932, 6872, 6810, 6747, 6682,
		6615, 6547, 6477, 6405, 6332, 6258, 6182, 6104, 6026, 5946, 5865, 5783,
		5700, 5616, 5531, 5445, 5359, 5272, 5185, 5098, 5010, 4922, 4834, 4746,
		4659, 4571, 4485, 4398, 4313, 4228, 4144, 4061, 3979, 3898, 3818, 3739,
		3662, 3586, 3512, 3439, 3367, 3297, 3229, 3162, 3097, 3034, 2972, 2912,
		2854, 2797, 2742, 2689, 2637, 2587, 2539, 2492, 2447, 2404, 2362, 2321,
		2282, 2244, 2208, 2173, 2140, 2108, 2077, 2047, 2018, 1991, 1965, 1940,
		1915, 1892, 1870, 1849, 1829, 1809, 1791, 1773, 1756, 1740, 1724, 1709,
		1695, 1682, 1669, 1656, 1645, 1633, 1623, 1612, 1603, 1593, 1584, 1576,
		1568, 1560, 1553, 1546, 1539, 1533, 1527, 1521, 1515, 1510, 1505, 1500,
		1496, 1492, 1487, 1484, 1480, 1476, 1473, 1470, 1467, 1464, 1461, 1458,
		1456, 1453 };
static uint32_t period1[202] = { 8438, 8384, 8381, 8378, 8375, 8372, 8369, 8365,
		8362, 8358, 8354, 8349, 8345, 8340, 8335, 8330, 8325, 8319, 8313, 8307,
		8300, 8293, 8286, 8278, 8270, 8262, 8253, 8244, 8234, 8224, 8214, 8203,
		8191, 8179, 8166, 8152, 8138, 8124, 8108, 8092, 8075, 8057, 8039, 8019,
		7999, 7978, 7956, 7933, 7909, 7883, 7857, 7829, 7801, 7771, 7740, 7707,
		7673, 7638, 7601, 7563, 7523, 7482, 7439, 7395, 7348, 7300, 7251, 7199,
		7146, 7091, 7034, 6975, 6915, 6852, 6787, 6721, 6652, 6582, 6510, 6436,
		6360, 6282, 6202, 6121, 6037, 5952, 5866, 5778, 5688, 5597, 5505, 5411,
		5317, 5221, 5124, 5026, 4928, 4829, 4730, 4630, 4530, 4430, 4330, 4229,
		4130, 4030, 3931, 3833, 3735, 3639, 3543, 3448, 3355, 3262, 3171, 3082,
		2993, 2907, 2822, 2739, 2657, 2578, 2500, 2424, 2350, 2277, 2207, 2139,
		2072, 2007, 1945, 1884, 1825, 1768, 1713, 1660, 1609, 1559, 1511, 1465,
		1420, 1377, 1336, 1296, 1258, 1221, 1186, 1152, 1120, 1089, 1059, 1030,
		1002, 976, 951, 927, 904, 881, 860, 840, 821, 802, 784, 767, 751, 736,
		721, 707, 694, 681, 669, 657, 646, 635, 625, 615, 606, 597, 589, 581,
		573, 566, 559, 553, 546, 540, 535, 529, 524, 519, 514, 510, 506, 502,
		498, 494, 491, 487, 484, 481, 478, 476 };
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
	HAL_UART_Receive_IT(&huart2, RxBuffer, 6);
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
	uint32_t PWMHoldTime = period1[0];
	uint32_t OldStep = Step;
	PWM1_DIR(Dir);
	PWM1_EN(0);
	for (; Step > 0; Step--) {
		//S加速
		if ((OldStep - Step) < 200 && (Step > (OldStep / 2))) {
			PWMHoldTime = period1[OldStep - Step];
		}
		//S减速
		if (Step <= 200 && (Step < (OldStep / 2))) {
			PWMHoldTime = period1[Step];
		}
		PWM1_OUT(1);
		for (iTime = 0; iTime < 5; iTime++)
			;
		PWM1_OUT(0);
		for (iTime = 0; iTime < PWMHoldTime; iTime++)
			;
	}
	PWM1_EN(1);
}
void OutPWM2(uint32_t Step, uint8_t Dir) {
	uint32_t PWMHoldTime = period1[0];
	uint32_t OldStep = Step;
	PWM2_DIR(Dir);
	PWM2_EN(0);
	for (; Step > 0; Step--) {
		//S加速
		if ((OldStep - Step) < 200 && (Step > (OldStep / 2))) {
			PWMHoldTime = period1[OldStep - Step];
		}
		//S减速
		if (Step <= 200 && (Step < (OldStep / 2))) {
			PWMHoldTime = period1[Step];
		}
		PWM2_OUT(1);
		for (iTime = 0; iTime < 5; iTime++)
			;
		PWM2_OUT(0);
		for (iTime = 0; iTime < PWMHoldTime; iTime++)
			;
	}
	PWM2_EN(1);
}
void OutPWM3(uint32_t Step, uint8_t Dir) {
	uint32_t PWMHoldTime = period[0];
	uint32_t OldStep = Step;
	PWM3_DIR(Dir);
	PWM3_EN(0);
	for (; Step > 0; Step--) {
		//S加速
		if ((OldStep - Step) < 200 && (Step > (OldStep / 2))) {
			PWMHoldTime = period[OldStep - Step];
		}
		//S减速
		if (Step <= 200 && (Step < (OldStep / 2))) {
			PWMHoldTime = period[Step];
		}
		PWM3_OUT(1);
		for (iTime = 0; iTime < 5; iTime++)
			;
		PWM3_OUT(0);
		for (iTime = 0; iTime < PWMHoldTime; iTime++)
			;
	}
	PWM3_EN(1);
}
void OutPWM4(uint32_t Step, uint8_t Dir) {
	uint32_t PWMHoldTime = period[0];
	uint32_t OldStep = Step;
	PWM4_DIR(Dir);
	PWM4_EN(0);
	for (; Step > 0; Step--) {
		//S加速
		if ((OldStep - Step) < 200 && (Step > (OldStep / 2))) {
			PWMHoldTime = period[OldStep - Step];
		}
		//S减速
		if (Step <= 200 && (Step < (OldStep / 2))) {
			PWMHoldTime = period[Step];
		}
		PWM4_OUT(1);
		for (iTime = 0; iTime < 5; iTime++)
			;
		PWM4_OUT(0);
		for (iTime = 0; iTime < PWMHoldTime; iTime++)
			;
	}
	PWM4_EN(1);
}
void OutPWM5(uint32_t Step, uint8_t Dir) {
	uint32_t PWMHoldTime = period[0];
	uint32_t OldStep = Step;
	PWM5_DIR(Dir);
	PWM5_EN(0);
	for (; Step > 0; Step--) {
		//S加速
		if ((OldStep - Step) < 200 && (Step > (OldStep / 2))) {
			PWMHoldTime = period[OldStep - Step];
		}
		//S减速
		if (Step <= 200 && (Step < (OldStep / 2))) {
			PWMHoldTime = period[Step];
		}
		PWM5_OUT(1);
		for (iTime = 0; iTime < 5; iTime++)
			;
		PWM5_OUT(0);
		for (iTime = 0; iTime < PWMHoldTime; iTime++)
			;
	}
	PWM5_EN(1);
}
void OutPWM6(uint32_t Step, uint8_t Dir) {
	uint32_t PWMHoldTime = period[0];
	uint32_t OldStep = Step;
	PWM6_DIR(Dir);
	PWM6_EN(0);
	for (; Step > 0; Step--) {
		//S加速
		if ((OldStep - Step) < 200 && (Step > (OldStep / 2))) {
			PWMHoldTime = period[OldStep - Step];
		}
		//S减速
		if (Step <= 200 && (Step < (OldStep / 2))) {
			PWMHoldTime = period[Step];
		}
		PWM6_OUT(1);
		for (iTime = 0; iTime < 5; iTime++)
			;
		PWM6_OUT(0);
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
		default:
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
	unsigned short CRCCal = CRC16(&RxBuffer[0], 4);
	/* 判断CRC校验 */
	if ((RxBuffer[4] == ((CRCCal & 0xFF00) >> 8))
			&& (RxBuffer[5] == (CRCCal & 0xFF))) {
		/* 命令缓存达到最大值, 重新循环 */
		if (CMDRevNum > MAX_CMD_SIZE)
			CMDRevNum = 0;
		/* 缓存命令 */
		CMDBuffer[CMDRevNum][0] = RxBuffer[0];
		CMDBuffer[CMDRevNum][1] = RxBuffer[1];
		CMDBuffer[CMDRevNum][2] = RxBuffer[2];
		CMDBuffer[CMDRevNum][3] = RxBuffer[3];
		CMDRevNum++;
		/* 处理特殊命令: 暂未定义 */
		if (RxBuffer[0] == 99) {
			memset(CMDBuffer, 0, sizeof(CMDBuffer));
			CMDRevNum = 0;
			CMDRunNum = 0;
		}

//		HAL_UART_Transmit(&huart2, RxBuffer, 6, 0xFFFF);

		/* 清除接收缓存 */
		memset(RxBuffer, 0, 6);
	}
	/* 重启串口接收缓存 */
	HAL_UART_Receive_IT(&huart2, RxBuffer, 6);
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
