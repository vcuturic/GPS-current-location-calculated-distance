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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "distance_lib.h"
#include "string.h"
#include "stdlib.h"
#include "ctype.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NEW_LINE "\r\n"
#define ERROR_CHAR 'e'
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
int writeOn = 0;
int confirmationInput = 0;
double distance;
double lat_1 = 52.5200;
double lon_1 = 13.4050;
double lat_2 = 48.8566;
double lon_2 = 2.3522;
char receivedCoordinates[32];
uint8_t UART1_rxBuffer[32] = { 0 };

char *listOfGpsData[] =
{
	"$GPRMC,093121.691,A,S404.2675,N,00159.7569,W,000.0,000.0,300723„,A*7C\r\n",
	"$GPGGA,093121.691,5404.2675,N,00159.7569,W,1,10,4.00,100.0,M,S0.0,M„*7A\r\n",
	"$GPGSA,A,3,13,11,20,28,14,18,16,21,22,19„,4.00,3.20,2.40*OB\r\n"
};

// MESSAGES

char msg_InvalidCoordinates[] = "You entered invalid coordinates, try again:\r\n";
char msg_CoordinateInputText[] = "Enter latitude and longitude coordinates:\r\n";
char msg_CoordinateInputTextFormat[] = "(format: 'latitude longitude*')\r\n";
char msg_Hint[] = "(Do not use backspace. if you entered incorrect character, type '...*' to start again.)\r\n";
char msg_KindQuestion[] = "\r\nThis is your input, if you are not satisfied type 0, otherwise type 1.\r\n";
char msg_Success[] = "\r\nYou successfully forwarded the coordinates, the result is calculating...\r\n";
char msg_DistanceInfo[] = "The distance between your position and entered one is:";
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void mystrcat(char *string, char c) {
	int len = strlen(string);

	string[len] = c;

	string[len+1] = '\0';
}
void intToStr(int num, char *str) {
	int length = 1;
	int temp = num;
	while (temp /= 10) {
		length++;
	}

	for (int i = 0; i < length; i++) {
		str[length - i - 1] = ('0' + num % 10);
		num /= 10;
	}
}

void removeCharacter(char *str, int i) {
    if (str == NULL || i < 0 || i >= strlen(str)) {
        return;
    }

    for (; str[i] != '\0'; i++) {
        str[i] = str[i + 1];
    }
}

void fixInput(char *receivedCoords) {

	for(int i=0; i<strlen(receivedCoords); i++) {
		char c = receivedCoords[i];
		if(!isdigit(c) && c != '.' && c != ' ') {
			removeCharacter(receivedCoords, i);
		}
	}
}

void UART_SEND(char *message) {
	HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen(message), 100);
}

int validCoordinates(char *receivedCoords) {
	int countPoints = 0;
	for(int i=0; i<strlen(receivedCoords); i++) {
		char c = receivedCoords[i];
		if(c == '.')
			countPoints++;
	}

	if(countPoints > 2)
		return 0;

	return 1;
}

int charToNumber(char c) {
	if(c == '0') return 0;
	if(c == '1') return 1;
	if(c == '2') return 2;
	if(c == '3') return 3;
	if(c == '4') return 4;
	if(c == '5') return 5;
	if(c == '6') return 6;
	if(c == '7') return 7;
	if(c == '8') return 8;
	if(c == '9') return 9;
	return -1;
}

double strToFloat(char *str) {
	int counter;
	int startOfDecimals = 0;
	double number = 0;
	int lenBeforeDecimals = 0;
	int lenAfterDecimals = 0;

	for(int i=0; i<strlen(str); i++) {
		if(str[i] == '.') {
			startOfDecimals = 1;
			continue;
		}
		if(!startOfDecimals)
			lenBeforeDecimals++;
		else
			lenAfterDecimals++;
	}

	counter = lenAfterDecimals;
	int temp_lenBeforeDecimals = lenBeforeDecimals;
	int temp_lenAfterDecimals = lenAfterDecimals;

	for(int i=0; i<strlen(str); i++) {
		// 878.2324, lenBefore = 3 => 8 * 10^2 +7 *10^1 + 8*10^0
		if(str[i] == '.')
			continue;
		if(i<lenBeforeDecimals)
			number += charToNumber(str[i]) * pow(10, --temp_lenBeforeDecimals);
		else {
			// 2 => lenAfterDecimals = 4 => 2 / 10^1 == 0.2
			// 3 => lenAfterDecimals = 4 => 3 / 10^2 == 0.03
			number += charToNumber(str[i]) / pow(10, temp_lenAfterDecimals-(--counter));
		}
	}

	return number;
}

void extractCoordinates(char *receivedCoords) {
	char latitude[16], longitude[16];
	int idx = 0;
	int longitudeInput = 0;

	receivedCoords[strlen(receivedCoords)-1] = '\0'; // drop *

	for(int i=0; i<strlen(receivedCoords); i++) {
		if(receivedCoords[i] != ' ') {
			if(!longitudeInput)
				latitude[idx++] = receivedCoords[i];
			else
				longitude[idx++] = receivedCoords[i];
		}
		else {
			latitude[idx] = '\0';
			longitudeInput = 1;
			idx = 0;
		}
	}

	longitude[idx] = '\0';

	lat_2 = strToFloat(latitude);
	lon_2 = strToFloat(longitude);
}

void reverse(char* str, int len)
{
    int i = 0, j = len - 1, temp;
    while (i < j) {
        temp = str[i];
        str[i] = str[j];
        str[j] = temp;
        i++;
        j--;
    }
}

// Converts a given integer x to string str[].
// d is the number of digits required in the output.
// If d is more than the number of digits in x,
// then 0s are added at the beginning.
int intToStr2(int x, char str[], int d)
{
    int i = 0;
    while (x) {
        str[i++] = (x % 10) + '0';
        x = x / 10;
    }

    // If number of digits required is more, then
    // add 0s at the beginning
    while (i < d)
        str[i++] = '0';

    reverse(str, i);
    str[i] = '\0';
    return i;
}

// Converts a floating-point/double number to a string.
void ftoa(float n, char* res, int afterpoint)
{
    // Extract integer part
    int ipart = (int)n;

    // Extract floating part
    float fpart = n - (float)ipart;

    // convert integer part to string
    int i = intToStr2(ipart, res, 0);

    // check for display option after point
    if (afterpoint != 0) {
        res[i] = '.'; // add dot

        // Get the value of fraction part upto given no.
        // of points after dot. The third parameter
        // is needed to handle cases like 233.007
        fpart = fpart * pow(10, afterpoint);

        intToStr2((int)fpart, res + i + 1, afterpoint);
    }
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_USART1_UART_Init();
	/* USER CODE BEGIN 2 */
	HAL_UART_Receive_IT(&huart1, UART1_rxBuffer, 1);
//  char *string = (char*)malloc(14 * sizeof(char));

//  intToStr(distance, string);
//  itoa(distance, string, 10);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		if(!writeOn) {
			HAL_UART_Transmit(&huart1, (uint8_t*)listOfGpsData[0], strlen(listOfGpsData[0]), 100);
			HAL_UART_Transmit(&huart1, (uint8_t*)listOfGpsData[1], strlen(listOfGpsData[1]), 100);
			HAL_UART_Transmit(&huart1, (uint8_t*)listOfGpsData[2], strlen(listOfGpsData[2]), 100);
			HAL_Delay(1000);
			writeOn = 1;
		}
		if(writeOn == 1) {
			UART_SEND(msg_CoordinateInputText);
			UART_SEND(msg_CoordinateInputTextFormat);
			UART_SEND(msg_Hint);
			writeOn = 2;
		}


//		HAL_UART_Transmit(&huart1, (uint8_t*)"Calculating..\r\n", strlen("Calculating..\r\n"), 100);
//		HAL_Delay(1000);
//
//		distance = haversineDistance(lat_1, lon_1, lat_2, lon_2);
//		HAL_UART_Transmit(&huart1, (uint8_t*)"distance\r\n", 10, 100);
//
//		char buffer[20];
//		snprintf(buffer, sizeof(buffer), "%.2fkm\r\n", distance);
////		intToStr(distance, buffer);
//		HAL_UART_Transmit(&huart1, (uint8_t*) buffer, strlen(buffer), 100);

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(OUTPUT_GPIO_Port, OUTPUT_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin : OUTPUT_Pin */
	GPIO_InitStruct.Pin = OUTPUT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(OUTPUT_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

	if(confirmationInput) {
		if((char)UART1_rxBuffer[0] == '1') {
			char buffer[20];
			UART_SEND(msg_Success);
			extractCoordinates(receivedCoordinates);
			distance = haversineDistance(lat_1, lon_1, lat_2, lon_2);
			ftoa(distance, buffer, 2);
			UART_SEND(NEW_LINE);
			UART_SEND(msg_DistanceInfo);
			UART_SEND(NEW_LINE);
			UART_SEND(buffer);
			UART_SEND("km");
			UART_SEND(NEW_LINE);
		}
		else {
			UART_SEND(NEW_LINE);
			UART_SEND(msg_InvalidCoordinates);
			receivedCoordinates[0] = '\0';
			confirmationInput = 0;
			HAL_UART_Receive_IT(&huart1, UART1_rxBuffer, 1);
		}
	}
	else {
		if((char)UART1_rxBuffer[0] != '*') {
			mystrcat(receivedCoordinates, (char)UART1_rxBuffer[0]);
			HAL_UART_Receive_IT(&huart1, UART1_rxBuffer, 1);
		}
		else {
			if(!validCoordinates(receivedCoordinates)) {
				UART_SEND(NEW_LINE);
				UART_SEND(msg_InvalidCoordinates);
			}
			else {
				fixInput(receivedCoordinates);
				UART_SEND(msg_KindQuestion);
				UART_SEND(NEW_LINE);
				UART_SEND(receivedCoordinates);
				UART_SEND(NEW_LINE);
				confirmationInput = 1;
				HAL_UART_Receive_IT(&huart1, UART1_rxBuffer, 1);
				HAL_GPIO_WritePin(OUTPUT_GPIO_Port, OUTPUT_Pin, GPIO_PIN_RESET);
			}
		}
	}
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
