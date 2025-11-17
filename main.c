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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "fatfs_sd.h"
#include "string.h"
#include "stdio.h"

#include "ili9341.h" // librería para funcionamiento de la pantalla LCD
#include "Bitmaps.h" // librería para recursos gráficos
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
SPI_HandleTypeDef hspi1;

/* --- Variables para el sistema FATFS y la SD --- */
FATFS fs;
FATFS *pfs;
FIL fil;
FRESULT fres;
DWORD fre_clust;
uint32_t totalSpace, freeSpace;
char buffer[100]; // Buffer

/* --- Variables para antirrebote --- */
uint32_t ultimo_tiempo_pb1 = 0;
uint32_t ultimo_tiempo_pb2 = 0;
uint32_t ultimo_tiempo_pb3 = 0;

/* --- Flags para los botones --- */
uint8_t estado1 = 0;
uint8_t estado2 = 0;
uint8_t estado3 = 0;

/* --- Contadores --- */
int8_t contador1 = 0;
int8_t contador2 = 0;
int8_t contador3 = 0;

uint8_t datoTEMP = 0; // Variable para almacenar la temperatura

extern uint16_t fondo[]; // Imagen de fondo

char temp_str[20]; // String para almacenar el texto
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* Función para transmitir strings por UART */
void transmit_uart(char *string){
	uint16_t len = strlen(string);
	HAL_UART_Transmit(&huart2, (uint8_t*) string, len, 1000);
}

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
	MX_SPI1_Init();
	MX_FATFS_Init();
	MX_USART2_UART_Init();
	MX_USART3_UART_Init();
	/* USER CODE BEGIN 2 */

	LCD_Init(); // Inicializa la pantalla LCD

	/* --- Pantalla de Carga --- */
	LCD_Clear(0x00);
	LCD_Bitmap(0, 0, 320, 240, fondo); // Dibuja el fondo
	LCD_Print("Inicializando...", 30, 112, 2, 0x0000, 0xFFFF);
	HAL_Delay(3000);

	/* --- Pantalla de Bienvenida --- */
	LCD_Clear(0x00);
	LCD_Bitmap(0, 0, 320, 240, fondo);
	LCD_Print("Bienvenido/a", 60, 112, 2, 0x0000, 0xFFFF);

	LCD_Bitmap(10, 170, 60, 60, katha);
	LCD_Bitmap(250, 170, 60, 60, pablo);

	/* --- Montaje de la tarjeta Micro SD --- */
	fres = f_mount(&fs, "/", 0);
	if (fres == FR_OK) {
		transmit_uart("Micro SD card is mounted successfully!\n");
	} else if (fres != FR_OK) {
		transmit_uart("Micro SD card's mount error!\n");
	}

	transmit_uart("\r\n--- Inicializando ---\r\n");
	transmit_uart("\r\n--- Termómetro activo ---\r\n");
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* Lectura de botones */
		if (estado1==1){
			estado1=0; // Limpia la bandera
			contador1++; // Incrementa el contador 1
			if(contador1==2){
				contador1=1;
			}
		}
		if (estado2==1){
			estado2=0;
			contador2++;
			if(contador2==2){
				contador2=1;
			}
		}
		if (estado3==1){
			estado3=0;
			contador3++;
			if(contador3==2){
				contador3=1;
			}
		}


		/* Estados */

		/* --- Acción 1: Medir Temperatura --- */
		if(contador1==1){
			// 1. Envía un comando al ESP32 por UART3 para pedir la temperatura
			uint8_t cmd_medir = 1;
			HAL_UART_Transmit(&huart3, &cmd_medir, 1, HAL_MAX_DELAY);

			// 2. Espera recibir un byte de respuesta
			HAL_StatusTypeDef res = HAL_UART_Receive(&huart3, &datoTEMP, sizeof(datoTEMP), HAL_MAX_DELAY);

			// 3. Procesa y muestra la respuesta en la LCD y UART
			if (res == HAL_OK) {
				sprintf(buffer, "Respuesta recibida de ESP32: %d\r\n", datoTEMP);
				transmit_uart(buffer);

				// Guarda el string de temperatura
				sprintf(temp_str, "%d C", datoTEMP);

				// 4. Actualiza la pantalla LCD
				LCD_Clear(0x00);
				LCD_Bitmap(0, 0, 320, 240, fondo);
				LCD_Print("Dato obtenido:", 50, 100, 2, 0x0000, 0xFFFF);
				LCD_Print(temp_str, 120, 125, 2, 0x0000, 0xFFFF);
				LCD_Bitmap(10, 170, 60, 60, katha);
				LCD_Bitmap(250, 170, 60, 60, pablo);

			} else {
				transmit_uart("Error: No se recibio respuesta del ESP32.\r\n");
			}

			contador1=0; // Resetea el contador
		}

		/* --- Acción 2: Guardar Dato en SD --- */
		if(contador2==1){

			// 1. Prepara el string a guardar
			sprintf(buffer, "Valor de temperatura guardado: %d\r\n", datoTEMP);

			// 2. Envía comando de "guardar" al ESP32
			uint8_t cmd_guardar = 2;
			HAL_UART_Transmit(&huart3, &cmd_guardar, 1, HAL_MAX_DELAY);

			// 3. Abre el archivo "Reporte.txt"
			fres = f_open(&fil, "Reporte.txt", FA_OPEN_APPEND | FA_WRITE | FA_READ);
			if (fres == FR_OK) {
				transmit_uart("File opened for reading.\n");
			} else if (fres != FR_OK) {
				transmit_uart("File was not opened for reading!\n");
			}
			// 4. Escribe el valor de temperatura en el archivo
			f_puts(buffer, &fil);

			// 5. Cierra el archivo
			fres = f_close(&fil);
			if (fres == FR_OK) {
				transmit_uart("The file is closed.\n");
			} else if (fres != FR_OK) {
				transmit_uart("The file was not closed.\n");
			}

			// 6. Muestra mensaje de "Guardando" en la LCD
			LCD_Clear(0x00);
			LCD_Bitmap(0, 0, 320, 240, fondo);
			LCD_Bitmap(10, 170, 60, 60, katha);
			LCD_Bitmap(250, 170, 60, 60, pablo);
			LCD_Print("Guardando dato...", 40, 112, 2, 0x0000, 0xFFFF);

			HAL_Delay(3000);

			// 7. Restaura la pantalla a como estaba (mostrando la última temp)
			LCD_Clear(0x00);
			LCD_Bitmap(0, 0, 320, 240, fondo);
			LCD_Print("Dato obtenido:", 50, 100, 2, 0x0000, 0xFFFF);
			LCD_Print(temp_str, 120, 125, 2, 0x0000, 0xFFFF); // Usa el string global
			LCD_Bitmap(10, 170, 60, 60, katha);
			LCD_Bitmap(250, 170, 60, 60, pablo);

			contador2=0;

		}

		/* --- Acción 3: Desmontar SD y Finalizar --- */
		if(contador3==1){

			// 1. Envía comando de "cerrar" al ESP32
			uint8_t cmd_cerrar = 3;
			HAL_UART_Transmit(&huart3, &cmd_cerrar, 1, HAL_MAX_DELAY);

			// 2. Desmonta la SD card
			f_mount(NULL, "", 1);
			if (fres == FR_OK) {
				transmit_uart("The Micro SD card is unmounted!\n");
				// 3. Muestra mensaje en la LCD
				LCD_Clear(0x00);
				LCD_Bitmap(0, 0, 320, 240, fondo);
				LCD_Print("SD Desmontada", 64, 112, 2, 0x0000, 0xFFFF);
				LCD_Bitmap(10, 170, 60, 60, katha);
				LCD_Bitmap(250, 170, 60, 60, pablo);

			} else if (fres != FR_OK) {
				transmit_uart("The Micro SD was not unmounted!");
			}

			HAL_Delay(3000); // Muestra "SD Desmontada"

			// 4. Muestra la pantalla final de "Felicidades"
			LCD_Clear(0x00);
			LCD_Bitmap(0, 0, 320, 240, fondo);
			LCD_Print("Felicidades!", 60, 100, 2, 0x0000, 0xFFFF);
			LCD_Print("Nota: 100 pts", 50, 125, 2, 0x0000, 0xFFFF);
			LCD_Bitmap(10, 170, 60, 60, katha);
			LCD_Bitmap(250, 170, 60, 60, pablo);

			contador3=0;

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
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 16;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	RCC_OscInitStruct.PLL.PLLQ = 2;
	RCC_OscInitStruct.PLL.PLLR = 2;
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
}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void)
{

	/* USER CODE BEGIN SPI1_Init 0 */

	/* USER CODE END SPI1_Init 0 */

	/* USER CODE BEGIN SPI1_Init 1 */

	/* USER CODE END SPI1_Init 1 */
	/* SPI1 parameter configuration*/
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN SPI1_Init 2 */

	/* USER CODE END SPI1_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void)
{

	/* USER CODE BEGIN USART3_Init 0 */

	/* USER CODE END USART3_Init 0 */

	/* USER CODE BEGIN USART3_Init 1 */

	/* USER CODE END USART3_Init 1 */
	huart3.Instance = USART3;
	huart3.Init.BaudRate = 115200;
	huart3.Init.WordLength = UART_WORDLENGTH_8B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.Mode = UART_MODE_TX_RX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart3) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART3_Init 2 */

	/* USER CODE END USART3_Init 2 */

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
	HAL_GPIO_WritePin(GPIOC, LCD_RST_Pin|LCD_D1_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, LCD_RD_Pin|LCD_WR_Pin|LCD_RS_Pin|LCD_D7_Pin
			|LCD_D0_Pin|LCD_D2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, LCD_CS_Pin|LCD_D6_Pin|LCD_D3_Pin|LCD_D5_Pin
			|LCD_D4_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(SD_SS_GPIO_Port, SD_SS_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin : PB1_Pin */
	GPIO_InitStruct.Pin = PB1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(PB1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : LCD_RST_Pin LCD_D1_Pin */
	GPIO_InitStruct.Pin = LCD_RST_Pin|LCD_D1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : LCD_RD_Pin LCD_WR_Pin LCD_RS_Pin LCD_D7_Pin
                           LCD_D0_Pin LCD_D2_Pin */
	GPIO_InitStruct.Pin = LCD_RD_Pin|LCD_WR_Pin|LCD_RS_Pin|LCD_D7_Pin
			|LCD_D0_Pin|LCD_D2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : LCD_CS_Pin LCD_D6_Pin LCD_D3_Pin LCD_D5_Pin
                           LCD_D4_Pin */
	GPIO_InitStruct.Pin = LCD_CS_Pin|LCD_D6_Pin|LCD_D3_Pin|LCD_D5_Pin
			|LCD_D4_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : SD_SS_Pin */
	GPIO_InitStruct.Pin = SD_SS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(SD_SS_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : PB3_Pin PB2_Pin */
	GPIO_InitStruct.Pin = PB3_Pin|PB2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

	/* USER CODE BEGIN MX_GPIO_Init_2 */

	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
 * @brief  Callback de interrupción externa (se llama cuando se presiona un botón)
 * @param  GPIO_Pin: El pin que generó la interrupción
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	uint32_t tiempo_actual = HAL_GetTick();
	if(GPIO_Pin == PB1_Pin){ // Si fue el Botón 1
		if (tiempo_actual - ultimo_tiempo_pb1 > 300){
			estado1 = 1; // Activa la bandera para el bucle principal
			transmit_uart("solicitando temp"); // Mensaje de debug
			ultimo_tiempo_pb1 = tiempo_actual; // Guarda el tiempo de esta pulsación
		}
	}
	if(GPIO_Pin == PB2_Pin){ // Si fue el Botón 2
		if (tiempo_actual - ultimo_tiempo_pb2 > 300){
			estado2 = 1;
			transmit_uart("guardando ");
			ultimo_tiempo_pb2 = tiempo_actual;
		}
	}
	if(GPIO_Pin == PB3_Pin){ // Si fue el Botón 3
		if (tiempo_actual - ultimo_tiempo_pb3 > 300){
			estado3 = 1;
			transmit_uart("desmontando");
			ultimo_tiempo_pb3 = tiempo_actual;
		}
	}

}

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
