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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Instruction Mnemonics */
#define R_REGISTER    0x00
#define W_REGISTER    0x20
#define REGISTER_MASK 0x1F
#define ACTIVATE      0x50
#define R_RX_PL_WID   0x60
#define R_RX_PAYLOAD  0x61
#define W_TX_PAYLOAD  0xA0
#define W_ACK_PAYLOAD 0xA8
#define FLUSH_TX      0xE1
#define FLUSH_RX      0xE2
#define REUSE_TX_PL   0xE3
#define NOP           0xFF

/* Memory Map */
#define CONFIG      0x00
#define EN_AA       0x01
#define EN_RXADDR   0x02
#define SETUP_AW    0x03
#define SETUP_RETR  0x04
#define RF_CH       0x05
#define RF_SETUP    0x06
#define STATUS      0x07
#define OBSERVE_TX  0x08
#define CD          0x09
#define RX_ADDR_P0  0x0A
#define RX_ADDR_P1  0x0B
#define RX_ADDR_P2  0x0C
#define RX_ADDR_P3  0x0D
#define RX_ADDR_P4  0x0E
#define RX_ADDR_P5  0x0F
#define TX_ADDR     0x10
#define RX_PW_P0    0x11
#define RX_PW_P1    0x12
#define RX_PW_P2    0x13
#define RX_PW_P3    0x14
#define RX_PW_P4    0x15
#define RX_PW_P5    0x16
#define FIFO_STATUS 0x17
#define DYNPD	    0x1C
#define FEATURE	    0x1D

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */
	void TransmitSetup(uint8_t DataT[]);
	void ReceiveSetup();
	void uSDelay(uint16_t delay);
	void Transmit(uint8_t data[]);
	void Receive();
  uint8_t data[2]={1,2};
	HAL_StatusTypeDef status1=!(HAL_OK);
	uint8_t StatusCheckx[2];
	uint8_t RxData2[3];
  uint8_t RxData22;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

		

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void){
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
  MX_TIM1_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start(&htim1);
	HAL_Delay(100);
	Receive();//Activate the receiver
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	data[0]=1;data[1]=4;
  
	Transmit(data);//Transmit an array of data(2 bytes)
	HAL_Delay(10);
	data[0]=9;data[1]=16;
	
	Transmit(data);//Retransmit an array of data(2 bytes)
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
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
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 16-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CE_Pin|CE1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CSN_Pin|CSN1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : CE_Pin CSN_Pin CSN1_Pin CE1_Pin */
  GPIO_InitStruct.Pin = CE_Pin|CSN_Pin|CSN1_Pin|CE1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : IRQ_TX_Pin */
  GPIO_InitStruct.Pin = IRQ_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IRQ_TX_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : IRQ_Rx_Pin */
  GPIO_InitStruct.Pin = IRQ_Rx_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IRQ_Rx_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void uSDelay(uint16_t Delay)
{
	__HAL_TIM_SET_COUNTER(&htim1,0);
	while(__HAL_TIM_GET_COUNTER(&htim1)<Delay);
}

	
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
	{
		if(GPIO_Pin==GPIO_PIN_0)
		{
			//Go to Standby-I Mode
		HAL_GPIO_WritePin(CE_GPIO_Port,CE_Pin,GPIO_PIN_RESET);
		
			
		//Check IRQ PIN TX
		//Flush TX
	  uint8_t FlushTx=FLUSH_TX;
		uint8_t RxData;
	  HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_RESET);
	  HAL_SPI_TransmitReceive(&hspi1,&FlushTx,&RxData22,1,1);
	  HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_SET);
	
	  //Flush RX
	  uint8_t FlushRx=FLUSH_RX;
	  HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_RESET);
	  HAL_SPI_TransmitReceive(&hspi1,&FlushRx,&RxData,1,1);
	  HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_SET);
		}
		
		else
		{
			uint8_t TxData[2];
			uint8_t RxDatax[2];
			//Go to Standby-I Mode
		HAL_GPIO_WritePin(CE1_GPIO_Port,CE1_Pin,GPIO_PIN_RESET);
		
		//Check IRQ PIN RX
		//Read Payload Data
		uint8_t R_Rx_Payload[3]={(R_REGISTER|R_RX_PAYLOAD),0,0};
		uint8_t RxData;
	  HAL_GPIO_WritePin(CSN1_GPIO_Port, CSN1_Pin, GPIO_PIN_RESET);
	  status1=HAL_SPI_TransmitReceive(&hspi2,(uint8_t *)R_Rx_Payload,(uint8_t *)RxData2,3,1);
	  HAL_GPIO_WritePin(CSN1_GPIO_Port, CSN1_Pin, GPIO_PIN_SET);
		
		//Flush TX
	  uint8_t FlushTx=FLUSH_TX;
	  HAL_GPIO_WritePin(CSN1_GPIO_Port, CSN1_Pin, GPIO_PIN_RESET);
	  HAL_SPI_TransmitReceive(&hspi2,&FlushTx,&RxData,1,1);
	  HAL_GPIO_WritePin(CSN1_GPIO_Port, CSN1_Pin, GPIO_PIN_SET);
	
	  //Flush RX
	  uint8_t FlushRx=FLUSH_RX;
	  HAL_GPIO_WritePin(CSN1_GPIO_Port, CSN1_Pin, GPIO_PIN_RESET);
	  HAL_SPI_TransmitReceive(&hspi2,&FlushRx,&RxData,1,1);
	  HAL_GPIO_WritePin(CSN1_GPIO_Port, CSN1_Pin, GPIO_PIN_SET);
		
		//Clear Status Register
	TxData[0]=(W_REGISTER|STATUS);
	uint8_t temp=(((0b0111)<<4)|((RxData)&0b00001111));
	TxData[1]=temp;
	HAL_GPIO_WritePin(CSN1_GPIO_Port, CSN1_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi2,(uint8_t *)TxData,(uint8_t *)RxDatax,2,1);
	HAL_GPIO_WritePin(CSN1_GPIO_Port, CSN1_Pin, GPIO_PIN_SET);
	
	//Go to RX Mode again to search for more packets
		HAL_GPIO_WritePin(CE1_GPIO_Port,CE1_Pin,GPIO_PIN_SET);
		
	  }
		
		
	}
	
	
void Transmit(uint8_t DataT[])
{
	
	uint8_t RxDataX[3];
	
	//Flush TX
	  uint8_t FlushTx=FLUSH_TX;
		uint8_t RxDataF;
	  HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_RESET);
	  HAL_SPI_TransmitReceive(&hspi1,&FlushTx,&RxDataF,1,1);
	  HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_SET);
	
	  //Flush RX
	  uint8_t FlushRx=FLUSH_RX;
	  HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_RESET);
	  HAL_SPI_TransmitReceive(&hspi1,&FlushRx,&RxDataF,1,1);
	  HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_SET);
	
	//Clear Status Register Bits
	uint8_t temp=(((0b0111)<<4)|((RxDataF)&0b00001111));
	uint8_t TxData[2]={(W_REGISTER|STATUS),temp};
	uint8_t RxData[2];
	HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1,(uint8_t *)TxData,(uint8_t *)RxData,2,1);
	HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_SET);
	
	
	//Enable AUTO ACK for data pipe 0 only
	TxData[0]=(W_REGISTER|EN_AA);
	TxData[1]=1;
	HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1,(uint8_t *)TxData,(uint8_t *)RxData,2,1);
	HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_SET);
	
	//Enable Rx Address 0 only
	TxData[0]=(W_REGISTER|EN_RXADDR);
	TxData[1]=1;
	HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1,(uint8_t *)TxData,(uint8_t *)RxData,2,1);
	HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_SET);
	

		//Payload Data
	//Write Tx Payload Data Pipe 0 width=2 bytes
	uint8_t TxData1[3]={W_TX_PAYLOAD,DataT[0],DataT[1]};
	HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1,(uint8_t *)TxData1,(uint8_t *)RxDataX,3,1);
	HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_SET);
	
	//Enable Address Width
	TxData[0]=(W_REGISTER|SETUP_AW);
	TxData[1]=0b00000011;
	HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1,(uint8_t *)TxData,(uint8_t *)RxData,2,1);
	HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_SET);
	
	//500 microseconds timeout for auto retransmission and 5 tries
	TxData[0]=(W_REGISTER|SETUP_RETR);
	TxData[1]=(0b0001<<4)|(0b0101);
	HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1,(uint8_t *)TxData,(uint8_t *)RxData,2,1);
	HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_SET);
	
	//Enable RF Channel
	TxData[0]=(W_REGISTER|RF_CH);
	TxData[1]=0b00000010;
	HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1,(uint8_t *)TxData,(uint8_t *)RxData,2,1);
	HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_SET);
	
	//RF Setup Register- Air Data Rate= 250Kbps, RF Output Power= -6dBm
	TxData[0]=(W_REGISTER|RF_SETUP);
	TxData[1]=(0b0<<6)|(0b100100);
	HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1,(uint8_t *)TxData,(uint8_t *)RxData,2,1);
	HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_SET);
	
	//Write Receive address of data pipe 0 as PRATH
	uint8_t RxAddr[6]={(W_REGISTER|RX_ADDR_P0),(uint8_t )'P',(uint8_t )'R',(uint8_t )'A',(uint8_t )'T',(uint8_t )'H'};
	uint8_t RxDataAddress[6];
	HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1,(uint8_t *)RxAddr,(uint8_t *)RxDataAddress,6,1);
	HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_SET);
	
	//Write Transmit address of data pipe 0 as PRATH
	uint8_t RxAddr1[6]={(W_REGISTER|TX_ADDR),(uint8_t )'P',(uint8_t )'R',(uint8_t )'A',(uint8_t )'T',(uint8_t )'H'};
	HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1,(uint8_t *)RxAddr1,(uint8_t *)RxDataAddress,6,1);
	HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_SET);
	
	//1. For Transmitter
	
		//PRIM RX=0, 1 byte of ACK, Assert TX_DS as active low interrupt for IRQ Pin
	uint8_t TxDataZ[2]={(W_REGISTER|CONFIG),0b01011010};
	
	uint8_t RxDataZ[2];
	
	HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1,(uint8_t *)TxDataZ,(uint8_t *)RxDataZ,2,1);
	HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_SET);
	

	//Wait for 1.5ms to transition from Power down to Standby Mode

	uSDelay(1500);
	
	//CE Pin pulse for TX Mode
	HAL_GPIO_WritePin(CE_GPIO_Port,CE_Pin,GPIO_PIN_SET);
	uSDelay(10);
	
	
}

void Receive()
{
	//2. For Receiver
	uint8_t TxData[2];
	uint8_t RxData[2];
	uint8_t RxDataAddress[6];
	uint8_t Stat;
	
	//Flush TX
	  uint8_t FlushTx=FLUSH_TX;
	  HAL_GPIO_WritePin(CSN1_GPIO_Port, CSN1_Pin, GPIO_PIN_RESET);
	  HAL_SPI_TransmitReceive(&hspi2,&FlushTx,&Stat,1,1);
	  HAL_GPIO_WritePin(CSN1_GPIO_Port, CSN1_Pin, GPIO_PIN_SET);
	
	  //Flush RX
	  uint8_t FlushRx=FLUSH_RX;
	  HAL_GPIO_WritePin(CSN1_GPIO_Port, CSN1_Pin, GPIO_PIN_RESET);
	  HAL_SPI_TransmitReceive(&hspi2,&FlushRx,&Stat,1,1);
	  HAL_GPIO_WritePin(CSN1_GPIO_Port, CSN1_Pin, GPIO_PIN_SET);
	
	//Clear Status Register Bits
	uint8_t temp=(((0b0111)<<4)|((Stat)&0b00001111));
	TxData[0]=(W_REGISTER|STATUS);
	TxData[1]=temp;
	HAL_GPIO_WritePin(CSN1_GPIO_Port, CSN1_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi2,(uint8_t *)TxData,(uint8_t *)RxData,2,1);
	HAL_GPIO_WritePin(CSN1_GPIO_Port, CSN1_Pin, GPIO_PIN_SET);
	
	//Enable AUTO ACK for data pipe 0 only
	uint8_t TxData3[2]={(W_REGISTER|EN_AA),1};
	uint8_t RxData3[2];
	HAL_GPIO_WritePin(CSN1_GPIO_Port, CSN1_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi2,(uint8_t *)TxData3,(uint8_t *)RxData,2,1);
	HAL_GPIO_WritePin(CSN1_GPIO_Port, CSN1_Pin, GPIO_PIN_SET);
	
	
	
	//Enable Rx Address 0 only
	TxData[0]=(W_REGISTER|EN_RXADDR);
	TxData[1]=1;
	HAL_GPIO_WritePin(CSN1_GPIO_Port, CSN1_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi2,(uint8_t *)TxData,(uint8_t *)RxData,2,1);
	HAL_GPIO_WritePin(CSN1_GPIO_Port, CSN1_Pin, GPIO_PIN_SET);
	

	//Enable Address Width
	TxData[0]=(W_REGISTER|SETUP_AW);
	TxData[1]=0b00000011;
	HAL_GPIO_WritePin(CSN1_GPIO_Port, CSN1_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi2,(uint8_t *)TxData,(uint8_t *)RxData,2,1);
	HAL_GPIO_WritePin(CSN1_GPIO_Port, CSN1_Pin, GPIO_PIN_SET);
	
	//Enable RF Channel
	TxData[0]=(W_REGISTER|RF_CH);
	TxData[1]=0b00000010;
	HAL_GPIO_WritePin(CSN1_GPIO_Port, CSN1_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi2,(uint8_t *)TxData,(uint8_t *)RxData,2,1);
	HAL_GPIO_WritePin(CSN1_GPIO_Port, CSN1_Pin, GPIO_PIN_SET);
	


	//RF Setup Register- Air Data Rate= 250Kbps, RF Output Power= -6dBm
	TxData[0]=(W_REGISTER|RF_SETUP);
	TxData[1]=(0b0<<6)|(0b100100);
	HAL_GPIO_WritePin(CSN1_GPIO_Port, CSN1_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi2,(uint8_t *)TxData,(uint8_t *)RxData,2,1);
	HAL_GPIO_WritePin(CSN1_GPIO_Port, CSN1_Pin, GPIO_PIN_SET);
	
	
	//RX_PW_P0 (Payload Register) --> Configure to 2 bytes
	TxData[0]=(W_REGISTER|RX_PW_P0);
	TxData[1]=(0b0<<6)|(0b000010);
	HAL_GPIO_WritePin(CSN1_GPIO_Port, CSN1_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi2,(uint8_t *)TxData,(uint8_t *)RxData,2,1);
	HAL_GPIO_WritePin(CSN1_GPIO_Port, CSN1_Pin, GPIO_PIN_SET);
		
	//Write Receive address of data pipe 0 as PRATH
	uint8_t RxAddr3[6]={(W_REGISTER|RX_ADDR_P0),(uint8_t )'P',(uint8_t )'R',(uint8_t )'A',(uint8_t )'T',(uint8_t )'H'};
	HAL_GPIO_WritePin(CSN1_GPIO_Port, CSN1_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi2,(uint8_t *)RxAddr3,(uint8_t *)RxDataAddress,6,1);
	HAL_GPIO_WritePin(CSN1_GPIO_Port, CSN1_Pin, GPIO_PIN_SET);
	
  //PRIM RX=1, 1 byte of ACK, Assert RX_DR as active low interrupt for IRQ Pin
	uint8_t TxDataY[2]={(W_REGISTER|CONFIG),0b00111011};
	uint8_t RxDataY[2];
	HAL_GPIO_WritePin(CSN1_GPIO_Port, CSN1_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi2,(uint8_t *)TxDataY,(uint8_t *)RxDataY,2,1);
	HAL_GPIO_WritePin(CSN1_GPIO_Port, CSN1_Pin, GPIO_PIN_SET);
	
	
	
	//Enter Standby-I or Standby-II from power down mode
	uSDelay(1500);
	
	//CE Pin pulse for RX Mode
	HAL_GPIO_WritePin(CE1_GPIO_Port,CE1_Pin,GPIO_PIN_SET);
	uSDelay(130);	
		
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
