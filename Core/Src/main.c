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
#include "ring_buf.h"
#include "utils.h"

#include <stdbool.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct{
	struct{
		uint32_t ide;
		uint32_t val;
		uint32_t rtr;
	}id;
	uint8_t data[8];
	uint8_t dlc;
}can_packet_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
enum{
	CAN_BITRATE_10K,
	CAN_BITRATE_20K,
	CAN_BITRATE_50K,
	CAN_BITRATE_100K,
	CAN_BITRATE_125K,
	CAN_BITRATE_250K,
	CAN_BITRATE_500K,
	CAN_BITRATE_800K,
	CAN_BITRATE_1M
};
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static struct ring input_ring;
static struct ring output_ring;

static uint8_t input_buf[RING_BUFFER_SIZE];
static uint8_t rx_ch[256];

static uint16_t last_dma_pos;
static uint16_t current_dma_pos;

volatile uint8_t commands_pending;

static uint8_t channel_open = 0;
static CAN_TxHeaderTypeDef tx_header;
static uint8_t uart_tx_busy;
uint32_t rx_active = 0;
uint32_t tx_active = 0;

#define CAN_BUF_SIZE		256

can_packet_t can_buf[CAN_BUF_SIZE];


static uint16_t can_buf_head = 0;
static uint16_t can_buf_tail = 0;

static int can_buf_get(can_packet_t *packet)
{
	uint16_t next;
	next = can_buf_tail + 1;
	if(next >= CAN_BUF_SIZE){
		next = 0;
	}

	if(can_buf_tail == can_buf_head){
		return 0;
	}

	if(packet){
		memcpy(packet, &can_buf[can_buf_tail], sizeof(can_packet_t));
	}
	can_buf_tail = next;

	return 1;
}

static void can_buf_add(can_packet_t *packet)
{
	uint16_t next;
	next = can_buf_head + 1;
	if(next >= CAN_BUF_SIZE){
		next = 0;
	}

	if(next == can_buf_tail){
		(void)can_buf_get(NULL);
	}

	memcpy(&can_buf[can_buf_head], packet, sizeof(can_packet_t));
	can_buf_head = next;
}


static void put_hex(uint8_t c)
{
	uint8_t s[2];
	bin2hex(s, c);
//	(void)ring_write(&output_ring, s, 2);
}

//void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
//{
//	(void)huart;
////	uart_tx_busy = 0;
//}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef rx_header;
	uint8_t can_rx[8];
	can_packet_t tmp;
	char c;

	if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, can_rx) == HAL_OK){
		tmp.id.rtr = rx_header.RTR;
		tmp.id.ide = rx_header.IDE;
		tmp.dlc = rx_header.DLC;

		if(tmp.id.ide == CAN_ID_STD){
			tmp.id.val = rx_header.StdId;
		}
		else{
			tmp.id.val = rx_header.ExtId;
		}

		memcpy(tmp.data, can_rx, tmp.dlc);
		can_buf_add(&tmp);
	}
}

static int uart_read_blocking(uint8_t *c)
{
	uint8_t rx_complete;
	uint32_t start_ms;

	start_ms = HAL_GetTick();
	rx_complete = 0;

	// Wait for DMA data
	do{
		uint32_t dma_counter = __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);
		current_dma_pos = (uint16_t)(sizeof(rx_ch)/sizeof(rx_ch[0])) - dma_counter;
		// Already data in buffer
		if(ring_read_ch(&input_ring, c)){
			rx_complete = 1;
			break;
		}

		if(HAL_GetTick() - start_ms > 10){
			return -10;
		}
	}while(current_dma_pos == last_dma_pos);

	if(!rx_complete){
		if(current_dma_pos > last_dma_pos){
			(void)ring_write(&input_ring, &rx_ch[last_dma_pos], current_dma_pos - last_dma_pos);
		}
		else if(current_dma_pos < last_dma_pos){
			(void)ring_write(&input_ring, &rx_ch[last_dma_pos], sizeof(rx_ch) - last_dma_pos);
			if(current_dma_pos > 0){
				(void)ring_write(&input_ring, &rx_ch[0], current_dma_pos);
			}
		}

		last_dma_pos = current_dma_pos;
		(void)ring_read_ch(&input_ring, c);
	}

	return 0;
}

static HAL_StatusTypeDef can_filter_setup()
{
	static CAN_FilterTypeDef can_filter;

	can_filter.FilterIdHigh = 0;
	can_filter.FilterIdLow = 0;
	can_filter.FilterMaskIdHigh = 0;
	can_filter.FilterMaskIdLow = 0;
	can_filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	can_filter.FilterBank = 0;
	can_filter.FilterMode = CAN_FILTERMODE_IDMASK;
	can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
	can_filter.FilterActivation = CAN_FILTER_ENABLE;
	can_filter.SlaveStartFilterBank = 6;

	return HAL_CAN_ConfigFilter(&hcan, &can_filter);
}

static HAL_StatusTypeDef can_reinit(uint32_t prescaler, uint32_t seg1, uint32_t seg2)
{
	HAL_CAN_Stop(&hcan);
	HAL_Delay(1);
	HAL_CAN_DeInit(&hcan);
	HAL_Delay(1);

	hcan.Instance = CAN1;
	hcan.Init.Prescaler = prescaler;
	hcan.Init.Mode = CAN_MODE_NORMAL;
	hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan.Init.TimeSeg1 = seg1;
	hcan.Init.TimeSeg2 = seg2;
	hcan.Init.TimeTriggeredMode = DISABLE;
	hcan.Init.AutoBusOff = ENABLE;
	hcan.Init.AutoWakeUp = DISABLE;
	hcan.Init.AutoRetransmission = ENABLE;
	hcan.Init.ReceiveFifoLocked = DISABLE;
	hcan.Init.TransmitFifoPriority = ENABLE;

	if(HAL_CAN_Init(&hcan) != HAL_OK){
		return HAL_ERROR;
	}

	if(can_filter_setup() != HAL_OK){
		return HAL_ERROR;
	}

	if(HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING)){
		return HAL_ERROR;
	}

	if(HAL_CAN_Start(&hcan) != HAL_OK){
		return HAL_ERROR;
	}

	return HAL_OK;
}

static int can_init(int index)
{
	int ret;
	switch(index){
	case CAN_BITRATE_10K:{
		if(can_reinit(200, CAN_BS1_13TQ, CAN_BS2_2TQ) == HAL_OK){
			ret = 0;
		}
		else{
			ret = -1;
		}
		break;
	}
	case CAN_BITRATE_20K:{
		if(can_reinit(100, CAN_BS1_13TQ, CAN_BS2_2TQ) == HAL_OK){
			ret = 0;
		}
		else{
			ret = -1;
		}
		break;
	}
	case CAN_BITRATE_50K:{
		if(can_reinit(40, CAN_BS1_13TQ, CAN_BS2_2TQ) == HAL_OK){
			ret = 0;
		}
		else{
			ret = -1;
		}
		break;
	}
	case CAN_BITRATE_100K:{
		if(can_reinit(20, CAN_BS1_13TQ, CAN_BS2_2TQ) == HAL_OK){
			ret = 0;
		}
		else{
			ret = -1;
		}
		break;
	}
	case CAN_BITRATE_125K:{
		if(can_reinit(16, CAN_BS1_13TQ, CAN_BS2_2TQ) == HAL_OK){
			ret = 0;
		}
		else{
			ret = -1;
		}
		break;
	}
	case CAN_BITRATE_250K:{
		if(can_reinit(8, CAN_BS1_13TQ, CAN_BS2_2TQ) == HAL_OK){
			ret = 0;
		}
		else{
			ret = -1;
		}
		break;
	}
	case CAN_BITRATE_500K:{
		if(can_reinit(4, CAN_BS1_13TQ, CAN_BS2_2TQ) == HAL_OK){
			ret = 0;
		}
		else{
			ret = -1;
		}
		break;
	}
	case CAN_BITRATE_800K:{
		if(can_reinit(4, CAN_BS1_8TQ, CAN_BS2_1TQ) == HAL_OK){
			ret = 0;
		}
		else{
			ret = -1;
		}
		break;
	}
	case CAN_BITRATE_1M:{
		if(can_reinit(2, CAN_BS1_13TQ, CAN_BS2_2TQ) == HAL_OK){
			ret = 0;
		}
		else{
			ret = -1;
		}
		break;
	}
	default:{
		ret = -1;
		break;
	}
	}

	return ret;
}

static int can_speed(int index)
{
	HAL_GPIO_WritePin(CAN_STDBY_GPIO_Port, CAN_STDBY_Pin, 0);
	DEBUG_PRINT("Speed %d", index);
	return can_init(index);
}

static uint32_t get_nibbles(int nibbles)
{
	int i;
	uint32_t id;
	uint8_t c;

	id = 0;
	for (i = 0; i < nibbles; i++) {
		if(uart_read_blocking(&c) == 0){
			id <<= 4;
			id |= nibble2bin(c);
		}
	}
	return id;
}

static void dump_can_messages()
{
	can_packet_t tmp;
	static uint8_t tx_buf[128];
	int tx_index = 0;
	char c;

	if(can_buf_get(&tmp) > 0){
		if(tmp.id.rtr == CAN_RTR_REMOTE){
			if(tmp.id.ide == CAN_ID_EXT){
				tx_buf[tx_index++] = 'R';
			}
			else{
				tx_buf[tx_index++] = 'r';
			}
		}
		else{
			if(tmp.id.ide == CAN_ID_EXT){
				tx_buf[tx_index++] = 'T';
			}
			else{
				tx_buf[tx_index++] = 't';
			}
		}

//		(void)ring_write_ch(&output_ring, c);

		if(tmp.id.ide == CAN_ID_EXT){
			c = (uint8_t)((tmp.id.val >> 24) & 0xff);
//			put_hex(c);
			bin2hex(&tx_buf[tx_index], c);
			tx_index += 2;
			c = (uint8_t)((tmp.id.val >> 16) & 0xff);
//			put_hex(c);
			bin2hex(&tx_buf[tx_index], c);
			tx_index += 2;
			c = (uint8_t)((tmp.id.val >> 8) & 0xff);
//			put_hex(c);
			bin2hex(&tx_buf[tx_index], c);
			tx_index += 2;
			c = (uint8_t)(tmp.id.val & 0xff);
//			put_hex(c);
			bin2hex(&tx_buf[tx_index], c);
			tx_index += 2;
		}
		else{
			/* bits 11-9 */
			c = (tmp.id.val >> 8) & 0x07;
			c += 0x30;

			tx_buf[tx_index++] = c;
			//	        put_hex(c);
//			(void)ring_write_ch(&output_ring, c);
			/* bits 8-1 */
			c = tmp.id.val & 0xff;
//			put_hex(c);
			bin2hex(&tx_buf[tx_index], c);
			tx_index += 2;
		}

		c = (uint8_t)(tmp.dlc);
		c += 0x30;
		tx_buf[tx_index++] = c;
//		(void)ring_write_ch(&output_ring, c);
		//		put_hex(c);

		for (uint32_t i = 0; i < tmp.dlc; i++){
//			put_hex(tmp.data[i]);
			bin2hex(&tx_buf[tx_index], tmp.data[i]);
			tx_index += 2;
		}

//		(void)ring_write_ch(&output_ring, '\r');
		tx_buf[tx_index++] = '\r';

//		uint8_t output_c;
//		while(ring_read_ch(&output_ring, &output_c)){
//			while(HAL_DMA_GetState(&hdma_usart2_tx) == HAL_DMA_STATE_BUSY){;}
//			HAL_UART_Transmit_DMA(&huart2, tx_buf, tx_index);
			HAL_UART_Transmit(&huart2, tx_buf, tx_index, HAL_MAX_DELAY);
//		}

		rx_active++;
	}
}

static int slcan_command(void)
{
	//    static bool sw_flow = true;
	bool ext, rtr;
	uint8_t i, dlc, data[8] = {0};
	uint32_t id;
	int32_t ret;
	uint8_t c;
	bool send;

	id = 0;
	dlc = 0;
	ext = true;
	send = true;
	rtr = false;
	ret = 0;
	c = 0;

//	if(HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0){
//		__asm__("nop");
//		return -1;
//	}

	if(uart_read_blocking(&c) != 0){
		return -1;
	}
//	while(uart_read_blocking(&c) < 0);

	DEBUG_PRINT("Uart RX");

	switch (c) {
	case 'T':
		id = get_nibbles(8);
		dlc = (uint8_t)get_nibbles(1);
		break;
	case 't':
		ext = false;
		id = get_nibbles(3);
		dlc = (uint8_t)get_nibbles(1);
		break;
	case 'R':
		rtr = true;
		ext = true;
		id = get_nibbles(8);
		dlc = (uint8_t)get_nibbles(1);
		break;
	case 'r':
		rtr = true;
		ext = false;
		id = get_nibbles(3);
		dlc = (uint8_t)get_nibbles(1);
		break;
	case 'S':
		c = (uint8_t)get_nibbles(1);
		ret = can_speed(c);
		send = false;
		break;
	case 'v':
		send = false;
		break;
	case 'V':
		send = false;
		break;
	case 'C':
		// Close channel
		//    	if(!channel_open){
		//    		ret = -1;
		//    	}

		channel_open = 0;
		send = false;
		break;
	case 'O':{
		//    	if(channel_open){
		//    		ret = -1;
		//    	}

		channel_open = 1;
		send = false;
		break;
	}
	default:
		send = false;
		break;
	}
	if (dlc > 8) {
		/* consume chars until eol reached */
		do {
			ret = uart_read_blocking(&c);
			if(ret < 0){
				break;
			}
		} while (c != '\r');
		return -1;
	}

	for (i = 0; i < dlc; i++) {
		data[i] = (uint8_t)get_nibbles(2);
	}

	/* consume chars until eol reached */
	do {
		ret = uart_read_blocking(&c);
		if(ret < 0){
			break;
		}
	} while (c != '\r');

#if 1
	if (send) {

		uint32_t mailbox;

		tx_header.DLC = dlc;
		tx_header.IDE = ext ? CAN_ID_EXT : CAN_ID_STD;
		tx_header.RTR = rtr ? CAN_RTR_REMOTE : CAN_RTR_DATA;
		if(tx_header.IDE == CAN_ID_STD){
			tx_header.StdId = id;
		}
		else{
			tx_header.ExtId = id;
		}

		if(HAL_CAN_AddTxMessage(&hcan, &tx_header, data, &mailbox) != HAL_OK){
			ret = -1;
		}
		else{
			tx_active++;
		}
	}
#else
	if (send) {
		int loop = CAN_MAX_RETRY;
		/* try to send data - omit if not possible */
		while (loop-- > 0) {
			if (can_available_mailbox(CAN1))
				break;
			/* TODO: LED overflow */
		}
		ret = can_transmit(CAN1, id, ext, rtr, dlc, data);
		gpio_debug(ret);
	}
#endif

	if (commands_pending)
		commands_pending--;

	return ret;
}


#ifdef DEBUG
int _write(int file, char *ptr, int len)
{
	(void)file;
	int i = 0;
	for(i=0; i<len; ++i){
		ITM_SendChar(ptr[i]);
	}

	return i;
}
#endif

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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */
	DEBUG_PRINT("Init..");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	ring_setup(&input_ring, input_buf, RING_BUFFER_SIZE);
//	ring_setup(&output_ring, output_buf, RING_BUFFER_SIZE);

	last_dma_pos = 0;
	commands_pending = 0;

	uart_tx_busy = 0;

	DEBUG_PRINT("Start..");
	can_speed(6);
	while(HAL_UART_Receive_DMA(&huart2, rx_ch, sizeof(rx_ch)/sizeof(rx_ch[0])) != HAL_OK);

	static uint8_t rsp;
	while (1)
	{
		int slcan_ret;
		slcan_ret = slcan_command();
		if(slcan_ret == 0){
			rsp = '\r';
//			while(HAL_DMA_GetState(&hdma_usart2_tx) == HAL_DMA_STATE_BUSY){;}
//			HAL_UART_Transmit_DMA(&huart2, &rsp, 1);
			HAL_UART_Transmit(&huart2, &rsp, 1, HAL_MAX_DELAY);
		}
		else if(slcan_ret == -1){
			rsp = 0x07;
//			while(HAL_DMA_GetState(&hdma_usart2_tx) == HAL_DMA_STATE_BUSY){;}
//			HAL_UART_Transmit_DMA(&huart2, &rsp, 1);
			HAL_UART_Transmit(&huart2, &rsp, 1, HAL_MAX_DELAY);
			HAL_CAN_AbortTxRequest(&hcan, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
		}

		dump_can_messages();
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 4;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_2TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = ENABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = ENABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = ENABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

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
  huart2.Init.BaudRate = 2000000;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CAN_STDBY_GPIO_Port, CAN_STDBY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RX_LED_Pin|TX_LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : CAN_STDBY_Pin */
  GPIO_InitStruct.Pin = CAN_STDBY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(CAN_STDBY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RX_LED_Pin TX_LED_Pin */
  GPIO_InitStruct.Pin = RX_LED_Pin|TX_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
