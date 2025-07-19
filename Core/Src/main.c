#include "main.h"
#include <stdbool.h>

TIM_HandleTypeDef htim2;
ADC_HandleTypeDef hadc1;
I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi1;

#define W25Q_SPI hspi1

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);

// 7 segments
#define DOT 128 //0b10000000

#define DS_Pin GPIO_PIN_0
#define DS_GPIO_Port GPIOB

#define CLK_Pin GPIO_PIN_1
#define CLK_GPIO_Port GPIOB

#define LATCH_Pin GPIO_PIN_10
#define LATCH_GPIO_Port GPIOB

// I2C
#define BMP280_ADDR 0x76 << 1

// SPI
#define W25Q64_CS_GPIO_Port GPIOA
#define W25Q64_CS_Pin GPIO_PIN_4

#define CS_LOW()  HAL_GPIO_WritePin(W25Q64_CS_GPIO_Port, W25Q64_CS_Pin, GPIO_PIN_RESET)
#define CS_HIGH() HAL_GPIO_WritePin(W25Q64_CS_GPIO_Port, W25Q64_CS_Pin, GPIO_PIN_SET)

#define W25Q64_CMD_READ_ID     0x9F
#define W25Q64_CMD_WRITE_EN    0x06
#define W25Q64_CMD_SECTOR_ERASE 0x20
#define W25Q64_CMD_PAGE_PROGRAM 0x02
#define W25Q64_CMD_READ_DATA   0x03
#define W25Q64_CMD_READ_STATUS 0x05

// Button
#define DEBOUNCE_DELAY 10

#define BUTTON_Pin GPIO_PIN_3
#define BUTTON_GPIO_Port GPIOA

// Led
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC

// Misc
#define HIGH GPIO_PIN_SET
#define LOW GPIO_PIN_RESET

typedef struct {
	uint16_t dig_T1;
	int16_t dig_T2;
	int16_t dig_T3;
} bmp280_calib_data;

bmp280_calib_data bmp280_calib;

const uint8_t segment_map[10] = {
	0xC0, // 0
	0xF9, // 1
	0xA4, // 2
	0xB0, // 3
	0x99, // 4
	0x92, // 5
	0x82, // 6
	0xF8, // 7
	0x80, // 8
	0x90  // 9
};

const uint8_t seg[4] = {0xF1, 0xF2, 0xF4, 0xF8};

uint8_t ptr2disp;
uint8_t buff[4] = {0, 0, 0, 0};
uint8_t button_state = 0;
uint8_t last_reading = 0;

uint8_t buffer_tx[256];

uint16_t count = 0;
uint16_t light = 0;

uint32_t last_debounce_time = 0;
uint32_t temp = 0;

bool state = false;
char m = 0;

void multiplex(int32_t number){
	for(uint8_t i = 0; i < 4; i++){
		switch(i){
			case 0:
				ptr2disp = number / 1000;
				break;
			case 1:
				ptr2disp = (number % 1000) / 100;
				break;
			case 2:
				ptr2disp = (number % 100) / 10;
				break;
			case 3:
				ptr2disp = number % 10;
				break;
		}
		buff[i] = (segment_map[ptr2disp]);
	}
}

void serializer(uint8_t data) {
	for (int8_t i = 7; i >= 0; i--) {
		HAL_GPIO_WritePin(CLK_GPIO_Port, CLK_Pin, LOW);
		HAL_GPIO_WritePin(DS_GPIO_Port, DS_Pin, (data & (1 << i)) ? HIGH : LOW );
		HAL_GPIO_WritePin(CLK_GPIO_Port, CLK_Pin, HIGH);
	}
}

void displayWrite(){
	for(int8_t i = 0; i< 4; i++){
		HAL_GPIO_WritePin(LATCH_GPIO_Port, LATCH_Pin, LOW);

		if(i == 1) serializer(buff[i] + DOT);
		else serializer(buff[i]);

		serializer(seg[i]);
		HAL_GPIO_WritePin(LATCH_GPIO_Port, LATCH_Pin, HIGH);
	}
}

uint16_t Read_LDR() {
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	return HAL_ADC_GetValue(&hadc1);
}

void BMP280_Read_Calibration(void) {
	uint8_t calib[6];
	HAL_I2C_Mem_Read(&hi2c1, BMP280_ADDR, 0x88, 1, calib, 6, HAL_MAX_DELAY);

	bmp280_calib.dig_T1 = (uint16_t)(calib[1] << 8 | calib[0]);
	bmp280_calib.dig_T2 = (int16_t)(calib[3] << 8 | calib[2]);
	bmp280_calib.dig_T3 = (int16_t)(calib[5] << 8 | calib[4]);
}

int32_t BMP280_Read_Temp() {
	uint8_t data[3];
	int32_t adc_T;

	HAL_I2C_Mem_Read(&hi2c1, BMP280_ADDR, 0xFA, 1, data, 3, HAL_MAX_DELAY);
	adc_T = (int32_t)((((uint32_t)data[0]) << 12) | (((uint32_t)data[1]) << 4) | ((data[2]) >> 4));

	int32_t var1, var2;
	var1 = ((((adc_T >> 3) - ((int32_t)bmp280_calib.dig_T1 << 1))) * ((int32_t)bmp280_calib.dig_T2)) >> 11;
	var2 = (((((adc_T >> 4) - ((int32_t)bmp280_calib.dig_T1)) * ((adc_T >> 4) - ((int32_t)bmp280_calib.dig_T1))) >> 12) *
			((int32_t)bmp280_calib.dig_T3)) >> 14;

	return ((var1 + var2) * 5 + 128) >> 8;
}

void BMP280_Init() {
	uint8_t config[2];

	config[0] = 0xF4;
	config[1] = 0x27;
	HAL_I2C_Master_Transmit(&hi2c1, BMP280_ADDR, config, 2, HAL_MAX_DELAY);

	config[0] = 0xF5;
	config[1] = 0xA0;
	HAL_I2C_Master_Transmit(&hi2c1, BMP280_ADDR, config, 2, HAL_MAX_DELAY);
}

void checkButton(void) {
    uint8_t reading = HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin);

    if (reading != last_reading) {
        last_debounce_time = HAL_GetTick();
    }

    if ((HAL_GetTick() - last_debounce_time) > DEBOUNCE_DELAY) {
        if (reading != button_state) {
            button_state = reading;

            /*if (button_state == HIGH) {
            	if(state){
            		count = 0;
            		state = false;
            		W25Q64_ReadData(0x000000, buffer_tx, strlen((char*)buffer_tx));
            		buffer_tx[255] = '\0';
            		int32_t temp_lida = -1;
            		int ret = sscanf(buffer_tx, "time: %*d | temp: %d", &temp_lida);
            		if(ret == 1) multiplex(temp_lida);
            		else multiplex(ret);
            	}else{
            		count = 0;
            		state = true;
            		multiplex(BMP280_Read_Temp());
            	}
            }*/
        }
    }

    last_reading = reading;
}

void W25Q_Delay (uint32_t time){
	HAL_Delay (time);
}

void csLOW(void)
{
	HAL_GPIO_WritePin(W25Q64_CS_GPIO_Port, W25Q64_CS_Pin, GPIO_PIN_RESET);
}

void csHIGH(void)
{
	HAL_GPIO_WritePin(W25Q64_CS_GPIO_Port, W25Q64_CS_Pin, GPIO_PIN_SET);
}

void SPI_Write (uint8_t *data, uint8_t len)
{
	HAL_SPI_Transmit(&W25Q_SPI, data, len, 2000);
}

void SPI_Read (uint8_t *data, uint8_t len)
{
	HAL_SPI_Receive(&W25Q_SPI, data, len, 5000);
}

void W25Q_Reset (void)
{
	uint8_t tData[2];

	tData[0] = 0x66;  // enable Reset
	tData[1] = 0x99;  // Reset
        csLOW();
	SPI_Write(tData, 2);
	csHIGH(); // pull the HIGH
        W25Q_Delay (100);
}

uint32_t W25Q_ReadID (void)
{
	uint8_t tData = 0x9f;  // Read ID
	uint8_t rData[3];
	csLOW();  // pull the CS LOW
	SPI_Write(&tData, 1);
	SPI_Read(rData, 3);
	csHIGH();  // pull the HIGH
	return ((rData[0]<<16)|(rData[1]<<8)|rData[2]); // MFN ID : MEM ID : CAPACITY ID
}

void W25Q_WriteEnable (void)
{
	uint8_t tData = 0x06;  // Read ID
	csLOW();  // pull the CS LOW
	SPI_Write(&tData, 1);
	csHIGH();  // pull the HIGH
}

void W25Q_isBusy (void)
{
	uint8_t tData = 0x05;
	uint8_t rData;// Read ID
	csLOW();  // pull the CS LOW
	SPI_Write(&tData, 1);
	do{
		SPI_Read(&rData, 1);
	} while(rData & 0x01);
	csHIGH();  // pull the HIGH
}

void W25Q_EraseSector (uint32_t address)
{
	uint8_t tData[4] = {
		0x20,
	    (address >> 16) & 0xFF,
	    (address >> 8) & 0xFF,
	    address & 0xFF
	};
	csLOW();  // pull the CS LOW
	SPI_Write(tData, 4);
	csHIGH();  // pull the HIGH
	W25Q_Delay (400);
}

void W25Q_WriteData (uint32_t address, uint8_t *data, uint16_t len)
{
	uint8_t tData[4] = {
		0x02,
	    (address >> 16) & 0xFF,
	    (address >> 8) & 0xFF,
	    address & 0xFF
	};
	csLOW();  // pull the CS LOW
	SPI_Write(tData, 4);
	SPI_Write(data, len);
	csHIGH();  // pull the HIGH
}

void W25Q_ReadData (uint32_t address, uint8_t *data, uint16_t len)
{
	uint8_t tData[4] = {
		0x03,
	    (address >> 16) & 0xFF,
	    (address >> 8) & 0xFF,
	    address & 0xFF
	};
	csLOW();  // pull the CS LOW
	SPI_Write(tData, 4);
	SPI_Read(data, len);
	csHIGH();  // pull the HIGH
}

void W25Q_Init(void) {
    uint8_t wren = 0x06;
    uint8_t wrsr = 0x01;
    uint8_t data[2] = { 0x00, 0x00 };  // Zera SR1 e SR2

    csLOW();
    SPI_Write(&wren, 1);
    csHIGH();
    HAL_Delay(1);

    csLOW();
    SPI_Write(&wrsr, 1);
    SPI_Write(data, 2);
    csHIGH();
    HAL_Delay(10);
}

void W25Q64_Test(void) {

	W25Q_Reset();

	W25Q_Init();
	//multiplex(id[0]);

	W25Q_WriteEnable();
	W25Q_EraseSector(0x000000);
	W25Q_isBusy();

	W25Q_WriteEnable();
	uint8_t write = 0xAB;
	W25Q_WriteData(0x000000, &write, 1);
	W25Q_isBusy();

	uint8_t data = 0;
	W25Q_ReadData(0x000000, &data, 1);
	W25Q_isBusy();

	multiplex((uint32_t) data);

}

int main(void)
{


  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();

  HAL_TIM_Base_Start_IT(&htim2);
  BMP280_Init();
  BMP280_Read_Calibration();

  temp = BMP280_Read_Temp();
  multiplex(temp);

  //W25Q64_Test();

  while (1)
  {
	  displayWrite();
	  checkButton();
  }

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM2){
		count++;
		if(count >= 5000) {
			count = 0;
			temp = BMP280_Read_Temp();
			multiplex(temp);

			/*if(m >= 15){
				m = 0;
				// gravar na mémoria e fazer uma média
			}
			else{
				m++;
			}*/
		}

		if(Read_LDR() > 3000) HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, HIGH);
		else HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, LOW);
	}
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
	Error_Handler();
  }

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

void MX_ADC1_Init(void) {
	ADC_ChannelConfTypeDef sConfig = {0};

	__HAL_RCC_ADC1_CLK_ENABLE();

	hadc1.Instance = ADC1;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;

	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
	HAL_ADC_ConfigChannel(&hadc1, &sConfig);
}

void MX_I2C1_Init(void) {
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 100000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	{
		Error_Handler();
	}
}

static void MX_SPI1_Init(void)
{

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

}

static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 35999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV2;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
	Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
	Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
	Error_Handler();

  }

}

static void MX_GPIO_Init(void)
{
	  GPIO_InitTypeDef GPIO_InitStruct = {0};

	  __HAL_RCC_GPIOA_CLK_ENABLE();
	  __HAL_RCC_GPIOB_CLK_ENABLE();
	  __HAL_RCC_GPIOC_CLK_ENABLE();
	  __HAL_RCC_GPIOD_CLK_ENABLE();

	  GPIO_InitStruct.Pin = LED_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);
	  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, HIGH);

	  GPIO_InitStruct.Pin = DS_Pin;
	  HAL_GPIO_Init(DS_GPIO_Port, &GPIO_InitStruct);

	  GPIO_InitStruct.Pin = CLK_Pin;
	  HAL_GPIO_Init(CLK_GPIO_Port, &GPIO_InitStruct);

	  GPIO_InitStruct.Pin = LATCH_Pin;
	  HAL_GPIO_Init(LATCH_GPIO_Port, &GPIO_InitStruct);

	  GPIO_InitStruct.Pin = BUTTON_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);

	  /*Configure GPIO pin : PA4 */
	  GPIO_InitStruct.Pin = GPIO_PIN_4;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

void Error_Handler(void)
{

  __disable_irq();
  while (1)
  {
  }
}

#ifdef  USE_FULL_ASSERT

void assert_failed(uint8_t *file, uint32_t line)
{

}
#endif
