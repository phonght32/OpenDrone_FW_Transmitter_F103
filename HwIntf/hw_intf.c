#include "i2c.h"
#include "adc.h"
#include "spi.h"
#include "tim.h"

#include "hd44780.h"
#include "joystick.h"
#include "nrf24l01.h"

#include "OpenDrone_Transmitter_Config.h"

#define APP_TIM 						htim2

#ifdef USE_HD44780_2004
#define HD44780_I2C_HANDLE				hi2c1
#define I2C_ADDR_HD44780  				(HD77480_I2C_ADDR_PCF8574<<1)
#endif

#ifdef USE_JOYSTICK_MODULE
#define LEFT_JOYSTICK_ADC               hadc2
#define LEFT_JOYSTICK_POSX_ADC_CHNL     ADC_CHANNEL_0
#define LEFT_JOYSTICK_POSY_ADC_CHNL     ADC_CHANNEL_1
#define LEFT_JOYSTICK_BUTTON_PORT       GPIOB
#define LEFT_JOYSTICK_BUTTON_PIN        GPIO_PIN_1

#define RIGHT_JOYSTICK_ADC              hadc2
#define RIGHT_JOYSTICK_POSX_ADC_CHNL    ADC_CHANNEL_2
#define RIGHT_JOYSTICK_POSY_ADC_CHNL    ADC_CHANNEL_3
#define RIGHT_JOYSTICK_BUTTON_PORT      GPIOB
#define RIGHT_JOYSTICK_BUTTON_PIN       GPIO_PIN_2

#define ADC_SAMPLE_CYCLES               ADC_SAMPLETIME_13CYCLES_5
#define ADC_CONV_TIMEOUT_MS             10
#endif

#ifdef USE_NRF24L01
#define NRF24L01_SPI                 	hspi2
#define NRF24L01_GPIO_PORT_CS         	GPIOB
#define NRF24L01_GPIO_PIN_CS      		GPIO_PIN_11
#define NRF24L01_GPIO_PORT_CE         	GPIOA
#define NRF24L01_GPIO_PIN_CE      		GPIO_PIN_8
#define NRF24L01_GPIO_PORT_IRQ        	GPIOA
#define NRF24L01_GPIO_PIN_IRQ         	GPIO_PIN_15
#endif

#ifdef USE_SX1278
#define SX1278_SPI                 		hspi2
#define SX1278_GPIO_PORT_CS         	GPIOA
#define SX1278_GPIO_PIN_CS      		GPIO_PIN_11
#define SX1278_GPIO_PORT_RST         	GPIOA
#define SX1278_GPIO_PIN_RST     		GPIO_PIN_12
#define SX1278_GPIO_PORT_IRQ            GPIOA
#define SX1278_GPIO_PIN_IRQ         	GPIO_PIN_10
#endif

uint32_t hwif_get_time_us(void)
{
	return HAL_GetTick() * 1000;
}

void hwif_delay_ms(uint32_t time_ms)
{
	HAL_Delay(time_ms);
}

#ifdef USE_HD44780_2004
hd44780_status_t hwif_hd44780_i2c_send(uint8_t *buf_send, uint16_t len)
{
	HAL_I2C_Master_Transmit(&HD44780_I2C_HANDLE, I2C_ADDR_HD44780, buf_send, len, 100);

	return HD44780_STATUS_SUCCESS;
}
#endif

#ifdef USE_JOYSTICK_MODULE
joystick_status_t hwif_left_joystick_get_pos_x(uint16_t *pos_x)
{
	ADC_ChannelConfTypeDef sConfig = {0};

	sConfig.Channel = LEFT_JOYSTICK_POSX_ADC_CHNL;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLE_CYCLES;
	if (HAL_ADC_ConfigChannel(&LEFT_JOYSTICK_ADC, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

	HAL_ADC_Start(&LEFT_JOYSTICK_ADC);
	HAL_ADC_PollForConversion(&LEFT_JOYSTICK_ADC, ADC_CONV_TIMEOUT_MS);
	*pos_x = HAL_ADC_GetValue(&LEFT_JOYSTICK_ADC);
	HAL_ADC_Stop(&LEFT_JOYSTICK_ADC);

	return JOYSTICK_STATUS_SUCCESS;
}

joystick_status_t hwif_left_joystick_get_pos_y(uint16_t *pos_y)
{
	ADC_ChannelConfTypeDef sConfig = {0};

	sConfig.Channel = LEFT_JOYSTICK_POSY_ADC_CHNL;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLE_CYCLES;
	if (HAL_ADC_ConfigChannel(&LEFT_JOYSTICK_ADC, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

	HAL_ADC_Start(&LEFT_JOYSTICK_ADC);
	HAL_ADC_PollForConversion(&LEFT_JOYSTICK_ADC, ADC_CONV_TIMEOUT_MS);
	*pos_y = HAL_ADC_GetValue(&LEFT_JOYSTICK_ADC);
	HAL_ADC_Stop(&LEFT_JOYSTICK_ADC);

	return JOYSTICK_STATUS_SUCCESS;
}

joystick_status_t hwif_left_joystick_get_bt_status(uint8_t *bt_status)
{
	*bt_status = HAL_GPIO_ReadPin(LEFT_JOYSTICK_BUTTON_PORT, LEFT_JOYSTICK_BUTTON_PIN);

	return JOYSTICK_STATUS_SUCCESS;
}

joystick_status_t hwif_right_joystick_get_pos_x(uint16_t *pos_x)
{
	ADC_ChannelConfTypeDef sConfig = {0};

	sConfig.Channel = RIGHT_JOYSTICK_POSX_ADC_CHNL;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLE_CYCLES;
	if (HAL_ADC_ConfigChannel(&RIGHT_JOYSTICK_ADC, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

	HAL_ADC_Start(&RIGHT_JOYSTICK_ADC);
	HAL_ADC_PollForConversion(&RIGHT_JOYSTICK_ADC, ADC_CONV_TIMEOUT_MS);
	*pos_x = HAL_ADC_GetValue(&RIGHT_JOYSTICK_ADC);
	HAL_ADC_Stop(&RIGHT_JOYSTICK_ADC);

	return JOYSTICK_STATUS_SUCCESS;
}

joystick_status_t hwif_right_joystick_get_pos_y(uint16_t *pos_y)
{
	ADC_ChannelConfTypeDef sConfig = {0};

	sConfig.Channel = RIGHT_JOYSTICK_POSY_ADC_CHNL;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLE_CYCLES;
	if (HAL_ADC_ConfigChannel(&RIGHT_JOYSTICK_ADC, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

	HAL_ADC_Start(&RIGHT_JOYSTICK_ADC);
	HAL_ADC_PollForConversion(&RIGHT_JOYSTICK_ADC, ADC_CONV_TIMEOUT_MS);
	*pos_y = HAL_ADC_GetValue(&RIGHT_JOYSTICK_ADC);
	HAL_ADC_Stop(&RIGHT_JOYSTICK_ADC);

	return JOYSTICK_STATUS_SUCCESS;
}

joystick_status_t hwif_right_joystick_get_bt_status(uint8_t *bt_status)
{
	*bt_status = HAL_GPIO_ReadPin(RIGHT_JOYSTICK_BUTTON_PORT, RIGHT_JOYSTICK_BUTTON_PIN);

	return JOYSTICK_STATUS_SUCCESS;
}
#endif

#ifdef USE_NRF24L01
nrf24l01_status_t hwif_nrf24l01_spi_send(uint8_t *buf_send, uint16_t len)
{
	HAL_SPI_Transmit(&NRF24L01_SPI, buf_send, len, 100);

	return NRF24L01_STATUS_SUCCESS;
}

nrf24l01_status_t hwif_nrf24l01_spi_recv(uint8_t *buf_recv, uint16_t len)
{
	HAL_SPI_Receive(&NRF24L01_SPI, buf_recv, len, 100);

	return NRF24L01_STATUS_SUCCESS;
}

nrf24l01_status_t hwif_nrf24l01_set_cs(uint8_t level)
{
	HAL_GPIO_WritePin(NRF24L01_GPIO_PORT_CS, NRF24L01_GPIO_PIN_CS, level);

	return NRF24L01_STATUS_SUCCESS;
}

nrf24l01_status_t hwif_nrf24l01_set_ce(uint8_t level)
{
	HAL_GPIO_WritePin(NRF24L01_GPIO_PORT_CE, NRF24L01_GPIO_PIN_CE, level);

	return NRF24L01_STATUS_SUCCESS;
}

nrf24l01_status_t hwif_nrf24l01_get_irq(uint8_t *level)
{
	*level = HAL_GPIO_ReadPin(NRF24L01_GPIO_PORT_IRQ, NRF24L01_GPIO_PIN_IRQ);

	return NRF24L01_STATUS_SUCCESS;
}
#endif

#ifdef USE_SX1278
err_code_t hwif_sx1278_spi_send(uint8_t *buf_send, uint16_t len)
{
	HAL_SPI_Transmit(&SX1278_SPI, buf_send, len, 100);

	return ERR_CODE_SUCCESS;
}

err_code_t hwif_sx1278_spi_recv(uint8_t *buf_recv, uint16_t len)
{
	HAL_SPI_Receive(&SX1278_SPI, buf_recv, len, 100);

	return ERR_CODE_SUCCESS;
}

err_code_t hwif_sx1278_set_cs(uint8_t level)
{
	HAL_GPIO_WritePin(SX1278_GPIO_PORT_CS, SX1278_GPIO_PIN_CS, level);

	return ERR_CODE_SUCCESS;
}

err_code_t hwif_sx1278_set_rst(uint8_t level)
{
	HAL_GPIO_WritePin(SX1278_GPIO_PORT_RST, SX1278_GPIO_PIN_RST, level);

	return ERR_CODE_SUCCESS;
}

err_code_t hwif_sx1278_get_irq(uint8_t *level)
{
	*level = HAL_GPIO_ReadPin(SX1278_GPIO_PORT_IRQ, SX1278_GPIO_PIN_IRQ);

	return ERR_CODE_SUCCESS;
}
#endif
