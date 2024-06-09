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
#include "adc.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "OpenDrone_Transmitter_HwIntf.h"
#include "Periph.h"
#include "OpenDrone_TxProto.h"
/* USER CODE END Includes */
/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */
/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FREQ_HZ_TO_TIME_US(x)       (1000000.0f/(x))
#define TIME_US_TO_FREQ_HZ(x)       (1000000.0f/(x))

#define IDX_TASK_500_HZ             0
#define IDX_TASK_50_HZ              1
#define IDX_TASK_30_HZ              2
#define IDX_TASK_10_HZ              3
#define IDX_TASK_5_HZ               4
#define NUM_OF_TASK                 5

#define FREQ_500_HZ_TIME_US         FREQ_HZ_TO_TIME_US(500)
#define FREQ_50_HZ_TIME_US          FREQ_HZ_TO_TIME_US(50)
#define FREQ_30_HZ_TIME_US          FREQ_HZ_TO_TIME_US(30)
#define FREQ_10_HZ_TIME_US          FREQ_HZ_TO_TIME_US(10)
#define FREQ_5_HZ_TIME_US           FREQ_HZ_TO_TIME_US(5)
/* USER CODE END PD */
/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */
/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
uint32_t last_time_us[NUM_OF_TASK] = {0};
periph_operator_data_t periph_operator_data = {0};
OpenDrone_TxProto_Msg_t OpenDrone_TxProto_Msg = {0};
OpenDrone_TxProto_Msg_OprCtrl_t OpenDrone_TxProto_Msg_OprCtrl = {0};
OpenDrone_TxProto_Msg_StabilizerCtrl_t OpenDrone_TxProto_Msg_StabilizerCtrl = {0};
/* USER CODE END PV */
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
err_code_t app_init_message(void);
err_code_t app_prepare_message_control(void);
err_code_t app_send_message_control(void);
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
    MX_I2C1_Init();
    MX_SPI1_Init();
    MX_TIM2_Init();
    MX_ADC2_Init();
    /* USER CODE BEGIN 2 */
    app_init_message();
    PeriphSensor_Init();
    PeriphRadio_Init();
    PeriphDisplay_Init();
    /* USER CODE END 2 */
    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        uint32_t current_time = hw_intf_get_time_us();

        /* Task 500 Hz */
        if ((current_time - last_time_us[IDX_TASK_500_HZ]) > FREQ_500_HZ_TIME_US)
        {
        	PeriphRadio_ClearTransmitIrqFlags();

            last_time_us[IDX_TASK_500_HZ] = current_time;
        }

        /* Task 50 Hz */
        if ((current_time - last_time_us[IDX_TASK_50_HZ]) > FREQ_50_HZ_TIME_US)
        {
        	PeriphSensor_GetJoystickScale(&periph_operator_data);

            app_prepare_message_control();
            app_send_message_control();

            last_time_us[IDX_TASK_50_HZ] = current_time;
        }

        /* Task 30 Hz */
        if ((current_time - last_time_us[IDX_TASK_30_HZ]) > FREQ_30_HZ_TIME_US)
        {
            last_time_us[IDX_TASK_30_HZ] = current_time;
        }

        /* Task 5 Hz */
        if ((current_time - last_time_us[IDX_TASK_5_HZ]) > FREQ_5_HZ_TIME_US)
        {
        	PeriphDisplay_ShowStabilizerMessage(OpenDrone_TxProto_Msg_StabilizerCtrl.throttle,
                                                  OpenDrone_TxProto_Msg_StabilizerCtrl.roll,
                                                  OpenDrone_TxProto_Msg_StabilizerCtrl.pitch,
                                                  OpenDrone_TxProto_Msg_StabilizerCtrl.yaw);

            last_time_us[IDX_TASK_5_HZ] = current_time;
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
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
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
    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
        Error_Handler();
    }
}
/* USER CODE BEGIN 4 */
err_code_t app_init_message(void)
{
    OpenDrone_TxProto_Msg.StartInd = 0xAA;
    OpenDrone_TxProto_Msg.PktLen = 8;
    OpenDrone_TxProto_Msg.PktSeq = 0x80;
    OpenDrone_TxProto_Msg.SrcId = 0x40;
    OpenDrone_TxProto_Msg.DesId = 0x41;
    OpenDrone_TxProto_Msg.MsgId = OPENDRONE_TXPROTO_MSG_ID_STABILIZER_CONTROL;
    OpenDrone_TxProto_Msg.Crc = 0x00;
    memset(&OpenDrone_TxProto_Msg.Payload, 0x00, sizeof(OpenDrone_TxProto_Payload_t));

    return ERR_CODE_SUCCESS;
}

err_code_t app_prepare_message_control(void)
{
    OpenDrone_TxProto_Msg_StabilizerCtrl.throttle = periph_operator_data.left_joystick_y;
    OpenDrone_TxProto_Msg_StabilizerCtrl.roll = periph_operator_data.right_joystick_x;
    OpenDrone_TxProto_Msg_StabilizerCtrl.pitch = periph_operator_data.right_joystick_y;
    OpenDrone_TxProto_Msg_StabilizerCtrl.yaw = periph_operator_data.left_joystick_x;

    OpenDrone_TxProto_Msg.Payload = (OpenDrone_TxProto_Payload_t)OpenDrone_TxProto_Msg_StabilizerCtrl;

    return ERR_CODE_SUCCESS;
}

err_code_t app_send_message_control(void)
{
	PeriphRadio_Send((uint8_t *)&OpenDrone_TxProto_Msg);

    return ERR_CODE_SUCCESS;
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
