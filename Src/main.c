/* USER CODE BEGIN Header */
/**
    ******************************************************************************
    * @file           : main.c
    * @brief          : Main program body
    ******************************************************************************
    * @attention
    *
    * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
    * All rights reserved.</center></h2>
    *
    * This software component is licensed by ST under BSD 3-Clause license,
    * the "License"; You may not use this file except in compliance with the
    * License. You may obtain a copy of the License at:
    *                        opensource.org/licenses/BSD-3-Clause
    *
    ******************************************************************************
    */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "eth.h"
#include "spi.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "NRF24L01.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint8_t tx_buf[25];
uint8_t rx_buf[25];
uint8_t TX_frequency = 0x5a;    //24L01频率初始化为18
uint8_t RX_frequency = 0x5a;  //24L01频率初始化为18
unsigned int time_count = 0;
uint8_t Check = 0x00;
// uint8_t CheckBuf[1] = {0xff};
uint8_t test_flag = 0;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
    MX_ETH_Init();
    MX_SPI4_Init();
    MX_SPI5_Init();
    MX_USART3_UART_Init();
    MX_USB_OTG_FS_PCD_Init();
    /* USER CODE BEGIN 2 */
        
    while (NRF24L01_TX_Check()){
        HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
        HAL_Delay(2000);
    }
        
    TX_Mode();
        
    HAL_UART_Receive_IT(&huart3, tx_buf, 25);
    
    // while (NRF24L01_RX_Check())
    // {
    //     HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    //     HAL_Delay(2000);
    // }
    
    // RX_Mode();
    
//    HAL_UART_Receive_IT(&huart3, rx_buf, 25);   //触发中断
    // while (1)
    // {
    //     if (HAL_UART_Receive(&huart3, tx_buf, 25, 0xff) == HAL_OK)
    //     {
    //         if (NRF24L01_TxPacket(tx_buf) == TX_OK)
    //         {
    //             time_count ++;
    //             if (time_count > 32)
    //             {
    //                 time_count = 0;
    //                 HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
    //                 HAL_GPIO_TogglePin(LD1_GPIO_Port, LD2_Pin);
    //                 HAL_GPIO_TogglePin(LD1_GPIO_Port, LD3_Pin);
    //             }
    //         }
    //     }
    // }

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
      /* USER CODE END WHILE */

      /* USER CODE BEGIN 3 */    
            
        // test_flag = NRF24L01_RxPacket(rx_buf);
        // if ( !test_flag )
        // {
        //     time_count ++;
        //     if (time_count > 32){
        //     time_count = 0;
        //     HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
        //     HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        //     HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
        //     }
        
        //     // 判断配置包
        //     Check = (rx_buf[0] + rx_buf[1] + rx_buf[2] + rx_buf[3]) & 0xff;
        //     if( rx_buf[0] == 0xf0 && rx_buf[4] == Check )
        //     {
        //         switch ( rx_buf[3] )
        //         {
        //             case 0x01:
        //                 TX_frequency = rx_buf[1];
        //                 break;
        //             case 0x02:
        //                 RX_frequency = rx_buf[2];
        //                 break;
        //             case 0x03:
        //                 TX_frequency = rx_buf[1];
        //                 RX_frequency = rx_buf[2];
        //                 break;
        //             default:
        //                 break;
        //         }
        //     }
        //     else
        //         HAL_UART_Transmit(&huart3, rx_buf, 25, 0xff);
        // }
            // if (HAL_UART_Receive(&huart3, tx_buf, 25, 0xff) == HAL_OK){
            //     if (NRF24L01_TxPacket(tx_buf) == TX_OK){
            //         time_count ++;
            //         if (time_count > 32){
            //             time_count = 0;
            //             HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
            //             HAL_GPIO_TogglePin(LD1_GPIO_Port, LD2_Pin);
            //             HAL_GPIO_TogglePin(LD1_GPIO_Port, LD3_Pin);
            //         }
            //     }, 
            // }
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
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

    /** Supply configuration update enable
    */
    HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
    /** Configure the main internal regulator output voltage
    */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

    while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
    /** Initializes the CPU, AHB and APB busses clocks
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 1;
    RCC_OscInitStruct.PLL.PLLN = 120;
    RCC_OscInitStruct.PLL.PLLP = 2;
    RCC_OscInitStruct.PLL.PLLQ = 20;
    RCC_OscInitStruct.PLL.PLLR = 2;
    RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
    RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
    RCC_OscInitStruct.PLL.PLLFRACN = 0;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
      Error_Handler();
    }
    /** Initializes the CPU, AHB and APB busses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
    RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
    {
      Error_Handler();
    }
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_SPI5
                              |RCC_PERIPHCLK_SPI4|RCC_PERIPHCLK_USB;
    PeriphClkInitStruct.Spi45ClockSelection = RCC_SPI45CLKSOURCE_D2PCLK1;
    PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
    PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }
    /** Enable USB Voltage detector
    */
    HAL_PWREx_EnableUSBVoltageDetector();
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

    if(huart->Instance == USART3)
    {
        // HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
        if (NRF24L01_TxPacket(tx_buf) == TX_OK){
            time_count ++;
            if (time_count > 32){
                time_count = 0;
                HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
                HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
                HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
            }

            // 判断配置包
            Check = (tx_buf[0] + tx_buf[1] + tx_buf[2] + tx_buf[3] + tx_buf[4]) & 0xff;
            // test
            // CheckBuf[0] = Check;
            // HAL_UART_Transmit(&huart3, CheckBuf, 1, 0xff);
            // CheckBuf[0] = 0xff;
            if( tx_buf[0] == 0xf0 && tx_buf[5] == Check )
            {
                switch ( tx_buf[3] )
                {
                    case 0x01:
                        TX_frequency = tx_buf[1];
                        TX_Mode();  // Updata frequency
                        break;
                    case 0x02:
                        RX_frequency = tx_buf[2];
                        RX_Mode();
                        break;
                    case 0x03:
                        TX_frequency = tx_buf[1];
                        RX_frequency = tx_buf[2];
                        TX_Mode();
                        RX_Mode();
                        break;
                    default:
                        break;
                }
                Check = 0x00;   // Initial value
                // test
                // // CheckBuf[0] = 0x88;  // Check Success
                // CheckBuf[0] = TX_frequency;  // Check Freq
                // HAL_UART_Transmit(&huart3, CheckBuf, 1, 0xff);
            }
            else
            {
                
            }
            
        }
        HAL_UART_Receive_IT(&huart3, tx_buf, 25);
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
    tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
