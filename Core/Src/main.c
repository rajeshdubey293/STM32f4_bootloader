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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint8_t supported_commands[] = {
		BL_GET_VER ,
		BL_GET_HELP,
		BL_GET_CID,
		BL_GET_RDP_STATUS,
		BL_GO_TO_ADDR,
		BL_FLASH_ERASE,
		BL_MEM_WRITE,
		BL_READ_SECTOR_P_STATUS,
		BL_BOOT_APPLICATION} ;

#define BL_DEBUG_MSG_EN
//#define INTERRUPT_ON

#define D_UART   &huart3
#define C_UART   &huart1
#define BL_RX_LEN  200
uint8_t bl_rx_buffer[BL_RX_LEN];

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_CRC_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//char pData[] = "Hey Buddy\r\n";
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
	MX_USART3_UART_Init();
	MX_CRC_Init();
	MX_USART1_UART_Init();
	/* USER CODE BEGIN 2 */

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	//{
	/* USER CODE END WHILE */

	/* USER CODE BEGIN 3 */
#ifdef INTERRUPT_ON
	//printmsg("Press User Button to Abort Auto Boot Within \r\n");
	printmsg("Press User Button to Boot Application2 Within \r\n");
	HAL_Delay(400);
	printmsg("3 Seconds...\r\n");
	HAL_Delay(1000);
	printmsg("2 Seconds..\r\n");
	HAL_Delay(1000);
	printmsg("1 Seconds.\r\n");
	HAL_Delay(1000);
	printmsg("BL_DEBUG_MSG:User Button is Not Pressed !... Executing User Application !\r\n");

	//jump to user application
	bootloader_jump_to_user_app_2();
#else
	/* Lets check whether button is pressed or not, if not pressed jump to user application */
	  if ( HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0) == GPIO_PIN_SET )
	  {
		  printmsg("BL_DEBUG_MSG:Button is pressed .. going to BL mode\r\n");

		  //we should continue in bootloader mode
		  //bootloader_uart_read_data();
		  bootloader_jump_to_user_app_2();

	  }
	  else
	  {
		  printmsg("BL_DEBUG_MSG:Button is not pressed .. executing user app\r\n");

			//jump to user application
			bootloader_jump_to_user_app_1();

	  }
#endif
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
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 84;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief CRC Initialization Function
 * @param None
 * @retval None
 */
static void MX_CRC_Init(void)
{

	/* USER CODE BEGIN CRC_Init 0 */

	/* USER CODE END CRC_Init 0 */

	/* USER CODE BEGIN CRC_Init 1 */

	/* USER CODE END CRC_Init 1 */
	hcrc.Instance = CRC;
	if (HAL_CRC_Init(&hcrc) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN CRC_Init 2 */

	/* USER CODE END CRC_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{

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
	if (HAL_UART_Init(&huart1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

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

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOG_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_RESET);

	/*Configure GPIO pin : PA0 */
	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : PD5 PD6 */
	GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pin : PG13 */
	GPIO_InitStruct.Pin = GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */
void  bootloader_uart_read_data(void)
{
	printmsg("Hello From BootLoader\r\n");
	printmsg("Connect This Board to Your PC using UART1(PA9/PA10) \r\n");
	printmsg("then, Run Programmer.py\r\n");
	printmsg("Enjoy..!! Made by Rajesh Dubey.!\r\n");
	uint8_t rcv_len=0;

	while(1)
	{
		memset(bl_rx_buffer,0,200);
		//here we will read and decode the commands coming from host
		//first read only one byte from the host , which is the "length" field of the command packet
		HAL_UART_Receive(C_UART,bl_rx_buffer,1,HAL_MAX_DELAY);
		rcv_len= bl_rx_buffer[0];
		HAL_UART_Receive(C_UART,&bl_rx_buffer[1],rcv_len,HAL_MAX_DELAY);
		switch(bl_rx_buffer[1])
		{
		case BL_GET_VER:
			bootloader_handle_getver_cmd(bl_rx_buffer);
			break;
		case BL_GET_HELP:
			bootloader_handle_gethelp_cmd(bl_rx_buffer);
			break;
		case BL_GET_CID:
			bootloader_handle_getcid_cmd(bl_rx_buffer);
			break;
		case BL_GET_RDP_STATUS:
			bootloader_handle_getrdp_cmd(bl_rx_buffer);
			break;
		case BL_GO_TO_ADDR:
			bootloader_handle_go_cmd(bl_rx_buffer);
			break;
		case BL_FLASH_ERASE:
			bootloader_handle_flash_erase_cmd(bl_rx_buffer);
			break;
		case BL_MEM_WRITE:
			bootloader_handle_mem_write_cmd(bl_rx_buffer);
			break;
		case BL_EN_RW_PROTECT:
			bootloader_handle_en_rw_protect(bl_rx_buffer);
			break;
		case BL_MEM_READ:
			bootloader_handle_mem_read(bl_rx_buffer);
			break;
		case BL_READ_SECTOR_P_STATUS:
			bootloader_handle_read_sector_protection_status(bl_rx_buffer);
			break;
		case BL_OTP_READ:
			bootloader_handle_read_otp(bl_rx_buffer);
			break;
		case BL_DIS_R_W_PROTECT:
			bootloader_handle_dis_rw_protect(bl_rx_buffer);
			break;
		case BL_BOOT_APPLICATION:
			bootloader_jump_to_user_app_1();
		default:
			printmsg("BL_DEBUG_MSG:Invalid command code received from host \r\n");
			break;
		}

	}

}


/*code to jump to user application
 *Here we are assuming FLASH_SECTOR2_BASE_ADDRESS
 *is where the user application is stored
 */


void bootloader_jump_to_user_app_1(void)
{
	//just a function pointer to hold the address of the reset handler of the user app.
	void (*app_reset_handler)(void);

	//printmsg("BL_DEBUG_MSG:bootloader_jump_to_user_app\r\n");


	// 1. configure the MSP by reading the value from the base address of the sector 2
	uint32_t msp_value = *(volatile uint32_t *)FLASH_SECTOR2_BASE_ADDRESS;
	//printmsg("BL_DEBUG_MSG:MSP value : %#x\r\n",msp_value);

	//This function comes from CMSIS.
	__set_MSP(msp_value);

	//SCB->VTOR = FLASH_SECTOR1_BASE_ADDRESS;

	/* 2. Now fetch the reset handler address of the user application
	 * from the location FLASH_SECTOR2_BASE_ADDRESS+4
	 */
	uint32_t resethandler_address = *(volatile uint32_t *) (FLASH_SECTOR2_BASE_ADDRESS + 4);
	//printmsg("BL_DEBUG_MSG: Reset Handler Addr_1st : %#x\r\n",resethandler_address);


	app_reset_handler = (void*) resethandler_address;
	//app_reset_handler =(void*) 0x08008789U;
	//printmsg("BL_DEBUG_MSG: Application Reset Handler Addr : %#x\r\n",app_reset_handler);


	//3. jump to reset handler of the user application
	app_reset_handler();

}

void bootloader_jump_to_user_app_2(void)
{
	//just a function pointer to hold the address of the reset handler of the user app.
	void (*app_reset_handler)(void);

	//printmsg("BL_DEBUG_MSG:bootloader_jump_to_user_app\r\n");


	// 1. configure the MSP by reading the value from the base address of the sector 2
	uint32_t msp_value = *(volatile uint32_t *)FLASH_SECTOR3_BASE_ADDRESS;
	//printmsg("BL_DEBUG_MSG:MSP value : %#x\r\n",msp_value);

	//This function comes from CMSIS.
	__set_MSP(msp_value);

	//SCB->VTOR = FLASH_SECTOR1_BASE_ADDRESS;

	/* 2. Now fetch the reset handler address of the user application
	 * from the location FLASH_SECTOR2_BASE_ADDRESS+4
	 */
	uint32_t resethandler_address = *(volatile uint32_t *) (FLASH_SECTOR3_BASE_ADDRESS + 4);
	//printmsg("BL_DEBUG_MSG: Reset Handler Addr_1st : %#x\r\n",resethandler_address);


	app_reset_handler = (void*) resethandler_address;
	//app_reset_handler =(void*) 0x08008789U;
	//printmsg("BL_DEBUG_MSG: Application Reset Handler Addr : %#x\r\n",app_reset_handler);


	//3. jump to reset handler of the user application
	app_reset_handler();

}
void printmsg(char *format,...)
{
#ifdef BL_DEBUG_MSG_EN
	char str[80];

	/*Extract the the argument list using VA apis */
	va_list args;
	va_start(args, format);
	vsprintf(str, format,args);
	HAL_UART_Transmit(D_UART,(uint8_t *)str, strlen(str),HAL_MAX_DELAY);
	va_end(args);
#endif
}

/**************Implementation of Boot-loader Command Handle functions *********/

/*Helper function to handle BL_GET_VER command */
void bootloader_handle_getver_cmd(uint8_t *bl_rx_buffer)
{
	uint8_t bl_version;

	// 1) verify the checksum
	printmsg("BL_DEBUG_MSG:bootloader_handle_getver_cmd\r\n");

	//Total length of the command packet
	uint32_t command_packet_len = bl_rx_buffer[0]+1 ;

	//extract the CRC32 sent by the Host
	uint32_t host_crc = *((uint32_t * ) (bl_rx_buffer+command_packet_len - 4) ) ;

	if (! bootloader_verify_crc(&bl_rx_buffer[0],command_packet_len-4,host_crc))
	{
		printmsg("BL_DEBUG_MSG:checksum success !!\r\n");
		// checksum is correct..
		bootloader_send_ack(bl_rx_buffer[0],1);
		bl_version=get_bootloader_version();
		printmsg("BL_DEBUG_MSG:BL_VER : %d %#x\r\n",bl_version,bl_version);
		bootloader_uart_write_data(&bl_version,1);


	}else
	{
		printmsg("BL_DEBUG_MSG:checksum fail !!\r\n");
		//checksum is wrong send nack
		bootloader_send_nack();
	}


}

/*Helper function to handle BL_GET_HELP command
 * Bootloader sends out All supported Command codes
 */
void bootloader_handle_gethelp_cmd(uint8_t *pBuffer)
{
	printmsg("BL_DEBUG_MSG:bootloader_handle_gethelp_cmd\r\n");

	//Total length of the command packet
	uint32_t command_packet_len = bl_rx_buffer[0]+1 ;

	//extract the CRC32 sent by the Host
	uint32_t host_crc = *((uint32_t * ) (bl_rx_buffer+command_packet_len - 4) ) ;

	if (! bootloader_verify_crc(&bl_rx_buffer[0],command_packet_len-4,host_crc))
	{
		printmsg("BL_DEBUG_MSG:checksum success !!\r\n");
		bootloader_send_ack(pBuffer[0],sizeof(supported_commands));
		bootloader_uart_write_data(supported_commands,sizeof(supported_commands) );

	}else
	{
		printmsg("BL_DEBUG_MSG:checksum fail !!\r\n");
		bootloader_send_nack();
	}

}

/*Helper function to handle BL_GET_CID command */
void bootloader_handle_getcid_cmd(uint8_t *pBuffer)
{
	uint16_t bl_cid_num = 0;
	printmsg("BL_DEBUG_MSG:bootloader_handle_getcid_cmd\r\n");

	//Total length of the command packet
	uint32_t command_packet_len = bl_rx_buffer[0]+1 ;

	//extract the CRC32 sent by the Host
	uint32_t host_crc = *((uint32_t * ) (bl_rx_buffer+command_packet_len - 4) ) ;

	if (! bootloader_verify_crc(&bl_rx_buffer[0],command_packet_len-4,host_crc))
	{
		printmsg("BL_DEBUG_MSG:checksum success !!\r\n");
		bootloader_send_ack(pBuffer[0],2);
		bl_cid_num = get_mcu_chip_id();
		printmsg("BL_DEBUG_MSG:MCU id : %d %#x !!\r\n",bl_cid_num, bl_cid_num);
		bootloader_uart_write_data((uint8_t *)&bl_cid_num,2);

	}else
	{
		printmsg("BL_DEBUG_MSG:checksum fail !!\r\n");
		bootloader_send_nack();
	}


}

/*Helper function to handle BL_GET_RDP_STATUS command */
void bootloader_handle_getrdp_cmd(uint8_t *pBuffer)
{
	uint8_t rdp_level = 0x00;
	printmsg("BL_DEBUG_MSG:bootloader_handle_getrdp_cmd\r\n");

	//Total length of the command packet
	uint32_t command_packet_len = bl_rx_buffer[0]+1 ;

	//extract the CRC32 sent by the Host
	uint32_t host_crc = *((uint32_t * ) (bl_rx_buffer+command_packet_len - 4) ) ;

	if (! bootloader_verify_crc(&bl_rx_buffer[0],command_packet_len-4,host_crc))
	{
		printmsg("BL_DEBUG_MSG:checksum success !!\r\n");
		bootloader_send_ack(pBuffer[0],1);
		rdp_level = get_flash_rdp_level();
		printmsg("BL_DEBUG_MSG:RDP level: %d %#x\r\n",rdp_level,rdp_level);
		bootloader_uart_write_data(&rdp_level,1);

	}else
	{
		printmsg("BL_DEBUG_MSG:checksum fail !!\r\n");
		bootloader_send_nack();
	}


}

/*Helper function to handle BL_GO_TO_ADDR command */
void bootloader_handle_go_cmd(uint8_t *pBuffer)
{
	uint32_t go_address = 0;
	uint8_t addr_valid = ADDR_VALID;
	uint8_t addr_invalid = ADDR_INVALID;

	printmsg("BL_DEBUG_MSG:bootloader_handle_go_cmd\r\n");

	//Total length of the command packet
	uint32_t command_packet_len = bl_rx_buffer[0]+1 ;

	//extract the CRC32 sent by the Host
	uint32_t host_crc = *((uint32_t * ) (bl_rx_buffer+command_packet_len - 4) ) ;

	if (! bootloader_verify_crc(&bl_rx_buffer[0],command_packet_len-4,host_crc))
	{
		printmsg("BL_DEBUG_MSG:checksum success !!\r\n");

		bootloader_send_ack(pBuffer[0],1);

		//extract the go address
		go_address = *((uint32_t *)&pBuffer[2] );
		printmsg("BL_DEBUG_MSG:GO addr: %#x\r\n",go_address);

		if( verify_address(go_address) == ADDR_VALID )
		{
			//tell host that address is fine
			bootloader_uart_write_data(&addr_valid,1);

			/*jump to "go" address.
            we dont care what is being done there.
            host must ensure that valid code is present over there
            Its not the duty of bootloader. so just trust and jump */

			/* Not doing the below line will result in hardfault exception for ARM cortex M */
			//watch : https://www.youtube.com/watch?v=VX_12SjnNhY

			go_address+=1; //make T bit =1

			void (*lets_jump)(void) = (void *)go_address;

			printmsg("BL_DEBUG_MSG: jumping to go address!\r \n");

			lets_jump();

		}else
		{
			printmsg("BL_DEBUG_MSG:GO addr invalid ! \r\n");
			//tell host that address is invalid
			bootloader_uart_write_data(&addr_invalid,1);
		}

	}else
	{
		printmsg("BL_DEBUG_MSG:checksum fail !!\r\n");
		bootloader_send_nack();
	}


}

/*Helper function to handle BL_FLASH_ERASE command */
void bootloader_handle_flash_erase_cmd(uint8_t *pBuffer)
{
	uint8_t erase_status = 0x00;
	printmsg("BL_DEBUG_MSG:bootloader_handle_flash_erase_cmd\r\n");

	//Total length of the command packet
	uint32_t command_packet_len = bl_rx_buffer[0]+1 ;

	//extract the CRC32 sent by the Host
	uint32_t host_crc = *((uint32_t * ) (bl_rx_buffer+command_packet_len - 4) ) ;

	if (! bootloader_verify_crc(&bl_rx_buffer[0],command_packet_len-4,host_crc))
	{
		printmsg("BL_DEBUG_MSG:checksum success !!\r\n");
		bootloader_send_ack(pBuffer[0],1);
		printmsg("BL_DEBUG_MSG:initial_sector : %d  no_ofsectors: %d\r\n",pBuffer[2],pBuffer[3]);

		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13,1);
		erase_status = execute_flash_erase(pBuffer[2] , pBuffer[3]);
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13,0);

		printmsg("BL_DEBUG_MSG: flash erase status: %#x\r\n",erase_status);

		bootloader_uart_write_data(&erase_status,1);

	}else
	{
		printmsg("BL_DEBUG_MSG:checksum fail !!\r\n");
		bootloader_send_nack();
	}
}

/*Helper function to handle BL_MEM_WRITE command */
void bootloader_handle_mem_write_cmd(uint8_t *pBuffer)
{
	uint8_t addr_valid = ADDR_VALID;
	uint8_t write_status = 0x00;
	uint8_t chksum =0, len=0;
	len = pBuffer[0];
	uint8_t payload_len = pBuffer[6];

	uint32_t mem_address = *((uint32_t *) ( &pBuffer[2]) );

	chksum = pBuffer[len];

	printmsg("BL_DEBUG_MSG:bootloader_handle_mem_write_cmd\r\n");

	//Total length of the command packet
	uint32_t command_packet_len = bl_rx_buffer[0]+1 ;

	//extract the CRC32 sent by the Host
	uint32_t host_crc = *((uint32_t * ) (bl_rx_buffer+command_packet_len - 4) ) ;


	if (! bootloader_verify_crc(&bl_rx_buffer[0],command_packet_len-4,host_crc))
	{
		printmsg("BL_DEBUG_MSG:checksum success !!\r\n");

		bootloader_send_ack(pBuffer[0],1);

		printmsg("BL_DEBUG_MSG: mem write address : %#x\r\n",mem_address);

		if( verify_address(mem_address) == ADDR_VALID )
		{

			printmsg("BL_DEBUG_MSG: valid mem write address\r\n");

			//glow the led to indicate bootloader is currently writing to memory
			HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_SET);

			//execute mem write
			write_status = execute_mem_write(&pBuffer[7],mem_address, payload_len);

			//turn off the led to indicate memory write is over
			HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_RESET);

			//inform host about the status
			bootloader_uart_write_data(&write_status,1);

		}else
		{
			printmsg("BL_DEBUG_MSG: invalid mem write address\r\n");
			write_status = ADDR_INVALID;
			//inform host that address is invalid
			bootloader_uart_write_data(&write_status,1);
		}


	}else
	{
		printmsg("BL_DEBUG_MSG:checksum fail !!\r\n");
		bootloader_send_nack();
	}

}

/*Helper function to handle BL_EN_RW_PROTECT  command */
void bootloader_handle_en_rw_protect(uint8_t *pBuffer)
{
	uint8_t status = 0x00;
	printmsg("BL_DEBUG_MSG:bootloader_handle_endis_rw_protect\r\n");

	//Total length of the command packet
	uint32_t command_packet_len = bl_rx_buffer[0]+1 ;

	//extract the CRC32 sent by the Host
	uint32_t host_crc = *((uint32_t * ) (bl_rx_buffer+command_packet_len - 4) ) ;

	if (! bootloader_verify_crc(&bl_rx_buffer[0],command_packet_len-4,host_crc))
	{
		printmsg("BL_DEBUG_MSG:checksum success !!\r\n");
		bootloader_send_ack(pBuffer[0],1);

		status = configure_flash_sector_rw_protection(pBuffer[2] , pBuffer[3],0);

		printmsg("BL_DEBUG_MSG: flash erase status: %#x\r\n",status);

		bootloader_uart_write_data(&status,1);

	}else
	{
		printmsg("BL_DEBUG_MSG:checksum fail !!\r\n");
		bootloader_send_nack();
	}


}


/*Helper function to handle BL_EN_RW_PROTECT  command */
void bootloader_handle_dis_rw_protect(uint8_t *pBuffer)
{
	uint8_t status = 0x00;
	printmsg("BL_DEBUG_MSG:bootloader_handle_dis_rw_protect\r\n");

	//Total length of the command packet
	uint32_t command_packet_len = bl_rx_buffer[0]+1 ;

	//extract the CRC32 sent by the Host
	uint32_t host_crc = *((uint32_t * ) (bl_rx_buffer+command_packet_len - 4) ) ;

	if (! bootloader_verify_crc(&bl_rx_buffer[0],command_packet_len-4,host_crc))
	{
		printmsg("BL_DEBUG_MSG:checksum success !!\r\n");
		bootloader_send_ack(pBuffer[0],1);

		status = configure_flash_sector_rw_protection(0,0,1);

		printmsg("BL_DEBUG_MSG: flash erase status: %#x\r\n",status);

		bootloader_uart_write_data(&status,1);

	}else
	{
		printmsg("BL_DEBUG_MSG:checksum fail !!\r\n");
		bootloader_send_nack();
	}


}

/*Helper function to handle BL_MEM_READ command */
void bootloader_handle_mem_read (uint8_t *pBuffer)
{


}

/*Helper function to handle _BL_READ_SECTOR_P_STATUS command */
void bootloader_handle_read_sector_protection_status(uint8_t *pBuffer)
{
	uint16_t status;
	printmsg("BL_DEBUG_MSG:bootloader_handle_read_sector_protection_status\r\n");

	//Total length of the command packet
	uint32_t command_packet_len = bl_rx_buffer[0]+1 ;

	//extract the CRC32 sent by the Host
	uint32_t host_crc = *((uint32_t * ) (bl_rx_buffer+command_packet_len - 4) ) ;

	if (! bootloader_verify_crc(&bl_rx_buffer[0],command_packet_len-4,host_crc))
	{
		printmsg("BL_DEBUG_MSG:checksum success !!\r\n");
		bootloader_send_ack(pBuffer[0],2);
		status=read_OB_rw_protection_status();
		printmsg("BL_DEBUG_MSG: nWRP status: %#x\r\n",status);
		bootloader_uart_write_data((uint8_t*)&status,2);

	}else
	{
		printmsg("BL_DEBUG_MSG:checksum fail !!\r\n");
		bootloader_send_nack();
	}

}

/*Helper function to handle BL_OTP_READ command */
void bootloader_handle_read_otp(uint8_t *pBuffer)
{


}

/*This function sends ACK if CRC matches along with "len to follow"*/
void bootloader_send_ack(uint8_t command_code, uint8_t follow_len)
{
	//here we send 2 byte.. first byte is ack and the second byte is len value
	uint8_t ack_buf[2];
	ack_buf[0] = BL_ACK;
	ack_buf[1] = follow_len;
	HAL_UART_Transmit(C_UART,ack_buf,2,HAL_MAX_DELAY);

}

/*This function sends NACK */
void bootloader_send_nack(void)
{
	uint8_t nack = BL_NACK;
	HAL_UART_Transmit(C_UART,&nack,1,HAL_MAX_DELAY);
}

//This verifies the CRC of the given buffer in pData .
uint8_t bootloader_verify_crc (uint8_t *pData, uint32_t len, uint32_t crc_host)
{
	uint32_t uwCRCValue=0xff;

	for (uint32_t i=0 ; i < len ; i++)
	{
		uint32_t i_data = pData[i];
		uwCRCValue = HAL_CRC_Accumulate(&hcrc, &i_data, 1);
	}

	/* Reset CRC Calculation Unit */
	__HAL_CRC_DR_RESET(&hcrc);

	if( uwCRCValue == crc_host)
	{
		return VERIFY_CRC_SUCCESS;
	}

	return VERIFY_CRC_FAIL;
}

/* This function writes data in to C_UART */
void bootloader_uart_write_data(uint8_t *pBuffer,uint32_t len)
{
	/*you can replace the below ST's USART driver API call with your MCUs driver API call */
	HAL_UART_Transmit(C_UART,pBuffer,len,HAL_MAX_DELAY);

}


//Just returns the macro value .
uint8_t get_bootloader_version(void)
{
	return (uint8_t)BL_VERSION;
}

//Read the chip identifier or device Identifier
uint16_t get_mcu_chip_id(void)
{
	/*
	The STM32F446xx MCUs integrate an MCU ID code. This ID identifies the ST MCU partnumber
	and the die revision. It is part of the DBG_MCU component and is mapped on the
	external PPB bus (see Section 33.16 on page 1304). This code is accessible using the
	JTAG debug pCat.2ort (4 to 5 pins) or the SW debug port (two pins) or by the user software.
	It is even accessible while the MCU is under system reset. */
	uint16_t cid;
	cid = (uint16_t)(DBGMCU->IDCODE) & 0x0FFF;
	return  cid;

}


/*This function reads the RDP ( Read protection option byte) value
 *For more info refer "Table 9. Description of the option bytes" in stm32f446xx RM
 */
uint8_t get_flash_rdp_level(void)
{

	uint8_t rdp_status=0;
#if 0
	FLASH_OBProgramInitTypeDef  ob_handle;
	HAL_FLASHEx_OBGetConfig(&ob_handle);
	rdp_status = (uint8_t)ob_handle.RDPLevel;
#else

	volatile uint32_t *pOB_addr = (uint32_t*) 0x1FFFC000;
	rdp_status =  (uint8_t)(*pOB_addr >> 8) ;
#endif

	return rdp_status;

}

//verify the address sent by the host .
uint8_t verify_address(uint32_t go_address)
{
	//so, what are the valid addresses to which we can jump ?
	//can we jump to system memory ? yes
	//can we jump to sram1 memory ?  yes
	//can we jump to sram2 memory ? yes
	//can we jump to backup sram memory ? yes
	//can we jump to peripheral memory ? its possible , but dont allow. so no
	//can we jump to external memory ? yes.

	//incomplete -poorly written .. optimize it
	if ( go_address >= SRAM1_BASE && go_address <= SRAM1_END)
	{
		return ADDR_VALID;
	}
	else if ( go_address >= SRAM2_BASE && go_address <= SRAM2_END)
	{
		return ADDR_VALID;
	}
	else if ( go_address >= FLASH_BASE && go_address <= FLASH_END)
	{
		return ADDR_VALID;
	}
	else if ( go_address >= BKPSRAM_BASE && go_address <= BKPSRAM_END)
	{
		return ADDR_VALID;
	}
	else
		return ADDR_INVALID;
}

uint8_t execute_flash_erase(uint8_t sector_number , uint8_t number_of_sector)
{
	//we have totally 8 sectors in STM32F446RE mcu .. sector[0 to 7]
	//number_of_sector has to be in the range of 0 to 7
	// if sector_number = 0xff , that means mass erase !
	//Code needs to modified if your MCU supports more flash sectors
	FLASH_EraseInitTypeDef flashErase_handle;
	uint32_t sectorError;
	HAL_StatusTypeDef status;


	if( number_of_sector > 8 )
		return INVALID_SECTOR;

	if( (sector_number == 0xff ) || (sector_number <= 7) )
	{
		if(sector_number == (uint8_t) 0xff)
		{
			flashErase_handle.TypeErase = FLASH_TYPEERASE_MASSERASE;
		}else
		{
			/*Here we are just calculating how many sectors needs to erased */
			uint8_t remanining_sector = 8 - sector_number;
			if( number_of_sector > remanining_sector)
			{
				number_of_sector = remanining_sector;
			}
			flashErase_handle.TypeErase = FLASH_TYPEERASE_SECTORS;
			flashErase_handle.Sector = sector_number; // this is the initial sector
			flashErase_handle.NbSectors = number_of_sector;
		}
		flashErase_handle.Banks = FLASH_BANK_1;

		/*Get access to touch the flash registers */
		HAL_FLASH_Unlock();
		flashErase_handle.VoltageRange = FLASH_VOLTAGE_RANGE_3;  // our mcu will work on this voltage range
		status = (uint8_t) HAL_FLASHEx_Erase(&flashErase_handle, &sectorError);
		HAL_FLASH_Lock();

		return status;
	}


	return INVALID_SECTOR;
}

/*This function writes the contents of pBuffer to  "mem_address" byte by byte */
//Note1 : Currently this function supports writing to Flash only .
//Note2 : This functions does not check whether "mem_address" is a valid address of the flash range.
uint8_t execute_mem_write(uint8_t *pBuffer, uint32_t mem_address, uint32_t len)
{
	uint8_t status=HAL_OK;

	//We have to unlock flash module to get control of registers
	HAL_FLASH_Unlock();

	for(uint32_t i = 0 ; i <len ; i++)
	{
		//Here we program the flash byte by byte
		status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE,mem_address+i,pBuffer[i] );
	}

	HAL_FLASH_Lock();

	return status;
}


/*
Modifying user option bytes
To modify the user option value, follow the sequence below:
1. Check that no Flash memory operation is ongoing by checking the BSY bit in the
FLASH_SR register
2. Write the desired option value in the FLASH_OPTCR register.
3. Set the option start bit (OPTSTRT) in the FLASH_OPTCR register
4. Wait for the BSY bit to be cleared.
 */
uint8_t configure_flash_sector_rw_protection(uint8_t sector_details, uint8_t protection_mode, uint8_t disable)
{
	//First configure the protection mode
	//protection_mode =1 , means write protect of the user flash sectors
	//protection_mode =2, means read/write protect of the user flash sectors
	//According to RM of stm32f446xx TABLE 9, We have to modify the address 0x1FFF C008 bit 15(SPRMOD)

	//Flash option control register (OPTCR)
	volatile uint32_t *pOPTCR = (uint32_t*) 0x40023C14;

	if(disable)
	{

		//disable all r/w protection on sectors

		//Option byte configuration unlock
		HAL_FLASH_OB_Unlock();

		//wait till no active operation on flash
		while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);

		//clear the 31st bit (default state)
		//please refer : Flash option control register (FLASH_OPTCR) in RM
		*pOPTCR &= ~(1 << 31);

		//clear the protection : make all bits belonging to sectors as 1
		*pOPTCR |= (0xFF << 16);

		//Set the option start bit (OPTSTRT) in the FLASH_OPTCR register
		*pOPTCR |= ( 1 << 1);

		//wait till no active operation on flash
		while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);

		HAL_FLASH_OB_Lock();

		return 0;

	}

	if(protection_mode == (uint8_t) 1)
	{
		//we are putting write protection on the sectors encoded in sector_details argument

		//Option byte configuration unlock
		HAL_FLASH_OB_Unlock();

		//wait till no active operation on flash
		while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);

		//here we are setting just write protection for the sectors
		//clear the 31st bit
		//please refer : Flash option control register (FLASH_OPTCR) in RM
		*pOPTCR &= ~(1 << 31);

		//put write protection on sectors
		*pOPTCR &= ~ (sector_details << 16);

		//Set the option start bit (OPTSTRT) in the FLASH_OPTCR register
		*pOPTCR |= ( 1 << 1);

		//wait till no active operation on flash
		while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);

		HAL_FLASH_OB_Lock();
	}

	else if (protection_mode == (uint8_t) 2)
	{
		//Option byte configuration unlock
		HAL_FLASH_OB_Unlock();

		//wait till no active operation on flash
		while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);

		//here wer are setting read and write protection for the sectors
		//set the 31st bit
		//please refer : Flash option control register (FLASH_OPTCR) in RM
		*pOPTCR |= (1 << 31);

		//put read and write protection on sectors
		*pOPTCR &= ~(0xff << 16);
		*pOPTCR |= (sector_details << 16);

		//Set the option start bit (OPTSTRT) in the FLASH_OPTCR register
		*pOPTCR |= ( 1 << 1);

		//wait till no active operation on flash
		while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);

		HAL_FLASH_OB_Lock();
	}

	return 0;
}

uint16_t read_OB_rw_protection_status(void)
{
	//This structure is given by ST Flash driver to hold the OB(Option Byte) contents .
	FLASH_OBProgramInitTypeDef OBInit;

	//First unlock the OB(Option Byte) memory access
	HAL_FLASH_OB_Unlock();
	//get the OB configuration details
	HAL_FLASHEx_OBGetConfig(&OBInit);
	//Lock back .
	HAL_FLASH_Lock();

	//We are just interested in r/w protection status of the sectors.
	return (uint16_t)OBInit.WRPSector;

}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void _Error_Handler(char * file, int line)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while(1)
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
