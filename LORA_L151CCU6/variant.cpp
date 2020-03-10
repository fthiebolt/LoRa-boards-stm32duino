/*
 *******************************************************************************
 * Copyright (c) 2018, STMicroelectronics
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of STMicroelectronics nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
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
 *******************************************************************************
 */

#include "pins_arduino.h"

#ifdef __cplusplus
extern "C" {
#endif

// Pin number
const PinName digitalPin[] = {
	/* SMA connector on the bottom, STM32L151 device display side */
	/* Left side, from bottom to top */
	PB_1,		//D0	- LoRa_DIO2
	PB_0,		//D1	- LoRa_DIO3
	PA_4,		//D2	- LoRa_NSS
	PA_7,		//D3	- LoRa_MOSI
	PA_6,		//D4	- LoRa_MISO
	PA_5,		//D5	- LoRa_SCK
	PA_2,		//D6	- Power detection
	PA_1,		//D7	- ADC_IN1
	PA_0,		//D8	- ADC_IN0
	PB_5,		//D9	- Alternate functions : SPI1_SPI3_MOSI
	PB_6,		//D10	- I2C1_SCL 
	PB_7,		//D11	- I2C1_SDA
	PB_9,		//D12	- Alternate functions : I2C1_SDA
	PB_8,		//D13	- Alternate functions : I2C1_SCL & LED
	/* Right side, from bottom to top */
	PB_10,	//D14	- LoRa_DIO1
	PB_11,	//D15	- LoRa_DIO0
	PA_3,		//D16	- LoRa_RST
	PB_12,	//D17	- Alternate functions : SPI2_NSS
	PB_4,		//D18	- Alternate functions : SPI1_SPI3_MISO
	PB_3,		//D19 - Vext control
	PA_15,	//D20 - Alternate functions : SPI1_SPI3_NSS
	PB_13,	//D21 - Alternate functions : SPI2_SCK
	PB_14,	//D22	- Alternate functions : SPI2_MISO
	PB_15,	//D23	- Alternate functions : SPI2_MOSI
	PA_8		//D24	- 
	PA_9,		//D25	- UART1_TX
	PA_10		//D26	- UART1_RX
	PA_13,	//D27 - STlink SWDIO: Serial Wire Debug Port 
	PA_14		//D28 - STlink SWCLK: Serial Wire Debug Port 
};

#ifdef __cplusplus
}
#endif

// ----------------------------------------------------------------------------

#ifdef __cplusplus
extern "C" {
#endif
/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = HSE
  *            SYSCLK(Hz)                     = 32000000
  *            HCLK(Hz)                       = 32000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            HSE Frequency(Hz)              = 8000000
  *            HSI Frequency(Hz)              = 16000000
  *            PLLMUL                         = 12
  *            PLLDIV                         = 3
  *            Flash Latency(WS)              = 1
  * @retval None
  */
WEAK void SystemClock_Config(void)
{
  	RCC_OscInitTypeDef RCC_OscInitStruct;
  	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /* WARNING: USB bootloader from BSFrance seems to disable CLK ...
     * ... means that you get bricked once you overwrite it through serial upload :(
     * so we re-enable it there :) */
    __HAL_RCC_PWR_CLK_ENABLE();

    /* Configure the main internal regulator output voltage */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /* Initializes the CPU, AHB and APB busses clocks */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.LSEState = RCC_LSE_ON;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;    // for ADC
    RCC_OscInitStruct.HSICalibrationValue = 16;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;   // MUL12 --> VCO/2 ==> 48MHz USB with 8MHz HSE
    RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
		// Initialization Error
	    _Error_Handler(__FILE__, __LINE__);
    }

    /* Initializes the CPU, AHB and APB busses clocks */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
    {
    	// Initialization Error
		_Error_Handler(__FILE__, __LINE__);
    }

    /* Initialise peripherals clocks */
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
	PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
    
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
    	// Initialization Error
		_Error_Handler(__FILE__, __LINE__);
	}
}

#ifdef __cplusplus
}
#endif

