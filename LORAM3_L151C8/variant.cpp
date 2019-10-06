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

/* 
 *  * [jan.19] addon for selecting clock source
 *   * select HSE (external clock) or HSI (internal)
 *    */
//#define _INTERNAL_CLOCK     // select HSI (HSE otherwise)

#ifdef __cplusplus
extern "C" {
#endif

// Pin number
const PinName digitalPin[] = {
/* USB connector on the top, OLED display side */
/* Left side, from bottom to top */
  PA_2,     //D0 - Radio RST
  PA_3,     //D1 - Radio DIO0
  PA_4,     //D2 - Radio NSS
  PA_5,     //D3 - RADIO SCK
  PA_6,     //D4 - Radio MISO
  PA_7,     //D5 - Radio MOSI
  PB_0,     //D6
  PB_1,     //D7
  PB_10,    //D8
  PB_11,    //D9
  PB_12,    //D10
  PB_13,    //D11
  PB_14,    //D12
  PB_15,    //D13
  PA_8,     //D14
  PA_9,     //D15 - Usart1 RX
  PA_10,    //D16 - Usart1 TX
/* Right side, from bottom to top */
  PA_1,     //D17/ADC1 - Bat mon
  PA_0,     //D18/ADC0
  PC_13,    //D19 - LED
  PB_9,     //D20
  PB_8,     //D21
  PB_7,     //D22 - OLED I2C1 SDA
  PB_6,     //D23 - OLED I2C1 SCL
  PB_5,     //D24 - OLED RST
  PB_4,     //D25
  PB_3,     //D26
  PA_15,    //D27
  PA_14,    //D28 - STlink SWCLK
  PA_13     //D29 - STlink SWDIO
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
#ifdef _INTERNAL_CLOCK
    #warning "STM32L151 HSI INTERNAL CLOCK SELECTED !!"
    // HSI only internal clock
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;    // for ADC
    RCC_OscInitStruct.HSICalibrationValue = 16;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
    RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV2;
#else
    // external clock
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.LSEState = RCC_LSE_ON;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;    // for ADC
    RCC_OscInitStruct.HSICalibrationValue = 16;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;   // MUL12 --> VCO/2 ==> 48MHz USB with 8MHz HSE
    RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
#endif

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

