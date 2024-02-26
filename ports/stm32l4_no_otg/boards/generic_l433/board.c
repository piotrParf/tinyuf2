#include "board_api.h"
#include "stm32l4xx_hal.h"

void Error_Handler(void) {
  for (;;) {
  }
}

#define HAL_CHECK(x)                                                           \
  if (x != HAL_OK)                                                             \
  Error_Handler()

//--------------------------------------------------------------------+
// RCC Clock
//--------------------------------------------------------------------+
void clock_init(void) {
  // enable the debugger while sleeping. Todo move somewhere more central (kind
  // of generally useful in a debug build)
  SET_BIT(DBGMCU->CR, DBGMCU_CR_DBG_SLEEP);

  //  Set tick interrupt priority, default HAL value is intentionally invalid
  //  Without this, USB does not function.
  HAL_InitTick((1UL << __NVIC_PRIO_BITS) - 1UL);

  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /* Enable Power Control clock */
  // Sets the drive strength of 32kHz external crystal, in line with
  // calculations specified in ST AN2867 sections 3.3, 3.4, and STM32L4
  // datasheet DS12023 Table 58. LSE oscillator characteristics. The drive
  // strength RCC_LSEDRIVE_LOW is marginal for the 32kHz crystal oscillator
  // stability, and RCC_LSEDRIVE_MEDIUMLOW meets the calculated drive strength
  // with a small margin for parasitic capacitance.
  //__HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_MEDIUMLOW);
  __HAL_RCC_PWR_CLK_ENABLE();

  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK) {
    Error_Handler();
  }

  /* Activate PLL with MSI */
  RCC_OscInitStruct.OscillatorType =
      RCC_OSCILLATORTYPE_HSI48 | RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  HAL_CHECK(HAL_RCC_OscConfig(&RCC_OscInitStruct));

  /* Enable MSI Auto-calibration through LSE */
  // HAL_RCCEx_EnableMSIPLLMode();

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
     clocks dividers */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  HAL_CHECK(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4));

  // PeriphClkInitStruct.PeriphClockSelection =
  //     RCC_PERIPHCLK_USB | RCC_PERIPHCLK_RTC;
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
  // PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  HAL_CHECK(HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit));
}