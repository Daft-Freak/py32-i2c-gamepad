/**
  ******************************************************************************
  * @file    system_py32f0xx.h
  * @brief   CMSIS Cortex-M0+ Device Peripheral Access Layer Header File for
    *                    PY32F0xx Device Series
  * @version v1.0.0
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 Puya Semiconductor Co.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by Puya under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* This file refers the CMSIS standard, some adjustments are made according to Puya chips */
#ifndef SYSTEM_PY32F0XX_H
#define SYSTEM_PY32F0XX_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported variables --------------------------------------------------------*/
extern uint32_t SystemCoreClock;     /*!< System Clock Frequency (Core Clock) */

/* Exported constants --------------------------------------------------------*/
extern const uint32_t AHBPrescTable[16];  /*!< AHB prescalers table values */
extern const uint32_t APBPrescTable[8];   /*!< APB prescalers table values */
extern const uint32_t HSIFreqTable[8];    /*!< HSI frequency table values */

/** @addtogroup PY32F0xx_System_Exported_Functions
  * @{
  */
  
extern void SystemInit(void);
extern void SystemCoreClockUpdate(void);
/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* SYSTEM_PY32F0XX_H */
