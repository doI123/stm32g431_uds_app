/**
 *******************************************************************************
 * @file  stm32g4xx_hal_conf.h
 * @brief HAL 配置（裁剪版）——只启用本工程实际用到的模块
 *
 *   启用：RCC / GPIO / CORTEX / FLASH / PWR / FDCAN
 *   其余模块一律注释掉，避免链接进无用代码（Flash 只有 128 KB）。
 *
 *   注意：本文件必须通过 include 路径优先被找到，
 *         否则会用固件包中的 hal_conf_template.h 默认全开配置。
 *******************************************************************************
 * Copyright (C) 2026, all rights reserved.
 *
 * This software component is licensed under BSD 3-Clause license.
 *******************************************************************************
 */
#ifndef STM32G4xx_HAL_CONF_H
#define STM32G4xx_HAL_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

/* ########################## Module Selection ############################## */
#define HAL_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED
#define HAL_GPIO_MODULE_ENABLED
#define HAL_RCC_MODULE_ENABLED
#define HAL_FLASH_MODULE_ENABLED
#define HAL_PWR_MODULE_ENABLED
#define HAL_FDCAN_MODULE_ENABLED
#define HAL_EXTI_MODULE_ENABLED

/* ########################## Register Callbacks ############################ */
#define USE_HAL_FDCAN_REGISTER_CALLBACKS      0U
#define USE_HAL_FLASH_REGISTER_CALLBACKS      0U
#define USE_HAL_RCC_REGISTER_CALLBACKS        0U

/* ########################## Oscillator Values ############################# */
#if !defined  (HSE_VALUE)
#define HSE_VALUE               (8000000UL)     /*!< 外部高速晶振 8 MHz */
#endif

#if !defined  (HSE_STARTUP_TIMEOUT)
#define HSE_STARTUP_TIMEOUT     (100UL)         /*!< HSE 起振超时 (ms)  */
#endif

#if !defined  (HSI_VALUE)
#define HSI_VALUE               (16000000UL)    /*!< 内部 RC 16 MHz     */
#endif

#if !defined  (HSI48_VALUE)
#define HSI48_VALUE             (48000000UL)    /*!< 内部 RC 48 MHz     */
#endif

#if !defined  (LSI_VALUE)
#define LSI_VALUE               (32000UL)       /*!< 内部低速 32 kHz    */
#endif

#if !defined  (LSE_VALUE)
#define LSE_VALUE               (32768UL)       /*!< 外部低速 32.768 kHz */
#endif

#if !defined  (LSE_STARTUP_TIMEOUT)
#define LSE_STARTUP_TIMEOUT     (5000UL)
#endif

#if !defined  (EXTERNAL_CLOCK_VALUE)
#define EXTERNAL_CLOCK_VALUE    (12288000UL)
#endif

/* ########################### System Configuration ######################### */
#define  VDD_VALUE                    (3300UL)  /*!< VDD 电压 (mV)      */
#define  TICK_INT_PRIORITY            (0x0FUL)  /*!< SysTick 中断优先级  */
#define  USE_RTOS                     0U
#define  PREFETCH_ENABLE              0U
#define  INSTRUCTION_CACHE_ENABLE     1U
#define  DATA_CACHE_ENABLE            1U

/* ########################## Assert Selection ############################## */
/* 发布版本关闭断言；调试时可打开以捕获参数错误 */
/* #define USE_FULL_ASSERT    1U */

/* ################## Register callback feature configuration ############### */
#define USE_SPI_CRC                   0U

/* Includes ------------------------------------------------------------------*/
#ifdef HAL_RCC_MODULE_ENABLED
#include "stm32g4xx_hal_rcc.h"
#endif /* HAL_RCC_MODULE_ENABLED */

#ifdef HAL_GPIO_MODULE_ENABLED
#include "stm32g4xx_hal_gpio.h"
#endif /* HAL_GPIO_MODULE_ENABLED */

#ifdef HAL_EXTI_MODULE_ENABLED
#include "stm32g4xx_hal_exti.h"
#endif /* HAL_EXTI_MODULE_ENABLED */

#ifdef HAL_FLASH_MODULE_ENABLED
#include "stm32g4xx_hal_flash.h"
#endif /* HAL_FLASH_MODULE_ENABLED */

#ifdef HAL_PWR_MODULE_ENABLED
#include "stm32g4xx_hal_pwr.h"
#endif /* HAL_PWR_MODULE_ENABLED */

#ifdef HAL_CORTEX_MODULE_ENABLED
#include "stm32g4xx_hal_cortex.h"
#endif /* HAL_CORTEX_MODULE_ENABLED */

#ifdef HAL_FDCAN_MODULE_ENABLED
#include "stm32g4xx_hal_fdcan.h"
#endif /* HAL_FDCAN_MODULE_ENABLED */

/* Exported macro ------------------------------------------------------------*/
#ifdef  USE_FULL_ASSERT
/**
  * @brief  The assert_param macro is used for function's parameters check.
  */
#define assert_param(expr) ((expr) ? (void)0U : assert_failed((uint8_t *)__FILE__, __LINE__))
void assert_failed(uint8_t *file, uint32_t line);
#else
#define assert_param(expr) ((void)0U)
#endif /* USE_FULL_ASSERT */

#ifdef __cplusplus
}
#endif

#endif /* STM32G4xx_HAL_CONF_H */
