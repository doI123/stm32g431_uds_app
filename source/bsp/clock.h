/**
 *******************************************************************************
 * @file  clock.h
 * @brief 时钟初始化与毫秒时基（8 MHz HSE -> 170 MHz 系统主频）
 *******************************************************************************
 * Copyright (C) 2026, all rights reserved.
 *
 * This software component is licensed under BSD 3-Clause license.
 *******************************************************************************
 */
#ifndef __CLOCK_H__
#define __CLOCK_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  初始化系统时钟：8 MHz HSE -> PLL -> 170 MHz。
 *
 *   时钟树：
 *     PLLM = /2  -> 4 MHz
 *     VCO  = 4 MHz * 85 = 340 MHz
 *     PLLR = /2  -> 170 MHz = CK_SYS
 *     PLLQ = /8  -> 42.5 MHz（保留配置，但不用于 FDCAN）
 *
 *   注意：FDCAN 内核时钟源已改为 **HSE 8 MHz**（见 mcan.c），
 *         原因见 board.h 的说明（8 MHz 才能精确整除 800 kbps）。
 *
 *   说明：必须在 main() 开头调用（HAL_Init() 之后）。
 *         170 MHz 需要 PWR_REGULATOR_VOLTAGE_SCALE1_BOOST 与 FLASH_LATENCY_4。
 *
 *   容错：时钟配置失败时保持 HSI 16 MHz 运行并返回 -1，不阻塞，
 *         以便主循环闪灯报错、故障可在现场观察到。
 *
 * @param  无
 * @retval int32_t 0 = 成功（主频 170 MHz）；-1 = 失败（已退回到 HSI）
 */
int32_t Board_ClockInit(void);

/**
 * @brief  启动 SysTick 毫秒时基（中断方式）。
 * @param  [in] u32FreqHz 时基频率（Hz），传 1000 表示 1 ms 一次中断。
 * @retval 无
 */
void SysTick_Init(uint32_t u32FreqHz);

/**
 * @brief  获取当前毫秒计数（回绕安全）。
 * @param  无
 * @retval uint32_t 毫秒计数
 */
uint32_t SysTick_GetTick(void);

/**
 * @brief  毫秒计数自增，供 SysTick_Handler 调用。
 * @param  无
 * @retval 无
 */
void SysTick_IncTick(void);

/**
 * @brief  阻塞延时。
 * @param  [in] u32Ms 延时毫秒数
 * @retval 无
 */
void SysTick_Delay(uint32_t u32Ms);

#ifdef __cplusplus
}
#endif

#endif /* __CLOCK_H__ */
