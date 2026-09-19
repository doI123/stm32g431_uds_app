/**
 *******************************************************************************
 * @file  led.h
 * @brief 状态指示灯接口（PC6，高电平有效）
 *******************************************************************************
 * Copyright (C) 2026, all rights reserved.
 *
 * This software component is licensed under BSD 3-Clause license.
 *******************************************************************************
 */
#ifndef __LED_H__
#define __LED_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  初始化状态指示灯 GPIO（PC6 推挽输出，初始熄灭）。
 * @param  无
 * @retval 无
 */
void Led_Init(void);

/**
 * @brief  点亮指示灯。
 * @param  无
 * @retval 无
 */
void Led_On(void);

/**
 * @brief  熄灭指示灯。
 * @param  无
 * @retval 无
 */
void Led_Off(void);

/**
 * @brief  翻转指示灯状态。
 * @param  无
 * @retval 无
 */
void Led_Toggle(void);

/**
 * @brief  非阻塞闪灯任务：按 u32PeriodMs 在半周期处翻转。
 *         需在主循环中频繁调用。
 * @param  [in] u32PeriodMs 闪烁周期（ms），0 表示常亮（表示进入服务模式）
 * @retval 无
 */
void Led_BlinkTask(uint32_t u32PeriodMs);

#ifdef __cplusplus
}
#endif

#endif /* __LED_H__ */
