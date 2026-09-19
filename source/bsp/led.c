/**
 *******************************************************************************
 * @file  led.c
 * @brief 状态指示灯实现（PC6 推挽输出，高电平有效）
 *
 *   指示灯语义（Bootloader 侧）：
 *     等待窗口内快速闪烁 -> 正在等待上位机连接
 *     常亮               -> 已进入服务模式（正在 / 已准备好刷写）
 *******************************************************************************
 * Copyright (C) 2026, all rights reserved.
 *
 * This software component is licensed under BSD 3-Clause license.
 *******************************************************************************
 */

/*******************************************************************************
 * 头文件
 ******************************************************************************/
#include "led.h"
#include "board.h"

#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_gpio.h"
#include "clock.h"

/*******************************************************************************
 * 宏定义
 ******************************************************************************/
#if (LED_ACTIVE_HIGH != 0)
#define LED_WRITE(on)   do { if (on) { LL_GPIO_SetOutputPin(LED_GPIO_PORT, LED_GPIO_PIN); } \
                             else   { LL_GPIO_ResetOutputPin(LED_GPIO_PORT, LED_GPIO_PIN); } } while (0)
#else
#define LED_WRITE(on)   do { if (on) { LL_GPIO_ResetOutputPin(LED_GPIO_PORT, LED_GPIO_PIN); } \
                             else   { LL_GPIO_SetOutputPin(LED_GPIO_PORT, LED_GPIO_PIN); } } while (0)
#endif

/*******************************************************************************
 * 内部变量
 ******************************************************************************/
static uint32_t m_u32LastToggleTick = 0U;
static uint8_t  m_u8State           = 0U;

/*******************************************************************************
 * 函数实现
 ******************************************************************************/

/**
 * @brief  初始化状态指示灯 GPIO（PC6 推挽输出，初始熄灭）。
 * @param  无
 * @retval 无
 */
void Led_Init(void)
{
    /* GPIOC 时钟（LED_GPIO_PORT 固定为 GPIOC，见 board.h） */
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);

    /* 推挽输出、低速、无上下拉 —— 与改造前的 HAL 配置逐项等价 */
    LL_GPIO_SetPinMode(LED_GPIO_PORT, LED_GPIO_PIN, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinOutputType(LED_GPIO_PORT, LED_GPIO_PIN, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinSpeed(LED_GPIO_PORT, LED_GPIO_PIN, LL_GPIO_SPEED_FREQ_LOW);
    LL_GPIO_SetPinPull(LED_GPIO_PORT, LED_GPIO_PIN, LL_GPIO_PULL_NO);

    Led_Off();
}

/**
 * @brief  点亮指示灯。
 * @param  无
 * @retval 无
 */
void Led_On(void)
{
    LED_WRITE(1);
    m_u8State = 1U;
}

/**
 * @brief  熄灭指示灯。
 * @param  无
 * @retval 无
 */
void Led_Off(void)
{
    LED_WRITE(0);
    m_u8State = 0U;
}

/**
 * @brief  翻转指示灯状态。
 * @param  无
 * @retval 无
 */
void Led_Toggle(void)
{
    if (m_u8State != 0U) {
        Led_Off();
    } else {
        Led_On();
    }
}

/**
 * @brief  非阻塞闪灯任务。
 * @param  [in] u32PeriodMs 闪烁周期（ms）；传 0 表示常亮。
 * @retval 无
 */
void Led_BlinkTask(uint32_t u32PeriodMs)
{
    uint32_t u32Now = SysTick_GetTick();

    /* 周期为 0：常亮，表示已进入服务模式 */
    if (u32PeriodMs == 0U) {
        if (m_u8State == 0U) {
            Led_On();
        }
        return;
    }

    /* 半周期翻转一次，实现整体周期 = u32PeriodMs 的闪烁 */
    if ((u32Now - m_u32LastToggleTick) >= (u32PeriodMs / 2U)) {
        m_u32LastToggleTick = u32Now;
        Led_Toggle();
    }
}

/*******************************************************************************
 * 文件结束
 ******************************************************************************/
