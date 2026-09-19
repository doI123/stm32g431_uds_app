/**
 *******************************************************************************
 * @file  clock.c
 * @brief 时钟初始化与毫秒时基（8 MHz HSE -> 170 MHz 系统主频）
 *
 *   时钟树：
 *     PLLM = /2  -> 4 MHz
 *     VCO  = 4 MHz * 85 = 340 MHz
 *     PLLR = /2  -> 170 MHz  = CK_SYS / CK_AHB
 *     PLLQ = /8  -> 42.5 MHz （保留配置但**不再**用于 FDCAN）
 *
 *   ⚠️ FDCAN 内核时钟源已改为 **HSE（8 MHz）**，见 mcan.c 的
 *      Mcan_ClockAndGpioInit()。原因：8 MHz 能整除 125k/250k/500k/800k/1M
 *      全部目标速率（波特率误差为 0），而 42.5 MHz 无法整除 800 kbps。
 *      这里仍然配置 PLLQ，只是为了不改动已验证通过的 PLL 配置
 *      （改动 PLLN/PLLQ 会影响 VCO 与主频）。
 *
 *   说明：本文件利用 HAL 的 RCC 接口完成配置，
 *         不修改固件包中的 system_stm32g4xx.c。
 *******************************************************************************
 * Copyright (C) 2026, all rights reserved.
 *
 * This software component is licensed under BSD 3-Clause license.
 *******************************************************************************
 */

/*******************************************************************************
 * 头文件
 ******************************************************************************/
#include "clock.h"
#include "board.h"

/*******************************************************************************
 * 宏定义
 ******************************************************************************/
/* PLL 参数：VCO = 8 MHz / 2 * 85 = 340 MHz，CK_SYS = VCO / 2 = 170 MHz */
#define BOARD_PLL_M                 (RCC_PLLM_DIV2)
#define BOARD_PLL_N                 (85U)
#define BOARD_PLL_P                 (RCC_PLLP_DIV2)
#define BOARD_PLL_Q                 (RCC_PLLQ_DIV8)     /* 42.5 MHz */
#define BOARD_PLL_R                 (RCC_PLLR_DIV2)     /* 170 MHz */

/* 170 MHz 下的 Flash 等待周期 */
#define BOARD_FLASH_LATENCY         (FLASH_LATENCY_4)

/*******************************************************************************
 * 内部变量
 ******************************************************************************/
/* 毫秒计数（SysTick 中断中自增，主循环中读取） */
static volatile uint32_t m_u32Tick = 0U;

/*******************************************************************************
 * 函数实现
 ******************************************************************************/

/**
 * @brief  HAL MSP 初始化回调（由 HAL_Init() 调用）。
 *
 *   HAL 库里的 HAL_MspInit() 是 __weak 空函数，必须在用户代码中实现。
 *   ST 的所有官方例程都在这里使能 PWR 与 SYSCFG 时钟。
 *
 *   为什么 PWR 时钟必不可少：
 *     HAL_PWREx_ControlVoltageScaling() 会写 PWR 寄存器（CR1/CR5）并等待
 *     VOSF 标志清零。若 PWR 时钟未使能，写入被静默丢弃、VOSF 永远置位，
 *     该函数会超时返回 HAL_TIMEOUT，导致 Board_ClockInit() 提前返回 ——
 *     主频停留在 HSI 16 MHz，CAN 位时序与毫秒时基全部错乱。
 *
 * @param  无
 * @retval 无
 */
void HAL_MspInit(void)
{
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();
}

/**
 * @brief  启动 SysTick 毫秒时基（中断方式）。
 * @param  [in] u32FreqHz 时基频率（Hz），1000 表示 1 ms 一次中断。
 * @retval 无
 */
void SysTick_Init(uint32_t u32FreqHz)
{
    m_u32Tick = 0U;

    /* 重装载值 = 系统主频 / 时基频率 - 1 */
    SysTick->LOAD = (SystemCoreClock / u32FreqHz) - 1UL;
    SysTick->VAL  = 0UL;
    /* 时钟源 = HCLK，使能中断，使能计数 */
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
                    SysTick_CTRL_TICKINT_Msk   |
                    SysTick_CTRL_ENABLE_Msk;
}

/**
 * @brief  获取当前毫秒计数。
 * @param  无
 * @retval uint32_t 毫秒计数
 */
uint32_t SysTick_GetTick(void)
{
    return m_u32Tick;
}

/**
 * @brief  毫秒计数自增，供 SysTick_Handler 调用。
 * @param  无
 * @retval 无
 */
void SysTick_IncTick(void)
{
    m_u32Tick++;
}

/**
 * @brief  阻塞延时。
 * @param  [in] u32Ms 延时毫秒数
 * @retval 无
 */
void SysTick_Delay(uint32_t u32Ms)
{
    uint32_t u32Start = m_u32Tick;

    /* 使用无符号减法，天然处理计数回绕 */
    while ((m_u32Tick - u32Start) < u32Ms) {
        ;
    }
}

/**
 * @brief  初始化系统时钟：8 MHz HSE -> PLL -> 170 MHz。
 * @param  无
 * @retval int32_t 0 = 时钟配置成功；-1 = 失败（已退回到安全频率）
 */
int32_t Board_ClockInit(void)
{
    RCC_OscInitTypeDef       stcOsc = {0};
    RCC_ClkInitTypeDef       stcClk = {0};

    /* 1. 配置调压器为 Boost 模式。
     *    主频超过 150 MHz 时必须使用 Scale1 Boost，否则内核无法稳定运行。
     *
     *    前提：PWR 时钟必须已使能（见本文件顶部的 HAL_MspInit()），
     *          否则本次写入会被静默丢弃并直接返回 HAL_TIMEOUT。 */
    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST)
        != HAL_OK) {
        /* 调压器无法进入 Boost 档：此时不能跑 170 MHz，
           退回 Range-1 常规档（上限 150 MHz）并降频运行。 */
        if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1)
            != HAL_OK) {
            return -1;      /* 调压器完全不可控：时钟无法可信配置 */
        }
    }

    /* 2. 配置 HSE 与 PLL */
    stcOsc.OscillatorType      = RCC_OSCILLATORTYPE_HSE;
    stcOsc.HSEState            = RCC_HSE_ON;
    stcOsc.HSIState            = RCC_HSI_ON;
    stcOsc.PLL.PLLState        = RCC_PLL_ON;
    stcOsc.PLL.PLLSource       = RCC_PLLSOURCE_HSE;
    stcOsc.PLL.PLLM            = BOARD_PLL_M;
    stcOsc.PLL.PLLN            = BOARD_PLL_N;
    stcOsc.PLL.PLLP            = BOARD_PLL_P;
    stcOsc.PLL.PLLQ            = BOARD_PLL_Q;
    stcOsc.PLL.PLLR            = BOARD_PLL_R;

    if (HAL_RCC_OscConfig(&stcOsc) != HAL_OK) {
        /* HSE 未起振或 PLL 未锁定：保持当前时钟（HSI 16 MHz）运行并返回失败，
           不在此处死等，以便主循环仍能闪灯报错、故障可在现场观察到。 */
        SystemCoreClockUpdate();
        return -1;
    }

    /* 3. 切换系统时钟源到 PLL，并设置总线分频 */
    stcClk.ClockType      = RCC_CLOCKTYPE_HCLK   | RCC_CLOCKTYPE_SYSCLK |
                            RCC_CLOCKTYPE_PCLK1  | RCC_CLOCKTYPE_PCLK2;
    stcClk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    stcClk.AHBCLKDivider  = RCC_SYSCLK_DIV1;    /* 170 MHz */
    stcClk.APB1CLKDivider = RCC_HCLK_DIV1;      /* 170 MHz */
    stcClk.APB2CLKDivider = RCC_HCLK_DIV1;      /* 170 MHz */

    if (HAL_RCC_ClockConfig(&stcClk, BOARD_FLASH_LATENCY) != HAL_OK) {
        SystemCoreClockUpdate();
        return -1;
    }

    /* 4. HAL_RCC_ClockConfig 会更新 SystemCoreClock；
     *    这里再显式刷新一次，确保时基/延时计算使用正确频率。 */
    SystemCoreClockUpdate();

    return 0;
}

/*******************************************************************************
 * 文件结束
 ******************************************************************************/
