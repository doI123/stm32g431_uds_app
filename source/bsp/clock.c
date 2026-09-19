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
 *   说明：本文件使用 **LL 库**（stm32g4xx_ll_rcc.h / ll_pwr.h / ll_system.h）
 *         完成配置，不修改固件包中的 system_stm32g4xx.c。
 *         自 LL 化改造起，本工程不再依赖 HAL。
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

#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_pwr.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_system.h"

/*******************************************************************************
 * 宏定义
 ******************************************************************************/
/* PLL 参数：VCO = 8 MHz / 2 * 85 = 340 MHz，CK_SYS = VCO / 2 = 170 MHz
 *
 * ⚠️ 注意 LL 与 HAL 的常量命名不同：LL 是 LL_RCC_PLLM_DIV_2（带下划线），
 *    HAL 是 RCC_PLLM_DIV2。迁移时已同步改名。
 *
 * 说明：PLLP/PLLQ 不再显式配置 —— LL_RCC_PLL_ConfigDomain_SYS() 只写
 *       PLLM/PLLN/PLLR，PLLP/PLLQ 保持复位值。二者本工程均未使用
 *       （FDCAN 内核时钟取自 HSE，ADC 取自 AHB）。 */
#define BOARD_PLL_M                 (LL_RCC_PLLM_DIV_2)
#define BOARD_PLL_N                 (85U)                   /* 乘数，直接给数值 */
#define BOARD_PLL_R                 (LL_RCC_PLLR_DIV_2)     /* 170 MHz */

/* 170 MHz 下的 Flash 等待周期 */
#define BOARD_FLASH_LATENCY         (LL_FLASH_LATENCY_4)

/* 时钟初始化各阶段的超时上限（ms）。
 * HSE 起振最慢，用 board.h 的 HSE_STARTUP_TIMEOUT（100 ms）；
 * 调压器与 PLL 锁定都在微秒级，10 ms 已极宽裕。 */
#define BOARD_HSE_TIMEOUT_MS        (HSE_STARTUP_TIMEOUT)
#define BOARD_PLL_TIMEOUT_MS        (10U)
#define BOARD_VOS_TIMEOUT_MS        (10U)

/*******************************************************************************
 * 内部变量
 ******************************************************************************/
/* 毫秒计数（SysTick 中断中自增，主循环中读取） */
static volatile uint32_t m_u32Tick = 0U;

/*******************************************************************************
 * 函数实现
 ******************************************************************************/

/**
 * @brief  使能 PWR 与 SYSCFG 时钟（时钟配置的最前置条件）。
 *
 *   改造前这一步由 HAL 回调 HAL_MspInit() 完成（HAL_Init() 内部调用）。
 *   LL 化后不再有 HAL_Init()，因此改为显式调用。
 *
 *   为什么 PWR 时钟必不可少：
 *     设置调压器档位会写 PWR->CR1(VOS) 与 PWR->CR5(R1MODE) 并等待
 *     SR2.VOSF 清零。若 PWR 时钟未使能，写入被静默丢弃、VOSF 永远置位，
 *     等待循环必然超时 → Board_ClockInit() 提前返回失败 →
 *     主频停留在 HSI 16 MHz，CAN 位时序与毫秒时基全部错乱。
 *
 * @param  无
 * @retval 无
 */
static void Clock_EnablePwrAndSysCfgClock(void)
{
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
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
    uint32_t u32Start;

    /* 0. 时钟配置期间也需要毫秒级超时。
     *    本函数在 main() 的 SysTick_Init() 之前执行，因此先用**当前**
     *    （复位后为 HSI 16 MHz）主频把时基跑起来；PLL 切换完成后
     *    main() 会再调用 SysTick_Init()，以 170 MHz 重新标定。
     *    这与改造前 HAL_Init() 内部先配好 SysTick 的做法等价。 */
    SysTick_Init(1000U);

    /* 1. 使能 PWR / SYSCFG 时钟 —— 写 PWR 寄存器的前提，必须最先做 */
    Clock_EnablePwrAndSysCfgClock();

    /* 2. 调压器：Range 1（VOS = 01）+ Boost（PWR_CR5.R1MODE = 0）
     *    主频超过 150 MHz 必须工作在 Range 1 boost 档，否则内核无法稳定运行。
     *    与改造前 HAL_PWREx_ControlVoltageScaling(SCALE1_BOOST) 等价。 */
    LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
    LL_PWR_EnableRange1BoostMode();

    u32Start = SysTick_GetTick();
    while (LL_PWR_IsActiveFlag_VOS() != 0U) {       /* 等 VOSF 清零 */
        if ((SysTick_GetTick() - u32Start) >= BOARD_VOS_TIMEOUT_MS) {
            SystemCoreClockUpdate();
            return -1;      /* 调压器不可控：不能跑 170 MHz */
        }
    }

    /* 3. HSE 起振（8 MHz 晶振）。
     *    失败时保持当前时钟（HSI 16 MHz）运行并返回失败，不在此处死等，
     *    以便主循环仍能闪灯报错、故障可在现场观察到。 */
    LL_RCC_HSE_Enable();

    u32Start = SysTick_GetTick();
    while (LL_RCC_HSE_IsReady() == 0U) {
        if ((SysTick_GetTick() - u32Start) >= BOARD_HSE_TIMEOUT_MS) {
            SystemCoreClockUpdate();
            return -1;      /* HSE 未起振 */
        }
    }

    /* 4. PLL：M = /2 → 4 MHz；VCO = 4 × 85 = 340 MHz；R = /2 → 170 MHz */
    LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE,
                                BOARD_PLL_M, BOARD_PLL_N, BOARD_PLL_R);
    LL_RCC_PLL_EnableDomain_SYS();

    u32Start = SysTick_GetTick();
    while (LL_RCC_PLL_IsReady() == 0U) {
        if ((SysTick_GetTick() - u32Start) >= BOARD_PLL_TIMEOUT_MS) {
            SystemCoreClockUpdate();
            return -1;      /* PLL 未锁定 */
        }
    }

    /* 5. 先设总线分频与 Flash 等待周期，再切换 SYSCLK 到 PLL。
     *    ⚠️ 顺序不能反：170 MHz 下 Flash 必须 ≥ 4 个等待周期，
     *       若先提速再补等待周期，取指会读到不确定数据。
     *       这与 HAL_RCC_ClockConfig 的内部顺序一致。 */
    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);     /* AHB  = 170 MHz */
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);      /* APB1 = 170 MHz */
    LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);      /* APB2 = 170 MHz */

    LL_FLASH_SetLatency(BOARD_FLASH_LATENCY);
    while (READ_BIT(FLASH->ACR, FLASH_ACR_LATENCY) != BOARD_FLASH_LATENCY) {
        ;           /* 等新等待周期真正生效 */
    }

    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
    while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL) {
        ;           /* 等系统时钟源切换完成 */
    }

    /* 6. 刷新 CMSIS 的 SystemCoreClock，供时基与延时计算使用 */
    SystemCoreClockUpdate();

    return 0;
}

/*******************************************************************************
 * 文件结束
 ******************************************************************************/
