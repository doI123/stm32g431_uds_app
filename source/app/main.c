/**
 *******************************************************************************
 * @file  main.c
 * @brief STM32G431CBU6 Application —— 完整 UDS 诊断 + 运行控制
 *
 *   本镜像链接在 APP_START_ADDR (0x08006000)，因此 Bootloader 会在
 *   0x34 RequestDownload 中接受它。
 *
 *   主要工作：
 *     1. 把 SCB->VTOR 指向本镜像（厂商 SystemInit() 只在定义了
 *        USER_VECT_TAB_ADDRESS 时才设置 VTOR，而本工程刻意不定义它，
 *        以免把向量表指向 Flash 基址即 Bootloader 的向量表）。
 *     2. 与 Bootloader 相同的 8 MHz HSE -> 170 MHz 时钟初始化。
 *     3. 初始化 CAN 与完整 UDS 诊断服务端。
 *     4. 主循环：
 *          - PC6 状态灯心跳；
 *          - Bus Off 自动恢复；
 *          - 轮询 0x7E0 / 0x7DF 上的诊断请求（0x10 / 0x11 / 0x14 / 0x19 /
 *            0x22 / 0x27 / 0x28 / 0x2E / 0x31 / 0x3E / 0x85）；
 *          - 收到"进入 BOOT"请求后置跨复位标志并复位，由 Bootloader 接手刷写；
 *          - 处理 0x11 ECUReset 与 CAN 波特率切换等"必须延后执行"的动作。
 *******************************************************************************
 * Copyright (C) 2026, all rights reserved.
 *
 * This software component is licensed under BSD 3-Clause license.
 *******************************************************************************/
#include "main.h"
#include "clock.h"
#include "mcan.h"
#include "led.h"
#include "boot_req.h"
#include "app_uds.h"
#include "can_cfg.h"

/*******************************************************************************
 * 内部函数声明
 ******************************************************************************/
static void App_VectorTableInit(void);
static void App_CoreInit(void);

/*******************************************************************************
 * 全局函数
 ******************************************************************************/

/**
 * @brief  SysTick 中断服务函数（1 ms）。
 *
 *   自 LL 化改造后，工程内已无 HAL，因此**不再需要** HAL_IncTick()。
 *   全部超时判断（CAN 发送、Tx 排空、BusOff 恢复限流、UDS S3 会话超时、
 *   帧间隔延时）统一读取 SysTick_GetTick()，只有这一个毫秒时基。
 *
 * @param  无
 * @retval 无
 */
void SysTick_Handler(void)
{
    SysTick_IncTick();
}

/**
 * @brief  内核级初始化（替代改造前的 HAL_Init()）。
 *
 *   HAL_Init() 原先做三件事，现已拆分：
 *     1) 配置中断优先级分组 —— 保留在本函数（下面这一条）；
 *     2) 配置 SysTick 毫秒时基 —— 已由 Board_ClockInit() 内部先按
 *        HSI 16 MHz 起时基（供时钟配置期间的超时使用），并在 main()
 *        中 PLL 稳定后用 170 MHz 重新标定；
 *     3) 回调 HAL_MspInit() 使能 PWR / SYSCFG 时钟 —— 已改为
 *        clock.c 内的 Clock_EnablePwrAndSysCfgClock()。
 *
 * @param  无
 * @retval 无
 */
static void App_CoreInit(void)
{
    /* 中断优先级分组 = 4 位抢占 + 0 位子优先级。
     *
     * 改造前是 HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4)，
     * 它最终就是往 SCB->AIRCR.PRIGROUP 写 3 —— Cortex-M4 未实现
     * 优先级位数大于 4 的位，因此 PRIGROUP=3 即"4 位全部用于抢占"。
     * NVIC_SetPriorityGrouping() 是 CMSIS 内核函数（不是 HAL），
     * 直接用数值 3 表达同一含义，避免再引入 HAL 的命名宏。
     */
    NVIC_SetPriorityGrouping(3U);
}

/**
 * @brief  把向量表指向本镜像。
 *
 *   厂商的 system_stm32g4xx.c 只在定义了 USER_VECT_TAB_ADDRESS 时才写
 *   SCB->VTOR，且写的是 FLASH_BASE（0x08000000），也就是 Bootloader 的
 *   向量表 —— 那是错误的。本工程不定义该宏，改由这里显式设置。
 *
 *   安全性：在使能任何外设中断之前调用即可。复位后 NVIC 与 SysTick 都处于
 *   关闭状态，SystemInit() 也只配置 FPU，因此从复位到本调用之间不会发生
 *   异常，重定向是安全的。
 *
 * @param  无
 * @retval 无
 */
static void App_VectorTableInit(void)
{
    SCB->VTOR = APP_VECT_TAB_ADDR;
    __DSB();
    __ISB();
}

/**
 * @brief  主函数。
 * @param  无
 * @retval int32_t 返回值（正常情况下不会返回）
 */
int main(void)
{
    /* 1. 先把向量表指向本镜像，再做内核级初始化（中断优先级分组）。
     *    注意：本工程已 100% 使用 LL，没有 HAL_Init()。 */
    App_VectorTableInit();
    App_CoreInit();

    /* 2. 系统时钟：8 MHz HSE -> 170 MHz（LL 实现）。
     *    注意：配置调压器前必须已使能 PWR 时钟 —— 这由 clock.c 内的
     *    Clock_EnablePwrAndSysCfgClock() 保证。若缺这一步，PWR 寄存器写入
     *    会被静默丢弃、SR2.VOSF 永远置位，本函数会超时失败，
     *    主频停留在 HSI 16 MHz（此时只有"极快闪"能提示这一故障）。 */
    if (Board_ClockInit() != 0) {
        /* 时钟配置失败：主频仍为 HSI 16 MHz。
         * 此时 CAN 位时序不可信，但状态灯仍可工作，
         * 因此用"极快闪"明确告知故障。
         *
         * 注意：必须先启动时基，否则 SysTick_GetTick() 永远不前进，
         *       Led_BlinkTask() 也就永远不翻转（灯将一直不亮）。 */
        SysTick_Init(1000U);
        Led_Init();
        for (;;) {
            Led_BlinkTask(LED_FAULT_BLINK_MS);
        }
    }

    /* 3. 启动 1 ms 时基（此时 SystemCoreClock 已是 170 MHz） */
    SysTick_Init(1000U);

    /* 4. 状态灯 */
    Led_Init();

    /* 4.1 先读回持久化的 CAN 波特率配置（只记录，不改硬件）。
     *     必须在 Mcan_Init() **之前**调用：这样 Mcan_Init() 会一次
     *     就用正确速率初始化，避免"先按默认速率起、再切一次"的抖动。
     *     未配置过的板子保持默认 800 k —— 与 BOOT 一致，
     *     因此从 BOOT 跳到 App 不会因速率变化而失联。 */
    CanCfg_Init();

    /* 5. CAN 与完整 UDS 诊断服务端（BootReq_Init() 内部会调 AppUds_Init()，
     *    后者初始化会话状态、DID 参数区与 DTC 表） */
    (void)Mcan_Init();
    Mcan_FlushRx();
    BootReq_Init();

    /* 6. 主循环 */
    for (;;) {
        uint32_t u32Tick = SysTick_GetTick();

        /* 6.1 状态灯心跳：表示 App 正在正常运行 */
        Led_BlinkTask(LED_BLINK_PERIOD_MS);

        /* 6.2 Bus Off 恢复检查。
         *
         * M_CAN 一旦 Bus Off 会自动置 CCCR.INIT 并永久停止收发，
         * 必须由软件清除 INIT 才能恢复。若不做这一步，会出现
         * "灯正常闪、程序正常跑，但总线上永远没有任何报文"的哑状态 ——
         * 届时上位机再也无法通过 0xFF00 把它请进 BOOT。
         * 未 Bus Off 时本调用只读一个寄存器，开销可忽略。 */
        (void)Mcan_RecoverIfBusOff();

        /* 6.3 轮询并处理诊断请求（ISO-TP 接收侧 + 全部 UDS 服务）。
         *     一旦收到**已鉴权**的固件更新请求，本函数返回 1。 */
        if (BootReq_Poll(u32Tick) != 0U) {
            /* 置跨复位标志（TAMP 备份寄存器）后复位；
             * Bootloader 复位后读取该标志即驻留等待刷写。本函数不返回。 */
            BootReq_Reboot();
        }

        /* 6.4 0x11 ECUReset：肯定响应已在处理中发完，这里执行复位。
         *     清掉"进入 BOOT"标志 -> 复位后 Bootloader 会正常跳回 App。 */
        if (AppUds_ResetPending() != 0U) {
            BootReq_RebootToApp();
        }

        /* 6.5 波特率切换（0x2E 写 DID 0xF1A0 后置位）。
         *
         *     必须放在这里 —— 也就是 BootReq_Poll() **返回之后**：
         *     0x6E 肯定响应已在轮询中按**旧**速率发完，此刻切换才不会
         *     把应答"发到新速率上去"（那样上位机在旧速率上必然收不到）。
         *
         *     切换会短暂中断总线（固有代价），随后按新速率重新初始化。 */
        if (AppUds_BaudChangePending() != 0U) {
            AppUds_ClearBaudChange();
            (void)Mcan_ReInit();
            Mcan_FlushRx();
        }
    }
}

/*******************************************************************************
 * 文件结束
 ******************************************************************************/
