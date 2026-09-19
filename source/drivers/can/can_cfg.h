/**
 *******************************************************************************
 * @file  can_cfg.h
 * @brief CAN 波特率配置的持久化（App 侧）
 *
 *   背景：
 *     App 支持运行时切换 CAN 波特率（UDS 0x2E 写 DID 0xF1A0）。
 *     切换结果必须跨复位保持，否则一复位就回到默认速率，
 *     上位机若已按新速率连接就会失联 —— 而此时 App 又"看起来正常在跑"，
 *     现场极难判断。故用 VBAT 域的备份寄存器保存。
 *
 *   介质：TAMP->BKP1R（VBAT 域，系统复位与掉电后仍保持）。
 *     与 BOOT 的"进入 BOOT"标志（BKP0R）分开使用，互不干扰。
 *
 *     为什么不用 Flash：改波特率是高频配置动作，写 Flash 需整页擦除，
 *     既慢又有磨损，且 App 自身就在被擦写的区域内，风险更高。
 *
 *   未配置的表示：0xFFFFFFFF。不能用 0 —— 档位码 0x00 是合法值（125 k），
 *     用 0 当"未配置"会把默认状态误判成"已配置为 125 k"。
 *******************************************************************************
 * Copyright (C) 2026, all rights reserved.
 *
 * This software component is licensed under BSD 3-Clause license.
 *******************************************************************************
 */
#ifndef __CAN_CFG_H__
#define __CAN_CFG_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  初始化：把持久化的档位码应用到 CAN 驱动（只记录，不改硬件）。
 *
 *   设计上**不**在这里重新初始化 FDCAN：本函数在 main() 中早于 Mcan_Init()
 *   被调用，顺序正好 —— 先把配置读进来，随后 Mcan_Init() 就会用正确速率
 *   初始化。这样避免了"先按默认速率起、再切一次"的无谓抖动。
 *
 *   注意：BOOT 固定 800 k，App 默认也是 800 k（CAN_DEFAULT_BAUDRATE），
 *   因此未配置过的板子从 BOOT 跳到 App 时速率不变，不会失联。
 *
 * @param  无
 * @retval 无
 */
void CanCfg_Init(void);

/**
 * @brief  把档位码写入备份寄存器并应用到 CAN 驱动（只记录，不改硬件）。
 *
 *   真正的硬件切换由调用方在回完应答后调用 Mcan_ReInit() 完成。
 *
 * @param  [in] u8Code 档位码（MCAN_BAUD_CODE_*）；0xFF 表示恢复默认
 * @retval int32_t 0 = 成功；-1 = 档位码不受支持（既有配置与硬件均不变）
 */
int32_t CanCfg_SetBaudCode(uint8_t u8Code);

/**
 * @brief  查询当前生效的档位码（优先备份寄存器，未配置时回落默认）。
 * @param  无
 * @retval uint8_t 档位码（MCAN_BAUD_CODE_*）
 */
uint8_t CanCfg_GetBaudCode(void);

/**
 * @brief  查询当前生效的波特率（速率值，如 800000）。
 * @param  无
 * @retval uint32_t 波特率
 */
uint32_t CanCfg_GetBaudrate(void);

/**
 * @brief  查询备份寄存器里是否存有"用户显式配置过"的档位码。
 * @param  无
 * @retval uint8_t 1 = 曾配置；0 = 从未配置（正在用默认值）
 */
uint8_t CanCfg_IsConfigured(void);

#ifdef __cplusplus
}
#endif

#endif /* __CAN_CFG_H__ */
