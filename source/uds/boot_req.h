/**
 *******************************************************************************
 * @file  boot_req.h
 * @brief 应用侧诊断轮询与"进入 BOOT"请求处理
 *
 *   背景：
 *     App 运行期间要实现完整 UDS 诊断服务，就需要一个 ISO-TP 接收侧来把
 *     CAN 帧拼成完整请求。本模块就是那个**唯一**的 Rx FIFO 消费者，
 *     拼好后交给 app_uds.c 的协议层处理。
 *
 *   交棒门禁（两道，缺一不可）：
 *     1. 必须已通过 0x27 解锁（密钥正确）；
 *     2. 必须发送 0xFF00 固件更新请求 DID。
 *   未解锁时对 0xFF00 回 NRC 0x33（SecurityAccessDenied），**不跳转**。
 *
 *   跨复位介质：TAMP 备份寄存器 TAMP->BKP0R（VBAT 域，系统复位与掉电后
 *   仍保持）。不用 Flash / 选项字节的原因：
 *     - 无需擦除 Flash 页（App 自身就在被擦写的区域内）；
 *     - 不涉及选项字节，无锁死芯片风险。
 *
 *   注意（STM32G4）：备份寄存器挂在 **TAMP** 外设（不是 RTC），
 *   其访问时钟由 RCC_APB1ENR1.RTCAPBEN 提供（本芯片无独立 TAMPEN 位），
 *   写保护由 PWR_CR1.DBP 控制。
 *******************************************************************************
 * Copyright (C) 2026, all rights reserved.
 *
 * This software component is licensed under BSD 3-Clause license.
 *******************************************************************************/
#ifndef __BOOT_REQ_H__
#define __BOOT_REQ_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  初始化诊断模块（含 UDS 服务端、参数区、DTC 表）。
 *         必须在 Mcan_Init() 之后调用。
 * @param  无
 * @retval 无
 */
void BootReq_Init(void);

/**
 * @brief  轮询 CAN，把完整请求交给 UDS 协议层处理。
 *
 *         实现 ISO-TP 接收侧（SF / FF + CF），并按 CAN ID 区分
 *         物理寻址（0x7E0）与功能寻址（0x7DF），把寻址类型透传给协议层。
 *
 * @param  [in] u32Tick 当前毫秒计数（喂给 S3 会话超时与种子生成）
 * @retval uint8_t 1 = 已收到**并已鉴权**的固件更新请求（主循环应复位）
 */
uint8_t BootReq_Poll(uint32_t u32Tick);

/**
 * @brief  写入跨复位"进入 BOOT"标志（TAMP->BKP0R，VBAT 域）。
 * @param  无
 * @retval 无
 */
void BootReq_Set(void);

/**
 * @brief  读取跨复位"进入 BOOT"标志。
 * @param  无
 * @retval uint8_t 1 = 已置位，0 = 未置位
 */
uint8_t BootReq_IsSet(void);

/**
 * @brief  清除跨复位"进入 BOOT"标志。
 * @param  无
 * @retval 无
 */
void BootReq_Clear(void);

/**
 * @brief  复位 MCU 使其进入 BOOT（不返回）。
 *         先置跨复位标志，Bootloader 读到后才驻留等待刷写，
 *         而不是在等待窗口结束后又跳回 App（那将形成复位循环）。
 * @param  无
 * @retval 无
 */
void BootReq_Reboot(void);

/**
 * @brief  复位 MCU 使其回到 App（不返回）。用于 0x11 ECUReset。
 *         会先清除"进入 BOOT"标志，复位后 Bootloader 立即跳回 App。
 * @param  无
 * @retval 无
 */
void BootReq_RebootToApp(void);

#ifdef __cplusplus
}
#endif

#endif /* __BOOT_REQ_H__ */
