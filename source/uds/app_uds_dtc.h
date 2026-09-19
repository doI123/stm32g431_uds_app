/**
 *******************************************************************************
 * @file  app_uds_dtc.h
 * @brief DTC（故障码）管理：存储、状态位、0x19 / 0x14 / 0x85 服务
 *
 *   存储介质：纯 RAM（本上电周期有效）。不占 TAMP 备份寄存器、不写 Flash ——
 *   刷写后 DTC 清零本就是正确语义。
 *
 *   应用层用法（FOC 侧）：
 *     AppUdsDtc_Init();                     // 上电调用一次
 *     AppUdsDtc_Report(dtc, 1, ext, 4);     // 检测到故障 -> 置位
 *     AppUdsDtc_Report(dtc, 0, NULL, 0);    // 故障消失   -> 清 testFailed
 *     AppUdsDtc_OperationCycle();           // 每个操作循环开始调用一次（老化）
 *******************************************************************************
 * Copyright (C) 2026, all rights reserved.
 *
 * This software component is licensed under BSD 3-Clause license.
 *******************************************************************************/
#ifndef __APP_UDS_DTC_H__
#define __APP_UDS_DTC_H__

#include <stdint.h>
#include "app_uds_cfg.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- DTC 状态字节（ISO 14229-1:2020 Table A.2） ---- */
#define APP_DTC_STATUS_TEST_FAILED              (1U << 0)
#define APP_DTC_STATUS_TEST_FAILED_THIS_CYCLE   (1U << 1)
#define APP_DTC_STATUS_PENDING                  (1U << 2)
#define APP_DTC_STATUS_CONFIRMED                (1U << 3)
#define APP_DTC_STATUS_NOT_COMPLETED_SINCE_CLR  (1U << 4)
#define APP_DTC_STATUS_FAILED_SINCE_CLR         (1U << 5)
#define APP_DTC_STATUS_NOT_COMPLETED_THIS_CYCLE (1U << 6)
#define APP_DTC_STATUS_WARNING_INDICATOR        (1U << 7)

/* 支持的状态位掩码（0x19 响应里的 DTCStatusAvailabilityMask） */
#define APP_DTC_AVAIL_MASK                      (0xFFU)

/* 老化阈值：连续 N 个操作循环无故障则清 confirmed 位 */
#define APP_DTC_AGING_LIMIT                     (20U)

/* 扩展数据长度 */
#define APP_DTC_EXT_DATA_LEN                    (4U)

/* 常用故障码（示例，FOC 侧按需扩充；3 字节 ISO 15031-6 编码） */
#define APP_DTC_OVERVOLTAGE                     (0x000101UL)  /* P0101 -> 过压   */
#define APP_DTC_UNDERVOLTAGE                    (0x000102UL)  /* P0102 -> 欠压   */
#define APP_DTC_OVERCURRENT                     (0x000300UL)  /* P0300 -> 过流   */
#define APP_DTC_OVERTEMP                        (0x000121UL)  /* U0121 -> 过温   */
#define APP_DTC_ENCODER_FAULT                   (0x000201UL)  /* P0201 -> 编码器 */
#define APP_DTC_PHASE_LOSS                      (0x000202UL)  /* P0202 -> 缺相   */

/**
 * @brief  初始化 DTC 表（清空）。上电调用一次。
 */
void AppUdsDtc_Init(void);

/**
 * @brief  应用层上报/清除一条故障。
 * @param  [in] u32Dtc     3 字节 DTC（ISO 15031-6 编码，如 P0101 -> 0x000101）
 * @param  [in] u8Failed   1 = 故障存在；0 = 故障消失
 * @param  [in] pu8ExtData 扩展数据（可为 NULL）；长度超过 APP_DTC_EXT_DATA_LEN 截断
 * @param  [in] u16ExtLen  扩展数据长度
 */
void AppUdsDtc_Report(uint32_t u32Dtc, uint8_t u8Failed,
                      const uint8_t *pu8ExtData, uint16_t u16ExtLen);

/**
 * @brief  标记"新操作循环开始"（老化计数 +1，更新 this-cycle 位）。
 */
void AppUdsDtc_OperationCycle(void);

/* ---- 供协议层调用（0x19 / 0x14 / 0x85） ---- */

/**
 * @brief  处理 0x19 ReadDTCInformation。
 * @param  [in]  pu8Req    请求字节（首字节 = 0x19）
 * @param  [in]  u16Len    请求长度
 * @param  [out] pu8Out    响应数据缓冲（**不含** SID 与子功能，从数据域开始）
 * @param  [in]  u16MaxLen 缓冲长度
 * @param  [out] pu16OutLen 实际写入长度
 * @retval uint8_t 0 = 成功；非 0 = NRC
 */
uint8_t AppUdsDtc_HandleRead(const uint8_t *pu8Req, uint16_t u16Len,
                             uint8_t *pu8Out, uint16_t u16MaxLen,
                             uint16_t *pu16OutLen);

/**
 * @brief  处理 0x14 ClearDiagnosticInformation。
 * @param  [in] u32Group 3 字节组号；0xFFFFFF = 全部
 * @retval uint8_t 0 = 成功；非 0 = NRC
 */
uint8_t AppUdsDtc_HandleClear(uint32_t u32Group);

/**
 * @brief  处理 0x85 ControlDTCSetting。
 * @param  [in] u8Type 0x01 = ON；0x02 = OFF
 * @retval uint8_t 0 = 成功；非 0 = NRC
 */
uint8_t AppUdsDtc_HandleSetting(uint8_t u8Type);

/**
 * @brief  查询 DTC 记录是否使能（0x85 OFF 时 Report 只更新计数不置位）。
 * @retval uint8_t 1 = 使能
 */
uint8_t AppUdsDtc_IsEnabled(void);

/**
 * @brief  查询当前已记录的 DTC 条数（诊断/调试用）。
 * @retval uint8_t 条数
 */
uint8_t AppUdsDtc_GetCount(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_UDS_DTC_H__ */
