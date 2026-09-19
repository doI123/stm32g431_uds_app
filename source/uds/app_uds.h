/**
 *******************************************************************************
 * @file  app_uds.h
 * @brief 应用层 UDS 协议服务端接口（完整版）
 *
 *   已实现服务：
 *     0x10 诊断会话控制          0x11 ECU 复位
 *     0x14 清除诊断信息          0x19 读 DTC 信息
 *     0x22 按标识符读数据        0x27 安全访问
 *     0x28 通信控制              0x2E 按标识符写数据
 *     0x31 例程控制              0x3E 会话保持
 *     0x85 DTC 设置
 *
 *   传输层复用 boot_req.c 的接收侧 + 本模块的阻塞式 ISO-TP 发送侧，
 *   全系统只有 boot_req.c 一个 Rx FIFO 消费者。
 *
 *   兼容性铁律（不可改）：
 *     0x27 key = seed ^ 0x5A5A5A5A，4 字节大端
 *     0x22 0xFF00 固件更新请求（需先解锁），收到后置跨复位标志并复位
 *******************************************************************************
 * Copyright (C) 2026, all rights reserved.
 *
 * This software component is licensed under BSD 3-Clause license.
 *******************************************************************************/
#ifndef __APP_UDS_H__
#define __APP_UDS_H__

#include <stdint.h>
#include "app_uds_cfg.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- 服务标识符 ---- */
#define APP_SID_DSC                 (0x10U)   /* DiagnosticSessionControl   */
#define APP_SID_ER                  (0x11U)   /* ECUReset                   */
#define APP_SID_CDI                 (0x14U)   /* ClearDiagnosticInformation */
#define APP_SID_RDTC                (0x19U)   /* ReadDTCInformation         */
#define APP_SID_RDABI               (0x22U)   /* ReadDataByIdentifier       */
#define APP_SID_RMBA                (0x23U)   /* ReadMemoryByAddress        */
#define APP_SID_SA                  (0x27U)   /* SecurityAccess             */
#define APP_SID_CC                  (0x28U)   /* CommunicationControl       */
#define APP_SID_WDABI               (0x2EU)   /* WriteDataByIdentifier      */
#define APP_SID_RC                  (0x31U)   /* RoutineControl             */
#define APP_SID_TP                  (0x3EU)   /* TesterPresent              */
#define APP_SID_CDTCS               (0x85U)   /* ControlDTCSetting          */

/* ---- 否定响应码（ISO 14229-1:2020 附录 A.1） ---- */
#define APP_NRC_GR                  (0x10U)   /* GeneralReject                   */
#define APP_NRC_SNS                 (0x11U)   /* ServiceNotSupported             */
#define APP_NRC_SFNS                (0x12U)   /* SubFunctionNotSupported         */
#define APP_NRC_IMLOIF              (0x13U)   /* IncorrectMessageLength          */
#define APP_NRC_RTL                 (0x14U)   /* ResponseTooLong                 */
#define APP_NRC_CNC                 (0x22U)   /* ConditionsNotCorrect            */
#define APP_NRC_RSE                 (0x24U)   /* RequestSequenceError            */
#define APP_NRC_ROOR                (0x31U)   /* RequestOutOfRange               */
#define APP_NRC_SAD                 (0x33U)   /* SecurityAccessDenied            */
#define APP_NRC_IK                  (0x35U)   /* InvalidKey                      */
#define APP_NRC_ENOA                (0x36U)   /* ExceedNumberOfAttempts          */
#define APP_NRC_RTDNE               (0x37U)   /* RequiredTimeDelayNotExpired     */
#define APP_NRC_SFNSIAS             (0x7EU)   /* SubFunctionNotSupportedInActiveSession */
#define APP_NRC_SNSIAS              (0x7FU)   /* ServiceNotSupportedInActiveSession     */

/* ---- 寻址类型 ---- */
#define APP_ADDR_PHYSICAL           (0U)
#define APP_ADDR_FUNCTIONAL         (1U)

/* ------------------------------ 生命周期 ------------------------------ */

/**
 * @brief  初始化 UDS 服务端（会话/安全状态、DID 参数区、DTC 表）。
 *         须在 Mcan_Init() 之后调用（由 BootReq_Init() 代调）。
 * @param  无
 * @retval 无
 */
void AppUds_Init(void);

/**
 * @brief  周期任务：S3 会话超时回落、安全访问超限延时到期。
 *         由 boot_req.c 在每轮轮询中调用。
 * @param  [in] u32Tick 当前毫秒计数
 * @retval 无
 */
void AppUds_Task(uint32_t u32Tick);

/**
 * @brief  处理一条完整 UDS 请求（含会话/安全/长度检查与 NRC 生成）。
 * @param  [in] pu8Req    请求字节（首字节 = SID）
 * @param  [in] u16Len    请求长度
 * @param  [in] u8AddrType APP_ADDR_PHYSICAL / APP_ADDR_FUNCTIONAL
 * @retval 无
 */
void AppUds_HandleRequest(const uint8_t *pu8Req, uint16_t u16Len,
                          uint8_t u8AddrType);

/**
 * @brief  取出"等待流控期间暂存的一帧"。
 *
 *   等流控帧时收到的非流控帧会被暂存在本模块，由 boot_req.c 的轮询优先
 *   消费 —— 否则上位机紧接着发来的下一请求会被静默丢弃（表现为响应超时）。
 *
 * @param  [out] pu32Id  CAN ID（可为 NULL）
 * @param  [out] pu8Data 数据缓冲（>= 8 字节，可为 NULL）
 * @param  [out] pu8Len  长度（可为 NULL）
 * @retval uint8_t 1 = 有暂存帧；0 = 无
 */
uint8_t AppUds_PopStashFrame(uint32_t *pu32Id, uint8_t *pu8Data, uint8_t *pu8Len);

/* ------------------------------ 状态查询 ------------------------------ */

/** @brief 查询当前诊断会话（0x01/0x02/0x03）。 */
uint8_t AppUds_GetSession(void);

/** @brief 查询是否已通过 0x27 解锁。 */
uint8_t AppUds_IsUnlocked(void);

/** @brief 查询是否已接受（且已鉴权的）固件更新请求。 */
uint8_t AppUds_BootRequested(void);

/** @brief 清除已接受的固件更新请求（一次性语义）。 */
void AppUds_ClearBootRequest(void);

/** @brief 查询是否有待执行的 0x11 复位请求。 */
uint8_t AppUds_ResetPending(void);

/** @brief 查询是否有待执行的 CAN 波特率切换（须在应答发完后处理）。 */
uint8_t AppUds_BaudChangePending(void);

/** @brief 置"待切换波特率"标志（由 DID 写回调调用）。 */
void AppUds_RequestBaudSwitch(void);

/** @brief 清除待执行的波特率切换标志（主循环切换完成后调用）。 */
void AppUds_ClearBaudChange(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_UDS_H__ */
