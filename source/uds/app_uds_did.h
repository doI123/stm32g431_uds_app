/**
 *******************************************************************************
 * @file  app_uds_did.h
 * @brief DID 业务表（0x22 读 / 0x2E 写）
 *
 *   设计：协议层（app_uds.c）只做"查表 + 会话检查 + 解锁检查 + 长度检查"，
 *   具体数据怎么来、怎么校验，全部由本模块的回调负责。新增一条 DID 只需：
 *     1) 在 app_uds_cfg.h 定义编号（可选）；
 *     2) 写一个静态读/写函数；
 *     3) 在 app_uds_did.c 的 s_astcDidTable[] 里加一行。
 *
 *   可读写参数区（0xF900 ~ 0xF930）单独由 app_uds_param.c 管理，
 *   不在本表中逐条列出 —— 协议层会先判断是否属于参数区
 *   （见 AppUdsDid_IsParamRange）。
 *******************************************************************************
 * Copyright (C) 2026, all rights reserved.
 *
 * This software component is licensed under BSD 3-Clause license.
 *******************************************************************************/
#ifndef __APP_UDS_DID_H__
#define __APP_UDS_DID_H__

#include <stdint.h>
#include "app_uds_cfg.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- 安全等级要求 ---- */
#define APP_SEC_NONE                (0U)   /* 无要求（只读标识类）        */
#define APP_SEC_UNLOCKED            (1U)   /* 读/写均需已通过 0x27        */
#define APP_SEC_WRITE_UNLOCKED      (2U)   /* 读不需要、写需要已通过 0x27 */

/* 读回调：把数据写入 pu8Out，返回 0 = 成功；非 0 = 直接作为 NRC 返回 */
typedef uint8_t (*AppUdsDidReadFn)(uint8_t *pu8Out, uint16_t u16MaxLen,
                                   uint16_t *pu16Len);
/* 写回调：校验并落地，返回 0 = 成功；非 0 = 直接作为 NRC 返回 */
typedef uint8_t (*AppUdsDidWriteFn)(const uint8_t *pu8In, uint16_t u16Len);

typedef struct {
    uint16_t         u16Did;      /* DID 编号                          */
    uint8_t          u8SessMask;  /* 允许的会话位掩码（APP_SESS_BIT_*） */
    uint8_t          u8Sec;       /* APP_SEC_*                         */
    uint16_t         u16Len;      /* 固定数据长度；0 = 变长            */
    AppUdsDidReadFn  pfRead;      /* NULL = 只写                       */
    AppUdsDidWriteFn pfWrite;     /* NULL = 只读                       */
} app_uds_did_t;

/**
 * @brief  按 DID 查表（不含参数区 0xF900~0xF930）。
 * @param  [in] u16Did DID 编号
 * @retval const app_uds_did_t* 命中返回表项；未命中返回 NULL
 */
const app_uds_did_t *AppUdsDid_Find(uint16_t u16Did);

/**
 * @brief  判断 DID 是否属于可读写参数区（0xF900 ~ 0xF930）。
 * @param  [in] u16Did DID 编号
 * @retval uint8_t 1 = 属于参数区
 */
uint8_t AppUdsDid_IsParamRange(uint16_t u16Did);

/* ================= FOC / 应用层接入点（弱函数，可被覆盖） =================
 * 默认返回 0，接入 FOC 时在应用层重新实现同名函数即可，无需改本模块 ——
 * 这样"UDS 诊断"与"电机控制"彻底解耦。
 */
uint16_t AppUdsData_GetMotorSpeedRpm(void);
int8_t   AppUdsData_GetMotorTemp(void);
uint16_t AppUdsData_GetBusVoltageMv(void);
void     AppUdsData_GetPhaseCurrentMa(int16_t *pi16Iabc);
uint32_t AppUdsData_GetMotorStateWord(void);
uint32_t AppUdsData_GetMotorFaultWord(void);
uint16_t AppUdsData_GetMotorAngleDeciDeg(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_UDS_DID_H__ */
