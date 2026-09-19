/**
 *******************************************************************************
 * @file  app_uds.c
 * @brief 应用层 UDS 协议服务端实现（完整版）
 *
 *   分层：
 *     app_uds.c      本文件：会话 / 安全 / 分发 / NRC / ISO-TP 发送
 *     app_uds_did.c        DID 业务表（0x22 / 0x2E 的数据源）
 *     app_uds_param.c      可读写参数区（0xF900 ~ 0xF930）
 *     app_uds_dtc.c        DTC 表与 0x19 / 0x14 / 0x85
 *     boot_req.c           ISO-TP 接收侧（唯一 Rx FIFO 消费者）
 *
 *   功能寻址（0x7DF）语义（ISO 14229-1:2020 §7.3）：
 *     1. **一律不应答**（肯定与否定响应都不发）—— 否则总线上多节点会同时
 *        应答而互相干扰；
 *     2. 不在白名单内的服务 **静默丢弃**，不回 NRC 0x11；
 *     3. 安全访问 / 写 DID / 例程控制 / 固件更新请求禁止功能寻址。
 *
 *   "抑制肯定响应"位（子功能 bit7）与功能寻址是两回事：
 *     前者只抑制**肯定**响应（物理寻址下使用），否定响应照发；
 *     后者彻底不响应。本文件用两个独立标志区分。
 *******************************************************************************
 * Copyright (C) 2026, all rights reserved.
 *
 * This software component is licensed under BSD 3-Clause license.
 *******************************************************************************/
#include <string.h>
#include "app_uds.h"
#include "app_uds_did.h"
#include "app_uds_dtc.h"
#include "app_uds_param.h"
#include "mcan.h"
#include "board.h"
#include "version.h"
#include "clock.h"

/*******************************************************************************
 * ISO-TP 协议控制信息（PCI）
 ******************************************************************************/
#define APP_ISOTP_PCI_MASK          (0xF0U)
#define APP_ISOTP_TYPE_SF           (0x00U)   /* SingleFrame       */
#define APP_ISOTP_TYPE_FF           (0x10U)   /* FirstFrame        */
#define APP_ISOTP_TYPE_CF           (0x20U)   /* ConsecutiveFrame  */
#define APP_ISOTP_TYPE_FC           (0x30U)   /* FlowControl       */

#define APP_ISOTP_MTU               (8U)      /* 经典 CAN 数据场长度 */
#define APP_ISOTP_SF_MAX            (7U)      /* 单帧最大数据字节   */
#define APP_ISOTP_FF_PAYLOAD        (6U)      /* 首帧携带数据字节   */

/* 流控等待超时（ms）：ISO 15765-2 的 N_Bs 为 1000 ms，取 500 ms 更激进 */
#define APP_ISOTP_FC_TIMEOUT_MS     (500U)

/* 子功能"抑制肯定响应"位（ISO 14229-1:2020 §7.4.5.1） */
#define APP_SUBFUNC_SUPPRESS_POS    (0x80U)
#define APP_SUBFUNC_MASK            (0x7FU)

/*******************************************************************************
 * 内部状态
 ******************************************************************************/
static uint8_t  m_u8Session          = APP_SESSION_DEFAULT;
static uint8_t  m_u8SecurityUnlocked = 0U;
static uint32_t m_u32Seed            = 0U;    /* 本次下发的种子        */
static uint8_t  m_u8SeedRequested    = 0U;    /* 1 = 已下发种子，允许随后发密钥 */
static uint8_t  m_u8SaAttempts       = 0U;    /* 密钥错误次数          */
static uint32_t m_u32SaUnlockAtTick  = 0U;    /* 超限后可再解锁的时刻  */
static uint32_t m_u32SeedTick        = 0U;    /* 生成种子用的时基      */
static uint32_t m_u32LastActivity    = 0U;    /* 最近一次诊断活动      */
static uint8_t  m_u8BootRequested    = 0U;    /* 已接受刷写请求        */
static uint8_t  m_u8BaudChangePending= 0U;    /* 待切换 CAN 波特率     */
static uint8_t  m_u8ResetPending     = 0U;    /* 待执行 ECU 复位       */

/* 寻址与响应抑制（两者语义不同，必须分开） */
static uint8_t  m_u8IsFuncAddr       = 0U;    /* 1 = 功能寻址：完全不响应 */
static uint8_t  m_u8SuppressPos      = 0U;    /* 1 = 抑制肯定响应（物理寻址） */

/* 响应组包缓冲（多帧由 AppUds_SendIsoTp 拆帧） */
static uint8_t  m_au8Resp[APP_UDS_TX_BUF_LEN];

/* "等待流控"期间的意外帧暂存（防止吃掉上位机的下一请求） */
static uint32_t m_au32StashId[APP_UDS_STASH_DEPTH];
static uint8_t  m_au8StashBuf[APP_UDS_STASH_DEPTH][APP_ISOTP_MTU];
static uint8_t  m_au8StashLen[APP_UDS_STASH_DEPTH];
static uint8_t  m_u8StashCnt = 0U;

/*******************************************************************************
 * 内部函数声明
 ******************************************************************************/
static void    AppUds_SendFrame(uint32_t u32Id, const uint8_t *pu8Data, uint8_t u8Len);
static void    AppUds_SendNrc(uint8_t u8Sid, uint8_t u8Nrc);
static void    AppUds_SendResponse(const uint8_t *pu8Data, uint16_t u16Len);
static int32_t AppUds_SendIsoTp(const uint8_t *pu8Data, uint16_t u16Len);
static int32_t AppUds_WaitFlowControl(uint8_t *pu8Bs);
static void    AppUds_StashFrame(uint32_t u32Id, const uint8_t *pu8Data, uint8_t u8Len);

static uint32_t AppUds_ComputeKey(uint32_t u32Seed);
static uint8_t  AppUds_IsSessionAllowed(uint8_t u8SessMask);
static uint8_t  AppUds_IsParamSessionAllowed(void);

static void    AppUds_HandleSessCtrl(const uint8_t *pu8Req, uint16_t u16Len,
                                     uint8_t u8Sub, uint8_t u8Sid);
static void    AppUds_HandleEcuReset(const uint8_t *pu8Req, uint16_t u16Len,
                                     uint8_t u8Sub, uint8_t u8Sid);
static void    AppUds_HandleClearDtc(const uint8_t *pu8Req, uint16_t u16Len,
                                     uint8_t u8Sid);
static void    AppUds_HandleReadDtc(const uint8_t *pu8Req, uint16_t u16Len,
                                    uint8_t u8Sub, uint8_t u8Sid);
static void    AppUds_HandleRdabi(const uint8_t *pu8Req, uint16_t u16Len,
                                  uint8_t u8Sid);
static void    AppUds_HandleSA(const uint8_t *pu8Req, uint16_t u16Len,
                               uint8_t u8Sub, uint8_t u8Sid);
static void    AppUds_HandleCommCtrl(const uint8_t *pu8Req, uint16_t u16Len,
                                     uint8_t u8Sub, uint8_t u8Sid);
static void    AppUds_HandleWdabi(const uint8_t *pu8Req, uint16_t u16Len,
                                  uint8_t u8Sid);
static void    AppUds_HandleRoutine(const uint8_t *pu8Req, uint16_t u16Len,
                                    uint8_t u8Sub, uint8_t u8Sid);
static void    AppUds_HandleTesterPresent(const uint8_t *pu8Req, uint16_t u16Len,
                                          uint8_t u8Sub, uint8_t u8Sid);
static void    AppUds_HandleDtcSetting(const uint8_t *pu8Req, uint16_t u16Len,
                                       uint8_t u8Sub, uint8_t u8Sid);

/*******************************************************************************
 * ISO-TP 发送侧
 ******************************************************************************/

/**
 * @brief  发送一帧原始 CAN 数据帧。
 */
static void AppUds_SendFrame(uint32_t u32Id, const uint8_t *pu8Data, uint8_t u8Len)
{
    (void)Mcan_Send(u32Id, pu8Data, u8Len);
}

/**
 * @brief  发送否定响应（0x7F SID NRC）。
 *
 *   功能寻址下一律不发（不仅是肯定响应）；"抑制肯定响应"位**不影响**它 ——
 *   按 ISO 14229-1，抑制位只针对肯定响应。
 */
static void AppUds_SendNrc(uint8_t u8Sid, uint8_t u8Nrc)
{
    uint8_t au8Frame[APP_ISOTP_MTU];

    if (m_u8IsFuncAddr != 0U) {
        return;
    }

    memset(au8Frame, 0, sizeof(au8Frame));
    au8Frame[0] = 0x03U;                    /* SF，3 字节 */
    au8Frame[1] = 0x7FU;
    au8Frame[2] = u8Sid;
    au8Frame[3] = u8Nrc;                    /* 第 4 字节不计入长度 */
    AppUds_SendFrame(UDS_CAN_TX_ID, au8Frame, 4U);
}

/**
 * @brief  把"等待流控期间收到的非流控帧"暂存起来。
 *
 *   必须暂存而不是丢弃：若丢弃，上位机紧接着发来的下一条请求就永远丢了，
 *   表现为"偶发响应超时"。暂存后由 boot_req.c 的轮询优先取回处理。
 */
static void AppUds_StashFrame(uint32_t u32Id, const uint8_t *pu8Data, uint8_t u8Len)
{
    uint8_t u8Idx;

    if (m_u8StashCnt >= (uint8_t)APP_UDS_STASH_DEPTH) {
        return;                             /* 暂存已满：只能丢弃 */
    }
    u8Idx = m_u8StashCnt;
    m_u8StashCnt++;
    m_au32StashId[u8Idx] = u32Id;
    memcpy(m_au8StashBuf[u8Idx], pu8Data, u8Len);
    m_au8StashLen[u8Idx] = u8Len;
}

/**
 * @brief  等待对端流控帧。
 * @param  [out] pu8Bs 输出块大小（BS）
 * @retval int32_t 0 = CTS（继续发送）；1 = WT（等待）；2 = OVFLW；
 *                 -1 = 超时或收到错误帧
 */
static int32_t AppUds_WaitFlowControl(uint8_t *pu8Bs)
{
    uint32_t u32Start = SysTick_GetTick();

    while ((SysTick_GetTick() - u32Start) < APP_ISOTP_FC_TIMEOUT_MS) {
        uint32_t u32Id;
        uint8_t  au8Rx[APP_ISOTP_MTU];
        uint8_t  u8Len;

        if (Mcan_Receive(&u32Id, au8Rx, &u8Len) != MCAN_OK) {
            continue;
        }
        if ((u8Len >= 3U) &&
            ((au8Rx[0] & APP_ISOTP_PCI_MASK) == APP_ISOTP_TYPE_FC)) {
            *pu8Bs = au8Rx[1];
            return (int32_t)(au8Rx[0] & 0x0FU);
        }
        AppUds_StashFrame(u32Id, au8Rx, u8Len);
    }
    return -1;
}

/**
 * @brief  阻塞式发送一条 ISO-TP 消息（SF 或 FF + FC 等待 + CF）。
 * @param  [in] pu8Data 数据
 * @param  [in] u16Len  长度（<= 4095）
 * @retval int32_t 0 = 成功；-1 = 失败/超时
 */
static int32_t AppUds_SendIsoTp(const uint8_t *pu8Data, uint16_t u16Len)
{
    uint8_t  au8Frame[APP_ISOTP_MTU];
    uint16_t u16Idx;
    uint8_t  u8Sn;
    uint8_t  u8Bs = 0U;
    uint16_t u16Sent = 0U;
    uint16_t u16BlockLeft;      /* 本块剩余可发帧数；0 = 不限块 */

    /* ---- 单帧 ---- */
    if (u16Len <= APP_ISOTP_SF_MAX) {
        memset(au8Frame, 0, sizeof(au8Frame));
        au8Frame[0] = (uint8_t)(APP_ISOTP_TYPE_SF | (uint8_t)u16Len);
        memcpy(&au8Frame[1], pu8Data, u16Len);
        AppUds_SendFrame(UDS_CAN_TX_ID, au8Frame, (uint8_t)(u16Len + 1U));
        return 0;
    }

    /* ---- 首帧 ---- */
    memset(au8Frame, 0, sizeof(au8Frame));
    au8Frame[0] = (uint8_t)(APP_ISOTP_TYPE_FF | ((u16Len >> 8U) & 0x0FU));
    au8Frame[1] = (uint8_t)(u16Len & 0xFFU);
    memcpy(&au8Frame[2], pu8Data, APP_ISOTP_FF_PAYLOAD);
    AppUds_SendFrame(UDS_CAN_TX_ID, au8Frame, APP_ISOTP_MTU);

    /* ---- 等首个流控帧 ---- */
    if (AppUds_WaitFlowControl(&u8Bs) != 0) {
        return -1;
    }
    u16BlockLeft = (u8Bs != 0U) ? (uint16_t)u8Bs : 0U;

    /* ---- 连续帧 ---- */
    u8Sn   = 1U;
    u16Idx = APP_ISOTP_FF_PAYLOAD;

    while (u16Idx < u16Len) {
        uint8_t u8Remain;

        memset(au8Frame, 0, sizeof(au8Frame));
        au8Frame[0] = (uint8_t)(APP_ISOTP_TYPE_CF | (u8Sn & 0x0FU));
        u8Sn = (uint8_t)((u8Sn + 1U) & 0x0FU);

        u8Remain = (uint8_t)(((u16Len - u16Idx) > 7U) ? 7U : (u16Len - u16Idx));
        memcpy(&au8Frame[1], &pu8Data[u16Idx], u8Remain);
        AppUds_SendFrame(UDS_CAN_TX_ID, au8Frame, (uint8_t)(u8Remain + 1U));

        u16Idx += u8Remain;
        u16Sent++;

        /* 对端声明了块大小时，每发满一块需重新等流控 */
        if ((u16BlockLeft != 0U) && (u16Sent >= u16BlockLeft)) {
            if (AppUds_WaitFlowControl(&u8Bs) != 0) {
                return -1;
            }
            u16Sent      = 0U;
            u16BlockLeft = (u8Bs != 0U) ? (uint16_t)u8Bs : 0U;
        }
    }

    return 0;
}

/**
 * @brief  发送肯定响应（长度 > 7 字节自动走多帧）。
 */
static void AppUds_SendResponse(const uint8_t *pu8Data, uint16_t u16Len)
{
    /* 功能寻址完全不响应；"抑制肯定响应"位只压制肯定响应 */
    if ((m_u8IsFuncAddr != 0U) || (m_u8SuppressPos != 0U)) {
        return;
    }

    if (u16Len <= APP_ISOTP_SF_MAX) {
        uint8_t au8Frame[APP_ISOTP_MTU];

        memset(au8Frame, 0, sizeof(au8Frame));
        au8Frame[0] = (uint8_t)(APP_ISOTP_TYPE_SF | (uint8_t)u16Len);
        memcpy(&au8Frame[1], pu8Data, u16Len);
        AppUds_SendFrame(UDS_CAN_TX_ID, au8Frame, (uint8_t)(u16Len + 1U));
        return;
    }
    (void)AppUds_SendIsoTp(pu8Data, u16Len);
}

/*******************************************************************************
 * 安全访问与通用检查
 ******************************************************************************/

/**
 * @brief  计算期望密钥（必须与 BOOT 侧 uds.c 及上位机 ac310_xor 一致）。
 */
static uint32_t AppUds_ComputeKey(uint32_t u32Seed)
{
    return (u32Seed ^ APP_SA_KEY_XOR_MASK);
}

/**
 * @brief  判断当前会话是否在给定掩码内。
 */
static uint8_t AppUds_IsSessionAllowed(uint8_t u8SessMask)
{
    if ((m_u8Session < APP_SESSION_DEFAULT) || (m_u8Session > APP_SESSION_EXTENDED)) {
        return 0U;
    }
    return ((u8SessMask & (uint8_t)(1U << (m_u8Session - 1U))) != 0U) ? 1U : 0U;
}

/**
 * @brief  参数区（0xF900~0xF930）的会话要求：必须扩展或编程会话。
 *
 *   参数会影响控制环，默认会话下不接受读写 —— 避免误连的诊断仪
 *   在上电默认状态下就把参数读走或改掉。
 */
static uint8_t AppUds_IsParamSessionAllowed(void)
{
    return ((m_u8Session == APP_SESSION_EXTENDED) ||
            (m_u8Session == APP_SESSION_PROGRAMMING)) ? 1U : 0U;
}

/*******************************************************************************
 * 0x27 安全访问
 *
 *   门禁顺序（ISO 14229-1 §10.4）：
 *     27 01 求种子 -> 27 02 发密钥 -> 解锁
 *   **必须先请求种子**，否则 `27 02` 回 NRC 0x24（RequestSequenceError）。
 *
 *   ⚠️ 为什么必须检查顺序：密钥算法是固定异或 key = seed ^ 0x5A5A5A5A，
 *      而 AppUds_Init() 里 seed 初值为 0 —— 若不检查，直接发
 *      `27 02 5A 5A 5A 5A` 就能在从未请求种子的情况下解锁（真实的鉴权绕过）。
 *      上位机侧的 _unlock_security() 本来就是 27 01 -> 27 02，不受影响。
 *
 *   种子一次性：验证（无论成功或失败）后立即作废，防止同一密钥重放。
 ******************************************************************************/
static void AppUds_HandleSA(const uint8_t *pu8Req, uint16_t u16Len,
                            uint8_t u8Sub, uint8_t u8Sid)
{
    uint8_t  au8Resp[2U + APP_SA_SEED_LEN];
    uint32_t u32Key;
    uint32_t u32Expected;

    /* ---------- 0x01 请求种子 ---------- */
    if (u8Sub == APP_SA_LEVEL_SEED) {
        /* 超限保护期内不允许再次请求种子（NRC 0x37） */
        if (m_u8SaAttempts >= (uint8_t)APP_SA_MAX_ATTEMPTS) {
            if ((int32_t)(SysTick_GetTick() - m_u32SaUnlockAtTick) < 0) {
                AppUds_SendNrc(u8Sid, APP_NRC_RTDNE);
                return;
            }
            m_u8SaAttempts = 0U;                /* 延时已到，重新计数 */
        }

        memset(au8Resp, 0, sizeof(au8Resp));
        au8Resp[0] = (uint8_t)(APP_SID_SA + 0x40U);
        au8Resp[1] = u8Sub;

        if (m_u8SecurityUnlocked != 0U) {
            /* 已解锁：按 ISO 14229-1 §10.4.4 回全 0 种子，并同步清零本地种子
             * 与"已发种子"标志 —— 零种子的语义就是"你已经解锁了"。 */
            m_u32Seed         = 0U;
            m_u8SeedRequested = 0U;
        } else {
            /* 非零种子：时基异或常量并强制最低位为 1 */
            m_u32Seed = ((m_u32SeedTick ^ APP_SA_KEY_XOR_MASK) | 1UL);
            m_u8SeedRequested = 1U;             /* 允许随后发密钥 */
            au8Resp[2] = (uint8_t)(m_u32Seed >> 24U);
            au8Resp[3] = (uint8_t)(m_u32Seed >> 16U);
            au8Resp[4] = (uint8_t)(m_u32Seed >> 8U);
            au8Resp[5] = (uint8_t)(m_u32Seed);
        }
        AppUds_SendResponse(au8Resp, (uint16_t)(2U + APP_SA_SEED_LEN));
        return;
    }

    /* ---------- 0x02 发送密钥 ---------- */
    if (u8Sub == APP_SA_LEVEL_KEY) {
        if (u16Len < (uint16_t)(2U + APP_SA_KEY_LEN)) {
            AppUds_SendNrc(u8Sid, APP_NRC_IMLOIF);
            return;
        }

        /* 已解锁：直接肯定响应。部分上位机拿到零种子后仍会补发一次密钥，
         * 此时回 NRC 会让人误判为解锁失败，故按"已解锁即成功"处理。 */
        if (m_u8SecurityUnlocked != 0U) {
            au8Resp[0] = (uint8_t)(APP_SID_SA + 0x40U);
            au8Resp[1] = u8Sub;
            AppUds_SendResponse(au8Resp, 2U);
            return;
        }

        /* 关键门禁：没有先请求种子就发密钥 -> 请求序列错误 */
        if (m_u8SeedRequested == 0U) {
            AppUds_SendNrc(u8Sid, APP_NRC_RSE);
            return;
        }

        if (m_u8SaAttempts >= (uint8_t)APP_SA_MAX_ATTEMPTS) {
            AppUds_SendNrc(u8Sid, APP_NRC_ENOA);
            return;
        }

        u32Key = ((uint32_t)pu8Req[2] << 24U) | ((uint32_t)pu8Req[3] << 16U) |
                 ((uint32_t)pu8Req[4] << 8U)  |  (uint32_t)pu8Req[5];

        /* 种子一次性：验证后立即作废（成功与失败都要作废） */
        u32Expected       = AppUds_ComputeKey(m_u32Seed);
        m_u32Seed         = 0U;
        m_u8SeedRequested = 0U;

        if (u32Key == u32Expected) {
            m_u8SecurityUnlocked = 1U;
            m_u8SaAttempts       = 0U;
            au8Resp[0] = (uint8_t)(APP_SID_SA + 0x40U);
            au8Resp[1] = u8Sub;
            AppUds_SendResponse(au8Resp, 2U);
        } else {
            m_u8SaAttempts++;
            if (m_u8SaAttempts >= (uint8_t)APP_SA_MAX_ATTEMPTS) {
                /* 进入禁止解锁延时，并按 ISO 回 0x36 */
                m_u32SaUnlockAtTick = SysTick_GetTick() + APP_SA_DELAY_MS;
                AppUds_SendNrc(u8Sid, APP_NRC_ENOA);
            } else {
                AppUds_SendNrc(u8Sid, APP_NRC_IK);
            }
        }
        return;
    }

    AppUds_SendNrc(u8Sid, APP_NRC_SFNS);
}

/*******************************************************************************
 * 0x10 诊断会话控制
 ******************************************************************************/
static void AppUds_HandleSessCtrl(const uint8_t *pu8Req, uint16_t u16Len,
                                  uint8_t u8Sub, uint8_t u8Sid)
{
    uint8_t au8Resp[2];
    (void)pu8Req;

    if (u16Len < 2U) {
        AppUds_SendNrc(u8Sid, APP_NRC_IMLOIF);
        return;
    }
    if ((u8Sub != APP_SESSION_DEFAULT) &&
        (u8Sub != APP_SESSION_PROGRAMMING) &&
        (u8Sub != APP_SESSION_EXTENDED)) {
        AppUds_SendNrc(u8Sid, APP_NRC_SFNS);
        return;
    }

    m_u8Session       = u8Sub;
    m_u32LastActivity = SysTick_GetTick();

    /* 注意：这里**不**清解锁状态。
     *
     * ISO 14229-1 只强制"回到默认会话时重新上锁"（见 AppUds_Task 的 S3 超时），
     * 是否在**任意**会话切换时上锁属 OEM 自定。本工程选择"不上锁"是为了
     * 兼容两种上位机流程：
     *   先 0x10 02 再 0x27（本工程上位机的做法）  —— 两种策略都能过
     *   先 0x27 再 0x10 02                       —— 只有不上锁才能过
     * 若你的整车规范要求"换会话即上锁"，在这里加一行
     * m_u8SecurityUnlocked = 0U; 即可（`m_u8SeedRequested` 建议一并清零）。 */

    /* 只镜像子功能号（2 字节响应），与 BOOT 保持一致：
     * 上位机 uds_stack.py 以 standard_version=2006 解析本 ECU 的响应，
     * 回 6 字节（带 P2/P2* 参数记录）反而可能触发响应长度校验。 */
    au8Resp[0] = (uint8_t)(APP_SID_DSC + 0x40U);
    au8Resp[1] = u8Sub;
    AppUds_SendResponse(au8Resp, 2U);
}

/*******************************************************************************
 * 0x11 ECU 复位
 ******************************************************************************/
static void AppUds_HandleEcuReset(const uint8_t *pu8Req, uint16_t u16Len,
                                  uint8_t u8Sub, uint8_t u8Sid)
{
    uint8_t au8Resp[2];
    (void)pu8Req;

    if (u16Len < 2U) {
        AppUds_SendNrc(u8Sid, APP_NRC_IMLOIF);
        return;
    }
    if ((u8Sub != 0x01U) && (u8Sub != 0x02U) && (u8Sub != 0x03U)) {
        AppUds_SendNrc(u8Sid, APP_NRC_SFNS);
        return;
    }

    /* 先发肯定响应，复位动作交给主循环 —— 保证上位机能收到应答，
     * 且响应确实发完才执行复位（ISO 14229-1 §10.2 要求）。 */
    au8Resp[0] = (uint8_t)(APP_SID_ER + 0x40U);
    au8Resp[1] = u8Sub;
    AppUds_SendResponse(au8Resp, 2U);
    m_u8ResetPending = 1U;
}

/*******************************************************************************
 * 0x14 清除诊断信息
 ******************************************************************************/
static void AppUds_HandleClearDtc(const uint8_t *pu8Req, uint16_t u16Len,
                                  uint8_t u8Sid)
{
    uint32_t u32Group;
    uint8_t  au8Resp[4];
    uint8_t  u8Nrc;

    if (u16Len < 4U) {
        AppUds_SendNrc(u8Sid, APP_NRC_IMLOIF);
        return;
    }
    /* 清故障码会影响设备状态，必须已解锁 */
    if (m_u8SecurityUnlocked == 0U) {
        AppUds_SendNrc(u8Sid, APP_NRC_SAD);
        return;
    }

    u32Group = ((uint32_t)pu8Req[1] << 16U) | ((uint32_t)pu8Req[2] << 8U) |
                (uint32_t)pu8Req[3];

    u8Nrc = AppUdsDtc_HandleClear(u32Group);
    if (u8Nrc != 0U) {
        AppUds_SendNrc(u8Sid, u8Nrc);
        return;
    }

    au8Resp[0] = (uint8_t)(APP_SID_CDI + 0x40U);
    au8Resp[1] = (uint8_t)(u32Group >> 16U);
    au8Resp[2] = (uint8_t)(u32Group >> 8U);
    au8Resp[3] = (uint8_t)(u32Group);
    AppUds_SendResponse(au8Resp, 4U);
}

/*******************************************************************************
 * 0x19 读 DTC 信息
 ******************************************************************************/
static void AppUds_HandleReadDtc(const uint8_t *pu8Req, uint16_t u16Len,
                                 uint8_t u8Sub, uint8_t u8Sid)
{
    uint16_t u16OutLen = 0U;
    uint8_t  u8Nrc;

    if (u16Len < 2U) {
        AppUds_SendNrc(u8Sid, APP_NRC_IMLOIF);
        return;
    }

    /* 响应格式：59 <sub> <数据...>，数据域由 DTC 模块填充 */
    m_au8Resp[0] = (uint8_t)(APP_SID_RDTC + 0x40U);
    m_au8Resp[1] = u8Sub;

    u8Nrc = AppUdsDtc_HandleRead(pu8Req, u16Len, &m_au8Resp[2],
                                 (uint16_t)(APP_UDS_TX_BUF_LEN - 2U), &u16OutLen);
    if (u8Nrc != 0U) {
        AppUds_SendNrc(u8Sid, u8Nrc);
        return;
    }

    AppUds_SendResponse(m_au8Resp, (uint16_t)(2U + u16OutLen));
}

/*******************************************************************************
 * 0x22 按标识符读数据（支持一次请求多个 DID）
 ******************************************************************************/

/**
 * @brief  读一个 DID 到响应缓冲（自动区分 DID 表与参数区）。
 * @retval uint8_t 0 = 成功；非 0 = NRC
 */
static uint8_t AppUds_ReadOneDid(uint16_t u16Did, uint8_t *pu8Out,
                                 uint16_t u16MaxLen, uint16_t *pu16OutLen)
{
    const app_uds_did_t *pEntry;

    /* ---------- 可读写参数区（0xF900 ~ 0xF93F） ---------- */
    if (AppUdsDid_IsParamRange(u16Did) != 0U) {
        if (AppUds_IsParamSessionAllowed() == 0U) {
            return APP_NRC_SNSIAS;
        }
        /* 类型/scale/量程全部由参数描述表决定（u16 是 2 字节，f32 是 4 字节），
         * 协议层不参与解释，避免"某条参数是 f32"这种知识泄漏到这一层。 */
        return AppUdsParam_Read(u16Did, pu8Out, u16MaxLen, pu16OutLen);
    }

    /* ---------- 常规 DID 表 ---------- */
    pEntry = AppUdsDid_Find(u16Did);
    if (pEntry == NULL) {
        return APP_NRC_ROOR;
    }
    if (pEntry->pfRead == NULL) {
        return APP_NRC_ROOR;
    }
    /* 会话不匹配回 0x7F（不是 0x11） */
    if (AppUds_IsSessionAllowed(pEntry->u8SessMask) == 0U) {
        return APP_NRC_SNSIAS;
    }
    /* 需要解锁的 DID */
    if ((pEntry->u8Sec == APP_SEC_UNLOCKED) && (m_u8SecurityUnlocked == 0U)) {
        return APP_NRC_SAD;
    }

    {
        uint8_t  u8Nrc;
        uint16_t u16Len2 = 0U;

        u8Nrc = pEntry->pfRead(pu8Out, u16MaxLen, &u16Len2);
        if (u8Nrc != 0U) {
            return u8Nrc;
        }
        /* 固定长度 DID 必须与声明一致，否则 NRC 0x13 */
        if ((pEntry->u16Len != 0U) && (u16Len2 != pEntry->u16Len)) {
            return APP_NRC_IMLOIF;
        }
        *pu16OutLen = u16Len2;
    }
    return 0U;
}

static void AppUds_HandleRdabi(const uint8_t *pu8Req, uint16_t u16Len,
                               uint8_t u8Sid)
{
    uint16_t u16Did;
    uint16_t u16Pos = 1U;               /* 62 之后即第一个 DID 的高字节 */
    uint16_t u16Idx;

    /* 长度校验：
     *   22 <DID(2)>              3 字节  -> 单 DID
     *   22 FF00 01               4 字节  -> 固件更新请求带子功能（特例）
     *   22 <DID(2)> [<DID(2)>...]  1+2n   -> 一次读多个 DID
     * 先放行 0xFF00 的 4 字节形式，再校验通用的"1 + 偶数"格式。 */
    if (u16Len < 3U) {
        AppUds_SendNrc(u8Sid, APP_NRC_IMLOIF);
        return;
    }
    if ((uint16_t)((u16Len - 1U) & 1U) != 0U) {
        /* 只允许 22 <FF00> <子功能> 这种 4 字节特例 */
        uint16_t u16SpecialDid =
            (uint16_t)(((uint16_t)pu8Req[1] << 8U) | pu8Req[2]);

        if ((u16Len != 4U) || (u16SpecialDid != UDS_DID_FW_UPDATE_REQ)) {
            AppUds_SendNrc(u8Sid, APP_NRC_IMLOIF);
            return;
        }
    }

    m_au8Resp[0] = (uint8_t)(APP_SID_RDABI + 0x40U);

    for (u16Idx = 1U; (uint16_t)(u16Idx + 1U) < u16Len; u16Idx = (uint16_t)(u16Idx + 2U)) {
        uint16_t u16OutLen = 0U;
        uint8_t  u8Nrc;

        u16Did = (uint16_t)(((uint16_t)pu8Req[u16Idx] << 8U) | pu8Req[u16Idx + 1U]);

        /* ---------- 特例：0xFF00 固件更新请求（交棒），不走 DID 表 ---------- */
        if (u16Did == UDS_DID_FW_UPDATE_REQ) {
            /* 禁止功能寻址：一条广播会把总线上所有节点都打回 BOOT */
            if (m_u8IsFuncAddr != 0U) {
                return;
            }
            /* 只允许"单独读这一个 DID"：22 FF00 或 22 FF00 01（带子功能），
             * 不允许与其它 DID 拼在一起（交棒是"命令"，不是"读取"）。 */
            if ((u16Idx != 1U) ||
                ((u16Len != 3U) && (u16Len != 4U))) {
                AppUds_SendNrc(u8Sid, APP_NRC_ROOR);
                return;
            }
#if (APP_UDS_REQUIRE_SECURITY != 0)
            if (m_u8SecurityUnlocked == 0U) {
                AppUds_SendNrc(u8Sid, APP_NRC_SAD);
                return;
            }
#endif
            /* 可选子功能：存在时必须为 0x01（开始更新） */
            if ((u16Len > 3U) && (pu8Req[3] != FW_UPDATE_SUBFUNC_START)) {
                AppUds_SendNrc(u8Sid, APP_NRC_SFNS);
                return;
            }

            /* 两道门禁都通过：接受刷写请求，先回应答，主循环随即可复位 */
            m_u8BootRequested = 1U;
            m_au8Resp[0] = (uint8_t)(APP_SID_RDABI + 0x40U);
            m_au8Resp[1] = (uint8_t)(u16Did >> 8U);
            m_au8Resp[2] = (uint8_t)(u16Did);
            AppUds_SendResponse(m_au8Resp, 3U);
            return;
        }

        /* ---------- 常规 DID：先占位 DID 字段，再填数据 ---------- */
        if (u16Pos > (uint16_t)(APP_UDS_TX_BUF_LEN - 4U)) {
            AppUds_SendNrc(u8Sid, APP_NRC_RTL);      /* 响应太长 */
            return;
        }
        m_au8Resp[u16Pos]      = (uint8_t)(u16Did >> 8U);
        m_au8Resp[u16Pos + 1U] = (uint8_t)(u16Did);

        u8Nrc = AppUds_ReadOneDid(u16Did, &m_au8Resp[u16Pos + 2U],
                                  (uint16_t)(APP_UDS_TX_BUF_LEN - u16Pos - 2U),
                                  &u16OutLen);
        if (u8Nrc != 0U) {
            AppUds_SendNrc(u8Sid, u8Nrc);
            return;
        }
        u16Pos = (uint16_t)(u16Pos + 2U + u16OutLen);
    }

    AppUds_SendResponse(m_au8Resp, u16Pos);
}

/*******************************************************************************
 * 0x2E 按标识符写数据
 ******************************************************************************/
static void AppUds_HandleWdabi(const uint8_t *pu8Req, uint16_t u16Len,
                               uint8_t u8Sid)
{
    uint16_t u16Did;
    uint16_t u16DataLen;
    uint8_t  au8Resp[3];

    if (u16Len < 4U) {
        AppUds_SendNrc(u8Sid, APP_NRC_IMLOIF);
        return;
    }
    if (m_u8IsFuncAddr != 0U) {
        return;                     /* 写操作严禁功能寻址（白名单已挡，双保险） */
    }

    u16Did     = (uint16_t)(((uint16_t)pu8Req[1] << 8U) | pu8Req[2]);
    u16DataLen = (uint16_t)(u16Len - 3U);

    /* ---------- 可读写参数区（0xF900 ~ 0xF93F） ---------- */
    if (AppUdsDid_IsParamRange(u16Did) != 0U) {
        uint8_t u8Nrc;

        if (AppUds_IsParamSessionAllowed() == 0U) {
            AppUds_SendNrc(u8Sid, APP_NRC_SNSIAS);
            return;
        }
        /* 参数影响控制环，必须已解锁 */
        if (m_u8SecurityUnlocked == 0U) {
            AppUds_SendNrc(u8Sid, APP_NRC_SAD);
            return;
        }

        /* 长度 / 类型 / 量程校验都在参数模块里完成
         * （u16 收 2 字节、f32 收 4 字节；长度不符回 NRC 0x13）。 */
        u8Nrc = AppUdsParam_Write(u16Did, &pu8Req[3], u16DataLen);
        if (u8Nrc != 0U) {
            AppUds_SendNrc(u8Sid, u8Nrc);
            return;
        }
    } else {
        /* ---------- 常规 DID 表 ---------- */
        const app_uds_did_t *pEntry = AppUdsDid_Find(u16Did);
        uint8_t u8Nrc;

        if (pEntry == NULL) {
            AppUds_SendNrc(u8Sid, APP_NRC_ROOR);
            return;
        }
        if (pEntry->pfWrite == NULL) {
            AppUds_SendNrc(u8Sid, APP_NRC_ROOR);
            return;
        }
        if (AppUds_IsSessionAllowed(pEntry->u8SessMask) == 0U) {
            AppUds_SendNrc(u8Sid, APP_NRC_SNSIAS);
            return;
        }
        /* 只要声明了非 NONE 的安全要求，写操作就必须已解锁 */
        if ((pEntry->u8Sec != APP_SEC_NONE) && (m_u8SecurityUnlocked == 0U)) {
            AppUds_SendNrc(u8Sid, APP_NRC_SAD);
            return;
        }
        if ((pEntry->u16Len != 0U) && (u16DataLen != pEntry->u16Len)) {
            AppUds_SendNrc(u8Sid, APP_NRC_IMLOIF);
            return;
        }

        u8Nrc = pEntry->pfWrite(&pu8Req[3], u16DataLen);
        if (u8Nrc != 0U) {
            AppUds_SendNrc(u8Sid, u8Nrc);
            return;
        }
    }

    au8Resp[0] = (uint8_t)(APP_SID_WDABI + 0x40U);
    au8Resp[1] = (uint8_t)(u16Did >> 8U);
    au8Resp[2] = (uint8_t)(u16Did);
    AppUds_SendResponse(au8Resp, 3U);
}

/*******************************************************************************
 * 0x28 通信控制（接受但本工程不真正关闭收发）
 ******************************************************************************/
static void AppUds_HandleCommCtrl(const uint8_t *pu8Req, uint16_t u16Len,
                                  uint8_t u8Sub, uint8_t u8Sid)
{
    uint8_t au8Resp[3];

    if (u16Len < 3U) {
        AppUds_SendNrc(u8Sid, APP_NRC_IMLOIF);
        return;
    }
    if (u8Sub > 0x03U) {
        AppUds_SendNrc(u8Sid, APP_NRC_SFNS);
        return;
    }
    if (m_u8SecurityUnlocked == 0U) {
        AppUds_SendNrc(u8Sid, APP_NRC_SAD);
        return;
    }

    /* 本工程的诊断通道即业务通道，关掉收发等于自我失联，
     * 因此只做协议应答而不改变总线行为（这点需在项目文档中说明）。 */
    au8Resp[0] = (uint8_t)(APP_SID_CC + 0x40U);
    au8Resp[1] = u8Sub;
    au8Resp[2] = pu8Req[2];
    AppUds_SendResponse(au8Resp, 3U);
}

/*******************************************************************************
 * 0x31 例程控制
 ******************************************************************************/
static void AppUds_HandleRoutine(const uint8_t *pu8Req, uint16_t u16Len,
                                 uint8_t u8Sub, uint8_t u8Sid)
{
    uint16_t u16Rid;
    uint8_t  au8Resp[4];
    uint8_t  u8Nrc = 0U;

    if (u16Len < 4U) {
        AppUds_SendNrc(u8Sid, APP_NRC_IMLOIF);
        return;
    }
    if ((u8Sub != 0x01U) && (u8Sub != 0x02U) && (u8Sub != 0x03U)) {
        AppUds_SendNrc(u8Sid, APP_NRC_SFNS);
        return;
    }

    u16Rid = (uint16_t)(((uint16_t)pu8Req[2] << 8U) | pu8Req[3]);

    switch (u16Rid) {
        /* 检查编程依赖：App 正在运行即视为通过（BOOT 侧用它判断能否跳转） */
        case APP_RID_CHECK_PROG_DEP:
            if (u8Sub != 0x01U) {
                u8Nrc = APP_NRC_SFNS;
            }
            break;

        /* 参数区恢复默认值：startRoutine 生效，requestRoutineResults 只查询 */
        case APP_RID_PARAM_RESET_DEFAULT:
            if (u8Sub == 0x01U) {
                if (m_u8SecurityUnlocked == 0U) {
                    u8Nrc = APP_NRC_SAD;
                } else {
                    u8Nrc = AppUdsParam_ResetAll();
                }
            } else if (u8Sub == 0x02U) {
                u8Nrc = APP_NRC_SFNS;       /* 不支持 stopRoutine */
            }
            break;

        /* 参数区一致性检查 */
        case APP_RID_PARAM_CHECK:
            if (u8Sub == 0x01U) {
                (void)AppUdsParam_CheckAll();   /* 越界项钳回默认值 */
            }
            break;

        /* 保存配置（本工程无掉电存储，仅调用弱函数钩子） */
        case APP_RID_SAVE_CONFIG:
            if (u8Sub == 0x01U) {
                if (m_u8SecurityUnlocked == 0U) {
                    u8Nrc = APP_NRC_SAD;
                } else {
                    u8Nrc = AppUdsParam_Save();
                }
            }
            break;

        /* 应用层自检（占位，实际逻辑由 FOC 侧实现） */
        case APP_RID_MOTOR_SELFTEST:
            if (u8Sub == 0x01U) {
                if (m_u8SecurityUnlocked == 0U) {
                    u8Nrc = APP_NRC_SAD;
                }
            } else {
                u8Nrc = APP_NRC_SFNS;
            }
            break;

        default:
            AppUds_SendNrc(u8Sid, APP_NRC_ROOR);
            return;
    }

    if (u8Nrc != 0U) {
        AppUds_SendNrc(u8Sid, u8Nrc);
        return;
    }

    au8Resp[0] = (uint8_t)(APP_SID_RC + 0x40U);
    au8Resp[1] = u8Sub;
    au8Resp[2] = (uint8_t)(u16Rid >> 8U);
    au8Resp[3] = (uint8_t)(u16Rid);
    AppUds_SendResponse(au8Resp, 4U);
}

/*******************************************************************************
 * 0x3E 会话保持
 ******************************************************************************/
static void AppUds_HandleTesterPresent(const uint8_t *pu8Req, uint16_t u16Len,
                                       uint8_t u8Sub, uint8_t u8Sid)
{
    uint8_t au8Resp[2];
    (void)pu8Req;

    if (u16Len != 2U) {
        AppUds_SendNrc(u8Sid, APP_NRC_IMLOIF);
        return;
    }
    /* 子功能只支持 0x00（或带抑制位的 0x80，已在上层剥离） */
    if (u8Sub != 0x00U) {
        AppUds_SendNrc(u8Sid, APP_NRC_SFNS);
        return;
    }

    /* 活动时间戳已在分发器入口更新，这里只需应答 */
    au8Resp[0] = (uint8_t)(APP_SID_TP + 0x40U);
    au8Resp[1] = 0x00U;
    AppUds_SendResponse(au8Resp, 2U);
}

/*******************************************************************************
 * 0x85 DTC 设置
 ******************************************************************************/
static void AppUds_HandleDtcSetting(const uint8_t *pu8Req, uint16_t u16Len,
                                    uint8_t u8Sub, uint8_t u8Sid)
{
    uint8_t au8Resp[2];
    uint8_t u8Nrc;
    (void)pu8Req;

    if (u16Len < 2U) {
        AppUds_SendNrc(u8Sid, APP_NRC_IMLOIF);
        return;
    }
    if (m_u8SecurityUnlocked == 0U) {
        AppUds_SendNrc(u8Sid, APP_NRC_SAD);
        return;
    }

    u8Nrc = AppUdsDtc_HandleSetting(u8Sub);
    if (u8Nrc != 0U) {
        AppUds_SendNrc(u8Sid, u8Nrc);
        return;
    }

    au8Resp[0] = (uint8_t)(APP_SID_CDTCS + 0x40U);
    au8Resp[1] = u8Sub;
    AppUds_SendResponse(au8Resp, 2U);
}

/*******************************************************************************
 * 分发器
 ******************************************************************************/

/**
 * @brief  判断某 SID 是否允许在功能寻址下处理。
 */
static uint8_t AppUds_IsFuncAllowed(uint8_t u8Sid)
{
#if (APP_UDS_ENABLE_FUNC_ADDR != 0)
    switch (u8Sid) {
        case APP_UDS_FUNC_SID_SESS_CTRL:
        case APP_UDS_FUNC_SID_ECU_RESET:
        case APP_UDS_FUNC_SID_CLEAR_DTC:
        case APP_UDS_FUNC_SID_READ_DTC:
        case APP_UDS_FUNC_SID_READ_DID:
        case APP_UDS_FUNC_SID_COMM_CTRL:
        case APP_UDS_FUNC_SID_TESTER_PRESENT:
        case APP_UDS_FUNC_SID_DTC_SETTING:
            return 1U;
        default:
            return 0U;
    }
#else
    (void)u8Sid;
    return 0U;
#endif
}

/**
 * @brief  判断服务是否带子功能字节（用于提取子功能与抑制位）。
 */
static uint8_t AppUds_HasSubfunc(uint8_t u8Sid)
{
    switch (u8Sid) {
        case APP_SID_DSC:
        case APP_SID_ER:
        case APP_SID_RDTC:
        case APP_SID_SA:
        case APP_SID_CC:
        case APP_SID_RC:
        case APP_SID_TP:
        case APP_SID_CDTCS:
            return 1U;
        default:
            return 0U;
    }
}

void AppUds_HandleRequest(const uint8_t *pu8Req, uint16_t u16Len,
                          uint8_t u8AddrType)
{
    uint8_t u8Sid;
    uint8_t u8Sub = 0U;

    if ((pu8Req == NULL) || (u16Len < 1U)) {
        return;
    }

    u8Sid             = pu8Req[0];
    m_u32LastActivity = SysTick_GetTick();

    /* ---- 寻址类型处理 ---- */
    if (u8AddrType == APP_ADDR_FUNCTIONAL) {
#if (APP_UDS_ENABLE_FUNC_ADDR != 0)
        if (AppUds_IsFuncAllowed(u8Sid) == 0U) {
            return;                          /* 白名单外：静默丢弃，不回 0x11 */
        }
        m_u8IsFuncAddr = 1U;
#else
        return;                              /* 未启用功能寻址：一律忽略 */
#endif
    } else {
        m_u8IsFuncAddr = 0U;
    }
    m_u8SuppressPos = 0U;

    /* ---- 提取子功能与"抑制肯定响应"位 ---- */
    if (AppUds_HasSubfunc(u8Sid) != 0U) {
        if (u16Len < 2U) {
            AppUds_SendNrc(u8Sid, APP_NRC_IMLOIF);
            return;
        }
        u8Sub = (uint8_t)(pu8Req[1] & APP_SUBFUNC_MASK);
        if ((pu8Req[1] & APP_SUBFUNC_SUPPRESS_POS) != 0U) {
            m_u8SuppressPos = 1U;
        }
    }

    switch (u8Sid) {
        case APP_SID_DSC:
            AppUds_HandleSessCtrl(pu8Req, u16Len, u8Sub, u8Sid);
            break;
        case APP_SID_ER:
            AppUds_HandleEcuReset(pu8Req, u16Len, u8Sub, u8Sid);
            break;
        case APP_SID_CDI:
            AppUds_HandleClearDtc(pu8Req, u16Len, u8Sid);
            break;
        case APP_SID_RDTC:
            AppUds_HandleReadDtc(pu8Req, u16Len, u8Sub, u8Sid);
            break;
        case APP_SID_RDABI:
            AppUds_HandleRdabi(pu8Req, u16Len, u8Sid);
            break;
        case APP_SID_SA:
            AppUds_HandleSA(pu8Req, u16Len, u8Sub, u8Sid);
            break;
        case APP_SID_CC:
            AppUds_HandleCommCtrl(pu8Req, u16Len, u8Sub, u8Sid);
            break;
        case APP_SID_WDABI:
            AppUds_HandleWdabi(pu8Req, u16Len, u8Sid);
            break;
        case APP_SID_RC:
            AppUds_HandleRoutine(pu8Req, u16Len, u8Sub, u8Sid);
            break;
        case APP_SID_TP:
            AppUds_HandleTesterPresent(pu8Req, u16Len, u8Sub, u8Sid);
            break;
        case APP_SID_CDTCS:
            AppUds_HandleDtcSetting(pu8Req, u16Len, u8Sub, u8Sid);
            break;

#if (APP_UDS_ENABLE_MEM_ACCESS != 0)
        case APP_SID_RMBA:
            /* 需要时自行实现，务必加地址白名单 */
            AppUds_SendNrc(u8Sid, APP_NRC_SNS);
            break;
#endif
        default:
            /* 物理寻址回 0x11 让上位机快速失败；功能寻址已在上方静默返回 */
            AppUds_SendNrc(u8Sid, APP_NRC_SNS);
            break;
    }

    /* 本次请求处理完毕，清掉暂态标志（不影响后续请求的独立判定） */
    m_u8IsFuncAddr  = 0U;
    m_u8SuppressPos = 0U;
}

/*******************************************************************************
 * 生命周期与周期任务
 ******************************************************************************/

void AppUds_Task(uint32_t u32Tick)
{
    m_u32SeedTick = u32Tick;

    /* S3：非默认会话超时 -> 回默认会话并清解锁（与 BOOT 行为一致） */
    if (m_u8Session != APP_SESSION_DEFAULT) {
        if ((u32Tick - m_u32LastActivity) >= UDS_S3_TIMEOUT_MS) {
            m_u8Session          = APP_SESSION_DEFAULT;
            m_u8SecurityUnlocked = 0U;
            m_u8SaAttempts       = 0U;
            m_u32Seed            = 0U;
            m_u8SeedRequested    = 0U;
        }
    }

    /* 安全访问超限延时到期后清计数（下次可重新请求种子） */
    if (m_u8SaAttempts >= (uint8_t)APP_SA_MAX_ATTEMPTS) {
        if ((int32_t)(u32Tick - m_u32SaUnlockAtTick) >= 0) {
            m_u8SaAttempts = 0U;
        }
    }
}

void AppUds_Init(void)
{
    m_u8Session           = APP_SESSION_DEFAULT;
    m_u8SecurityUnlocked  = 0U;
    m_u32Seed             = 0U;
    m_u8SeedRequested     = 0U;
    m_u8SaAttempts        = 0U;
    m_u32SaUnlockAtTick   = 0U;
    m_u32SeedTick         = 0U;
    m_u32LastActivity     = SysTick_GetTick();
    m_u8BootRequested     = 0U;
    m_u8BaudChangePending = 0U;
    m_u8ResetPending      = 0U;
    m_u8IsFuncAddr        = 0U;
    m_u8SuppressPos       = 0U;
    m_u8StashCnt          = 0U;

    /* 参数区与 DTC 表 */
    AppUdsParam_Init();
    AppUdsDtc_Init();
}

uint8_t AppUds_PopStashFrame(uint32_t *pu32Id, uint8_t *pu8Data, uint8_t *pu8Len)
{
    if (m_u8StashCnt == 0U) {
        return 0U;
    }

    if (pu32Id != NULL) {
        *pu32Id = m_au32StashId[0];
    }
    if (pu8Data != NULL) {
        memcpy(pu8Data, m_au8StashBuf[0], m_au8StashLen[0]);
    }
    if (pu8Len != NULL) {
        *pu8Len = m_au8StashLen[0];
    }

    /* 队列前移 */
    m_u8StashCnt--;
    if (m_u8StashCnt > 0U) {
        memmove(&m_au32StashId[0], &m_au32StashId[1],
                (size_t)m_u8StashCnt * sizeof(m_au32StashId[0]));
        memmove(&m_au8StashBuf[0][0], &m_au8StashBuf[1][0],
                (size_t)m_u8StashCnt * APP_ISOTP_MTU);
        memmove(&m_au8StashLen[0], &m_au8StashLen[1], (size_t)m_u8StashCnt);
    }
    return 1U;
}

/* ---------------------------- 状态查询接口 ---------------------------- */

uint8_t AppUds_GetSession(void)        { return m_u8Session; }
uint8_t AppUds_IsUnlocked(void)        { return m_u8SecurityUnlocked; }
uint8_t AppUds_BootRequested(void)     { return m_u8BootRequested; }
uint8_t AppUds_ResetPending(void)      { return m_u8ResetPending; }
uint8_t AppUds_BaudChangePending(void) { return m_u8BaudChangePending; }

void AppUds_ClearBootRequest(void)     { m_u8BootRequested = 0U; }
void AppUds_RequestBaudSwitch(void)    { m_u8BaudChangePending = 1U; }
void AppUds_ClearBaudChange(void)      { m_u8BaudChangePending = 0U; }

/*******************************************************************************
 * 文件结束
 ******************************************************************************/
