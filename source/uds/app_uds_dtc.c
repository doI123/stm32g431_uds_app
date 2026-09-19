/**
 *******************************************************************************
 * @file  app_uds_dtc.c
 * @brief DTC 表实现与 0x19 / 0x14 / 0x85 服务
 *
 *   存储：纯 RAM 表（无掉电保持）。
 *   每条记录：DTC(3B) + 状态(1B) + 老化计数(1B) + 发生次数(2B) + 首次发生时刻
 *             + 扩展数据(4B)。
 *
 *   0x19 子功能实现情况（ISO 14229-1:2020 §14.7 报文格式）：
 *     0x01 reportNumberOfDTCByStatusMask        : 59 01 <availMask> <format> <cnt(2)>
 *     0x02 reportDTCByStatusMask                : 59 02 <availMask> <DTCAndStatus[]>
 *     0x03 reportDTCSnapshotIdentification      : 59 03 <DTCAndStatus[]>
 *     0x04 reportDTCSnapshotRecordByDTCNumber   : 59 04 <DTC+st> <recNum> <snapshot[]>
 *     0x06 reportDTCExtDataRecordByDTCNumber    : 59 06 <DTC+st> <recNum> <extData[]>
 *     0x0A reportSupportedDTC                   : 59 0A <availMask> <DTCAndStatus[]>
 *     0x14 reportDTCFaultDetectionCounter       : 59 14 <DTC+st> <FDC>
 *
 *   FDC（故障检测计数器）本工程约定：
 *     0x00..0x7F = 未失败（0x7F 表示计数已满但仍未失败）
 *     0x80..0xFF = 已失败（0x80 刚失败，0xFF 计数已满）
 *   这是业界最常见的表示法，与 ISO 14229-1 的"有符号 -128..+127"等价，
 *   只是这里用无符号字节的直观读法。
 *******************************************************************************
 * Copyright (C) 2026, all rights reserved.
 *
 * This software component is licensed under BSD 3-Clause license.
 *******************************************************************************/
#include <string.h>
#include "app_uds_dtc.h"
#include "app_uds.h"
#include "clock.h"

/* ============================== 内部类型 ============================== */
typedef struct {
    uint32_t u32Dtc;                            /* 3 字节有效           */
    uint8_t  u8Status;                          /* APP_DTC_STATUS_*     */
    uint8_t  u8Aging;                           /* 老化计数             */
    uint16_t u16Occur;                          /* 发生次数             */
    uint32_t u32FirstTick;                      /* 首次发生（ms）       */
    uint8_t  au8ExtData[APP_DTC_EXT_DATA_LEN];  /* 扩展数据             */
} app_uds_dtc_entry_t;

/* ============================== 内部变量 ============================== */
static app_uds_dtc_entry_t m_astcDtc[APP_UDS_DTC_MAX];
static uint8_t  m_u8DtcCount   = 0U;     /* 当前有效条数 */
static uint8_t  m_u8DtcEnabled = 1U;     /* 0x85 开关    */

/* ============================== 内部函数 ============================== */

/**
 * @brief  在表中查找 DTC（未找到返回 NULL）。
 */
static app_uds_dtc_entry_t *Dtc_Find(uint32_t u32Dtc)
{
    uint8_t i;

    for (i = 0U; i < m_u8DtcCount; i++) {
        if (m_astcDtc[i].u32Dtc == u32Dtc) {
            return &m_astcDtc[i];
        }
    }
    return NULL;
}

/**
 * @brief  追加一条 DTC 记录（表满返回 NULL）。
 */
static app_uds_dtc_entry_t *Dtc_Append(uint32_t u32Dtc)
{
    app_uds_dtc_entry_t *pEntry;

    if (m_u8DtcCount >= APP_UDS_DTC_MAX) {
        return NULL;
    }
    pEntry = &m_astcDtc[m_u8DtcCount];
    m_u8DtcCount++;

    memset(pEntry, 0, sizeof(*pEntry));
    pEntry->u32Dtc = u32Dtc & 0xFFFFFFUL;
    /* 新记录：自上次清除后尚未完成测试 */
    pEntry->u8Status = APP_DTC_STATUS_NOT_COMPLETED_SINCE_CLR;
    pEntry->u32FirstTick = SysTick_GetTick();
    return pEntry;
}

/**
 * @brief  向响应缓冲追加一条"DTC + 状态"（4 字节）。
 * @retval uint8_t 1 = 已写入；0 = 缓冲不足
 */
static uint8_t Dtc_EmitOne(const app_uds_dtc_entry_t *pEntry,
                           uint8_t *pu8Out, uint16_t u16MaxLen, uint16_t *pu16Len)
{
    if ((uint16_t)(*pu16Len + 4U) > u16MaxLen) {
        return 0U;
    }
    pu8Out[*pu16Len]      = (uint8_t)((pEntry->u32Dtc >> 16U) & 0xFFU);
    pu8Out[*pu16Len + 1U] = (uint8_t)((pEntry->u32Dtc >> 8U) & 0xFFU);
    pu8Out[*pu16Len + 2U] = (uint8_t)(pEntry->u32Dtc & 0xFFU);
    pu8Out[*pu16Len + 3U] = pEntry->u8Status;
    *pu16Len = (uint16_t)(*pu16Len + 4U);
    return 1U;
}

/**
 * @brief  按状态掩码统计匹配条数。
 */
static uint16_t Dtc_CountByMask(uint8_t u8Mask)
{
    uint16_t u16Cnt = 0U;
    uint8_t  i;

    for (i = 0U; i < m_u8DtcCount; i++) {
        if ((m_astcDtc[i].u8Status & u8Mask) != 0U) {
            u16Cnt++;
        }
    }
    return u16Cnt;
}

/**
 * @brief  从请求中解析 3 字节 DTC。
 */
static uint32_t Dtc_ParseDtc(const uint8_t *pu8Req, uint8_t u8Offset)
{
    return (((uint32_t)pu8Req[u8Offset] << 16U) |
            ((uint32_t)pu8Req[u8Offset + 1U] << 8U) |
             (uint32_t)pu8Req[u8Offset + 2U]);
}

/* ============================== 对外实现 ============================== */

void AppUdsDtc_Init(void)
{
    memset(m_astcDtc, 0, sizeof(m_astcDtc));
    m_u8DtcCount   = 0U;
    m_u8DtcEnabled = 1U;
}

void AppUdsDtc_Report(uint32_t u32Dtc, uint8_t u8Failed,
                      const uint8_t *pu8ExtData, uint16_t u16ExtLen)
{
    app_uds_dtc_entry_t *pEntry;

    u32Dtc &= 0xFFFFFFUL;

    pEntry = Dtc_Find(u32Dtc);
    if (pEntry == NULL) {
        if (u8Failed == 0U) {
            return;                     /* 故障消失且本来没记录：无需建表项 */
        }
        pEntry = Dtc_Append(u32Dtc);
        if (pEntry == NULL) {
            return;                     /* 表满：丢弃，不覆盖已有故障      */
        }
    }

    if (u8Failed != 0U) {
        if (m_u8DtcEnabled == 0U) {
            return;                     /* 0x85 OFF：冻结记录，不更新状态  */
        }
        pEntry->u8Status |= (APP_DTC_STATUS_TEST_FAILED |
                             APP_DTC_STATUS_TEST_FAILED_THIS_CYCLE |
                             APP_DTC_STATUS_FAILED_SINCE_CLR |
                             APP_DTC_STATUS_CONFIRMED);
        pEntry->u8Status &= (uint8_t)~APP_DTC_STATUS_PENDING;
        pEntry->u8Aging   = 0U;
        if (pEntry->u16Occur < 0xFFFFU) {
            pEntry->u16Occur++;
        }
        if (pu8ExtData != NULL) {
            uint16_t u16Copy = (u16ExtLen > APP_DTC_EXT_DATA_LEN)
                             ? (uint16_t)APP_DTC_EXT_DATA_LEN : u16ExtLen;
            memcpy(pEntry->au8ExtData, pu8ExtData, u16Copy);
        }
    } else {
        /* 故障消失：清 testFailed / this-cycle / pending，保留 confirmed
         * 交由 AppUdsDtc_OperationCycle() 做老化判定。 */
        pEntry->u8Status &= (uint8_t)~(APP_DTC_STATUS_TEST_FAILED |
                                       APP_DTC_STATUS_TEST_FAILED_THIS_CYCLE |
                                       APP_DTC_STATUS_PENDING);
    }
}

void AppUdsDtc_OperationCycle(void)
{
    uint8_t i;

    for (i = 0U; i < m_u8DtcCount; i++) {
        app_uds_dtc_entry_t *pEntry = &m_astcDtc[i];

        if ((pEntry->u8Status & APP_DTC_STATUS_TEST_FAILED) != 0U) {
            pEntry->u8Aging = 0U;
        } else {
            if (pEntry->u8Aging < 0xFFU) {
                pEntry->u8Aging++;
            }
            /* 连续 APP_DTC_AGING_LIMIT 个循环无故障 -> 清 confirmed */
            if (pEntry->u8Aging >= (uint8_t)APP_DTC_AGING_LIMIT) {
                pEntry->u8Status &= (uint8_t)~APP_DTC_STATUS_CONFIRMED;
                pEntry->u8Aging = 0U;
            }
        }
        /* 新一轮：清"本轮未完成/本轮失败"位 */
        pEntry->u8Status &= (uint8_t)~(APP_DTC_STATUS_NOT_COMPLETED_THIS_CYCLE |
                                       APP_DTC_STATUS_TEST_FAILED_THIS_CYCLE);
        if ((pEntry->u8Status & APP_DTC_STATUS_TEST_FAILED) == 0U) {
            pEntry->u8Status |= APP_DTC_STATUS_NOT_COMPLETED_THIS_CYCLE;
        }
    }
}

uint8_t AppUdsDtc_IsEnabled(void)
{
    return m_u8DtcEnabled;
}

uint8_t AppUdsDtc_GetCount(void)
{
    return m_u8DtcCount;
}

uint8_t AppUdsDtc_HandleClear(uint32_t u32Group)
{
    uint8_t i;

    u32Group &= 0xFFFFFFUL;

    if (u32Group == 0xFFFFFFUL) {
        memset(m_astcDtc, 0, sizeof(m_astcDtc));
        m_u8DtcCount = 0U;
        return 0U;
    }

    /* 指定组：删除匹配的 DTC（把后续条目前移） */
    i = 0U;
    while (i < m_u8DtcCount) {
        if (m_astcDtc[i].u32Dtc == u32Group) {
            uint8_t j;

            for (j = i; (uint8_t)(j + 1U) < m_u8DtcCount; j++) {
                m_astcDtc[j] = m_astcDtc[j + 1U];
            }
            m_u8DtcCount--;
        } else {
            i++;
        }
    }
    return 0U;
}

uint8_t AppUdsDtc_HandleSetting(uint8_t u8Type)
{
    switch (u8Type) {
        case 0x01U: m_u8DtcEnabled = 1U; break;   /* ON  */
        case 0x02U: m_u8DtcEnabled = 0U; break;   /* OFF */
        default:    return APP_NRC_SFNS;
    }
    return 0U;
}

uint8_t AppUdsDtc_HandleRead(const uint8_t *pu8Req, uint16_t u16Len,
                             uint8_t *pu8Out, uint16_t u16MaxLen,
                             uint16_t *pu16OutLen)
{
    uint8_t  u8Sub;
    uint8_t  u8Mask;
    uint8_t  i;
    uint16_t u16Pos = 0U;

    if (u16Len < 2U) {
        return APP_NRC_IMLOIF;
    }
    u8Sub = (uint8_t)(pu8Req[1] & 0x7FU);

    switch (u8Sub) {
        /* ---- 0x01 按状态掩码报告 DTC 数量 ----
         * 59 01 <availMask> <format> <countHi> <countLo> */
        case 0x01U: {
            uint16_t u16Cnt;

            if (u16Len < 3U) { return APP_NRC_IMLOIF; }
            if (u16MaxLen < 5U) { return APP_NRC_CNC; }

            u8Mask = pu8Req[2];
            u16Cnt = Dtc_CountByMask(u8Mask);

            pu8Out[0] = APP_DTC_AVAIL_MASK;   /* DTCStatusAvailabilityMask */
            pu8Out[1] = 0x01U;                /* DTCFormatIdentifier(ISO14229-1) */
            pu8Out[2] = (uint8_t)(u16Cnt >> 8U);
            pu8Out[3] = (uint8_t)(u16Cnt);
            *pu16OutLen = 4U;
            return 0U;
        }

        /* ---- 0x02 按状态掩码报告 DTC 列表 ----
         * 59 02 <availMask> [<DTC(3)><status>]... */
        case 0x02U:
            if (u16Len < 3U) { return APP_NRC_IMLOIF; }
            if (u16MaxLen < 1U) { return APP_NRC_CNC; }

            u8Mask      = pu8Req[2];
            pu8Out[0]   = APP_DTC_AVAIL_MASK;
            u16Pos      = 1U;
            for (i = 0U; i < m_u8DtcCount; i++) {
                if ((m_astcDtc[i].u8Status & u8Mask) != 0U) {
                    if (Dtc_EmitOne(&m_astcDtc[i], pu8Out, u16MaxLen, &u16Pos) == 0U) {
                        break;      /* 缓冲不足：返回已收集部分 */
                    }
                }
            }
            *pu16OutLen = u16Pos;
            return 0U;

        /* ---- 0x03 快照标识（本工程只有扩展数据，按 DTC+状态 返回）----
         * 59 03 [<DTC(3)><status>]... */
        case 0x03U:
            u16Pos = 0U;
            for (i = 0U; i < m_u8DtcCount; i++) {
                if (Dtc_EmitOne(&m_astcDtc[i], pu8Out, u16MaxLen, &u16Pos) == 0U) {
                    break;
                }
            }
            *pu16OutLen = u16Pos;
            return 0U;

        /* ---- 0x04 按 DTC 报告快照记录 ----
         * 请求：19 04 <DTC(3)> <recordNumber>
         * 响应：59 04 <DTC(3)><status> <recordNumber> <recordData[]>
         * 本工程无独立冻结帧，用扩展数据充当快照内容。 */
        case 0x04U: {
            app_uds_dtc_entry_t *pEntry;
            uint32_t u32Dtc;

            if (u16Len < 6U) { return APP_NRC_IMLOIF; }
            if (u16MaxLen < (uint16_t)(6U + APP_DTC_EXT_DATA_LEN)) {
                return APP_NRC_CNC;
            }
            u32Dtc = Dtc_ParseDtc(pu8Req, 2U);
            pEntry = Dtc_Find(u32Dtc);
            if (pEntry == NULL) { return APP_NRC_ROOR; }
            if (pu8Req[5] != 0x01U) { return APP_NRC_ROOR; }

            Dtc_EmitOne(pEntry, pu8Out, u16MaxLen, &u16Pos);   /* DTC + status */
            pu8Out[u16Pos] = 0x01U;                            /* recordNumber */
            u16Pos++;
            memcpy(&pu8Out[u16Pos], pEntry->au8ExtData, APP_DTC_EXT_DATA_LEN);
            u16Pos = (uint16_t)(u16Pos + APP_DTC_EXT_DATA_LEN);
            *pu16OutLen = u16Pos;
            return 0U;
        }

        /* ---- 0x06 按 DTC 报告扩展数据 ----
         * 请求：19 06 <DTC(3)> <recordNumber>
         * 响应：59 06 <DTC(3)><status> <recordNumber> <extDataRecord[]> */
        case 0x06U: {
            app_uds_dtc_entry_t *pEntry;
            uint32_t u32Dtc;
            uint16_t u16Occ;

            if (u16Len < 6U) { return APP_NRC_IMLOIF; }
            if (u16MaxLen < 12U) { return APP_NRC_CNC; }

            u32Dtc = Dtc_ParseDtc(pu8Req, 2U);
            pEntry = Dtc_Find(u32Dtc);
            if (pEntry == NULL) { return APP_NRC_ROOR; }
            /* 0xFF = 全部记录；0x01 = 第一条 */
            if ((pu8Req[5] != 0x01U) && (pu8Req[5] != 0xFFU)) { return APP_NRC_ROOR; }

            Dtc_EmitOne(pEntry, pu8Out, u16MaxLen, &u16Pos);   /* DTC + status */
            pu8Out[u16Pos] = 0x01U;                            /* recordNumber */
            u16Pos++;

            /* 扩展数据记录：occurCnt(2) + aging(1) + firstTick(2) + extData(4) */
            u16Occ = pEntry->u16Occur;
            pu8Out[u16Pos]      = (uint8_t)(u16Occ >> 8U);
            pu8Out[u16Pos + 1U] = (uint8_t)(u16Occ);
            pu8Out[u16Pos + 2U] = pEntry->u8Aging;
            pu8Out[u16Pos + 3U] = (uint8_t)(pEntry->u32FirstTick >> 8U);
            pu8Out[u16Pos + 4U] = (uint8_t)(pEntry->u32FirstTick);
            u16Pos = (uint16_t)(u16Pos + 5U);
            memcpy(&pu8Out[u16Pos], pEntry->au8ExtData, APP_DTC_EXT_DATA_LEN);
            u16Pos = (uint16_t)(u16Pos + APP_DTC_EXT_DATA_LEN);
            *pu16OutLen = u16Pos;
            return 0U;
        }

        /* ---- 0x0A 报告所有支持的 DTC ----
         * 59 0A <availMask> [<DTC(3)><status>]... */
        case 0x0AU:
            if (u16MaxLen < 1U) { return APP_NRC_CNC; }

            pu8Out[0] = APP_DTC_AVAIL_MASK;
            u16Pos    = 1U;
            for (i = 0U; i < m_u8DtcCount; i++) {
                if (Dtc_EmitOne(&m_astcDtc[i], pu8Out, u16MaxLen, &u16Pos) == 0U) {
                    break;
                }
            }
            *pu16OutLen = u16Pos;
            return 0U;

        /* ---- 0x14 报告 DTC 故障检测计数器 ----
         * 请求：19 14 <DTC(3)>
         * 响应：59 14 <DTC(3)><status> <FDC> */
        case 0x14U: {
            app_uds_dtc_entry_t *pEntry;
            uint32_t u32Dtc;

            if (u16Len < 5U) { return APP_NRC_IMLOIF; }
            if (u16MaxLen < 5U) { return APP_NRC_CNC; }

            u32Dtc = Dtc_ParseDtc(pu8Req, 2U);
            pEntry = Dtc_Find(u32Dtc);
            if (pEntry == NULL) { return APP_NRC_ROOR; }

            Dtc_EmitOne(pEntry, pu8Out, u16MaxLen, &u16Pos);
            /* FDC：0x00~0x7F 未失败，0x80~0xFF 已失败（见文件头说明） */
            if ((pEntry->u8Status & APP_DTC_STATUS_TEST_FAILED) != 0U) {
                uint8_t u8Cnt = (pEntry->u16Occur > 0x7FU)
                              ? 0x7FU : (uint8_t)pEntry->u16Occur;
                pu8Out[u16Pos] = (uint8_t)(0x80U | u8Cnt);
            } else {
                pu8Out[u16Pos] = 0x00U;
            }
            u16Pos++;
            *pu16OutLen = u16Pos;
            return 0U;
        }

        /* ---- 其余子功能未实现 ---- */
        default:
            return APP_NRC_SFNS;
    }
}

/*******************************************************************************
 * 文件结束
 ******************************************************************************/
