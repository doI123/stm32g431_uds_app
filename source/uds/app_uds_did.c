/**
 *******************************************************************************
 * @file  app_uds_did.c
 * @brief DID 业务实现（0x22 / 0x2E 的数据源与数据落地）
 *
 *   本文件是"应用层 UDS 诊断"里最常改的地方：
 *     - 加/改一条 DID -> 改 s_astcDidTable[] 一行 + 写一个回调
 *     - 接入 FOC 数据 -> 在应用层实现 AppUdsData_Get* 覆盖弱函数
 *******************************************************************************
 * Copyright (C) 2026, all rights reserved.
 *
 * This software component is licensed under BSD 3-Clause license.
 *******************************************************************************/
#include <string.h>
#include "app_uds_did.h"
#include "app_uds_param.h"
#include "app_uds.h"
#include "mcan.h"
#include "board.h"
#include "version.h"
#include "can_cfg.h"

/* ======================= FOC 接入点（弱函数默认实现） ======================= */
__attribute__((weak)) uint16_t AppUdsData_GetMotorSpeedRpm(void)     { return 0U; }
__attribute__((weak)) int8_t   AppUdsData_GetMotorTemp(void)         { return 0; }
__attribute__((weak)) uint16_t AppUdsData_GetBusVoltageMv(void)      { return 0U; }
__attribute__((weak)) void     AppUdsData_GetPhaseCurrentMa(int16_t *pi16Iabc)
{
    if (pi16Iabc != NULL) { pi16Iabc[0] = 0; pi16Iabc[1] = 0; pi16Iabc[2] = 0; }
}
__attribute__((weak)) uint32_t AppUdsData_GetMotorStateWord(void)    { return 0U; }
__attribute__((weak)) uint32_t AppUdsData_GetMotorFaultWord(void)    { return 0U; }
__attribute__((weak)) uint16_t AppUdsData_GetMotorAngleDeciDeg(void) { return 0U; }

/* ============================ 可写配置的本地镜像 ============================ */
static uint8_t m_u8NodeAddr = 0x01U;   /* 节点地址（本上电周期内有效） */

/* ============================== 工具函数 ============================== */

/**
 * @brief  把 0..255 的十进制整数写入缓冲区（无前导零）。
 * @param  [in]  u8Val 待转换值
 * @param  [out] pcOut 输出缓冲（>= 3 字节）
 * @retval uint8_t 写入字符数
 */
static uint8_t Did_U8ToDec(uint8_t u8Val, char *pcOut)
{
    char    acTmp[3];
    uint8_t u8Len = 0U;
    uint8_t i;

    if (u8Val == 0U) { pcOut[0] = '0'; return 1U; }
    while (u8Val > 0U) {
        acTmp[u8Len] = (char)('0' + (u8Val % 10U));
        u8Len++;
        u8Val = (uint8_t)(u8Val / 10U);
    }
    for (i = 0U; i < u8Len; i++) { pcOut[i] = acTmp[u8Len - 1U - i]; }
    return u8Len;
}

/**
 * @brief  取 fw_info（无效时返回 NULL，由调用方回 NRC 0x31）。
 */
static const fw_info_t *Did_GetFwInfo(void)
{
    const fw_info_t *pInfo = FwInfo_Get();

    return (FwInfo_IsValid(pInfo) != 0U) ? pInfo : NULL;
}

/**
 * @brief  拷贝定长/变长块到输出缓冲。
 * @retval uint8_t 0 = 成功；非 0 = NRC
 */
static uint8_t Did_Copy(const void *pvSrc, uint16_t u16Len,
                        uint8_t *pu8Out, uint16_t u16MaxLen, uint16_t *pu16OutLen)
{
    if (pvSrc == NULL) { return APP_NRC_ROOR; }
    if (u16Len > u16MaxLen) { return APP_NRC_CNC; }
    memcpy(pu8Out, pvSrc, u16Len);
    *pu16OutLen = u16Len;
    return 0U;
}

/**
 * @brief  大端写入 uint16 / uint32。
 */
static void Did_PutU16(uint8_t *pu8Out, uint16_t u16Val)
{
    pu8Out[0] = (uint8_t)(u16Val >> 8U);
    pu8Out[1] = (uint8_t)(u16Val);
}

static void Did_PutU32(uint8_t *pu8Out, uint32_t u32Val)
{
    pu8Out[0] = (uint8_t)(u32Val >> 24U);
    pu8Out[1] = (uint8_t)(u32Val >> 16U);
    pu8Out[2] = (uint8_t)(u32Val >> 8U);
    pu8Out[3] = (uint8_t)(u32Val);
}

/* ============================== 读回调 ============================== */

static uint8_t Did_ReadAppSwIdent(uint8_t *pu8Out, uint16_t u16MaxLen, uint16_t *pu16Len)
{
    const fw_info_t *pInfo = Did_GetFwInfo();

    if (pInfo == NULL) { return APP_NRC_ROOR; }
    return Did_Copy(pInfo->acSwVersion, (uint16_t)strlen(pInfo->acSwVersion),
                    pu8Out, u16MaxLen, pu16Len);
}

static uint8_t Did_ReadAppFingerprint(uint8_t *pu8Out, uint16_t u16MaxLen, uint16_t *pu16Len)
{
    const fw_info_t *pInfo = Did_GetFwInfo();
    uint8_t au8Crc[4];

    if (pInfo == NULL) { return APP_NRC_ROOR; }
    Did_PutU32(au8Crc, pInfo->u32ImageCrc32);
    return Did_Copy(au8Crc, 4U, pu8Out, u16MaxLen, pu16Len);
}

static uint8_t Did_ReadActiveSession(uint8_t *pu8Out, uint16_t u16MaxLen, uint16_t *pu16Len)
{
    uint8_t u8Sess = AppUds_GetSession();

    return Did_Copy(&u8Sess, 1U, pu8Out, u16MaxLen, pu16Len);
}

static uint8_t Did_ReadHwVersion(uint8_t *pu8Out, uint16_t u16MaxLen, uint16_t *pu16Len)
{
    const fw_info_t *pInfo = Did_GetFwInfo();

    if (pInfo == NULL) { return APP_NRC_ROOR; }
    return Did_Copy(pInfo->acHwVersion, (uint16_t)strlen(pInfo->acHwVersion),
                    pu8Out, u16MaxLen, pu16Len);
}

static uint8_t Did_ReadSwVersion(uint8_t *pu8Out, uint16_t u16MaxLen, uint16_t *pu16Len)
{
    const fw_info_t *pInfo = Did_GetFwInfo();
    uint8_t u8Len = 0U;

    if (pInfo == NULL) { return APP_NRC_ROOR; }
    if (u16MaxLen < 12U) { return APP_NRC_CNC; }

    u8Len = (uint8_t)(u8Len + Did_U8ToDec(pInfo->u8VerMajor, (char *)&pu8Out[u8Len]));
    pu8Out[u8Len] = '.';
    u8Len++;
    u8Len = (uint8_t)(u8Len + Did_U8ToDec(pInfo->u8VerMinor, (char *)&pu8Out[u8Len]));
    pu8Out[u8Len] = '.';
    u8Len++;
    u8Len = (uint8_t)(u8Len + Did_U8ToDec(pInfo->u8VerPatch, (char *)&pu8Out[u8Len]));

    *pu16Len = u8Len;
    return 0U;
}

static uint8_t Did_ReadSystemName(uint8_t *pu8Out, uint16_t u16MaxLen, uint16_t *pu16Len)
{
    static const char acName[8] = { 'F', 'O', 'C', '_', 'G', '4', '3', '1' };

    return Did_Copy(acName, 8U, pu8Out, u16MaxLen, pu16Len);
}

static uint8_t Did_ReadProgrammingDate(uint8_t *pu8Out, uint16_t u16MaxLen, uint16_t *pu16Len)
{
    const fw_info_t *pInfo = Did_GetFwInfo();

    if (pInfo == NULL) { return APP_NRC_ROOR; }
    if (u16MaxLen < 19U) { return APP_NRC_CNC; }
    memcpy(&pu8Out[0], pInfo->acBuildDate, 10U);
    pu8Out[10] = ' ';
    memcpy(&pu8Out[11], pInfo->acBuildTime, 8U);
    *pu16Len = 19U;
    return 0U;
}

static uint8_t Did_ReadSerialNumber(uint8_t *pu8Out, uint16_t u16MaxLen, uint16_t *pu16Len)
{
    /* 本工程未在 fw_info 中存序列号，用 git 短散列做等价唯一标识（8 字符 hex） */
    const fw_info_t *pInfo = Did_GetFwInfo();
    static const char acHex[] = "0123456789ABCDEF";
    uint8_t i;

    if (pInfo == NULL) { return APP_NRC_ROOR; }
    if (u16MaxLen < 8U) { return APP_NRC_CNC; }

    for (i = 0U; i < 8U; i++) {
        pu8Out[i] = (uint8_t)acHex[(pInfo->u32GitHash >> (28U - (i * 4U))) & 0x0FU];
    }
    *pu16Len = 8U;
    return 0U;
}

static uint8_t Did_ReadCanBaud(uint8_t *pu8Out, uint16_t u16MaxLen, uint16_t *pu16Len)
{
    uint8_t u8Code = CanCfg_GetBaudCode();

    return Did_Copy(&u8Code, 1U, pu8Out, u16MaxLen, pu16Len);
}

static uint8_t Did_ReadNodeAddr(uint8_t *pu8Out, uint16_t u16MaxLen, uint16_t *pu16Len)
{
    return Did_Copy(&m_u8NodeAddr, 1U, pu8Out, u16MaxLen, pu16Len);
}

static uint8_t Did_ReadMotorSpeed(uint8_t *pu8Out, uint16_t u16MaxLen, uint16_t *pu16Len)
{
    uint8_t au8Buf[2];

    Did_PutU16(au8Buf, AppUdsData_GetMotorSpeedRpm());
    return Did_Copy(au8Buf, 2U, pu8Out, u16MaxLen, pu16Len);
}

static uint8_t Did_ReadMotorTemp(uint8_t *pu8Out, uint16_t u16MaxLen, uint16_t *pu16Len)
{
    int8_t i8Val = AppUdsData_GetMotorTemp();

    return Did_Copy(&i8Val, 1U, pu8Out, u16MaxLen, pu16Len);
}

static uint8_t Did_ReadBusVoltage(uint8_t *pu8Out, uint16_t u16MaxLen, uint16_t *pu16Len)
{
    uint8_t au8Buf[2];

    Did_PutU16(au8Buf, AppUdsData_GetBusVoltageMv());
    return Did_Copy(au8Buf, 2U, pu8Out, u16MaxLen, pu16Len);
}

static uint8_t Did_ReadPhaseCurrent(uint8_t *pu8Out, uint16_t u16MaxLen, uint16_t *pu16Len)
{
    int16_t ai16Iabc[3];
    uint8_t au8Buf[6];
    uint8_t i;

    if (u16MaxLen < 6U) { return APP_NRC_CNC; }
    AppUdsData_GetPhaseCurrentMa(ai16Iabc);
    for (i = 0U; i < 3U; i++) {
        Did_PutU16(&au8Buf[i * 2U], (uint16_t)ai16Iabc[i]);
    }
    return Did_Copy(au8Buf, 6U, pu8Out, u16MaxLen, pu16Len);
}

static uint8_t Did_ReadMotorState(uint8_t *pu8Out, uint16_t u16MaxLen, uint16_t *pu16Len)
{
    uint8_t au8Buf[4];

    Did_PutU32(au8Buf, AppUdsData_GetMotorStateWord());
    return Did_Copy(au8Buf, 4U, pu8Out, u16MaxLen, pu16Len);
}

static uint8_t Did_ReadMotorFault(uint8_t *pu8Out, uint16_t u16MaxLen, uint16_t *pu16Len)
{
    uint8_t au8Buf[4];

    Did_PutU32(au8Buf, AppUdsData_GetMotorFaultWord());
    return Did_Copy(au8Buf, 4U, pu8Out, u16MaxLen, pu16Len);
}

static uint8_t Did_ReadMotorAngle(uint8_t *pu8Out, uint16_t u16MaxLen, uint16_t *pu16Len)
{
    uint8_t au8Buf[2];

    Did_PutU16(au8Buf, AppUdsData_GetMotorAngleDeciDeg());
    return Did_Copy(au8Buf, 2U, pu8Out, u16MaxLen, pu16Len);
}

/* ============================== 写回调 ============================== */

static uint8_t Did_WriteCanBaud(const uint8_t *pu8In, uint16_t u16Len)
{
    uint32_t u32OldBaud;

    if (u16Len != 1U) { return APP_NRC_IMLOIF; }

    /* 必须在 CanCfg_SetBaudCode() **之前**读当前速率：
     * 该函数会顺带更新驱动里记录的速率，之后再比较永远相同。 */
    u32OldBaud = Mcan_GetBaudrate();

    if (CanCfg_SetBaudCode(pu8In[0]) != 0) { return APP_NRC_ROOR; }

    /* 速率真的变了才要求主循环做硬件切换（先发应答，再切硬件） */
    if (Mcan_GetBaudrate() != u32OldBaud) {
        AppUds_RequestBaudSwitch();
    }
    return 0U;
}

static uint8_t Did_WriteNodeAddr(const uint8_t *pu8In, uint16_t u16Len)
{
    if (u16Len != 1U) { return APP_NRC_IMLOIF; }
    if (pu8In[0] == 0U) { return APP_NRC_ROOR; }   /* 0 非法 */
    m_u8NodeAddr = pu8In[0];
    return 0U;
}

/* ============================== DID 表 ==============================
 * 新增 DID：在这里加一行即可。u16Len = 固定长度，0 = 变长。
 * 注意：0xF900~0xF930 由 app_uds_param.c 统一处理，不要写进本表。
 */
static const app_uds_did_t s_astcDidTable[] = {
    /* DID                          会话掩码      安全要求                 长度  读                      写 */
    { APP_DID_APP_SW_IDENT,         APP_SESS_ANY, APP_SEC_NONE,            0U,   Did_ReadAppSwIdent,     NULL },
    { APP_DID_APP_SW_FINGERPRINT,   APP_SESS_ANY, APP_SEC_NONE,            4U,   Did_ReadAppFingerprint, NULL },
    { APP_DID_ACTIVE_SESSION,       APP_SESS_ANY, APP_SEC_NONE,            1U,   Did_ReadActiveSession,  NULL },
    { APP_DID_SERIAL_NUMBER,        APP_SESS_ANY, APP_SEC_NONE,            8U,   Did_ReadSerialNumber,   NULL },
    { APP_DID_HW_VERSION,           APP_SESS_ANY, APP_SEC_NONE,            0U,   Did_ReadHwVersion,      NULL },
    { APP_DID_SW_VERSION,           APP_SESS_ANY, APP_SEC_NONE,            0U,   Did_ReadSwVersion,      NULL },
    { APP_DID_SYSTEM_NAME,          APP_SESS_ANY, APP_SEC_NONE,            8U,   Did_ReadSystemName,     NULL },
    { APP_DID_PROGRAMMING_DATE,     APP_SESS_ANY, APP_SEC_NONE,           19U,   Did_ReadProgrammingDate, NULL },

    /* 配置类：读不需要解锁，写必须解锁 */
    { APP_DID_CAN_BAUD,             APP_SESS_ANY, APP_SEC_WRITE_UNLOCKED,  1U,   Did_ReadCanBaud,        Did_WriteCanBaud },
    { APP_DID_NODE_ADDR,            APP_SESS_ANY, APP_SEC_WRITE_UNLOCKED,  1U,   Did_ReadNodeAddr,       Did_WriteNodeAddr },

    /* 实时运行数据（只读） */
    { APP_DID_MOTOR_SPEED,          APP_SESS_ANY, APP_SEC_NONE,            2U,   Did_ReadMotorSpeed,     NULL },
    { APP_DID_MOTOR_TEMP,           APP_SESS_ANY, APP_SEC_NONE,            1U,   Did_ReadMotorTemp,      NULL },
    { APP_DID_BUS_VOLTAGE,          APP_SESS_ANY, APP_SEC_NONE,            2U,   Did_ReadBusVoltage,     NULL },
    { APP_DID_PHASE_CURRENT,        APP_SESS_ANY, APP_SEC_NONE,            6U,   Did_ReadPhaseCurrent,   NULL },
    { APP_DID_MOTOR_STATE,          APP_SESS_ANY, APP_SEC_NONE,            4U,   Did_ReadMotorState,     NULL },
    { APP_DID_MOTOR_FAULT,          APP_SESS_ANY, APP_SEC_NONE,            4U,   Did_ReadMotorFault,     NULL },
    { APP_DID_MOTOR_ANGLE,          APP_SESS_ANY, APP_SEC_NONE,            2U,   Did_ReadMotorAngle,     NULL },
};

#define APP_DID_TABLE_LEN \
    (sizeof(s_astcDidTable) / sizeof(s_astcDidTable[0]))

const app_uds_did_t *AppUdsDid_Find(uint16_t u16Did)
{
    uint32_t i;

    for (i = 0U; i < APP_DID_TABLE_LEN; i++) {
        if (s_astcDidTable[i].u16Did == u16Did) {
            return &s_astcDidTable[i];
        }
    }
    return NULL;
}

/**
 * @brief  判断 DID 是否属于可读写参数区。
 *
 *   直接用参数表的**实际命中**判断，而不是判断"是否落在 F900~F93F 区间" ——
 *   上位机配置的 did_hi 是 0xFA00，区间内有大量未定义的 DID，那些必须回
 *   NRC 0x31 而不是被当成参数处理。
 */
uint8_t AppUdsDid_IsParamRange(uint16_t u16Did)
{
    return AppUdsParam_Exists(u16Did);
}

/*******************************************************************************
 * 文件结束
 ******************************************************************************/
