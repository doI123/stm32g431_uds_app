/**
 *******************************************************************************
 * @file  app_uds_param.c
 * @brief 可读写参数区实现（DID 0xF900 ~ 0xF93F）
 *
 *   与上位机 `params.items` 一一对应：DID、数据类型、scale、min/max、
 *   可写性全部按上位机配置实现。详见 app_uds_param.h 的对照表。
 *
 *   设计要点：
 *     1. **RAM 里统一存"物理值"（float）**，读写时按该 DID 的类型转换成
 *        线上字节。好处是"类型 / scale / 量程"三件事只在描述表里写一次，
 *        协议层与应用层都不必知道某条参数是 u16 还是 f32。
 *     2. **逐条量程**（min/max 是物理值），越界回 NRC 0x31。
 *     3. **长度按类型校验**：u16/i16 = 2 字节，f32 = 4 字节。长度不符回
 *        NRC 0x13，且不得先写入再报错。
 *     4. 参数只在 RAM 中保存（本工程无掉电存储）。要持久化就实现
 *        AppUdsParam_Save() 的弱函数钩子。
 *
 *   ⚠️ 本文件与上位机 config 是一对必须同步的清单。改任一侧都要改另一侧，
 *      否则会出现"写了没效果"或"读出来是垃圾"这类极难定位的问题。
 *******************************************************************************
 * Copyright (C) 2026, all rights reserved.
 *
 * This software component is licensed under BSD 3-Clause license.
 *******************************************************************************/
#include <string.h>
#include "app_uds_param.h"
#include "app_uds.h"

/* ============================== 参数描述表 ==============================
 * 与上位机 config 的 params.items 逐项对应，**顺序即遍历顺序**。
 * f32Min/f32Max/f32Default 都是物理值（应用 f32Scale 之后）。
 *
 *   DID     类型              可写 scale  物理下限  物理上限   默认值   名称 / 单位
 */
static const app_uds_param_desc_t s_astcParamDesc[] = {
    /* ---------------- 电机参数 (F900-F90F) ---------------- */
    { 0xF900U, APP_PARAM_T_U16, 1U,  0.1f,    0.0f,   100.0f,    5.0f },  /* 额定电流    A     */
    { 0xF901U, APP_PARAM_T_U16, 1U,  1.0f,    0.0f,  6000.0f, 3000.0f },  /* 额定转速    rpm   */
    { 0xF902U, APP_PARAM_T_U16, 1U,  1.0f,    1.0f,    64.0f,    7.0f },  /* 极对数      —     */
    { 0xF903U, APP_PARAM_T_F32, 0U,  1.0f,    0.0f,  1000.0f,    0.0f },  /* 反电势常数  V/krpm 只读 */

    /* ---------------- 电流环参数 (F910-F91F) ---------------- */
    { 0xF910U, APP_PARAM_T_F32, 1U,  1.0f,    0.0f,   100.0f,    1.0f },  /* 电流环 Kp   —     */
    { 0xF911U, APP_PARAM_T_F32, 1U,  1.0f,    0.0f,   100.0f,    0.1f },  /* 电流环 Ki   —     */
    { 0xF912U, APP_PARAM_T_U16, 1U,  1.0f,  100.0f,  5000.0f, 2000.0f },  /* 电流环带宽  Hz    */

    /* ---------------- 速度环参数 (F920-F92F) ---------------- */
    { 0xF920U, APP_PARAM_T_F32, 1U,  1.0f,    0.0f,   100.0f,    0.5f },  /* 速度环 Kp   —     */
    { 0xF921U, APP_PARAM_T_F32, 1U,  1.0f,    0.0f,   100.0f,   0.05f },  /* 速度环 Ki   —     */
    { 0xF922U, APP_PARAM_T_U16, 1U,  1.0f,    0.0f,  6000.0f, 3000.0f },  /* 速度限幅    rpm   */

    /* ---------------- 保护参数 (F930-F93F) ---------------- */
    { 0xF930U, APP_PARAM_T_U16, 1U,  0.1f,    1.0f,   200.0f,   20.0f },  /* 过流阈值    A     */
    { 0xF931U, APP_PARAM_T_U16, 1U,  0.1f,   10.0f,   600.0f,   60.0f },  /* 过压阈值    V     */
    { 0xF932U, APP_PARAM_T_U16, 1U,  0.1f,   10.0f,   600.0f,   40.0f },  /* 欠压阈值    V     */
    { 0xF933U, APP_PARAM_T_I16, 1U,  1.0f,    0.0f,   150.0f,  100.0f },  /* 过温阈值    ℃     */
};

#define APP_PARAM_TABLE_LEN \
    (sizeof(s_astcParamDesc) / sizeof(s_astcParamDesc[0]))

/* 表长度必须与 APP_PARAM_COUNT 一致（改宏忘改表会在这里报错） */
#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
_Static_assert(APP_PARAM_TABLE_LEN >= 1U, "参数描述表不能为空");
#endif

/* ============================== RAM 值（物理值） ============================== */
static float    m_af32Val[APP_PARAM_TABLE_LEN];
static uint32_t m_u32ParamChangeCnt = 0U;

/* ============================== 内部函数 ============================== */

/**
 * @brief  按 DID 查表并返回下标。
 * @retval int32_t >=0 = 下标；-1 = 未命中
 */
static int32_t Param_FindIndex(uint16_t u16Did)
{
    uint32_t i;

    for (i = 0U; i < APP_PARAM_TABLE_LEN; i++) {
        if (s_astcParamDesc[i].u16Did == u16Did) {
            return (int32_t)i;
        }
    }
    return -1;
}

/**
 * @brief  四舍五入取整（不依赖 libm 的 roundf，避免引入额外代码）。
 */
static int32_t Param_RoundF(float f32Val)
{
    return (int32_t)((f32Val >= 0.0f) ? (f32Val + 0.5f) : (f32Val - 0.5f));
}

/**
 * @brief  物理值 -> 原始整数值（scale 的反变换）。
 */
static int32_t Param_PhysToRaw(float f32Phys, float f32Scale)
{
    return Param_RoundF(f32Phys / f32Scale);
}

/**
 * @brief  原始整数值 -> 物理值。
 */
static float Param_RawToPhys(int32_t s32Raw, float f32Scale)
{
    return ((float)s32Raw * f32Scale);
}

/**
 * @brief  大端写入 / 读取 IEEE754 单精度。
 *         用 memcpy 做类型双关是安全的（位模式原样搬运，无对齐要求）。
 */
static void Param_PutF32(uint8_t *pu8Out, float f32Val)
{
    uint32_t u32Bits;

    memcpy(&u32Bits, &f32Val, sizeof(u32Bits));
    pu8Out[0] = (uint8_t)(u32Bits >> 24U);
    pu8Out[1] = (uint8_t)(u32Bits >> 16U);
    pu8Out[2] = (uint8_t)(u32Bits >> 8U);
    pu8Out[3] = (uint8_t)(u32Bits);
}

static float Param_GetF32(const uint8_t *pu8In)
{
    uint32_t u32Bits;
    float    f32Val;

    u32Bits = ((uint32_t)pu8In[0] << 24U) | ((uint32_t)pu8In[1] << 16U) |
              ((uint32_t)pu8In[2] << 8U)  |  (uint32_t)pu8In[3];
    memcpy(&f32Val, &u32Bits, sizeof(f32Val));
    return f32Val;
}

/**
 * @brief  物理值编码为线上字节（按类型）。
 * @retval uint16_t 写入字节数
 */
static uint16_t Param_Encode(const app_uds_param_desc_t *pDesc, float f32Phys,
                             uint8_t *pu8Out)
{
    int32_t  s32Raw = Param_PhysToRaw(f32Phys, pDesc->f32Scale);
    uint32_t u32Raw;

    switch (pDesc->u8Type) {
        case APP_PARAM_T_I16:
        case APP_PARAM_T_U16:
            u32Raw = (uint32_t)(uint16_t)(int16_t)s32Raw;
            pu8Out[0] = (uint8_t)(u32Raw >> 8U);
            pu8Out[1] = (uint8_t)(u32Raw);
            return 2U;

        case APP_PARAM_T_I32:
        case APP_PARAM_T_U32:
            u32Raw = (uint32_t)s32Raw;
            pu8Out[0] = (uint8_t)(u32Raw >> 24U);
            pu8Out[1] = (uint8_t)(u32Raw >> 16U);
            pu8Out[2] = (uint8_t)(u32Raw >> 8U);
            pu8Out[3] = (uint8_t)(u32Raw);
            return 4U;

        case APP_PARAM_T_F32:
        default:
            Param_PutF32(pu8Out, f32Phys);
            return 4U;
    }
}

/**
 * @brief  线上字节解码为物理值（按类型）。
 */
static float Param_Decode(const app_uds_param_desc_t *pDesc, const uint8_t *pu8In)
{
    int32_t  s32Raw;
    uint32_t u32Raw;

    switch (pDesc->u8Type) {
        case APP_PARAM_T_I16:
            u32Raw = ((uint32_t)pu8In[0] << 8U) | (uint32_t)pu8In[1];
            s32Raw = (int32_t)(int16_t)(uint16_t)u32Raw;
            break;

        case APP_PARAM_T_U16:
            u32Raw = ((uint32_t)pu8In[0] << 8U) | (uint32_t)pu8In[1];
            s32Raw = (int32_t)u32Raw;
            break;

        case APP_PARAM_T_I32:
        case APP_PARAM_T_U32:
            u32Raw = ((uint32_t)pu8In[0] << 24U) | ((uint32_t)pu8In[1] << 16U) |
                     ((uint32_t)pu8In[2] << 8U)  |  (uint32_t)pu8In[3];
            s32Raw = (int32_t)u32Raw;
            break;

        case APP_PARAM_T_F32:
        default:
            return Param_GetF32(pu8In);
    }

    return Param_RawToPhys(s32Raw, pDesc->f32Scale);
}

/* ============================== 对外实现 ============================== */

/* 持久化钩子：默认空操作（本工程无掉电存储）。
 * 接入 EEPROM 时在应用层实现同名函数覆盖即可。 */
__attribute__((weak)) uint8_t AppUdsParam_Save(void)
{
    return 0U;
}

void AppUdsParam_Init(void)
{
    uint32_t i;

    for (i = 0U; i < APP_PARAM_TABLE_LEN; i++) {
        m_af32Val[i] = s_astcParamDesc[i].f32Default;
    }
    m_u32ParamChangeCnt = 0U;
}

uint8_t AppUdsParam_Exists(uint16_t u16Did)
{
    return (Param_FindIndex(u16Did) >= 0) ? 1U : 0U;
}

const app_uds_param_desc_t *AppUdsParam_GetDesc(uint16_t u16Did)
{
    int32_t i32Idx = Param_FindIndex(u16Did);

    return (i32Idx < 0) ? NULL : &s_astcParamDesc[i32Idx];
}

uint8_t AppUdsParam_GetLen(uint16_t u16Did)
{
    const app_uds_param_desc_t *pDesc = AppUdsParam_GetDesc(u16Did);

    if (pDesc == NULL) {
        return 0U;
    }
    if ((pDesc->u8Type == APP_PARAM_T_I16) || (pDesc->u8Type == APP_PARAM_T_U16)) {
        return 2U;
    }
    return 4U;
}

uint32_t AppUdsParam_GetCount(void)
{
    return (uint32_t)APP_PARAM_TABLE_LEN;
}

const app_uds_param_desc_t *AppUdsParam_GetDescByIndex(uint32_t u32Idx)
{
    if (u32Idx >= APP_PARAM_TABLE_LEN) {
        return NULL;
    }
    return &s_astcParamDesc[u32Idx];
}

uint8_t AppUdsParam_GetPhysical(uint16_t u16Did, float *pf32Val)
{
    int32_t i32Idx = Param_FindIndex(u16Did);

    if (i32Idx < 0) {
        return APP_NRC_ROOR;
    }
    if (pf32Val != NULL) {
        *pf32Val = m_af32Val[i32Idx];
    }
    return 0U;
}

uint8_t AppUdsParam_SetPhysical(uint16_t u16Did, float f32Val)
{
    int32_t i32Idx = Param_FindIndex(u16Did);
    const app_uds_param_desc_t *pDesc;

    if (i32Idx < 0) {
        return APP_NRC_ROOR;
    }
    pDesc = &s_astcParamDesc[i32Idx];

    if (pDesc->u8Writable == 0U) {
        return APP_NRC_ROOR;                    /* 只读参数 */
    }
    /* 逐条量程：上位机配置称其为"防止手误写坏标定的第一道闸门" */
    if ((f32Val < pDesc->f32Min) || (f32Val > pDesc->f32Max)) {
        return APP_NRC_ROOR;
    }

    if (m_af32Val[i32Idx] != f32Val) {
        m_af32Val[i32Idx] = f32Val;
        m_u32ParamChangeCnt++;
    }
    return 0U;
}

uint8_t AppUdsParam_Read(uint16_t u16Did, uint8_t *pu8Out,
                         uint16_t u16MaxLen, uint16_t *pu16OutLen)
{
    int32_t  i32Idx = Param_FindIndex(u16Did);
    uint16_t u16Len;

    if (i32Idx < 0) {
        return APP_NRC_ROOR;
    }
    u16Len = AppUdsParam_GetLen(u16Did);
    if (u16MaxLen < u16Len) {
        return APP_NRC_CNC;
    }

    *pu16OutLen = Param_Encode(&s_astcParamDesc[i32Idx], m_af32Val[i32Idx], pu8Out);
    return 0U;
}

uint8_t AppUdsParam_Write(uint16_t u16Did, const uint8_t *pu8In, uint16_t u16Len)
{
    int32_t i32Idx = Param_FindIndex(u16Did);
    const app_uds_param_desc_t *pDesc;
    float f32Val;

    if (i32Idx < 0) {
        return APP_NRC_ROOR;
    }
    pDesc = &s_astcParamDesc[i32Idx];

    if (pDesc->u8Writable == 0U) {
        return APP_NRC_ROOR;
    }
    /* 长度按类型校验：不符直接拒绝，绝不落地 */
    if (u16Len != (uint16_t)AppUdsParam_GetLen(u16Did)) {
        return APP_NRC_IMLOIF;
    }

    f32Val = Param_Decode(pDesc, pu8In);
    if ((f32Val < pDesc->f32Min) || (f32Val > pDesc->f32Max)) {
        return APP_NRC_ROOR;
    }

    if (m_af32Val[i32Idx] != f32Val) {
        m_af32Val[i32Idx] = f32Val;
        m_u32ParamChangeCnt++;
    }
    return 0U;
}

uint32_t AppUdsParam_GetChangeCounter(void)
{
    return m_u32ParamChangeCnt;
}

uint8_t AppUdsParam_ResetAll(void)
{
    AppUdsParam_Init();
    m_u32ParamChangeCnt++;
    return 0U;
}

uint8_t AppUdsParam_CheckAll(void)
{
    uint32_t i;
    uint8_t  u8Bad = 0U;

    for (i = 0U; i < APP_PARAM_TABLE_LEN; i++) {
        if ((m_af32Val[i] < s_astcParamDesc[i].f32Min) ||
            (m_af32Val[i] > s_astcParamDesc[i].f32Max)) {
            m_af32Val[i] = s_astcParamDesc[i].f32Default;   /* 钳回默认值 */
            u8Bad = 1U;
        }
    }
    if (u8Bad != 0U) {
        m_u32ParamChangeCnt++;
    }
    return u8Bad;
}

/*******************************************************************************
 * 文件结束
 ******************************************************************************/
