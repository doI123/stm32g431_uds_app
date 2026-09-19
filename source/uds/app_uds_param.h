/**
 *******************************************************************************
 * @file  app_uds_param.h
 * @brief 可读写参数区（DID 0xF900 ~ 0xF93F）—— 类型/量程/单位与上位机对齐
 *
 *   本模块**按上位机 `params.items` 的定义实现**，可作为参数区的唯一真值
 *   来源使用。上位机配置摘录（改动时两边必须同步）：
 *
 *     ┌──────┬────────────┬──────┬───────┬────────┬──────────────┬──────┐
 *     │ DID  │ 名称       │ 类型 │ scale │ 单位   │ 允许范围     │ 可写 │
 *     ├──────┼────────────┼──────┼───────┼────────┼──────────────┼──────┤
 *     │ F900 │ 额定电流   │ u16  │  0.1  │ A      │   0.0~100.0  │  ✔   │
 *     │ F901 │ 额定转速   │ u16  │  1.0  │ rpm    │   0.0~6000.0 │  ✔   │
 *     │ F902 │ 极对数     │ u16  │  1.0  │ —      │   1.0~64.0   │  ✔   │
 *     │ F903 │ 反电势常数 │ f32  │  1.0  │ V/krpm │   —          │  ✘   │
 *     │ F910 │ 电流环 Kp  │ f32  │  1.0  │ —      │   0.0~100.0  │  ✔   │
 *     │ F911 │ 电流环 Ki  │ f32  │  1.0  │ —      │   0.0~100.0  │  ✔   │
 *     │ F912 │ 电流环带宽 │ u16  │  1.0  │ Hz     │ 100.0~5000.0 │  ✔   │
 *     │ F920 │ 速度环 Kp  │ f32  │  1.0  │ —      │   0.0~100.0  │  ✔   │
 *     │ F921 │ 速度环 Ki  │ f32  │  1.0  │ —      │   0.0~100.0  │  ✔   │
 *     │ F922 │ 速度限幅   │ u16  │  1.0  │ rpm    │   0.0~6000.0 │  ✔   │
 *     │ F930 │ 过流阈值   │ u16  │  0.1  │ A      │   1.0~200.0  │  ✔   │
 *     │ F931 │ 过压阈值   │ u16  │  0.1  │ V      │  10.0~600.0  │  ✔   │
 *     │ F932 │ 欠压阈值   │ u16  │  0.1  │ V      │  10.0~600.0  │  ✔   │
 *     │ F933 │ 过温阈值   │ i16  │  1.0  │ ℃      │   0.0~150.0  │  ✔   │
 *     └──────┴────────────┴──────┴───────┴────────┴──────────────┴──────┘
 *
 *   ⚠️ 注意 F903/F910/F911/F920/F921 是 **f32（4 字节 IEEE754 大端）**，
 *      其余是 2 字节整数。长度校验按类型进行，写错长度回 NRC 0x13。
 *
 *   ⚠️ `min`/`max` 是**物理值**（应用 scale 之后）。上位机配置里明确写着
 *      "这是防止手误写坏标定的第一道闸门"，因此每条参数有独立量程，
 *      不使用统一量程。越界一律回 NRC 0x31。
 *
 *   报文示例：
 *     读 22 F9 00             -> 62 F9 00 00 32          (50 -> 5.0 A)
 *     写 2E F9 00 00 64       -> 6E F9 00                (100 -> 10.0 A)
 *     写 2E F9 00 03 E8       -> 7F 2E 31                (1000 -> 100.0 A 越界)
 *     读 22 F9 10             -> 62 F9 10 3F 80 00 00    (1.0 的 f32 位模式)
 *     写 2E F9 10 41 20 00 00 -> 6E F9 10                (10.0)
 *******************************************************************************
 * Copyright (C) 2026, all rights reserved.
 *
 * This software component is licensed under BSD 3-Clause license.
 *******************************************************************************/
#ifndef __APP_UDS_PARAM_H__
#define __APP_UDS_PARAM_H__

#include <stdint.h>
#include "app_uds_cfg.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 参数区 DID 范围（与上位机 params.did_lo / did_hi 对应）。
 * 上位机写的是 did_hi = 0xFA00，但实际只定义到 F933；
 * 未定义的 DID 一律回 NRC 0x31，不影响上位机工作。 */
#define APP_PARAM_DID_BASE          (0xF900U)
#define APP_PARAM_DID_END           (0xF93FU)

/* 参数数据类型（决定线上字节数与解释方式） */
#define APP_PARAM_T_I16             (0U)   /* int16   2 字节大端补码 */
#define APP_PARAM_T_U16             (1U)   /* uint16  2 字节大端     */
#define APP_PARAM_T_I32             (2U)   /* int32   4 字节大端补码 */
#define APP_PARAM_T_U32             (3U)   /* uint32  4 字节大端     */
#define APP_PARAM_T_F32             (4U)   /* float   4 字节 IEEE754 */

/**
 * @brief  参数静态描述（const，放 Flash）。
 *
 *   f32Min/f32Max/f32Default 都是**物理值**（应用 f32Scale 之后）。
 *   不存参数名 —— 名字是上位机的展示需求，ECU 侧不需要，省 Flash。
 */
typedef struct {
    uint16_t u16Did;        /* DID 编号                          */
    uint8_t  u8Type;        /* APP_PARAM_T_*                     */
    uint8_t  u8Writable;    /* 1 = 可被 0x2E 写入                */
    float    f32Scale;      /* 物理值 = 原始值 × scale           */
    float    f32Min;        /* 允许的物理下限                    */
    float    f32Max;        /* 允许的物理上限                    */
    float    f32Default;    /* 出厂默认物理值                    */
} app_uds_param_desc_t;

/* ------------------------------ 生命周期 ------------------------------ */

/**
 * @brief  初始化参数区：全部恢复出厂默认值，变更计数清零。
 *         由 AppUds_Init() 调用。
 */
void AppUdsParam_Init(void);

/* ------------------------------ 查询 ------------------------------ */

/** @brief 判断 DID 是否在参数表内。 */
uint8_t AppUdsParam_Exists(uint16_t u16Did);

/** @brief 取参数描述；未命中返回 NULL。 */
const app_uds_param_desc_t *AppUdsParam_GetDesc(uint16_t u16Did);

/** @brief 取参数线上字节数（2 或 4）；未命中返回 0。 */
uint8_t AppUdsParam_GetLen(uint16_t u16Did);

/** @brief 参数表条目数。 */
uint32_t AppUdsParam_GetCount(void);

/** @brief 按下标取描述（遍历/自检用）；越界返回 NULL。 */
const app_uds_param_desc_t *AppUdsParam_GetDescByIndex(uint32_t u32Idx);

/** @brief 查询参数区自初始化以来被修改的次数。 */
uint32_t AppUdsParam_GetChangeCounter(void);

/* ------------------------ 值访问（物理值，float） ------------------------
 * 这两个接口给**应用层（FOC）**用：调用方拿到的是物理值（A / rpm / Hz），
 * 不必关心该参数在总线上是 u16 还是 f32、scale 是多少。
 */

/** @brief 读物理值；0 = 成功，APP_NRC_ROOR = 不是参数 DID。 */
uint8_t AppUdsParam_GetPhysical(uint16_t u16Did, float *pf32Val);

/** @brief 写物理值（含可写性与量程校验）；返回 0 或 NRC。 */
uint8_t AppUdsParam_SetPhysical(uint16_t u16Did, float f32Val);

/* ------------------------ 字节访问（供协议层 0x22/0x2E） ------------------ */

/**
 * @brief  按线上字节格式读一个参数。
 * @param  [in]  u16Did    DID
 * @param  [out] pu8Out    输出缓冲
 * @param  [in]  u16MaxLen 缓冲长度
 * @param  [out] pu16OutLen 实际字节数（2 或 4）
 * @retval uint8_t 0 = 成功；非 0 = NRC
 */
uint8_t AppUdsParam_Read(uint16_t u16Did, uint8_t *pu8Out,
                         uint16_t u16MaxLen, uint16_t *pu16OutLen);

/**
 * @brief  按线上字节格式写一个参数（长度按类型校验 + 量程校验）。
 * @retval uint8_t 0 = 成功；非 0 = NRC
 */
uint8_t AppUdsParam_Write(uint16_t u16Did, const uint8_t *pu8In, uint16_t u16Len);

/* ------------------------------ 批量操作 ------------------------------ */

/** @brief 全部参数恢复默认值（RID 0x0202）。 */
uint8_t AppUdsParam_ResetAll(void);

/** @brief 检查全部参数是否在量程内（RID 0x0203）；越界项钳回默认值。 */
uint8_t AppUdsParam_CheckAll(void);

/* ------------------------------ 持久化钩子 ------------------------------
 * 本工程无掉电存储，"保存"只保证 RAM 内容有效。
 * 若后续接入 EEPROM/备份寄存器，在应用层实现同名函数即可覆盖
 * （app_uds_param.c 中的默认实现为 __attribute__((weak))）。
 */
uint8_t AppUdsParam_Save(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_UDS_PARAM_H__ */
