/**
 *******************************************************************************
 * @file  app_uds_cfg.h
 * @brief 应用层 UDS 诊断参数集中定义
 *
 *   本文件只放"配置"，不放逻辑：DID/RID 编号、会话位掩码、安全等级要求、
 *   表容量、功能寻址白名单。新增一条 DID 只需改这里的宏（可选）
 *   + app_uds_did.c 的表里加一行。
 *
 *   兼容性铁律（不可改，否则上位机与 BOOT 三方不一致）：
 *     0x27 种子/密钥 = 4 字节大端，key = seed ^ 0x5A5A5A5A（ac310_xor）
 *     0x22 0xFF00  = 固件更新请求（必须先 0x27 解锁）
 *******************************************************************************
 * Copyright (C) 2026, all rights reserved.
 *
 * This software component is licensed under BSD 3-Clause license.
 *******************************************************************************/
#ifndef __APP_UDS_CFG_H__
#define __APP_UDS_CFG_H__

#include "board.h"

/* ============================== 一、会话类型 ============================== */
#define APP_SESSION_DEFAULT         (0x01U)
#define APP_SESSION_PROGRAMMING     (0x02U)
#define APP_SESSION_EXTENDED        (0x03U)

/* 会话位掩码（DID 表用）：bit0=默认 bit1=编程 bit2=扩展 */
#define APP_SESS_BIT_DEFAULT        (1U << 0)
#define APP_SESS_BIT_PROG           (1U << 1)
#define APP_SESS_BIT_EXT            (1U << 2)
#define APP_SESS_ANY                (APP_SESS_BIT_DEFAULT | APP_SESS_BIT_PROG | \
                                     APP_SESS_BIT_EXT)

/* ============================== 二、缓冲与表容量 ============================== */
/* 发送缓冲：需容纳两种最大响应，取较大者再加余量 ——
 *   a) 0x22 一次连读全部 49 个参数 DID：1 + 49*(DID2+数据2) = 197 字节
 *   b) 0x19 0x02 报满 16 条 DTC：2 + 1 + 16*4 = 67 字节
 * 故取 256 字节（多帧发送会按 ISO-TP 拆成 FF + CF）。 */
#define APP_UDS_TX_BUF_LEN          (256U)

/* DTC 表条数（纯 RAM，每条 16 字节 -> 256 字节） */
#define APP_UDS_DTC_MAX             (16U)

/* 等待流控期间"非流控帧"的暂存深度（防止吃掉上位机的下一请求） */
#define APP_UDS_STASH_DEPTH         (2U)

/* ========================= 三、安全访问（不可改） ========================= */
#define APP_SA_LEVEL_SEED           (0x01U)
#define APP_SA_LEVEL_KEY            (0x02U)
#define APP_SA_SEED_LEN             (4U)
#define APP_SA_KEY_LEN              (4U)
#define APP_SA_KEY_XOR_MASK         (0x5A5A5A5AU)
#define APP_SA_MAX_ATTEMPTS         (3U)
/* 尝试次数超限后的禁止解锁时长（ms；ISO 14229-1 建议 >= 10 s） */
#define APP_SA_DELAY_MS             (10000U)

/* 是否强制要求先 0x27 才能接受 0xFF00（必须与上位机 seed_key_algo 一致） */
#ifndef APP_UDS_REQUIRE_SECURITY
#define APP_UDS_REQUIRE_SECURITY    (1)
#endif

/* ========================= 四、功能寻址（0x7DF） =========================
 * 功能寻址语义（ISO 14229-1:2020 §7.3 / §8）：
 *   1. ECU 对功能寻址请求**不应答**（肯定与否定响应都不发）；
 *   2. 对不支持的服务**必须静默**，不能回 NRC 0x11 —— 否则总线上多节点
 *      同时应答会互相干扰（这是功能寻址最典型的错误实现）；
 *   3. 只有"对全局有意义"的服务才允许功能寻址：会话控制、复位、读/清 DTC、
 *      DTC 使能、读 DID、通信控制、会话保持。
 *      安全访问 / 写 DID / 例程控制 / 固件更新请求 **禁止** 功能寻址，
 *      否则一条广播就能解锁或改动总线上的所有节点。
 *   4. 功能寻址仅支持单帧请求（ISO 15765-2 不定义广播多帧）。
 */
#define APP_UDS_ENABLE_FUNC_ADDR    (1)

/* 功能寻址允许的服务白名单（SID） */
#define APP_UDS_FUNC_SID_SESS_CTRL      (0x10U)
#define APP_UDS_FUNC_SID_ECU_RESET      (0x11U)
#define APP_UDS_FUNC_SID_CLEAR_DTC      (0x14U)
#define APP_UDS_FUNC_SID_READ_DTC       (0x19U)
#define APP_UDS_FUNC_SID_READ_DID       (0x22U)
#define APP_UDS_FUNC_SID_COMM_CTRL      (0x28U)
#define APP_UDS_FUNC_SID_TESTER_PRESENT (0x3EU)
#define APP_UDS_FUNC_SID_DTC_SETTING    (0x85U)

/* ====================== 五、DID 编号（按区分区分配） ======================
 *   0xF180 ~ 0xF1FF  标准标识类（只读为主）
 *   0xF900 ~ 0xF930  可读写参数区（共 49 个 16 位参数，见 app_uds_param.c）
 *   0xF400 ~ 0xF4FF  实时运行数据（只读，FOC 填充）
 */
#define APP_DID_APP_SW_IDENT        (0xF181U)  /* 应用软件标识（版本字符串） */
#define APP_DID_APP_SW_FINGERPRINT  (0xF184U)  /* 应用软件指纹（镜像 CRC32） */
#define APP_DID_ACTIVE_SESSION      (0xF186U)  /* 当前诊断会话             */
#define APP_DID_SERIAL_NUMBER       (0xF18CU)  /* 序列号（git 短散列）     */
#define APP_DID_HW_VERSION          (0xF193U)  /* 硬件版本号               */
#define APP_DID_SW_VERSION          (0xF195U)  /* 软件版本 "M.m.p"         */
#define APP_DID_SYSTEM_NAME         (0xF197U)  /* 系统名称                 */
#define APP_DID_PROGRAMMING_DATE    (0xF199U)  /* 编译时间                 */

#define APP_DID_CAN_BAUD            (CAN_BAUD_DID) /* 0xF1A0 CAN 波特率档位 */
#define APP_DID_NODE_ADDR           (0xF1B0U)  /* 节点地址（RAM，需解锁）  */

/* ---- 实时运行数据（FOC 接入点，见 app_uds_did.c 的 AppUdsData_* 弱函数） ---- */
#define APP_DID_MOTOR_SPEED         (0xF401U)  /* 电转速 rpm        uint16  */
#define APP_DID_MOTOR_TEMP          (0xF402U)  /* 温度 °C           int8    */
#define APP_DID_BUS_VOLTAGE         (0xF403U)  /* 母线电压 mV       uint16  */
#define APP_DID_PHASE_CURRENT       (0xF404U)  /* 三相电流 mA       int16×3 */
#define APP_DID_MOTOR_STATE         (0xF405U)  /* 状态字            uint32  */
#define APP_DID_MOTOR_FAULT         (0xF406U)  /* 故障字            uint32  */
#define APP_DID_MOTOR_ANGLE         (0xF407U)  /* 电角度 0.1°       uint16  */

/* ====================== 六、例程（0x31）RID ====================== */
#define APP_RID_CHECK_PROG_DEP      (0xFF01U)  /* 检查编程依赖（结果查询）  */
#define APP_RID_MOTOR_SELFTEST      (0x0201U)  /* 应用层自检（占位）        */
#define APP_RID_PARAM_RESET_DEFAULT (0x0202U)  /* 参数区恢复默认值          */
#define APP_RID_PARAM_CHECK         (0x0203U)  /* 参数区一致性检查          */
#define APP_RID_SAVE_CONFIG         (0x0204U)  /* 保存配置（本工程无存储）  */

/* ====================== 七、功能开关 ====================== */
/* 0x23 读内存 / 0x3D 写内存：默认关闭。
 * 打开前请自行确认地址白名单，否则等于把整个地址空间暴露给诊断仪。 */
#ifndef APP_UDS_ENABLE_MEM_ACCESS
#define APP_UDS_ENABLE_MEM_ACCESS   (0)
#endif

#endif /* __APP_UDS_CFG_H__ */
