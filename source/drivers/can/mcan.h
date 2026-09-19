/**
 *******************************************************************************
 * @file  mcan.h
 * @brief CAN 驱动接口（FDCAN1，经典 CAN）
 *
 *   命名沿用参考工程的 Mcan_* 前缀，使上层协议（isotp.c / uds.c）
 *   与具体芯片无关；在 STM32G431 上由 FDCAN（M_CAN）控制器实现。
 *******************************************************************************
 * Copyright (C) 2026, all rights reserved.
 *
 * This software component is licensed under BSD 3-Clause license.
 *******************************************************************************
 */
#ifndef __MCAN_H__
#define __MCAN_H__

#include <stdint.h>
#include "board.h"      /* 提供 stm32g4xx.h（CMSIS 寄存器定义）与板级宏 */

#ifdef __cplusplus
extern "C" {
#endif

/* CAN 单帧最大数据长度（经典 CAN） */
#define MCAN_PAYLOAD_MAX            (8U)

/* 通用返回码 */
#define MCAN_OK                     (0)
#define MCAN_ERR                    (-1)

/* ---------------------------------------------------------------------------
 * 控制器工作模式（Mcan_LoopbackTest 的 u32Mode 参数）
 *
 *   ⚠️ ST 的 LL 库**不提供 FDCAN 驱动**（无 stm32g4xx_ll_fdcan.h），
 *      只有 HAL。因此本文件是 FDCAN 的唯一驱动层，直接操作 CMSIS 寄存器；
 *      这两种回环模式也只在这里定义，数值与 HAL 的 FDCAN_MODE_* 保持一致，
 *      便于与 ST 文档/RM0440 对照。
 *
 *   回环模式的用途（见 Mcan_LoopbackTest 说明）：
 *     正常模式      0  接总线，需外部节点应答
 *     内部回环      1  芯片内部把 TX 接回 RX，不驱动 TX 引脚
 *     外部回环      2  自收自发，但真实驱动 TX 引脚
 * ------------------------------------------------------------------------- */
#define MCAN_MODE_NORMAL            (0x00000000UL)
#define MCAN_MODE_INTERNAL_LOOPBACK (0x00000001UL)
#define MCAN_MODE_EXTERNAL_LOOPBACK (0x00000002UL)

/* ---------------------------------------------------------------------------
 * 支持的 CAN 波特率（位时序表在 mcan.c 的 s_astcBaudTable 中，唯一真值来源）
 *
 * FDCAN 内核时钟 = HSE 8 MHz，预分频统一为 1（tq = 125 ns），
 * 因此每位都能整除成整数个 tq，波特率误差为 0。
 * 采样点取 CiA 推荐的 87.5 %（800 k 无法整除到 87.5 %，取 80 %）。
 * ------------------------------------------------------------------------- */
#define MCAN_BAUD_125K              (125000UL)
#define MCAN_BAUD_250K              (250000UL)
#define MCAN_BAUD_500K              (500000UL)
#define MCAN_BAUD_800K              (800000UL)
#define MCAN_BAUD_1M                (1000000UL)

/* 波特率"档位码"：用于 UDS DID 下发（比直接传 4 字节速率更紧凑、易校验） */
#define MCAN_BAUD_CODE_125K         (0x00U)
#define MCAN_BAUD_CODE_250K         (0x01U)
#define MCAN_BAUD_CODE_500K         (0x02U)
#define MCAN_BAUD_CODE_800K         (0x03U)
#define MCAN_BAUD_CODE_1M           (0x04U)
/* 恢复出厂默认（= CAN_DEFAULT_BAUDRATE） */
#define MCAN_BAUD_CODE_DEFAULT      (0xFFU)

/* 一次成功的波特率配置结果（供上报/回读） */
typedef struct {
    uint32_t u32Baudrate;   /* 实际速率，如 800000 */
    uint32_t u32Prescaler;  /* 预分频（本工程恒为 1） */
    uint32_t u32TimeSeg1;   /* PS1，实际值（写 NBTP 时硬件再自动减 1） */
    uint32_t u32TimeSeg2;   /* PS2，实际值 */
    uint32_t u32Sjw;        /* 同步跳转宽度 */
    uint8_t  u8Code;        /* 档位码 */
} mcan_baud_info_t;

/* ---------------------------------------------------------------------------
 * 诊断信息（用于总线上定位"完全无应答"这类故障）
 *
 * 排障思路：若连本机**主动发送**的心跳帧都收不到，说明问题在 TX 侧
 * （FDCAN 外设 / 收发器 / 总线接线），而不是 UDS/ISO-TP 协议层；
 * 反之若心跳正常但不应答请求，则应怀疑验收滤波或 RX 侧。
 *
 * au8Raw 布局（共 8 字节，便于直接塞进 CAN 数据场）：
 *   [0] 错误状态指示：0 = 正常
 *   [1] TX 成功计数（低 8 位）
 *   [2] TX 失败计数（低 8 位）
 *   [3] TEC 发送错误计数（0~255）
 *   [4] REC 接收错误计数（0~127）
 *   [5] PSR.lec（最近错误码，M_CAN 定义）
 *   [6] CCCR 低 8 位（bit0=INIT, bit1=CCE, bit5=MON, bit7=ASM）
 *   [7] TXFQS.TFFL（Tx FIFO 剩余空位；为 0 说明发送队列已满）
 */
typedef struct {
    uint32_t u32TxOk;       /* 累计发送成功次数   */
    uint32_t u32TxFail;     /* 累计发送失败次数   */
    uint32_t u32BusOffCnt;  /* 累计进入 Bus Off 的次数 */
    uint8_t  u8ErrorState;  /* 0 = 正常，1 = 被动错误，2 = 总线关闭 */
    uint8_t  u8TxErrorCnt;  /* TEC */
    uint8_t  u8RxErrorCnt;  /* REC */
    uint8_t  u8LastErrCode; /* PSR.lec */
    uint8_t  u8Cccr;        /* CCCR 低 8 位 */
    uint8_t  u8TxFreeLevel; /* TXFQS.TFFL */
} mcan_diag_t;

/**
 * @brief  初始化 FDCAN1（经典 CAN，波特率见 mcan.c）。
 *
 *   使用模块内保存的波特率（初值来自 CAN_DEFAULT_BAUDRATE）；
 *   若曾调用 Mcan_SetBaudrate()，则用设置后的值。
 *
 * @param  无
 * @retval int32_t MCAN_OK 成功，MCAN_ERR 失败（含"波特率不受支持"）
 */
int32_t Mcan_Init(void);

/**
 * @brief  反初始化 FDCAN1（停止控制器），供跳转 App 前清理现场。
 * @param  无
 * @retval int32_t MCAN_OK 成功，MCAN_ERR 失败
 */
int32_t Mcan_DeInit(void);

/**
 * @brief  按"当前已记录的波特率"重新初始化 FDCAN1（运行时切换速率用）。
 *
 *   等价于 Mcan_DeInit() 后紧接 Mcan_Init()，但把两步合在一起，
 *   避免调用方漏掉 Stop/DeInit 而在已启动的控制器上重复 Init。
 *   切换期间会短暂丢失总线（这是 CAN 速率切换的固有代价）。
 *
 * @param  无
 * @retval int32_t MCAN_OK 成功，MCAN_ERR 失败
 */
int32_t Mcan_ReInit(void);

/**
 * @brief  设置 CAN 波特率（仅记录，需再次调用 Mcan_Init() 才生效）。
 *
 *   这样设计是刻意的：调用方（尤其是 App 的 0x2E 处理）需要先在**旧**波特率上
 *   把肯定响应发出去，再重新初始化切到新速率 —— 若本函数立即改变硬件，
 *   响应就会用新速率发出，而此刻上位机还停在旧速率，必然收不到。
 *
 * @param  [in] u32Baudrate 目标速率，必须是表中支持的档位之一
 * @retval int32_t MCAN_OK 支持；MCAN_ERR 不受支持（不改动当前设置）
 */
int32_t Mcan_SetBaudrate(uint32_t u32Baudrate);

/**
 * @brief  按档位码设置波特率（供 UDS DID 下发使用）。
 * @param  [in] u8Code 档位码（MCAN_BAUD_CODE_*）
 * @retval int32_t MCAN_OK 支持；MCAN_ERR 未知档位码
 */
int32_t Mcan_SetBaudrateByCode(uint8_t u8Code);

/**
 * @brief  查询当前（已记录的）波特率。
 * @param  无
 * @retval uint32_t 波特率，如 800000
 */
uint32_t Mcan_GetBaudrate(void);

/**
 * @brief  查询当前波特率对应的档位码。
 * @param  无
 * @retval uint8_t 档位码（MCAN_BAUD_CODE_*；当前值不在表中时返回 0xFF）
 */
uint8_t Mcan_GetBaudrateCode(void);

/**
 * @brief  把指定档位码换算成速率值。
 * @param  [in]  u8Code  档位码（MCAN_BAUD_CODE_*）
 * @param  [out] pu32Baud 输出速率（可为 NULL）
 * @retval int32_t MCAN_OK 有效；MCAN_ERR 未知档位码
 */
int32_t Mcan_CodeToBaudrate(uint8_t u8Code, uint32_t *pu32Baud);

/**
 * @brief  把速率值换算成档位码。
 * @param  [in]  u32Baudrate 速率
 * @param  [out] pu8Code     输出档位码（可为 NULL）
 * @retval int32_t MCAN_OK 有效；MCAN_ERR 不受支持
 */
int32_t Mcan_BaudrateToCode(uint32_t u32Baudrate, uint8_t *pu8Code);

/**
 * @brief  读取当前波特率下的完整位时序参数（调试/上报用）。
 * @param  [out] pInfo 输出结构（不可为 NULL）
 * @retval int32_t MCAN_OK 成功；MCAN_ERR 参数无效
 */
int32_t Mcan_GetBaudInfo(mcan_baud_info_t *pInfo);

/**
 * @brief  发送一帧 CAN 数据帧（阻塞，带超时）。
 * @param  [in] u32Id    标准帧 ID（11 位）
 * @param  [in] pu8Data  数据指针
 * @param  [in] u8Len    数据长度（0 ~ 8）
 * @retval int32_t MCAN_OK 成功，MCAN_ERR 失败
 */
int32_t Mcan_Send(uint32_t u32Id, const uint8_t *pu8Data, uint8_t u8Len);

/**
 * @brief  非阻塞轮询接收一帧 CAN 数据帧。
 * @param  [out] pu32Id   接收到的 ID（可为 NULL）
 * @param  [out] pu8Data  接收缓冲区（>= 8 字节，可为 NULL）
 * @param  [out] pu8Len   接收到的长度（可为 NULL）
 * @retval int32_t MCAN_OK 收到一帧，MCAN_ERR 无数据
 */
int32_t Mcan_Receive(uint32_t *pu32Id, uint8_t *pu8Data, uint8_t *pu8Len);

/**
 * @brief  清空接收 FIFO 中的残留帧。
 * @param  无
 * @retval 无
 */
void Mcan_FlushRx(void);

/**
 * @brief  获取最近一次接收到的 CAN ID（调试用）。
 * @param  无
 * @retval uint32_t 最近一次接收的 CAN ID（尚未收到时为 0xFFFFFFFF）
 */
uint32_t Mcan_GetLastRxId(void);

/**
 * @brief  读取 FDCAN 诊断信息（发送计数 / 错误计数器 / 控制器状态）。
 * @param  [out] pDiag 输出结构体（不可为 NULL）
 * @retval 无
 */
void Mcan_GetDiagnostics(mcan_diag_t *pDiag);

/**
 * @brief  获取累计发送成功次数（调试用）。
 * @param  无
 * @retval uint32_t 发送成功次数
 */
uint32_t Mcan_GetTxOkCount(void);

/**
 * @brief  获取累计发送失败次数（调试用）。
 * @param  无
 * @retval uint32_t 发送失败次数
 */
uint32_t Mcan_GetTxFailCount(void);

/**
 * @brief  获取 FDCAN 外设实例指针（供调试时直接读寄存器）。
 * @param  无
 * @retval FDCAN_GlobalTypeDef* FDCAN1 寄存器基址
 */
FDCAN_GlobalTypeDef *Mcan_GetInstance(void);

/**
 * @brief  若控制器已 Bus Off，则执行恢复（清 INIT 重新接入总线）。
 *
 *   为什么必需：M_CAN 一旦进入 Bus Off，硬件会**自动把 CCCR.INIT 置 1**
 *   并永久停止收发，除非软件显式清除 INIT。若不做恢复，设备会彻底哑掉 ——
 *   而且现象极具误导性：
 *     灯正常闪、程序正常跑，但总线上永远不再出现任何报文。
 *
 *   典型触发场景：上电时上位机（CAN 盒软件）尚未打开，总线上无人应答，
 *   自动重传使 TEC 累加到 256 -> Bus Off -> 之后再打开上位机也收不到任何东西。
 *
 *   本函数需在主循环中周期调用（建议每个心跳周期一次）。
 *   未 Bus Off 时它只读一个寄存器，开销可忽略。
 *
 * @param  无
 * @retval 1 = 本次执行了恢复动作；0 = 无需恢复
 */
uint8_t Mcan_RecoverIfBusOff(void);

/**
 * @brief  获取累计 Bus Off 次数（调试用）。
 * @param  无
 * @retval uint32_t Bus Off 次数
 */
uint32_t Mcan_GetBusOffCount(void);

/**
 * @brief  FDCAN 内部回环自检（用于定位"总线上完全无报文"故障）。
 *
 *   内部回环（Internal LoopBack）把发送器输出直接接到接收器输入，
 *   且**不需要总线上的其他节点应答**，因此可把故障域一刀切开：
 *
 *     自检通过 -> FDCAN 外设、时钟、位时序、验收滤波、收发代码全部正常，
 *                 问题在**外部硬件**（收发器 / 接线 / 终端电阻 / 引脚）
 *     自检失败 -> 问题在**固件/配置**（时钟源、位时序、外设初始化）
 *
 *   注意：本函数会重新初始化 FDCAN 为内部回环模式，调用后必须由调用方
 *   再次执行正常初始化（Mcan_Init）才能恢复通信；因此它只适合在启动
 *   早期做一次性诊断，不适合在正常运行时调用。
 *
 * @param  [in] u32TestId 自检使用的标准帧 ID
 * @param  [out] pu8Ok    1 = 通过，0 = 失败
 * @retval 无
 */
void Mcan_InternalLoopbackSelfTest(uint32_t u32TestId, uint8_t *pu8Ok);

/**
 * @brief  FDCAN 回环测试（内部 / 外部），用于定位发送链路故障。
 *
 *   两种模式的区别：
 *     内部回环(INTERNAL)：TX 在芯片内部接回 RX，**不驱动 TX 引肢**。
 *        通过 => FDCAN 外设 / 时钟 / 位时序 / Message RAM / 收发代码正常
 *     外部回环(EXTERNAL)：同样自收自发，但**会真实驱动 TX 引肢**。
 *        通过 => TX 引脚配置正确且引脚能输出
 *
 *   注意：两者都**不需要总线上其他节点应答**，因此可在"总线上
 *   没有任何报文"时独立验证 MCU 侧。若两者都通过但正常模式仍 Bus Off，
 *   则问题在收发器或总线侧（或对端不回 ACK）。
 *
 *   本函数会把 FDCAN 重新初始化为回环模式，调用后必须再执行
 *   Mcan_Init() 才能恢复通信。
 *
 * @param  [in]  u32Mode   MCAN_MODE_INTERNAL_LOOPBACK 或 MCAN_MODE_EXTERNAL_LOOPBACK
 * @param  [in]  u32TestId 自检使用的标准帧 ID
 * @param  [out] pu8Ok     1 = 通过，0 = 失败
 * @retval 无
 */
void Mcan_LoopbackTest(uint32_t u32Mode, uint32_t u32TestId, uint8_t *pu8Ok);

/* 回环自检结果（供 SWD 直接读取；0xFF = 尚未执行）
 * 在 OpenOCD 下：mdw <地址> 1 即可读到结果。 */
extern uint8_t g_u8CanLoopbackInternalOk;
extern uint8_t g_u8CanLoopbackExternalOk;

/**
 * @brief  获取累计接收到的 CAN 帧数（调试用）。
 *         若总线上有对端在发帧，此计数会增长 —— 可用于验证 RX 链路。
 * @param  无
 * @retval uint32_t 接收帧数
 */
uint32_t Mcan_GetRxCount(void);

#ifdef __cplusplus
}
#endif

#endif /* __MCAN_H__ */
