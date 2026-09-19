/**
 *******************************************************************************
 * @file  mcan.c
 * @brief CAN 驱动（FDCAN1，经典 CAN，波特率可配置）
 *
 *   本 Bootloader 只需要 8 字节数据场的经典 CAN，用于 UDS over ISO-TP，
 *   因此 FDCAN 配置为 Classic 格式（不使用 CAN FD 的更长数据场与可变速率）。
 *
 *   引脚：
 *     PB9  = FDCAN1_TX，AF9
 *     PA11 = FDCAN1_RX，AF9
 *
 *   FDCAN 内核时钟 = HSE = 8 MHz（CKDIV = /1）：
 *     时间量子 tq = 1 / 8 MHz = 125 ns
 *     位时间     = (1 + PS1 + PS2) tq
 *
 *   波特率由 s_astcBaudTable 决定（本文件是唯一真值来源），
 *   8 MHz 是 125k/250k/500k/800k/1M 的公共整数倍，全部整除、误差为 0。
 *******************************************************************************
 * Copyright (C) 2026, all rights reserved.
 *
 * This software component is licensed under BSD 3-Clause license.
 *******************************************************************************
 */

/*******************************************************************************
 * 头文件
 ******************************************************************************/
#include <string.h>
#include "mcan.h"
#include "board.h"
#include "clock.h"      /* SysTick_GetTick：用于限制 BusOff 恢复频率 */

/*******************************************************************************
 * 宏定义
 ******************************************************************************/
/* FDCAN 句柄 */
static FDCAN_HandleTypeDef s_stcFdcan;

/* 标准滤波器数量与 ID 表 —— 由 board.h 提供，本文件不写死。
 *
 * 这样两个工程（BOOT / App）的 mcan.c 可以保持**逐字节一致**：
 *   BOOT 的 board.h 不定义这些宏 -> 默认为"只用 1 个滤波器收 0x7E0"；
 *   App  的 board.h 定义 MCAN_STD_FILTER_COUNT=2 + MCAN_EXTRA_FILTER_ID
 *        即可额外放行功能寻址 0x7DF。
 *
 * 统一放进同一个 Rx FIFO 0：上层用收到的 CAN ID 区分物理/功能寻址，
 * 因此无需第二个 FIFO，也就不必改动 HAL 的消息 RAM 分配。
 */
#ifndef MCAN_STD_FILTER_COUNT
#define MCAN_STD_FILTER_COUNT       (1U)
#endif

#define MCAN_EXT_FILTER_COUNT       (0U)

/* 物理寻址滤波器索引（额外的滤波器依次使用 +1、+2 …） */
#define MCAN_FILTER_INDEX           (0U)

/* 单帧发送的轮询超时上限（近似循环次数） */
#define MCAN_TX_TIMEOUT             (1000000UL)

/* 内部回环自检的等待上限（近似循环次数，约数十毫秒） */
#define MCAN_SELFTEST_TIMEOUT       (200000UL)

/* BusOff 恢复的最小重试间隔（ms）：见 Mcan_RecoverIfBusOff 的说明 */
#define MCAN_BUSOFF_RETRY_MS        (100U)

/* 波特率切换前，等待 Tx FIFO 排空的最长时间（ms）。
 * 正常一两帧只需几毫秒；超时说明总线异常（如无节点应答导致重传），
 * 此时不必继续傻等。 */
#define MCAN_TX_DRAIN_TIMEOUT_MS    (100U)

/* DLC 编码 -> 实际字节数映射表（经典 CAN 只用前 9 项，索引 0~8） */
static const uint8_t s_au8DlcToBytes[16] = {
    0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U, 12U, 16U, 20U, 24U, 32U, 48U, 64U
};

/* ---------------------------------------------------------------------------
 * 波特率 -> 位时序表（FDCAN 内核时钟 = HSE 8 MHz，预分频统一为 1）
 *
 *   位时间 tq 总数 = 1(同步段) + PS1 + PS2
 *   波特率         = 8 MHz / (预分频 * 位时间 tq 总数)
 *   采样点         = (1 + PS1) / 位时间 tq 总数
 *
 *   波特率   tq  PS1  PS2  SJW   采样点   误差
 *   125 k    64   55    8    8   87.5 %    0
 *   250 k    32   27    4    4   87.5 %    0
 *   500 k    16   13    2    2   87.5 %    0
 *   800 k    10    7    2    2   80.0 %    0   <- BOOT 默认
 *     1 M     8    6    1    1   87.5 %    0
 *
 *   约束：SJW <= min(PS1, PS2)（CiA 推荐 SJW <= min(PS1,PS2,4)，本表满足）。
 *   采样点：除 800 k（10 tq 无法得到 87.5 %，取最接近的 80 %）外均为推荐值。
 *
 *   ⚠️ 修改本表即改变了硬件实际波特率。App 侧用的是同一张表
 *      （source/mcan.c），两处必须保持一致，否则 App 配好速率后
 *      BOOT 会以不同速率通信。
 * ------------------------------------------------------------------------- */
typedef struct {
    uint32_t u32Baudrate;
    uint8_t  u8Code;        /* 档位码（供 UDS DID 下发） */
    uint32_t u32Prescaler;
    uint32_t u32Ps1;
    uint32_t u32Ps2;
    uint32_t u32Sjw;
} mcan_baud_entry_t;

static const mcan_baud_entry_t s_astcBaudTable[] = {
    /* 速率        档位                      预分频  PS1  PS2  SJW */
    { MCAN_BAUD_125K, MCAN_BAUD_CODE_125K,    1U,   55U,   8U,   8U },
    { MCAN_BAUD_250K, MCAN_BAUD_CODE_250K,    1U,   27U,   4U,   4U },
    { MCAN_BAUD_500K, MCAN_BAUD_CODE_500K,    1U,   13U,   2U,   2U },
    { MCAN_BAUD_800K, MCAN_BAUD_CODE_800K,    1U,    7U,   2U,   2U },
    { MCAN_BAUD_1M,   MCAN_BAUD_CODE_1M,      1U,    6U,   1U,   1U },
};

#define MCAN_BAUD_TABLE_LEN \
    (sizeof(s_astcBaudTable) / sizeof(s_astcBaudTable[0]))

/*******************************************************************************
 * 内部变量
 ******************************************************************************/
/* 最近一次接收到的 CAN ID（用于调试观察） */
static volatile uint32_t m_u32LastRxId = 0xFFFFFFFFUL;

/* 发送统计（用于诊断"总线上完全看不到本机"这类故障） */
static volatile uint32_t m_u32TxOk      = 0U;
static volatile uint32_t m_u32TxFail    = 0U;
static volatile uint32_t m_u32BusOffCnt = 0U;

/* 接收帧计数（非 0 说明 RX 链路通了：总线上确实有帧进来） */
static volatile uint32_t m_u32RxCnt     = 0U;

/* 当前波特率（初值 = 板级默认；Mcan_SetBaudrate 只改这里，
 * 真正生效需重新调用 Mcan_Init） */
static uint32_t m_u32Baudrate = CAN_DEFAULT_BAUDRATE;

/* 回环自检结果（全局可见，供 SWD 直接读取） */
uint8_t g_u8CanLoopbackInternalOk = 0xFFU;
uint8_t g_u8CanLoopbackExternalOk = 0xFFU;

/*******************************************************************************
 * 内部函数声明
 ******************************************************************************/
static void     Mcan_GpioConfig(void);
static uint32_t Mcan_BytesToDlc(uint8_t u8Len);
static int32_t  Mcan_ClockAndGpioInit(void);
static const mcan_baud_entry_t *Mcan_FindBaudEntry(uint32_t u32Baudrate);
static const mcan_baud_entry_t *Mcan_FindBaudEntryByCode(uint8_t u8Code);
static void     Mcan_FillBitTiming(const mcan_baud_entry_t *pEntry,
                                   FDCAN_InitTypeDef *pInit);
static uint8_t  Mcan_WaitTxDrain(uint32_t u32TimeoutMs);

/*******************************************************************************
 * 函数实现
 ******************************************************************************/

/**
 * @brief  按速率查找位时序表项。
 * @param  [in] u32Baudrate 目标速率
 * @retval const mcan_baud_entry_t* 找到则返回表项，否则返回 NULL
 */
static const mcan_baud_entry_t *Mcan_FindBaudEntry(uint32_t u32Baudrate)
{
    uint32_t i;

    for (i = 0U; i < MCAN_BAUD_TABLE_LEN; i++) {
        if (s_astcBaudTable[i].u32Baudrate == u32Baudrate) {
            return &s_astcBaudTable[i];
        }
    }
    return NULL;
}

/**
 * @brief  按档位码查找位时序表项。
 * @param  [in] u8Code 档位码
 * @retval const mcan_baud_entry_t* 找到则返回表项，否则返回 NULL
 */
static const mcan_baud_entry_t *Mcan_FindBaudEntryByCode(uint8_t u8Code)
{
    uint32_t i;

    for (i = 0U; i < MCAN_BAUD_TABLE_LEN; i++) {
        if (s_astcBaudTable[i].u8Code == u8Code) {
            return &s_astcBaudTable[i];
        }
    }
    return NULL;
}

/**
 * @brief  把位时序表项填入 FDCAN 初始化结构。
 * @param  [in]  pEntry 表项（不可为 NULL）
 * @param  [out] pInit  初始化结构（不可为 NULL）
 * @retval 无
 */
static void Mcan_FillBitTiming(const mcan_baud_entry_t *pEntry,
                               FDCAN_InitTypeDef *pInit)
{
    pInit->NominalPrescaler     = pEntry->u32Prescaler;
    pInit->NominalSyncJumpWidth = pEntry->u32Sjw;
    pInit->NominalTimeSeg1      = pEntry->u32Ps1;
    pInit->NominalTimeSeg2      = pEntry->u32Ps2;
}

/**
 * @brief  FDCAN 时钟源 + 外设时钟 + 引脚配置（Mcan_Init 与自检共用）。
 *
 *   这部分必须与后续的 HAL_FDCAN_Init 分开：自检需要在正常初始化之前
 *   独立完成时钟与引脚配置，否则外设无时钟、HAL 调用会超时失败。
 *
 *   ⚠️ 时钟源选 **HSE（8 MHz）** 而不是 PLLQ：
 *     8 MHz 能整除 125k/250k/500k/800k/1M 全部目标速率（误差为 0），
 *     而 42.5 MHz 无法整除 800 k。且由晶振直接驱动，无 PLL 抖动。
 *     HSE 已在 Board_ClockInit() 中使能，此处只切换 FDCANSEL。
 *
 * @param  无
 * @retval int32_t MCAN_OK 成功，MCAN_ERR 失败
 */
static int32_t Mcan_ClockAndGpioInit(void)
{
    RCC_PeriphCLKInitTypeDef stcPeriphClk = {0};

    /* 选择 FDCAN 内核时钟源 = HSE（8 MHz）。
     * 不选 PCLK1 是为了让位时序独立于 APB1 分频，便于后续调整主频。 */
    stcPeriphClk.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    stcPeriphClk.FdcanClockSelection  = RCC_FDCANCLKSOURCE_HSE;
    if (HAL_RCCEx_PeriphCLKConfig(&stcPeriphClk) != HAL_OK) {
        return MCAN_ERR;
    }

    __HAL_RCC_FDCAN_CLK_ENABLE();
    Mcan_GpioConfig();

    return MCAN_OK;
}

/**
 * @brief  配置 FDCAN1 收发引脚（PB9 = TX，PA11 = RX，AF9）
 *         以及收发器的 S（Standby）控制引脚（PC11）。
 *
 *   ⚠️ S 引脚是关键：CAN 收发器（TJA1042/1051/1044 等）的 S 引脚
 *   若悬空，内部上拉会使器件停在 **Standby** 模式，收发通路关闭。
 *   此时 MCU 侧一切正常（回环自检全过），但：
 *     - TXD 的支配位传不到总线 -> PSR.LEC = 5 (Bit0Error)
 *     - RXD 不发数据          -> 接收帧数恒为 0
 *   表现为"总线上完全没有本机的任何报文"，极难定位。
 *
 *   因此必须显式驱动该引脚到 Normal 电平。
 *
 * @param  无
 * @retval 无
 */
static void Mcan_GpioConfig(void)
{
    GPIO_InitTypeDef stcGpio = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* TX / RX 均为复用推挽；RX 内部上拉使总线悬空时保持隐性电平 */
    stcGpio.Mode      = GPIO_MODE_AF_PP;
    stcGpio.Pull      = GPIO_PULLUP;
    stcGpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    stcGpio.Alternate = CAN_GPIO_AF;

    stcGpio.Pin = CAN_TX_GPIO_PIN;
    HAL_GPIO_Init(CAN_TX_GPIO_PORT, &stcGpio);

    stcGpio.Pin = CAN_RX_GPIO_PIN;
    HAL_GPIO_Init(CAN_RX_GPIO_PORT, &stcGpio);

    /* ---- 收发器 S（Standby）引脚：必须显式拉到 Normal 电平 ---- */
    stcGpio.Mode      = GPIO_MODE_OUTPUT_PP;
    stcGpio.Pull      = GPIO_NOPULL;
    stcGpio.Speed     = GPIO_SPEED_FREQ_LOW;
    stcGpio.Pin       = CAN_STB_GPIO_PIN;

    HAL_GPIO_Init(CAN_STB_GPIO_PORT, &stcGpio);

#if (CAN_STB_ACTIVE_HIGH != 0)
    HAL_GPIO_WritePin(CAN_STB_GPIO_PORT, CAN_STB_GPIO_PIN, GPIO_PIN_SET);
#else
    HAL_GPIO_WritePin(CAN_STB_GPIO_PORT, CAN_STB_GPIO_PIN, GPIO_PIN_RESET);
#endif
}

/**
 * @brief  将字节数转换为 FDCAN 的 DLC 编码值。
 * @param  [in] u8Len 数据字节数（0 ~ 8）
 * @retval uint32_t DLC 编码（0 ~ 8）
 */
static uint32_t Mcan_BytesToDlc(uint8_t u8Len)
{
    /* 经典 CAN 下 DLC 与字节数一一对应（0~8） */
    return ((uint32_t)u8Len & 0x0FU);
}

/**
 * @brief  初始化 FDCAN1（经典 CAN）。
 *
 *   波特率取自 m_u32Baudrate（初值 CAN_DEFAULT_BAUDRATE，可用
 *   Mcan_SetBaudrate() 修改）。若该速率不在位时序表中，直接失败 ——
 *   宁可不通信，也不要用错误的位时序在总线上制造隐性故障。
 *
 * @param  无
 * @retval int32_t MCAN_OK 成功，MCAN_ERR 失败
 */
int32_t Mcan_Init(void)
{
    FDCAN_FilterTypeDef stcFilter = {0};
    const mcan_baud_entry_t *pEntry;

    /* 0. 波特率必须在位时序表中（否则位时序无从取值） */
    pEntry = Mcan_FindBaudEntry(m_u32Baudrate);
    if (pEntry == NULL) {
        return MCAN_ERR;
    }

    /* 1. 时钟源 + 外设时钟 + 引脚（与自检共用同一份配置） */
    if (Mcan_ClockAndGpioInit() != MCAN_OK) {
        return MCAN_ERR;
    }

    /* 2. 配置 FDCAN 控制器（经典 CAN 格式） */
    s_stcFdcan.Instance                  = FDCAN1;
    s_stcFdcan.Init.ClockDivider         = FDCAN_CLOCK_DIV1;
    s_stcFdcan.Init.FrameFormat          = FDCAN_FRAME_CLASSIC;
    s_stcFdcan.Init.Mode                 = FDCAN_MODE_NORMAL;
    s_stcFdcan.Init.AutoRetransmission   = ENABLE;
    s_stcFdcan.Init.TransmitPause        = ENABLE;
    s_stcFdcan.Init.ProtocolException    = DISABLE;

    /* 位时序按当前波特率查表填入 */
    Mcan_FillBitTiming(pEntry, &s_stcFdcan.Init);

    /* 经典 CAN 格式下数据段位时序不生效，但 HAL 要求填合法值 */
    s_stcFdcan.Init.DataPrescaler        = 1U;
    s_stcFdcan.Init.DataSyncJumpWidth    = 1U;
    s_stcFdcan.Init.DataTimeSeg1         = 1U;
    s_stcFdcan.Init.DataTimeSeg2         = 1U;

    s_stcFdcan.Init.StdFiltersNbr        = MCAN_STD_FILTER_COUNT;
    s_stcFdcan.Init.ExtFiltersNbr        = MCAN_EXT_FILTER_COUNT;
    s_stcFdcan.Init.TxFifoQueueMode      = FDCAN_TX_FIFO_OPERATION;

    if (HAL_FDCAN_Init(&s_stcFdcan) != HAL_OK) {
        return MCAN_ERR;
    }

    /* 4. 配置验收滤波器：放行 UDS 请求 ID 到 Rx FIFO 0。
     *    FilterType = MASK 时，FilterID1 为期望值，FilterID2 为掩码；
     *    掩码全 1 表示 11 位 ID 必须完全匹配。
     *
     *    额外的滤波器（如 App 侧的功能寻址 0x7DF）由 board.h 的
     *    MCAN_EXTRA_FILTER_ID / MCAN_STD_FILTER_COUNT 声明，
     *    它们与物理寻址共用同一个 Rx FIFO 0。 */
    stcFilter.IdType       = FDCAN_STANDARD_ID;
    stcFilter.FilterIndex  = MCAN_FILTER_INDEX;
    stcFilter.FilterType   = FDCAN_FILTER_MASK;
    stcFilter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    stcFilter.FilterID1    = UDS_CAN_RX_ID;
    stcFilter.FilterID2    = 0x7FFU;            /* 掩码：逐位比较 */
    if (HAL_FDCAN_ConfigFilter(&s_stcFdcan, &stcFilter) != HAL_OK) {
        return MCAN_ERR;
    }

#ifdef MCAN_EXTRA_FILTER_ID
    stcFilter.FilterIndex  = (uint32_t)(MCAN_FILTER_INDEX + 1U);
    stcFilter.FilterID1    = MCAN_EXTRA_FILTER_ID;
    stcFilter.FilterID2    = 0x7FFU;
    if (HAL_FDCAN_ConfigFilter(&s_stcFdcan, &stcFilter) != HAL_OK) {
        return MCAN_ERR;
    }
#endif

    /* 5. 未匹配的标准帧一律拒绝（不放进 FIFO），并拒绝远程帧 */
    if (HAL_FDCAN_ConfigGlobalFilter(&s_stcFdcan,
                                     FDCAN_REJECT, FDCAN_REJECT,
                                     FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE)
        != HAL_OK) {
        return MCAN_ERR;
    }

    /* 6. 启动 FDCAN */
    if (HAL_FDCAN_Start(&s_stcFdcan) != HAL_OK) {
        return MCAN_ERR;
    }

    return MCAN_OK;
}

/**
 * @brief  反初始化 FDCAN1（停止控制器并复位外设寄存器）。
 *         供跳转 App 前清理现场，避免残留状态干扰 App 的初始化。
 * @param  无
 * @retval int32_t MCAN_OK 成功，MCAN_ERR 失败
 */
int32_t Mcan_DeInit(void)
{
    if (HAL_FDCAN_DeInit(&s_stcFdcan) != HAL_OK) {
        return MCAN_ERR;
    }

    return MCAN_OK;
}

/**
 * @brief  设置 CAN 波特率（只记录，需重新 Mcan_Init 才生效）。
 *
 *   刻意不立即改动硬件：调用方（App 的 0x2E 处理）必须先在**旧**速率上
 *   把肯定响应发出去，然后才重新初始化切到新速率。若本函数直接改硬件，
 *   响应会以新速率发出，而此刻上位机仍在旧速率监听，必然收不到。
 *
 * @param  [in] u32Baudrate 目标速率（必须是表中档位之一）
 * @retval int32_t MCAN_OK 支持；MCAN_ERR 不受支持（不改动当前设置）
 */
int32_t Mcan_SetBaudrate(uint32_t u32Baudrate)
{
    if (Mcan_FindBaudEntry(u32Baudrate) == NULL) {
        return MCAN_ERR;
    }

    m_u32Baudrate = u32Baudrate;
    return MCAN_OK;
}

/**
 * @brief  等待 Tx FIFO 里的帧全部真正发到总线上。
 *
 *   为什么必须有这一步：Mcan_Send() 只是把帧塞进 Tx FIFO 就返回，
 *   并不代表已经发上总线。若紧接着 HAL_FDCAN_Stop()（内部会立即置
 *   CCCR.INIT），未发出的帧会被丢弃 —— 这正是"改完波特率后上位机
 *   收不到 0x6E 应答"的原因。
 *
 *   判据用 TXBRP（Tx Buffer Request Pending）：某位为 1 表示对应
 *   缓冲仍待发送。全部为 0 即发送完毕。
 *
 *   超时保护：总线上若无其他节点应答（比如只有本机），帧会因自动重传
 *   而长期挂起；此时不能无限等，超时后照样继续（丢一帧总好过卡死）。
 *
 * @param  [in] u32TimeoutMs 最长等待毫秒数
 * @retval uint8_t 1 = 已全部发完；0 = 超时（可能有帧未发出）
 */
static uint8_t Mcan_WaitTxDrain(uint32_t u32TimeoutMs)
{
    uint32_t u32Start = SysTick_GetTick();

    if (s_stcFdcan.Instance == NULL) {
        return 1U;
    }

    while ((s_stcFdcan.Instance->TXBRP & FDCAN_TXBRP_TRP_Msk) != 0U) {
        if ((SysTick_GetTick() - u32Start) >= u32TimeoutMs) {
            return 0U;
        }
    }

    /* 缓冲已空，但最后一位可能刚发完。再等一个最短帧时间
     * （约 47 us @ 800 kbps，取 1 ms 足够宽裕），
     * 确保 ACK 位也已在总线上完成，随后再改速率才不会截断该帧。 */
    {
        uint32_t u32T = SysTick_GetTick();

        while ((SysTick_GetTick() - u32T) < 1U) {
            ;
        }
    }

    return 1U;
}

/**
 * @brief  按"当前已记录的波特率"重新初始化 FDCAN1。
 *
 *   运行时切换波特率用。必须在 HAL_FDCAN_DeInit() 之后重新 Init：
 *   HAL 只在 State == RESET 时才调用 MspInit 并写 CKDIV，
 *   若跳过 DeInit 直接 Init，CKDIV 与位时序不会更新。
 *
 *   ⚠️ 调用前会先把待发帧送完 —— 否则切换时的 Stop 会把它们丢掉。
 *      调用方（App 的 0x2E 处理）已经把 0x6E 应答排入队列，
 *      因此这一步是应答能真正到达上位机的关键。
 *
 * @param  无
 * @retval int32_t MCAN_OK 成功，MCAN_ERR 失败
 */
int32_t Mcan_ReInit(void)
{
    /* 先等队列里的帧（含 0x6E 应答）真正发上总线。
     * 上限取 100 ms：正常一两帧几个毫秒就够，超时说明总线有问题，
     * 此时继续切换也无意义，不去无谓地阻塞主循环。 */
    (void)Mcan_WaitTxDrain(MCAN_TX_DRAIN_TIMEOUT_MS);

    /* 先停控制器再复位外设，确保 HAL_FDCAN_Init 会重新配置时钟分频与位时序 */
    (void)HAL_FDCAN_Stop(&s_stcFdcan);
    (void)HAL_FDCAN_DeInit(&s_stcFdcan);

    return Mcan_Init();
}

/**
 * @brief  按档位码设置波特率（供 UDS DID 下发使用）。
 * @param  [in] u8Code 档位码；MCAN_BAUD_CODE_DEFAULT 表示恢复板级默认
 * @retval int32_t MCAN_OK 支持；MCAN_ERR 未知档位码
 */
int32_t Mcan_SetBaudrateByCode(uint8_t u8Code)
{
    const mcan_baud_entry_t *pEntry;

    /* 0xFF = 恢复出厂默认（与 CAN_DEFAULT_BAUDRATE 一致） */
    if (u8Code == MCAN_BAUD_CODE_DEFAULT) {
        m_u32Baudrate = CAN_DEFAULT_BAUDRATE;
        return (Mcan_FindBaudEntry(m_u32Baudrate) != NULL) ? MCAN_OK : MCAN_ERR;
    }

    pEntry = Mcan_FindBaudEntryByCode(u8Code);
    if (pEntry == NULL) {
        return MCAN_ERR;
    }

    m_u32Baudrate = pEntry->u32Baudrate;
    return MCAN_OK;
}

/**
 * @brief  查询当前（已记录的）波特率。
 * @param  无
 * @retval uint32_t 波特率，如 800000
 */
uint32_t Mcan_GetBaudrate(void)
{
    return m_u32Baudrate;
}

/**
 * @brief  查询当前波特率对应的档位码。
 * @param  无
 * @retval uint8_t 档位码；当前值不在表中时返回 MCAN_BAUD_CODE_DEFAULT
 */
uint8_t Mcan_GetBaudrateCode(void)
{
    const mcan_baud_entry_t *pEntry = Mcan_FindBaudEntry(m_u32Baudrate);

    return (pEntry != NULL) ? pEntry->u8Code : MCAN_BAUD_CODE_DEFAULT;
}

/**
 * @brief  把指定档位码换算成速率值。
 * @param  [in]  u8Code   档位码
 * @param  [out] pu32Baud 输出速率（可为 NULL）
 * @retval int32_t MCAN_OK 有效；MCAN_ERR 未知档位码
 */
int32_t Mcan_CodeToBaudrate(uint8_t u8Code, uint32_t *pu32Baud)
{
    const mcan_baud_entry_t *pEntry;

    if (u8Code == MCAN_BAUD_CODE_DEFAULT) {
        if (pu32Baud != NULL) {
            *pu32Baud = CAN_DEFAULT_BAUDRATE;
        }
        return MCAN_OK;
    }

    pEntry = Mcan_FindBaudEntryByCode(u8Code);
    if (pEntry == NULL) {
        return MCAN_ERR;
    }

    if (pu32Baud != NULL) {
        *pu32Baud = pEntry->u32Baudrate;
    }
    return MCAN_OK;
}

/**
 * @brief  把速率值换算成档位码。
 * @param  [in]  u32Baudrate 速率
 * @param  [out] pu8Code     输出档位码（可为 NULL）
 * @retval int32_t MCAN_OK 有效；MCAN_ERR 不受支持
 */
int32_t Mcan_BaudrateToCode(uint32_t u32Baudrate, uint8_t *pu8Code)
{
    const mcan_baud_entry_t *pEntry;

    if (u32Baudrate == CAN_DEFAULT_BAUDRATE) {
        if (pu8Code != NULL) {
            *pu8Code = MCAN_BAUD_CODE_DEFAULT;
        }
        return MCAN_OK;
    }

    pEntry = Mcan_FindBaudEntry(u32Baudrate);
    if (pEntry == NULL) {
        return MCAN_ERR;
    }

    if (pu8Code != NULL) {
        *pu8Code = pEntry->u8Code;
    }
    return MCAN_OK;
}

/**
 * @brief  读取当前波特率下的完整位时序参数（调试/上报用）。
 * @param  [out] pInfo 输出结构（不可为 NULL）
 * @retval int32_t MCAN_OK 成功；MCAN_ERR 参数无效或波特率不受支持
 */
int32_t Mcan_GetBaudInfo(mcan_baud_info_t *pInfo)
{
    const mcan_baud_entry_t *pEntry;

    if (pInfo == NULL) {
        return MCAN_ERR;
    }

    pEntry = Mcan_FindBaudEntry(m_u32Baudrate);
    if (pEntry == NULL) {
        return MCAN_ERR;
    }

    pInfo->u32Baudrate  = pEntry->u32Baudrate;
    pInfo->u32Prescaler = pEntry->u32Prescaler;
    pInfo->u32TimeSeg1  = pEntry->u32Ps1;
    pInfo->u32TimeSeg2  = pEntry->u32Ps2;
    pInfo->u32Sjw       = pEntry->u32Sjw;
    pInfo->u8Code       = pEntry->u8Code;

    return MCAN_OK;
}

/**
 * @brief  发送一帧 CAN 数据帧（阻塞，带超时）。
 * @param  [in] u32Id    标准帧 ID（11 位）
 * @param  [in] pu8Data  数据指针
 * @param  [in] u8Len    数据长度（0 ~ 8）
 * @retval int32_t MCAN_OK 成功，MCAN_ERR 失败
 */
int32_t Mcan_Send(uint32_t u32Id, const uint8_t *pu8Data, uint8_t u8Len)
{
    FDCAN_TxHeaderTypeDef stcTxHeader = {0};
    uint32_t              u32Timeout  = MCAN_TX_TIMEOUT;
    uint8_t               au8Pad[MCAN_PAYLOAD_MAX] = {0};
    uint32_t              u32FreeLevel;

    if ((pu8Data == NULL) || (u8Len > MCAN_PAYLOAD_MAX)) {
        m_u32TxFail++;
        return MCAN_ERR;
    }

    /* FDCAN 的 Tx FIFO 只有 3 个元素，先等待有空位。
     *
     * 注意 1：若总线无其他节点应答（例如上位机还没打开、收发器未使能、
     *         无终端电阻、波特率不匹配），FDCAN 会因 AutoRetransmission
     *         而不断重发，导致 Tx FIFO 长期占满 —— 这个循环会一直等到超时。
     *         因此它超时本身就说明"总线上发不出去"，是一个强诊断信号。
     *
     * 注意 2：Bus Off 时 FIFO 永远不会排空。若只靠超时计数，这里会白等
     *         满 MCAN_TX_TIMEOUT 次（约 30 ms）才返回，每次心跳都拖累主循环。
     *         因此检测到 Bus Off 立即放弃并尝试恢复。 */
    while ((u32FreeLevel = HAL_FDCAN_GetTxFifoFreeLevel(&s_stcFdcan)) == 0U) {
        if ((s_stcFdcan.Instance->PSR & FDCAN_PSR_BO) != 0U) {
            (void)Mcan_RecoverIfBusOff();
            m_u32TxFail++;
            return MCAN_ERR;
        }
        if (u32Timeout == 0UL) {
            m_u32TxFail++;
            return MCAN_ERR;
        }
        u32Timeout--;
    }

    stcTxHeader.Identifier          = (u32Id & 0x7FFUL);
    stcTxHeader.IdType              = FDCAN_STANDARD_ID;
    stcTxHeader.TxFrameType         = FDCAN_DATA_FRAME;
    stcTxHeader.DataLength          = Mcan_BytesToDlc(u8Len);
    stcTxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    stcTxHeader.BitRateSwitch       = FDCAN_BRS_OFF;
    stcTxHeader.FDFormat            = FDCAN_CLASSIC_CAN;
    stcTxHeader.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    stcTxHeader.MessageMarker       = 0U;

    /* HAL 会按 DataLength 拷贝对应字节数，源缓冲需保证足够长 */
    if (u8Len < MCAN_PAYLOAD_MAX) {
        memcpy(au8Pad, pu8Data, u8Len);
        pu8Data = au8Pad;
    }

    if (HAL_FDCAN_AddMessageToTxFifoQ(&s_stcFdcan, &stcTxHeader, pu8Data)
        != HAL_OK) {
        m_u32TxFail++;
        return MCAN_ERR;
    }

    m_u32TxOk++;
    return MCAN_OK;
}

/**
 * @brief  非阻塞轮询接收一帧 CAN 数据帧。
 * @param  [out] pu32Id   接收到的 ID（可为 NULL）
 * @param  [out] pu8Data  接收缓冲区（>= 8 字节，可为 NULL）
 * @param  [out] pu8Len   接收到的长度（可为 NULL）
 * @retval int32_t MCAN_OK 收到一帧，MCAN_ERR 无数据
 */
int32_t Mcan_Receive(uint32_t *pu32Id, uint8_t *pu8Data, uint8_t *pu8Len)
{
    FDCAN_RxHeaderTypeDef stcRxHeader = {0};
    uint8_t               au8Data[MCAN_PAYLOAD_MAX] = {0};
    uint8_t               u8Bytes;

    /* FIFO 为空则立即返回，不做任何等待 */
    if (HAL_FDCAN_GetRxFifoFillLevel(&s_stcFdcan, FDCAN_RX_FIFO0) == 0U) {
        return MCAN_ERR;
    }

    if (HAL_FDCAN_GetRxMessage(&s_stcFdcan, FDCAN_RX_FIFO0,
                               &stcRxHeader, au8Data) != HAL_OK) {
        return MCAN_ERR;
    }

    /* DataLength 是 DLC 编码值，需查表换算为实际字节数。
     * 本工程只接收经典 CAN 帧，DLC 0~8 与字节数一一对应。 */
    u8Bytes = (stcRxHeader.DataLength < 16U)
              ? s_au8DlcToBytes[stcRxHeader.DataLength] : 8U;
    if (u8Bytes > MCAN_PAYLOAD_MAX) {
        u8Bytes = MCAN_PAYLOAD_MAX;
    }

    m_u32LastRxId = stcRxHeader.Identifier;
    m_u32RxCnt++;

    if (pu32Id != NULL) {
        *pu32Id = stcRxHeader.Identifier;
    }
    if (pu8Data != NULL) {
        memcpy(pu8Data, au8Data, MCAN_PAYLOAD_MAX);
    }
    if (pu8Len != NULL) {
        *pu8Len = u8Bytes;
    }

    return MCAN_OK;
}

/**
 * @brief  清空接收 FIFO 中的残留帧。
 * @param  无
 * @retval 无
 */
void Mcan_FlushRx(void)
{
    FDCAN_RxHeaderTypeDef stcRxHeader;
    uint8_t               au8Data[MCAN_PAYLOAD_MAX];

    while (HAL_FDCAN_GetRxFifoFillLevel(&s_stcFdcan, FDCAN_RX_FIFO0) != 0U) {
        if (HAL_FDCAN_GetRxMessage(&s_stcFdcan, FDCAN_RX_FIFO0,
                                   &stcRxHeader, au8Data) != HAL_OK) {
            break;
        }
    }
}

/**
 * @brief  获取最近一次接收到的 CAN ID（调试用）。
 * @param  无
 * @retval uint32_t 最近一次接收的 CAN ID（尚未收到时为 0xFFFFFFFF）
 */
uint32_t Mcan_GetLastRxId(void)
{
    return m_u32LastRxId;
}

/**
 * @brief  获取累计发送成功次数（调试用）。
 * @param  无
 * @retval uint32_t 发送成功次数
 */
uint32_t Mcan_GetTxOkCount(void)
{
    return m_u32TxOk;
}

/**
 * @brief  获取累计发送失败次数（调试用）。
 * @param  无
 * @retval uint32_t 发送失败次数
 */
uint32_t Mcan_GetTxFailCount(void)
{
    return m_u32TxFail;
}

/**
 * @brief  获取 FDCAN 外设实例指针（供调试时直接读寄存器）。
 * @param  无
 * @retval FDCAN_GlobalTypeDef* FDCAN1 寄存器基址
 */
FDCAN_GlobalTypeDef *Mcan_GetInstance(void)
{
    return s_stcFdcan.Instance;
}

/**
 * @brief  读取 FDCAN 诊断信息。
 * @param  [out] pDiag 输出结构体（不可为 NULL）
 * @retval 无
 */
void Mcan_GetDiagnostics(mcan_diag_t *pDiag)
{
    uint32_t u32Psr;

    if (pDiag == NULL) {
        return;
    }

    pDiag->u32TxOk      = m_u32TxOk;
    pDiag->u32TxFail    = m_u32TxFail;
    pDiag->u32BusOffCnt = m_u32BusOffCnt;

    /* FDCAN 外设可能尚未初始化（例如时钟配置失败后未走到 Mcan_Init），
     * 此时不能解引用句柄，直接返回零值即可。 */
    if (s_stcFdcan.Instance == NULL) {
        pDiag->u8ErrorState  = 0U;
        pDiag->u8TxErrorCnt  = 0U;
        pDiag->u8RxErrorCnt  = 0U;
        pDiag->u8LastErrCode = 0U;
        pDiag->u8Cccr        = 0U;
        pDiag->u8TxFreeLevel = 0U;
        return;
    }

    /* ECR：TEC[7:0] 发送错误计数，REC[14:8] 接收错误计数 */
    pDiag->u8TxErrorCnt = (uint8_t)(s_stcFdcan.Instance->ECR &
                                    FDCAN_ECR_TEC_Msk);
    pDiag->u8RxErrorCnt = (uint8_t)((s_stcFdcan.Instance->ECR &
                                     FDCAN_ECR_REC_Msk) >> FDCAN_ECR_REC_Pos);

    /* PSR：LEC[2:0] 最近一次错误码，BO[7] 总线关闭，EP[5] 错误被动 */
    u32Psr = s_stcFdcan.Instance->PSR;
    pDiag->u8LastErrCode = (uint8_t)(u32Psr & FDCAN_PSR_LEC_Msk);

    if ((u32Psr & FDCAN_PSR_BO) != 0U) {
        pDiag->u8ErrorState = 2U;       /* 总线关闭（Bus Off） */
    } else if ((u32Psr & FDCAN_PSR_EP) != 0U) {
        pDiag->u8ErrorState = 1U;       /* 错误被动 */
    } else {
        pDiag->u8ErrorState = 0U;       /* 错误主动（正常） */
    }

    /* CCCR 低 8 位：bit0=INIT, bit1=CCE, bit5=MON, bit7=ASM */
    pDiag->u8Cccr = (uint8_t)(s_stcFdcan.Instance->CCCR & 0xFFU);

    /* TXFQS.TFFL：Tx FIFO 剩余空位数 */
    pDiag->u8TxFreeLevel = (uint8_t)(s_stcFdcan.Instance->TXFQS &
                                     FDCAN_TXFQS_TFFL_Msk);
}

/**
 * @brief  若控制器已 Bus Off，则执行恢复（清 INIT 重新接入总线）。
 *
 *   为什么必需：M_CAN 一旦进入 Bus Off，硬件会**自动把 CCCR.INIT 置 1**
 *   并永久停止收发，除非软件显式清除 INIT。若不做恢复，设备会彻底哑掉，
 *   而且现象极具误导性：
 *     灯正常闪、程序正常跑，但总线上永远不再出现任何报文。
 *
 *   典型触发场景：上电时上位机（CAN 盒软件）尚未打开，总线上无人应答，
 *   自动重传使 TEC 累加到 256 -> Bus Off -> 之后再打开上位机也收不到东西。
 *
 *   恢复流程（M_CAN 手册 / ISO 11898-1）：
 *     置 INIT=1 -> 置 CCE=1 -> 清 INIT=0；
 *     之后硬件需观察到 128 次连续 11 个隐性位才重新接入总线。
 *
 * @param  无
 * @retval 1 = 本次执行了恢复动作；0 = 无需恢复
 */
uint8_t Mcan_RecoverIfBusOff(void)
{
    static uint32_t s_u32LastTryTick = 0U;
    FDCAN_GlobalTypeDef *pInst = s_stcFdcan.Instance;
    uint32_t u32Now;

    if (pInst == NULL) {
        return 0U;
    }

    /* 既没 Bus Off、也不在初始化模式：无需处理 */
    if (((pInst->PSR  & FDCAN_PSR_BO)    == 0U) &&
        ((pInst->CCCR & FDCAN_CCCR_INIT) == 0U)) {
        return 0U;
    }

    /* 限流：M_CAN 清 INIT 后需要观察到 128 次连续 11 个隐性位才会重新
     * 接入总线，期间若再次失败会自行把 INIT 置回 1。
     *
     * 若不限流，每轮主循环都会"检测到 BusOff -> 清 INIT"，
     * 导致恢复动作以主循环频率反复触发（实测 >40 万次），
     * 既统计失真，也白耗 CPU。这里限制为每 MCAN_BUSOFF_RETRY_MS 一次。 */
    u32Now = SysTick_GetTick();
    if ((u32Now - s_u32LastTryTick) < MCAN_BUSOFF_RETRY_MS) {
        return 0U;
    }
    s_u32LastTryTick = u32Now;

    m_u32BusOffCnt++;

    /* 进入配置模式 -> 清除挂起请求 -> 重新接入总线 */
    SET_BIT(pInst->CCCR, FDCAN_CCCR_INIT);
    SET_BIT(pInst->CCCR, FDCAN_CCCR_CCE);

    /* 清掉队列里已无意义的挂起请求，否则 FIFO 会一直占满，
     * 后续正常发送也进不来。写 1 到 TXBCR 即请求取消对应缓冲。 */
    pInst->TXBCR = pInst->TXBRP;

    CLEAR_BIT(pInst->CCCR, FDCAN_CCCR_INIT);

    return 1U;
}

/**
 * @brief  获取累计 Bus Off 次数（调试用）。
 * @param  无
 * @retval uint32_t Bus Off 次数
 */
uint32_t Mcan_GetBusOffCount(void)
{
    return m_u32BusOffCnt;
}

/**
 * @brief  获取累计接收到的 CAN 帧数（调试用）。
 * @param  无
 * @retval uint32_t 接收帧数
 */
uint32_t Mcan_GetRxCount(void)
{
    return m_u32RxCnt;
}

/**
 * @brief  FDCAN 回环测试（内部 / 外部）。
 *
 *   内部回环：TX 在芯片内部接回 RX，不驱动 TX 引脚。
 *   外部回环：同样自收自发，但会真实驱动 TX 引脚。
 *   两者都不需要总线上其他节点应答，因此可在总线完全无报文时
 *   独立验证 MCU 侧。
 *
 * @param  [in]  u32Mode   FDCAN_MODE_INTERNAL_LOOPBACK / FDCAN_MODE_EXTERNAL_LOOPBACK
 * @param  [in]  u32TestId 自检使用的标准帧 ID
 * @param  [out] pu8Ok     1 = 通过，0 = 失败
 * @retval 无
 */
void Mcan_LoopbackTest(uint32_t u32Mode, uint32_t u32TestId, uint8_t *pu8Ok)
{
    FDCAN_InitTypeDef     stcInit = {0};
    FDCAN_FilterTypeDef   stcFilter = {0};
    FDCAN_TxHeaderTypeDef stcTx = {0};
    FDCAN_RxHeaderTypeDef stcRx = {0};
    uint8_t               au8TxData[MCAN_PAYLOAD_MAX] = {0};
    uint8_t               au8RxData[MCAN_PAYLOAD_MAX] = {0};
    uint32_t              u32Wait;
    const mcan_baud_entry_t *pEntry;

    if (pu8Ok == NULL) {
        return;
    }
    *pu8Ok = 0U;

    /* 位时序同样按当前波特率查表；查不到则自检无意义，直接判失败 */
    pEntry = Mcan_FindBaudEntry(m_u32Baudrate);
    if (pEntry == NULL) {
        return;
    }

    au8TxData[0] = (uint8_t)(u32TestId & 0xFFU);
    au8TxData[1] = 0xA5U;
    au8TxData[2] = 0x5AU;

    /* 时钟 / 引脚必须已就绪 */
    if (Mcan_ClockAndGpioInit() != MCAN_OK) {
        return;
    }

    (void)HAL_FDCAN_Stop(&s_stcFdcan);
    (void)HAL_FDCAN_DeInit(&s_stcFdcan);

    s_stcFdcan.Instance          = FDCAN1;
    stcInit.ClockDivider         = FDCAN_CLOCK_DIV1;
    stcInit.FrameFormat          = FDCAN_FRAME_CLASSIC;
    stcInit.Mode                 = u32Mode;
    stcInit.AutoRetransmission   = DISABLE;   /* 回环下禁止重传，避免拖时间 */
    stcInit.TransmitPause        = ENABLE;
    stcInit.ProtocolException    = DISABLE;
    Mcan_FillBitTiming(pEntry, &stcInit);
    stcInit.DataPrescaler        = 1U;
    stcInit.DataSyncJumpWidth    = 1U;
    stcInit.DataTimeSeg1         = 1U;
    stcInit.DataTimeSeg2         = 1U;
    stcInit.StdFiltersNbr        = MCAN_STD_FILTER_COUNT;
    stcInit.ExtFiltersNbr        = MCAN_EXT_FILTER_COUNT;
    stcInit.TxFifoQueueMode      = FDCAN_TX_FIFO_OPERATION;

    s_stcFdcan.Init = stcInit;

    if (HAL_FDCAN_Init(&s_stcFdcan) != HAL_OK) {
        return;
    }

    stcFilter.IdType       = FDCAN_STANDARD_ID;
    stcFilter.FilterIndex  = MCAN_FILTER_INDEX;
    stcFilter.FilterType   = FDCAN_FILTER_MASK;
    stcFilter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    stcFilter.FilterID1    = u32TestId;
    stcFilter.FilterID2    = 0x7FFU;
    if (HAL_FDCAN_ConfigFilter(&s_stcFdcan, &stcFilter) != HAL_OK) {
        return;
    }

    (void)HAL_FDCAN_ConfigGlobalFilter(&s_stcFdcan,
                                       FDCAN_REJECT, FDCAN_REJECT,
                                       FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);

    if (HAL_FDCAN_Start(&s_stcFdcan) != HAL_OK) {
        return;
    }

    stcTx.Identifier          = u32TestId & 0x7FFUL;
    stcTx.IdType              = FDCAN_STANDARD_ID;
    stcTx.TxFrameType         = FDCAN_DATA_FRAME;
    stcTx.DataLength          = Mcan_BytesToDlc(3U);
    stcTx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    stcTx.BitRateSwitch       = FDCAN_BRS_OFF;
    stcTx.FDFormat            = FDCAN_CLASSIC_CAN;
    stcTx.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    stcTx.MessageMarker       = 0U;

    {
        uint8_t au8Pad[MCAN_PAYLOAD_MAX] = {0};

        memcpy(au8Pad, au8TxData, sizeof(au8TxData));
        if (HAL_FDCAN_AddMessageToTxFifoQ(&s_stcFdcan, &stcTx, au8Pad)
            != HAL_OK) {
            return;
        }
    }

    /* 轮询接收：回环下应很快收到自己发的帧 */
    for (u32Wait = 0U; u32Wait < MCAN_SELFTEST_TIMEOUT; u32Wait++) {
        if (HAL_FDCAN_GetRxFifoFillLevel(&s_stcFdcan, FDCAN_RX_FIFO0) != 0U) {
            if (HAL_FDCAN_GetRxMessage(&s_stcFdcan, FDCAN_RX_FIFO0,
                                       &stcRx, au8RxData) != HAL_OK) {
                return;
            }
            if ((stcRx.Identifier == (u32TestId & 0x7FFUL)) &&
                (au8RxData[1] == 0xA5U) && (au8RxData[2] == 0x5AU)) {
                *pu8Ok = 1U;
            }
            return;
        }
    }
}

/**
 * @brief  FDCAN 内部回环自检。
 *
 *   内部回环不需要外部收发器与总线应答，因此可以把"总线上无报文"
 *   这一故障切成两半：外设/配置问题 vs 外部硬件问题。
 *
 *   实现要点：以 FDCAN_MODE_INTERNAL_LOOPBACK 重新初始化 FDCAN，
 *   发送一帧后轮询 Rx FIFO 是否收到同一帧（ID 与数据都核对）。
 *
 * @param  [in] u32TestId 自检使用的标准帧 ID
 * @param  [out] pu8Ok    1 = 通过，0 = 失败
 * @retval 无
 */
void Mcan_InternalLoopbackSelfTest(uint32_t u32TestId, uint8_t *pu8Ok)
{
    /* 直接复用通用回环测试的实现，避免两份重复代码 */
    Mcan_LoopbackTest(FDCAN_MODE_INTERNAL_LOOPBACK, u32TestId, pu8Ok);
}

/*******************************************************************************
 * 文件结束
 ******************************************************************************/
