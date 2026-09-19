/**
 *******************************************************************************
 * @file  mcan.c
 * @brief CAN 驱动（FDCAN1，经典 CAN，波特率可配置）
 *
 *   本工程只需要 8 字节数据场的经典 CAN，用于 UDS over ISO-TP，
 *   因此 FDCAN 配置为 Classic 格式（不使用 CAN FD 的更长数据场与可变速率）。
 *
 *   引脚：
 *     PB9  = FDCAN1_TX，AF9
 *     PA11 = FDCAN1_RX，AF9
 *     PC11 = 收发器 S（Standby）控制
 *
 *   FDCAN 内核时钟 = HSE = 8 MHz（CKDIV = /1）：
 *     时间量子 tq = 1 / 8 MHz = 125 ns
 *     位时间     = (1 + PS1 + PS2) tq
 *
 *   波特率由 s_astcBaudTable 决定（本文件是唯一真值来源），
 *   8 MHz 是 125k/250k/500k/800k/1M 的公共整数倍，全部整除、误差为 0。
 *
 *   ---------------------------------------------------------------------------
 *   本文件为何是"寄存器直写"而不是 LL 调用：
 *     ST 的 STM32G4 LL 库**不提供 FDCAN 驱动**（没有 stm32g4xx_ll_fdcan.h），
 *     FDCAN 只有 HAL 实现。本工程要求 100 % 去 HAL，因此这里：
 *       - 时钟/总线/引脚仍走 LL（LL_RCC_* / LL_APB1_GRP1_* / LL_AHB2_GRP1_* /
 *         LL_GPIO_*），与其它模块风格一致；
 *       - FDCAN 自身的寄存器用 CMSIS 的 FDCAN_GlobalTypeDef / 位域宏直接访问，
 *         这正是 LL 未覆盖时的等价写法。
 *
 *   ---------------------------------------------------------------------------
 *   消息 RAM（关键，与 HAL 的差别最大）：
 *     STM32G4 的 FDCAN **没有** SIDFC / XIDFC / RXF0C / RXBC / TXEFC 这类
 *     "消息 RAM 段起始地址"配置寄存器（TXBC 也仅第 24 位 TFQM 可写），
 *     即各段的起始地址与元素个数**由硬件固定**，见 RM0440 §44.3.6 图 670：
 *
 *       段                 字节偏移  元素        占用
 *       11 位滤波器表      0x0000   28 × 1 字    112 B
 *       29 位滤波器表      0x0070    8 × 2 字     64 B
 *       Rx FIFO 0          0x00B0    3 × 18 字   216 B
 *       Rx FIFO 1          0x0188    3 × 18 字   216 B
 *       Tx 事件 FIFO       0x0260    3 × 2 字     24 B
 *       Tx 缓冲区          0x0278    3 × 18 字   216 B
 *       合计                                0x0350 B = 212 字
 *
 *     （与手册"the FDCAN module is configured to allocate up to 212 words"吻合，
 *       也解释了为何 ST 只给出 CKDIV 一个 "config" 寄存器。）
 *
 *     元素尺寸固定：Rx FIFO / Tx 缓冲区 = 2 字头 + 16 字数据（64 B 数据场）；
 *     Tx 事件 = 2 字；标准滤波元素 = 1 字；扩展滤波元素 = 2 字。
 *     元素格式见 RM0440 表 411/413/417。
 *
 *     地址按 **32 位字对齐**：手册明确"只评估 bit15~2，低两位被忽略"。
 *******************************************************************************
 * Copyright (C) 2026, all rights reserved.
 *
 * This software component is licensed under BSD 3-Clause license.
 *******************************************************************************/

/*******************************************************************************
 * 头文件
 ******************************************************************************/
#include <string.h>
#include "mcan.h"
#include "board.h"
#include "clock.h"      /* SysTick_GetTick：超时与 BusOff 恢复限流 */

#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_rcc.h"

/*******************************************************************************
 * 宏定义
 ******************************************************************************/

/* 标准滤波器数量与 ID 表 —— 由 board.h 提供，本文件不写死。
 *
 * 这样两个工程（BOOT / App）的 mcan.c 可以保持**逐字节一致**：
 *   BOOT 的 board.h 不定义这些宏 -> 默认为"只用 1 个滤波器收 0x7E0"；
 *   App  的 board.h 定义 MCAN_STD_FILTER_COUNT=2 + MCAN_EXTRA_FILTER_ID
 *        即可额外放行功能寻址 0x7DF。
 *
 * ⚠️ LL 改造后这条"逐字节一致"约定已被打破（BOOT 仍是 HAL 版），
 *    因此 App 侧现在可以自由重构本文件；但宏的语义保留，方便两工程对照。
 *
 * 统一放进同一个 Rx FIFO 0：上层用收到的 CAN ID 区分物理/功能寻址，
 * 因此无需第二个 FIFO。
 */
#ifndef MCAN_STD_FILTER_COUNT
#define MCAN_STD_FILTER_COUNT       (1U)
#endif

#define MCAN_EXT_FILTER_COUNT       (0U)

/* 物理寻址滤波器索引（额外的滤波器依次使用 +1、+2 …） */
#define MCAN_FILTER_INDEX           (0U)

/* ---------------------------------------------------------------------------
 * FDCAN1 消息 RAM 基址与固定布局（RM0440 §44.3.6 图 670）
 *
 *   SRAMCAN_BASE 由 CMSIS 提供 = APB1PERIPH_BASE + 0xA400 = 0x4000A400。
 *   （注：FDCAN1 寄存器在 0x40006400，CKDIV 在 0x40006500，
 *     消息 RAM 却在 0x4000A400 —— 是本芯片比较反直觉的一点。）
 *
 *   元素尺寸（字）：
 *     标准滤波 1、扩展滤波 2、Tx 事件 2、Rx/Tx 缓冲 18（2 头 + 16 数据）
 * ------------------------------------------------------------------------- */
#define MCAN_RAM_BASE               ((uint32_t)SRAMCAN_BASE)

#define MCAN_RAM_STD_FLT_OFF        (0x0000UL)   /* 28 元素 × 1 字  */
#define MCAN_RAM_EXT_FLT_OFF        (0x0070UL)   /*  8 元素 × 2 字  */
#define MCAN_RAM_RXF0_OFF           (0x00B0UL)   /*  3 元素 × 18 字 */
#define MCAN_RAM_RXF1_OFF           (0x0188UL)   /*  3 元素 × 18 字 */
#define MCAN_RAM_TXEF_OFF           (0x0260UL)   /*  3 元素 × 2 字  */
#define MCAN_RAM_TXBUF_OFF          (0x0278UL)   /*  3 元素 × 18 字 */

#define MCAN_RAM_RX_ELEM_BYTES      (18UL * 4UL) /* 72 B = 18 字 */
#define MCAN_RAM_TX_ELEM_BYTES      (18UL * 4UL) /* 72 B = 18 字 */

#define MCAN_STD_FILTER_MAX         (28UL)       /* 硬件上限 */
#define MCAN_TX_BUF_COUNT           (3UL)        /* 硬件固定 */
#define MCAN_RX_FIFO_COUNT          (3UL)        /* 硬件固定 */

/* ---- 标准滤波元素位域（RM0440 表 417/418；消息 RAM 不是寄存器，故无 CMSIS 宏）----
 *   [31:30] SFT  = 10  经典位掩码：SFID1 = 期望 ID，SFID2 = 掩码
 *   [29:27] SFEC = 001 匹配则存入 Rx FIFO 0
 *   [26:16] SFID1[10:0]
 *   [10:0]  SFID2[10:0]
 */
#define MCAN_SF_SFT_Pos             (30U)
#define MCAN_SF_SFEC_Pos            (27U)
#define MCAN_SF_SFID1_Pos           (16U)
#define MCAN_SF_SFID2_Pos           (0U)

#define MCAN_SF_SFT_CLASSIC_MASK    (0x2UL)      /* 10B：SFID1 为滤波器，SFID2 为掩码 */
#define MCAN_SF_SFEC_RXFIFO0        (0x1UL)      /* 001B：存 Rx FIFO 0               */
#define MCAN_SF_SFEC_DISABLE        (0x0UL)      /* 000B：禁用该元素                 */

#define MCAN_SF_ID_MASK             (0x7FFUL)    /* 11 位标准 ID */

/* ---- Rx FIFO 元素位域（RM0440 表 411/412）----
 *   R0 [31] ESI  [30] XTD  [29] RTR  [28:0] ID（标准帧放 ID[28:18]）
 *   R1 [31] ANMF [30:24] FIDX [21] FDF [20] BRS [19:16] DLC [15:0] 时间戳
 *   R2/R3 = 数据字节（低位在前）
 */
#define MCAN_RX_ELEM_ID_SHIFT       (18U)        /* 标准 ID 在 ID[28:18] */
#define MCAN_RX_ELEM_DLC_SHIFT      (16U)
#define MCAN_RX_ELEM_DLC_MASK       (0xFUL)

/* ---- Tx 缓冲元素位域（RM0440 表 413/414）----
 *   T0 [31] ESI  [30] XTD  [29] RTR  [28:0] ID
 *   T1 [31:24] MM [23] EFC [21] FDF [20] BRS [19:16] DLC
 *   T2/T3 = 数据字节（低位在前）
 */
#define MCAN_TX_ELEM_ID_SHIFT       (18U)
#define MCAN_TX_ELEM_DLC_SHIFT      (16U)

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

/* 进入 / 退出初始化模式的等待上限（ms）。
 * CCCR.INIT 的置位需等硬件完成当前发送并等到总线空闲，
 * 正常情况下几个位时间即可；上限只是防止硬件异常时死循环。 */
#define MCAN_INIT_TIMEOUT_MS        (10U)

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
 *   800 k    10    7    2    2   80.0 %    0   <- 默认
 *     1 M     8    6    1    1   87.5 %    0
 *
 *   约束：SJW <= min(PS1, PS2)（CiA 推荐 SJW <= min(PS1,PS2,4)，本表满足）。
 *   采样点：除 800 k（10 tq 无法得到 87.5 %，取最接近的 80 %）外均为推荐值。
 *
 *   ⚠️ 修改本表即改变了硬件实际波特率。BOOT 侧用的是同一张表
 *      （STM32G431_UDS_Boot/source/mcan.c），两处必须保持一致，
 *      否则 App 配好速率后 BOOT 会以不同速率通信。
 *
 *   本表的数值语义（与写入 NBTP 的关系，RM0440 §44.4.7）：
 *     硬件实际使用值 = 编程值 + 1，因此编程时需写 (PS1-1)、(PS2-1) 等。
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

/* 硬件是否已初始化。
 *
 * 原 HAL 版用 "s_stcFdcan.Instance != NULL" 表达这一状态；
 * 去 HAL 后句柄不存在了，改用显式标志 —— 语义完全等价：
 * Mcan_GetInstance()/Mcan_GetDiagnostics() 在未初始化时不会去碰寄存器，
 * 从而避免"外设时钟还没开就读寄存器"导致的硬件异常。 */
static uint8_t m_u8HwReady = 0U;

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
static volatile uint32_t *Mcan_RamWord(uint32_t u32ByteOffset);
static void     Mcan_ProgramBitTiming(const mcan_baud_entry_t *pEntry);
static int32_t  Mcan_EnterInitMode(void);
static int32_t  Mcan_LeaveInitMode(void);
static int32_t  Mcan_WriteStdFilter(uint32_t u32Index, uint32_t u32Id,
                                    uint32_t u32Mask);
static int32_t  Mcan_QueueFrame(uint32_t u32Id, const uint8_t *pu8Data,
                                uint8_t u8Len);
static int32_t  Mcan_Configure(uint32_t u32Mode, uint8_t u8AutoRetransmit);
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
 * @brief  取得消息 RAM 中指定字节偏移处的字指针。
 *
 *   消息 RAM 不是外设寄存器，而是一段普通 SRAM 区域，可直接按 32 位访问。
 *   手册要求按字对齐访问（"只有 bit15~2 参与译码"），本文件的调用点
 *   全部由固定段偏移 + 元素索引 × 元素字节数得出，天然 4 字节对齐。
 *
 * @param  [in] u32ByteOffset 相对 MCAN_RAM_BASE 的字节偏移
 * @retval volatile uint32_t* 该地址处的字指针
 */
static volatile uint32_t *Mcan_RamWord(uint32_t u32ByteOffset)
{
    return (volatile uint32_t *)(MCAN_RAM_BASE + u32ByteOffset);
}

/**
 * @brief  FDCAN 时钟源 + 外设时钟 + 引脚配置（Mcan_Init 与自检共用）。
 *
 *   这部分必须与后续的控制器配置分开：自检需要在正常初始化之前
 *   独立完成时钟与引脚配置，否则外设无时钟、寄存器读写无效。
 *
 *   ⚠️ 时钟源选 **HSE（8 MHz）** 而不是 PLLQ：
 *     8 MHz 能整除 125k/250k/500k/800k/1M 全部目标速率（误差为 0），
 *     而 42.5 MHz 无法整除 800 k。且由晶振直接驱动，无 PLL 抖动。
 *     HSE 已在 Board_ClockInit() 中使能，此处只切换 FDCANSEL。
 *
 * @param  无
 * @retval int32_t MCAN_OK 成功（LL 直写寄存器，无失败分支）
 */
static int32_t Mcan_ClockAndGpioInit(void)
{
    /* 选择 FDCAN 内核时钟源 = HSE（8 MHz）。
     * 不选 PCLK1 是为了让位时序独立于 APB1 分频，便于后续调整主频。
     * HAL 里这一步是 HAL_RCCEx_PeriphCLKConfig()，LL 下就是一条 MODIFY_REG。 */
    LL_RCC_SetFDCANClockSource(LL_RCC_FDCAN_CLKSOURCE_HSE);

    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_FDCAN);
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
 *   LL 与 HAL 的差异：HAL 用一个 GPIO_InitTypeDef 一次配好一个引脚；
 *   LL 需按"模式/输出类型/速度/上下拉/复用号"分别调用，效果一致。
 *   复用号寄存器 AFR 分 AFR[0]（pin0~7）与 AFR[1]（pin8~15）两组，
 *   本板两个引脚都在 8~15，故用 LL_GPIO_SetAFPin_8_15()。
 *
 * @param  无
 * @retval 无
 */
static void Mcan_GpioConfig(void)
{
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA |
                             LL_AHB2_GRP1_PERIPH_GPIOB |
                             LL_AHB2_GRP1_PERIPH_GPIOC);

    /* ---- TX / RX：复用推挽；上拉使总线悬空时保持隐性电平 ---- */
    LL_GPIO_SetPinMode(CAN_TX_GPIO_PORT, CAN_TX_GPIO_PIN, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinOutputType(CAN_TX_GPIO_PORT, CAN_TX_GPIO_PIN,
                             LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinSpeed(CAN_TX_GPIO_PORT, CAN_TX_GPIO_PIN,
                        LL_GPIO_SPEED_FREQ_VERY_HIGH);
    LL_GPIO_SetPinPull(CAN_TX_GPIO_PORT, CAN_TX_GPIO_PIN, LL_GPIO_PULL_UP);
    LL_GPIO_SetAFPin_8_15(CAN_TX_GPIO_PORT, CAN_TX_GPIO_PIN, CAN_GPIO_AF);

    LL_GPIO_SetPinMode(CAN_RX_GPIO_PORT, CAN_RX_GPIO_PIN, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinOutputType(CAN_RX_GPIO_PORT, CAN_RX_GPIO_PIN,
                             LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinSpeed(CAN_RX_GPIO_PORT, CAN_RX_GPIO_PIN,
                        LL_GPIO_SPEED_FREQ_VERY_HIGH);
    LL_GPIO_SetPinPull(CAN_RX_GPIO_PORT, CAN_RX_GPIO_PIN, LL_GPIO_PULL_UP);
    LL_GPIO_SetAFPin_8_15(CAN_RX_GPIO_PORT, CAN_RX_GPIO_PIN, CAN_GPIO_AF);

    /* ---- 收发器 S（Standby）引脚：必须显式拉到 Normal 电平 ---- */
    LL_GPIO_SetPinMode(CAN_STB_GPIO_PORT, CAN_STB_GPIO_PIN, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinOutputType(CAN_STB_GPIO_PORT, CAN_STB_GPIO_PIN,
                             LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinSpeed(CAN_STB_GPIO_PORT, CAN_STB_GPIO_PIN,
                        LL_GPIO_SPEED_FREQ_LOW);
    LL_GPIO_SetPinPull(CAN_STB_GPIO_PORT, CAN_STB_GPIO_PIN, LL_GPIO_PULL_NO);

#if (CAN_STB_ACTIVE_HIGH != 0)
    LL_GPIO_SetOutputPin(CAN_STB_GPIO_PORT, CAN_STB_GPIO_PIN);
#else
    LL_GPIO_ResetOutputPin(CAN_STB_GPIO_PORT, CAN_STB_GPIO_PIN);
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
 * @brief  置 CCCR.INIT，进入初始化/配置模式。
 *
 *   M_CAN 要求：只有在 INIT=1 且 CCE=1 时才能写位时序等受保护寄存器。
 *   置位后需等硬件确认（硬件要等当前帧发完、总线空闲才真正进入），
 *   手册规定读回 CCCR.INIT 为 1 才算进入。
 *
 * @param  无
 * @retval int32_t MCAN_OK 已进入；MCAN_ERR 超时
 */
static int32_t Mcan_EnterInitMode(void)
{
    uint32_t u32Start = SysTick_GetTick();

    SET_BIT(FDCAN1->CCCR, FDCAN_CCCR_INIT);

    while ((FDCAN1->CCCR & FDCAN_CCCR_INIT) == 0U) {
        if ((SysTick_GetTick() - u32Start) >= MCAN_INIT_TIMEOUT_MS) {
            return MCAN_ERR;
        }
    }

    /* CCE = 1：打开受保护配置寄存器的写权限 */
    SET_BIT(FDCAN1->CCCR, FDCAN_CCCR_CCE);

    return MCAN_OK;
}

/**
 * @brief  清 CCCR.INIT，退出初始化模式并接入总线。
 * @param  无
 * @retval int32_t MCAN_OK 已退出；MCAN_ERR 超时
 */
static int32_t Mcan_LeaveInitMode(void)
{
    uint32_t u32Start = SysTick_GetTick();

    CLEAR_BIT(FDCAN1->CCCR, FDCAN_CCCR_CCE);
    CLEAR_BIT(FDCAN1->CCCR, FDCAN_CCCR_INIT);

    while ((FDCAN1->CCCR & FDCAN_CCCR_INIT) != 0U) {
        if ((SysTick_GetTick() - u32Start) >= MCAN_INIT_TIMEOUT_MS) {
            return MCAN_ERR;
        }
    }

    return MCAN_OK;
}

/**
 * @brief  写入一个标准 ID 滤波元素。
 *
 *   元素格式见 RM0440 表 417/418（消息 RAM 不是寄存器，无 CMSIS 位域宏，
 *   故位域在本文件开头自行定义）。
 *
 *   本工程用"经典位掩码"类型（SFT = 10）：
 *     SFID1 = 期望 ID，SFID2 = 掩码，掩码为 1 的位参与比较。
 *     掩码 0x7FF（全 1）表示 11 位 ID 必须逐位完全相同 —— 正是验收滤波
 *     想要的效果（只收 UDS 请求 ID）。
 *
 * @param  [in] u32Index 滤波器索引（0 ~ 27）
 * @param  [in] u32Id    期望的标准 ID（11 位）
 * @param  [in] u32Mask  掩码（11 位）
 * @retval int32_t MCAN_OK 成功；MCAN_ERR 索引越界
 */
static int32_t Mcan_WriteStdFilter(uint32_t u32Index, uint32_t u32Id,
                                   uint32_t u32Mask)
{
    uint32_t u32Elem;

    if (u32Index >= MCAN_STD_FILTER_MAX) {
        return MCAN_ERR;
    }

    u32Elem = ((MCAN_SF_SFT_CLASSIC_MASK << MCAN_SF_SFT_Pos)          |
               (MCAN_SF_SFEC_RXFIFO0    << MCAN_SF_SFEC_Pos)          |
               ((u32Id   & MCAN_SF_ID_MASK) << MCAN_SF_SFID1_Pos)     |
               ((u32Mask & MCAN_SF_ID_MASK) << MCAN_SF_SFID2_Pos));

    *Mcan_RamWord(MCAN_RAM_STD_FLT_OFF + (u32Index * 4UL)) = u32Elem;

    return MCAN_OK;
}

/**
 * @brief  按位时序表项编程 NBTP（标称位时序）。
 *
 *   寄存器字段（RM0440 §44.4.7）：
 *     [31:25] NSJW[6:0]   [24:16] NBRP[8:0]   [15:8] NTSEG1[7:0]   [6:0] NTSEG2[6:0]
 *   硬件实际使用值 = 编程值 + 1，故全部需要减 1 再写。
 *
 *   DBTP（数据段位时序）在经典 CAN 格式下不参与，但为保持寄存器处于
 *   合法值（也便于日后若启用 CAN FD 时复用）显式写 0：
 *   对应 DBRP=1、DTSEG1=1、DTSEG2=1、DSJW=1，即 HAL 的默认填法。
 *
 * @param  [in] pEntry 位时序表项（不可为 NULL）
 * @retval 无
 */
static void Mcan_ProgramBitTiming(const mcan_baud_entry_t *pEntry)
{
    FDCAN1->NBTP = (((uint32_t)pEntry->u32Sjw - 1UL) << FDCAN_NBTP_NSJW_Pos)      |
                   (((uint32_t)pEntry->u32Prescaler - 1UL) << FDCAN_NBTP_NBRP_Pos) |
                   (((uint32_t)pEntry->u32Ps1 - 1UL) << FDCAN_NBTP_NTSEG1_Pos)     |
                   (((uint32_t)pEntry->u32Ps2 - 1UL) << FDCAN_NBTP_NTSEG2_Pos);

    /* 经典 CAN：数据段不生效，显式清零（= 分频/各段均为 1） */
    FDCAN1->DBTP = 0U;
}

/**
 * @brief  配置控制器本体：帧格式、工作模式、位时序、滤波器、Tx 队列。
 *
 *   调用前提：时钟与引脚已就绪（Mcan_ClockAndGpioInit），
 *   且**不在**初始化模式（本函数内部会自行进入/退出）。
 *
 * @param  [in] u32Mode            MCAN_MODE_* 之一
 * @param  [in] u8AutoRetransmit   1 = 允许自动重传，0 = 关闭（回环自检用）
 * @retval int32_t MCAN_OK 成功，MCAN_ERR 失败
 */
static int32_t Mcan_Configure(uint32_t u32Mode, uint8_t u8AutoRetransmit)
{
    uint32_t u32CccrSet = 0U;
    uint32_t u32Test    = 0U;

    const mcan_baud_entry_t *pEntry = Mcan_FindBaudEntry(m_u32Baudrate);

    if (pEntry == NULL) {
        return MCAN_ERR;
    }

    /* 0. 内核时钟分频 CKDIV（PDIV = 0000 -> /1）。
     *
     *    ⚠️ 该寄存器不在 FDCAN1 的寄存器块里，而在独立的配置窗口
     *       FDCAN_CONFIG_BASE = 0x40006500（CMSIS 用 FDCAN_Config_TypeDef
     *       表示，且只有一个成员 CKDIV）。它是所有 FDCAN 实例共用的，
     *       因此必须在配置实例之前设置。 */
    FDCAN_CONFIG->CKDIV = 0U;

    /* 1. 进入初始化模式（INIT=1、CCE=1） */
    if (Mcan_EnterInitMode() != MCAN_OK) {
        return MCAN_ERR;
    }

    /* 2. 工作模式 / 帧格式（全部是 CCCR 中受 CCE 保护的位）
     *
     *    FDOE = 0、BRSE = 0 -> 经典 CAN（本工程不需要 CAN FD）
     *    MON / ASM / TEST = 0 -> 正常模式（按 u32Mode 覆盖）
     *    DAR = 0 -> 允许自动重传；DAR = 1 -> 关闭
     *    TXP = 1 -> 两次传输之间插入 2 个位时间（等价 HAL TransmitPause=ENABLE，
     *              避免连续帧太密集时收发器来不及释放总线）
     *    PXHD = 1 -> 关闭协议异常处理（等价 HAL ProtocolException=DISABLE）
     */
    switch (u32Mode) {
    case MCAN_MODE_INTERNAL_LOOPBACK:
        /* 内部回环：MON = 1（不发 ACK / 错误帧） + TEST.LBCK = 1，
         * 且 TX 不驱动引脚（TEST.TX = 00 保持不变即为该行为）。 */
        u32CccrSet |= FDCAN_CCCR_MON;
        u32Test    |= FDCAN_TEST_LBCK;
        break;

    case MCAN_MODE_EXTERNAL_LOOPBACK:
        /* 外部回环：只置 TEST.LBCK，TX 引脚仍真实驱动 */
        u32Test    |= FDCAN_TEST_LBCK;
        break;

    case MCAN_MODE_NORMAL:
    default:
        break;
    }

    if (u8AutoRetransmit == 0U) {
        u32CccrSet |= FDCAN_CCCR_DAR;
    }

    u32CccrSet |= FDCAN_CCCR_TXP | FDCAN_CCCR_PXHD;

    MODIFY_REG(FDCAN1->CCCR,
               FDCAN_CCCR_FDOE | FDCAN_CCCR_BRSE | FDCAN_CCCR_DAR   |
               FDCAN_CCCR_TXP  | FDCAN_CCCR_PXHD | FDCAN_CCCR_MON   |
               FDCAN_CCCR_ASM  | FDCAN_CCCR_TEST,
               u32CccrSet);

    /* TEST 寄存器仅在 CCCR.TEST = 1 时才能写 */
    if (u32Test != 0U) {
        SET_BIT(FDCAN1->CCCR, FDCAN_CCCR_TEST);
        FDCAN1->TEST = u32Test;
    } else {
        CLEAR_BIT(FDCAN1->CCCR, FDCAN_CCCR_TEST);
    }

    /* 3. 位时序 */
    Mcan_ProgramBitTiming(pEntry);

    /* 4. Tx 缓冲区工作方式：TFQM = 0 -> Tx FIFO（先进先出）。
     *    ⚠️ TXBC 是消息 RAM 配置寄存器里唯一可写的（本芯片其余段地址固定），
     *       bit23:0 全为保留位，只有 bit24 TFQM 可用。 */
    CLEAR_BIT(FDCAN1->TXBC, FDCAN_TXBC_TFQM);

    /* 5. 验收滤波器：放行 UDS 请求 ID 到 Rx FIFO 0 */
    (void)Mcan_WriteStdFilter(MCAN_FILTER_INDEX, UDS_CAN_RX_ID, MCAN_SF_ID_MASK);

#ifdef MCAN_EXTRA_FILTER_ID
    /* 额外的滤波器（App 侧的功能寻址 0x7DF）与物理寻址共用同一个 Rx FIFO 0 */
    (void)Mcan_WriteStdFilter((uint32_t)(MCAN_FILTER_INDEX + 1U),
                              MCAN_EXTRA_FILTER_ID, MCAN_SF_ID_MASK);
#endif

    /* 6. 全局滤波配置（RXGFC，受 CCE 保护）
     *      LSS = 标准滤波表长度（硬件只扫描前 LSS 个元素）
     *      LSE = 0（不用扩展 ID）
     *      ANFS = 10 -> 未匹配的标准帧**拒绝**（HAL 的 FDCAN_REJECT 同值）
     *      ANFE = 10 -> 未匹配的扩展帧同样拒绝
     *      RRFS / RRFE = 1 -> 拒绝远程帧
     *      F0OM / F1OM = 0 -> FIFO 阻塞模式（满则丢弃新帧，不覆盖旧帧） */
    FDCAN1->RXGFC = ((uint32_t)MCAN_STD_FILTER_COUNT << FDCAN_RXGFC_LSS_Pos) |
                    ((uint32_t)MCAN_EXT_FILTER_COUNT << FDCAN_RXGFC_LSE_Pos) |
                    (0x2UL << FDCAN_RXGFC_ANFS_Pos)                          |
                    (0x2UL << FDCAN_RXGFC_ANFE_Pos)                          |
                    FDCAN_RXGFC_RRFS | FDCAN_RXGFC_RRFE;

    /* 7. 清掉上电/复位后可能残留的中断标志，避免误判 */
    FDCAN1->IR = 0xFFFFFFFFU;

    /* 8. 退出初始化模式，正式接入总线 */
    if (Mcan_LeaveInitMode() != MCAN_OK) {
        return MCAN_ERR;
    }

    m_u8HwReady = 1U;

    return MCAN_OK;
}

/**
 * @brief  把一帧塞进 Tx FIFO 并触发发送（不做统计、不做参数校验）。
 *
 *   之所以把"入队"单独抽出来：Mcan_Send() 需要维护发送统计并处理 BusOff，
 *   而回环自检只想纯粹地发一帧、不想污染统计值。
 *
 *   元素格式见 RM0440 表 413/414：
 *     T0 [31] ESI=0 [30] XTD=0（标准帧） [29] RTR=0（数据帧）
 *        [28:0] ID（标准 ID 必须写在 ID[28:18]，故左移 18）
 *     T1 [31:24] MM=0 [23] EFC=0（不产生 Tx 事件） [21] FDF=0（经典帧）
 *        [20] BRS=0 [19:16] DLC
 *     T2/T3 数据字节（低位在前：DB3|DB2|DB1|DB0）
 *
 *   写入顺序也很关键：必须**先写数据、最后写 TXBAR**。先置 TXBAR 会让硬件
 *   在元素还没写完时就去发送（总线空闲时几乎是立即发起），导致帧内容错乱。
 *
 * @param  [in] u32Id    标准帧 ID（11 位）
 * @param  [in] pu8Data  数据指针（不可为 NULL）
 * @param  [in] u8Len    数据长度（0 ~ 8）
 * @retval int32_t MCAN_OK 已入队；MCAN_ERR 无空位或参数非法
 */
static int32_t Mcan_QueueFrame(uint32_t u32Id, const uint8_t *pu8Data,
                               uint8_t u8Len)
{
    volatile uint32_t *pElem;
    uint32_t u32PutIndex;
    uint32_t u32Dlc;
    uint32_t u32Data2 = 0U;
    uint32_t u32Data3 = 0U;
    uint32_t i;

    if ((pu8Data == NULL) || (u8Len > MCAN_PAYLOAD_MAX)) {
        return MCAN_ERR;
    }

    /* Tx FIFO 已满（TFFL = 0）则失败，由调用方决定等待还是放弃 */
    if ((FDCAN1->TXFQS & FDCAN_TXFQS_TFFL_Msk) == 0U) {
        return MCAN_ERR;
    }

    u32Dlc      = Mcan_BytesToDlc(u8Len);
    u32PutIndex = (FDCAN1->TXFQS & FDCAN_TXFQS_TFQPI_Msk) >>
                  FDCAN_TXFQS_TFQPI_Pos;

    /* 数据先装箱再一次性写元素，避免逐字节读改写（也避免发到 SRAM 残留数据） */
    for (i = 0U; i < u8Len; i++) {
        uint32_t u32Shift = (i % 4U) * 8U;

        if (i < 4U) {
            u32Data2 |= ((uint32_t)pu8Data[i] << u32Shift);
        } else {
            u32Data3 |= ((uint32_t)pu8Data[i] << u32Shift);
        }
    }

    pElem = Mcan_RamWord(MCAN_RAM_TXBUF_OFF +
                         (u32PutIndex * MCAN_RAM_TX_ELEM_BYTES));

    pElem[0] = ((u32Id & MCAN_SF_ID_MASK) << MCAN_TX_ELEM_ID_SHIFT);
    pElem[1] = (u32Dlc << MCAN_TX_ELEM_DLC_SHIFT);
    pElem[2] = u32Data2;
    pElem[3] = u32Data3;

    /* 置 TXBAR 对应位 -> 硬件开始发送 */
    FDCAN1->TXBAR = (1UL << u32PutIndex);

    return MCAN_OK;
}

/**
 * @brief  初始化 FDCAN1（经典 CAN，正常模式）。
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
    /* 0. 波特率必须在位时序表中（否则位时序无从取值） */
    if (Mcan_FindBaudEntry(m_u32Baudrate) == NULL) {
        return MCAN_ERR;
    }

    /* 1. 时钟源 + 外设时钟 + 引脚（与自检共用同一份配置） */
    if (Mcan_ClockAndGpioInit() != MCAN_OK) {
        return MCAN_ERR;
    }

    /* 2. 配置控制器（正常模式，允许自动重传） */
    return Mcan_Configure(MCAN_MODE_NORMAL, 1U);
}

/**
 * @brief  反初始化 FDCAN1（停止控制器并复位外设寄存器）。
 *         供跳转 App 前清理现场，避免残留状态干扰 App 的初始化。
 *
 *   HAL 的 HAL_FDCAN_DeInit 做两件事：把 CCCR.INIT 置 1 停总线，
 *   再复位外设。这里用 LL 的总线复位 API 等价实现。
 *
 *   ⚠️ 复位**不会**清除消息 RAM（它是普通 SRAM），不过 Mcan_Init 每次都
 *      会重写滤波器并重设 LSS，因此不影响正确性。
 *
 * @param  无
 * @retval int32_t MCAN_OK 成功，MCAN_ERR 失败
 */
int32_t Mcan_DeInit(void)
{
    if (m_u8HwReady == 0U) {
        return MCAN_OK;
    }

    /* 先进入 INIT 让控制器停止收发（等待总线空闲） */
    if (Mcan_EnterInitMode() != MCAN_OK) {
        return MCAN_ERR;
    }

    m_u8HwReady = 0U;

    /* 复位 FDCAN 外设：APB1RSTR1.FDCANRST 置位再释放 */
    LL_APB1_GRP1_ForceReset(LL_APB1_GRP1_PERIPH_FDCAN);
    LL_APB1_GRP1_ReleaseReset(LL_APB1_GRP1_PERIPH_FDCAN);

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
 *   为什么必须有这一步：Mcan_QueueFrame() 只是把帧塞进 Tx FIFO 就返回，
 *   并不代表已经发上总线。若紧接着进入 INIT（停止控制器），未发出的帧
 *   会被丢弃 —— 这正是"改完波特率后上位机收不到 0x6E 应答"的原因。
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

    if (m_u8HwReady == 0U) {
        return 1U;
    }

    while ((FDCAN1->TXBRP & FDCAN_TXBRP_TRP_Msk) != 0U) {
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
 *   运行时切换波特率用。必须真正复位并重新配置外设：位时序寄存器
 *   只有在 INIT + CCE 下才能改写，且 CKDIV 也需重新写入。
 *
 *   ⚠️ 调用前会先把待发帧送完 —— 否则切换时的停止会把它们丢掉。
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

    /* 复位外设后再初始化，确保位时序与 CKDIV 被真正重写 */
    (void)Mcan_DeInit();

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
    uint32_t u32Timeout = MCAN_TX_TIMEOUT;

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
    while ((FDCAN1->TXFQS & FDCAN_TXFQS_TFFL_Msk) == 0U) {
        if ((FDCAN1->PSR & FDCAN_PSR_BO) != 0U) {
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

    if (Mcan_QueueFrame(u32Id, pu8Data, u8Len) != MCAN_OK) {
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
    volatile uint32_t *pElem;
    uint32_t u32GetIndex;
    uint32_t u32R0;
    uint32_t u32R1;
    uint32_t u32Dlc;
    uint32_t u32Data[2];
    uint8_t  u8Bytes;
    uint32_t i;

    /* FIFO 为空则立即返回，不做任何等待 */
    if ((FDCAN1->RXF0S & FDCAN_RXF0S_F0FL_Msk) == 0U) {
        return MCAN_ERR;
    }

    /* 取索引：RXF0S.F0GI 指向"下一个待读元素"，据此算出元素地址 */
    u32GetIndex = (FDCAN1->RXF0S & FDCAN_RXF0S_F0GI_Msk) >>
                  FDCAN_RXF0S_F0GI_Pos;

    pElem = Mcan_RamWord(MCAN_RAM_RXF0_OFF +
                         (u32GetIndex * MCAN_RAM_RX_ELEM_BYTES));

    u32R0 = pElem[0];
    u32R1 = pElem[1];
    u32Data[0] = pElem[2];
    u32Data[1] = pElem[3];

    /* DLC 是编码值，需查表换算为实际字节数。
     * 本工程只接收经典 CAN 帧，DLC 0~8 与字节数一一对应
     * （DLC 9~15 在经典 CAN 下表示 8 字节）。 */
    u32Dlc  = (u32R1 >> MCAN_RX_ELEM_DLC_SHIFT) & MCAN_RX_ELEM_DLC_MASK;
    u8Bytes = (u32Dlc < 16U) ? s_au8DlcToBytes[u32Dlc] : 8U;
    if (u8Bytes > MCAN_PAYLOAD_MAX) {
        u8Bytes = MCAN_PAYLOAD_MAX;
    }

    /* 标准帧的 11 位 ID 存放在 ID[28:18] */
    m_u32LastRxId = (u32R0 >> MCAN_RX_ELEM_ID_SHIFT) & MCAN_SF_ID_MASK;
    m_u32RxCnt++;

    if (pu32Id != NULL) {
        *pu32Id = m_u32LastRxId;
    }
    if (pu8Data != NULL) {
        /* 未用到的字节清 0：上层按固定 8 字节缓冲处理，
         * 残留旧数据会让调试时的十六进制显示产生误解 */
        for (i = 0U; i < MCAN_PAYLOAD_MAX; i++) {
            pu8Data[i] = 0U;
        }
        for (i = 0U; i < u8Bytes; i++) {
            pu8Data[i] = (uint8_t)((u32Data[i / 4U] >> ((i % 4U) * 8U)) & 0xFFUL);
        }
    }
    if (pu8Len != NULL) {
        *pu8Len = u8Bytes;
    }

    /* 写 RXF0A：告诉硬件"索引为 u32GetIndex 的元素已取走"，
     * 硬件据此推进 get index 并更新填充度。不写这一笔，FIFO 很快会满。 */
    FDCAN1->RXF0A = u32GetIndex;

    return MCAN_OK;
}

/**
 * @brief  清空接收 FIFO 中的残留帧。
 * @param  无
 * @retval 无
 */
void Mcan_FlushRx(void)
{
    while ((FDCAN1->RXF0S & FDCAN_RXF0S_F0FL_Msk) != 0U) {
        uint32_t u32GetIndex = (FDCAN1->RXF0S & FDCAN_RXF0S_F0GI_Msk) >>
                               FDCAN_RXF0S_F0GI_Pos;

        FDCAN1->RXF0A = u32GetIndex;
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
 *
 *   未初始化时返回 NULL —— 与 HAL 版的行为一致（原实现直接返回句柄的
 *   Instance 成员，初始化前该成员为 NULL）。这样调用方可以安全地用
 *   "非 NULL" 判断外设是否已可用，不会读到未开时钟的寄存器。
 *
 * @param  无
 * @retval FDCAN_GlobalTypeDef* FDCAN1 寄存器基址；未初始化时为 NULL
 */
FDCAN_GlobalTypeDef *Mcan_GetInstance(void)
{
    return (m_u8HwReady != 0U) ? FDCAN1 : NULL;
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
     * 此时不能去碰寄存器（外设时钟未开），直接返回零值即可。 */
    if (m_u8HwReady == 0U) {
        pDiag->u8ErrorState  = 0U;
        pDiag->u8TxErrorCnt  = 0U;
        pDiag->u8RxErrorCnt  = 0U;
        pDiag->u8LastErrCode = 0U;
        pDiag->u8Cccr        = 0U;
        pDiag->u8TxFreeLevel = 0U;
        return;
    }

    /* ECR：TEC[7:0] 发送错误计数，REC[14:8] 接收错误计数 */
    pDiag->u8TxErrorCnt = (uint8_t)(FDCAN1->ECR & FDCAN_ECR_TEC_Msk);
    pDiag->u8RxErrorCnt = (uint8_t)((FDCAN1->ECR & FDCAN_ECR_REC_Msk) >>
                                    FDCAN_ECR_REC_Pos);

    /* PSR：LEC[2:0] 最近一次错误码，BO[7] 总线关闭，EP[5] 错误被动 */
    u32Psr = FDCAN1->PSR;
    pDiag->u8LastErrCode = (uint8_t)(u32Psr & FDCAN_PSR_LEC_Msk);

    if ((u32Psr & FDCAN_PSR_BO) != 0U) {
        pDiag->u8ErrorState = 2U;       /* 总线关闭（Bus Off） */
    } else if ((u32Psr & FDCAN_PSR_EP) != 0U) {
        pDiag->u8ErrorState = 1U;       /* 错误被动 */
    } else {
        pDiag->u8ErrorState = 0U;       /* 错误主动（正常） */
    }

    /* CCCR 低 8 位：bit0=INIT, bit1=CCE, bit5=MON, bit7=ASM */
    pDiag->u8Cccr = (uint8_t)(FDCAN1->CCCR & 0xFFU);

    /* TXFQS.TFFL：Tx FIFO 剩余空位数 */
    pDiag->u8TxFreeLevel = (uint8_t)(FDCAN1->TXFQS & FDCAN_TXFQS_TFFL_Msk);
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
    uint32_t u32Now;

    if (m_u8HwReady == 0U) {
        return 0U;
    }

    /* 既没 Bus Off、也不在初始化模式：无需处理 */
    if (((FDCAN1->PSR  & FDCAN_PSR_BO)    == 0U) &&
        ((FDCAN1->CCCR & FDCAN_CCCR_INIT) == 0U)) {
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
    SET_BIT(FDCAN1->CCCR, FDCAN_CCCR_INIT);
    SET_BIT(FDCAN1->CCCR, FDCAN_CCCR_CCE);

    /* 清掉队列里已无意义的挂起请求，否则 FIFO 会一直占满，
     * 后续正常发送也进不来。写 1 到 TXBCR 即请求取消对应缓冲。 */
    FDCAN1->TXBCR = FDCAN1->TXBRP;

    CLEAR_BIT(FDCAN1->CCCR, FDCAN_CCCR_INIT);

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
 * @param  [in]  u32Mode   MCAN_MODE_INTERNAL_LOOPBACK / MCAN_MODE_EXTERNAL_LOOPBACK
 * @param  [in]  u32TestId 自检使用的标准帧 ID
 * @param  [out] pu8Ok     1 = 通过，0 = 失败
 * @retval 无
 */
void Mcan_LoopbackTest(uint32_t u32Mode, uint32_t u32TestId, uint8_t *pu8Ok)
{
    uint8_t  au8TxData[MCAN_PAYLOAD_MAX] = {0};
    uint8_t  au8RxData[MCAN_PAYLOAD_MAX] = {0};
    uint8_t  u8RxLen = 0U;
    uint32_t u32RxId = 0U;
    uint32_t u32Wait;
    uint32_t u32Timeout;

    if (pu8Ok == NULL) {
        return;
    }
    *pu8Ok = 0U;

    /* 位时序同样按当前波特率查表；查不到则自检无意义，直接判失败 */
    if (Mcan_FindBaudEntry(m_u32Baudrate) == NULL) {
        return;
    }

    au8TxData[0] = (uint8_t)(u32TestId & 0xFFU);
    au8TxData[1] = 0xA5U;
    au8TxData[2] = 0x5AU;

    /* 时钟 / 引脚必须已就绪 */
    if (Mcan_ClockAndGpioInit() != MCAN_OK) {
        return;
    }

    /* 以回环模式重新初始化：关闭自动重传（回环下无人应答，
     * 开着重传会一直重发、白等超时） */
    m_u8HwReady = 0U;
    if (Mcan_Configure(u32Mode, 0U) != MCAN_OK) {
        return;
    }

    /* 等有空位（回环下 FIFO 必然是空的，这里的等待只是兜底） */
    u32Timeout = MCAN_TX_TIMEOUT;
    while (((FDCAN1->TXFQS & FDCAN_TXFQS_TFFL_Msk) == 0U) &&
           (u32Timeout != 0UL)) {
        u32Timeout--;
    }

    /* 直接入队：不更新发送统计，避免诊断计数被自检污染 */
    if (Mcan_QueueFrame(u32TestId, au8TxData, 3U) != MCAN_OK) {
        return;
    }

    /* 轮询接收：回环下应很快收到自己发的帧 */
    for (u32Wait = 0U; u32Wait < MCAN_SELFTEST_TIMEOUT; u32Wait++) {
        if (Mcan_Receive(&u32RxId, au8RxData, &u8RxLen) == MCAN_OK) {
            if ((u32RxId == (u32TestId & MCAN_SF_ID_MASK)) &&
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
 *   实现要点：以 MCAN_MODE_INTERNAL_LOOPBACK 重新初始化 FDCAN，
 *   发送一帧后轮询 Rx FIFO 是否收到同一帧（ID 与数据都核对）。
 *
 * @param  [in] u32TestId 自检使用的标准帧 ID
 * @param  [out] pu8Ok    1 = 通过，0 = 失败
 * @retval 无
 */
void Mcan_InternalLoopbackSelfTest(uint32_t u32TestId, uint8_t *pu8Ok)
{
    /* 直接复用通用回环测试的实现，避免两份重复代码 */
    Mcan_LoopbackTest(MCAN_MODE_INTERNAL_LOOPBACK, u32TestId, pu8Ok);
}

/*******************************************************************************
 * 文件结束
 ******************************************************************************/
