/**
 *******************************************************************************
 * @file  board.h
 * @brief 板级配置（Application 侧）——存储器映射、时钟、CAN 引脚、状态灯
 *
 *   目标芯片: STM32G431CBU6 (Cortex-M4F, 48 引脚, UFQFPN48)
 *     Flash : 128 KB @ 0x08000000（页大小 2 KB，共 64 页）
 *     SRAM  :  32 KB @ 0x20000000（SRAM1 16K + SRAM2 6K，另 CCM 10K）
 *     晶振  : 8 MHz 外部晶振 (HSE)
 *     主频  : 170 MHz
 *
 *   IMPORTANT: APP_START_ADDR 必须与 Bootloader 的配置以及链接脚本的
 *   ORIGIN 完全一致，否则 Bootloader 会以 NRC 0x31 拒绝 0x34 RequestDownload。
 *
 *   本文件内容必须与 STM32G431_UDS_Boot/drivers/board.h 保持同步
 *   （尤其是 Flash 分区、CAN 引脚与位时序）。
 *******************************************************************************
 * Copyright (C) 2026, all rights reserved.
 *
 * This software component is licensed under BSD 3-Clause license.
 *******************************************************************************
 */
#ifndef __BOARD_H__
#define __BOARD_H__

/* 本工程 100 % 使用 LL 库，不再包含 HAL 头。
 * stm32g4xx.h 会按 STM32G431xx 宏选入 stm32g431xx.h，
 * 提供 TAMP/FDCAN/GPIO/RCC 等全部外设寄存器定义。
 *
 * 另外需要 LL 的 GPIO 头：引脚掩码与复用功能号都用 LL 的命名
 * （LL_GPIO_PIN_x / LL_GPIO_AF_x），HAL 的 GPIO_PIN_x / GPIO_AFx_xxx
 * 已随 HAL 一起移出本工程。该头是纯 inline 实现，不引入任何源码。 */
#include "stm32g4xx.h"
#include "stm32g4xx_ll_gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================== 一、时钟配置 ==============================
 *   PLLM = /2  -> 4 MHz；VCO = 4 * 85 = 340 MHz
 *   PLLR = /2  -> 170 MHz（CK_SYS）
 *   PLLQ = /8  -> 42.5 MHz   本工程**不再用于 FDCAN**，保留原值不动 PLL
 *
 * FDCAN 内核时钟源 = **HSE（8 MHz）**，不是 PLLQ。
 *   原因同 Bootloader：8 MHz 是 125k/250k/500k/800k/1M 的公共整数倍，
 *   每位都能整除成整数个 tq（波特率误差为 0）；42.5 MHz 无法整除 800 k。
 *   由晶振直接驱动 CAN，也没有 PLL 抖动影响位时序。
 */
#define BOARD_HSE_FREQ                (8000000UL)
#define BOARD_SYSCLK_FREQ             (170000000UL)

/* FDCAN 内核时钟（直接来自 HSE，CKDIV = /1） */
#define BOARD_FDCAN_CLK_FREQ          (8000000UL)

/* ============================= 二、Flash 分区 =============================
 *   第 0 ~ 11 页 : 0x08000000 ~ 0x08005FFF   24 KB  Bootloader
 *   第 12 ~ 63 页: 0x08006000 ~ 0x0801FFFF  104 KB  Application
 *
 * 命名说明：这里刻意不叫 FLASH_PAGE_SIZE —— HAL 的 stm32g4xx_hal_flash.h
 *           曾用该名字定义 0x800U（同为 2 KB 页）。本工程已 100 % 改用 LL
 *           （LL 不提供该宏），保留 BOARD_ 前缀可避免日后引入其它库时撞名。
 */
#define BOARD_FLASH_PAGE_SIZE         (0x00000800UL)   /* 2 KB */
#define FLASH_PAGE_COUNT              (64U)            /* 128 KB / 2 KB */

#define BOOT_REGION_START             (0x08000000UL)
#define BOOT_REGION_SIZE              (0x00006000UL)   /* 24 KB */
#define APP_START_ADDR                (0x08006000UL)
#define APP_REGION_START              (APP_START_ADDR)
#define APP_REGION_END                (0x0801FFFFUL)
#define APP_MAX_SIZE                  (APP_REGION_END - APP_REGION_START + 1UL)

/* 绝对向量表地址（SCB->VTOR 接收绝对地址） */
#define APP_VECT_TAB_ADDR             (APP_START_ADDR)

/* 有效的 RAM 区间（供 Bootloader 校验 App 的栈顶指针）
 * 注意：STM32G431CB 的 SRAM 为 22 KB（SRAM1 16K + SRAM2 6K），
 *       末端地址是 0x200057FF，不是 RB 型号的 0x20007FFF。 */
#define BOARD_RAM_START               (0x20000000UL)
#define BOARD_RAM_END                 (0x200057FFUL)

/* 合法的初始栈顶（MSP）上限：RAM 末端 + 1（栈向下生长） */
#define BOARD_MSP_MAX                 (BOARD_RAM_END + 1UL)

/* ======================== 三、Bootloader 运行行为 ======================== */

/* 复位后等待上位机诊断请求的窗口（ms）—— 与 Bootloader 保持一致 */
#define BOOT_WAIT_JUMP_MS             (500U)

/* "请求进入 BOOT" 标志：TAMP 备份寄存器 TAMP->BKP0R（VBAT 域） */
#define BOOT_REQ_MAGIC                (0x424F4F54UL)  /* "BOOT" */

/* 固件更新请求握手 DID */
#define UDS_DID_FW_UPDATE_REQ         (0xFF00U)
#define UDS_DID_FW_UPDATE_ACK         (0xFF01U)
#define FW_UPDATE_SUBFUNC_START       (0x01U)

/* App 侧：判定"本板未处于已刷写状态"的前导标志 */
#define APP_INVALID_MARK              (0xFFFFFFFEUL)

/* ========================== 四、诊断 CAN 参数 ========================== */
#define UDS_CAN_BAUDRATE              (800000UL)        /* 期望值/文档用 */
#define UDS_CAN_RX_ID                 (0x7E0UL)         /* 上位机 -> ECU 物理请求 */
#define UDS_CAN_TX_ID                 (0x7E8UL)         /* ECU -> 上位机 响应     */

/* 功能寻址请求 ID（ISO 15765-4:2016 表 2：11 位功能寻址固定 0x7DF）。
 *
 *   ⚠️ 这两个宏**只有 App 侧**定义。mcan.c 通过 #ifdef MCAN_EXTRA_FILTER_ID
 *      决定是否多装一个验收滤波器，因此 BOOT 侧不定义时行为完全不变 ——
 *      两个工程的 mcan.c 仍然保持逐字节一致。
 *
 *   功能寻址语义（ISO 14229-1:2020 §7.3）：
 *     - ECU 对功能寻址请求**一律不应答**（肯定与否定响应都不发），
 *       否则总线上多节点会同时应答而互相干扰；
 *     - 不在白名单内的服务静默丢弃，不回 NRC 0x11；
 *     - 只支持单帧请求（ISO 15765-2 不定义广播多帧）。 */
#define UDS_CAN_FUNC_RX_ID            (0x7DFUL)
#define MCAN_EXTRA_FILTER_ID          (UDS_CAN_FUNC_RX_ID)
#define MCAN_STD_FILTER_COUNT         (2U)

/* App 默认波特率（未通过 0x2E 配置过时使用）。
 *
 * 与 BOOT 的 CAN_DEFAULT_BAUDRATE 保持一致：这样"App 从未被配置过"
 * 与"BOOT 固定速率"天然相同，不会出现刷写完 App 后因速率不同而失联。
 *
 * App 支持运行时切换（见下），BOOT 固定用本速率。
 *
 * ⚠️ 位时序表在 source/mcan.c 的 s_astcBaudTable 中（唯一真值来源）。
 *    本宏若改成表里没有的速率，Mcan_Init() 会直接失败（返回 MCAN_ERR）。
 */
#define CAN_DEFAULT_BAUDRATE          (800000UL)

/* ------------------- App 运行时波特率切换 -------------------
 * 通过 UDS 0x2E WriteDataByIdentifier 写 DID 0xF1A0，数据为 1 字节档位码：
 *   0x00=125k  0x01=250k  0x02=500k  0x03=800k  0x04=1M  0xFF=恢复默认
 *
 * 档位码定义在 mcan.h 的 MCAN_BAUD_CODE_*，与位时序表一一对应。
 *
 * 持久化：写入 TAMP->BKP1R（VBAT 域，系统复位与掉电后仍保持），
 *         因此配置一次即长期生效，无需每次上电重新下发。
 *         读取用 0x22 DID 0xF1A0（1 = 当前档位码）。
 *
 * 生效时机：收到 0x2E 后先在**当前**速率上回肯定响应，再重新初始化
 *           FDCAN 切到新速率 —— 否则响应会用新速率发出，而此刻上位机
 *           还停在旧速率，必然收不到。
 */
#define CAN_BAUD_DID                  (0xF1A0U)         /* 波特率配置 DID */
#define CAN_BAUD_CODE_INVALID         (0xFFU)           /* 表示"当前无对应档位" */

/* TAMP 备份寄存器分配（VBAT 域，掉电保持）
 *   BKP0R : "进入 BOOT" 请求标志（BOOT_REQ_MAGIC）
 *   BKP1R : CAN 波特率档位码（见下）
 */
#define CAN_BAUD_BKP_REG              (TAMP->BKP1R)

/* ---------- 档位码在 BKP1R 中的编码（⚠️ 不可想当然） ----------
 * 实测：STM32G4 的 TAMP 备份寄存器**上电初值是 0x00000000**，
 *       不是 0xFFFFFFFF（VBAT 域复位值就是全 0）。
 *
 * 因此不能用"0xFFFFFFFF 表示未配置"这种哨兵 —— 全新板子上电读回 0，
 * 会被误判为"已配置、档位码 = 0x00 = 125 k"，导致 App 以 125 k 初始化，
 * 而 BOOT 用 800 k，**总线直接对不上**（本项目就踩过这个坑）。
 *
 * 改用显式 MAGIC 高位标记（取 ASCII "BD" = 0x4244，便于在调试器里一眼认出）：
 *   [31:16] = 0x4244 表示"本寄存器内容有效"
 *   [15:8]  = 保留
 *   [7:0]   = 档位码；0xFF = 未配置哨兵
 *
 * 判读顺序：先看 MAGIC，再看档位码是否为哨兵；
 * 任一不满足即视为"未配置"，回落 CAN_DEFAULT_BAUDRATE。
 */
#define CAN_BAUD_BKP_MAGIC            (0x42440000UL)      /* "BD" 高 16 位标记 */
#define CAN_BAUD_BKP_MAGIC_MASK       (0xFFFF0000UL)
#define CAN_BAUD_BKP_CODE_MASK        (0x000000FFUL)
#define CAN_BAUD_BKP_CODE_UNSET       (0xFFU)             /* 未配置哨兵 */

/* "未配置"时写入寄存器的规范化值（含 MAGIC，便于下次快速判定） */
#define CAN_BAUD_BKP_UNSET            (CAN_BAUD_BKP_MAGIC | CAN_BAUD_BKP_CODE_UNSET)

/* 完整编码：MAGIC | 档位码 */
#define CAN_BAUD_BKP_ENC(code)        (CAN_BAUD_BKP_MAGIC | ((uint32_t)(code) & 0xFFU))

/* CAN 引脚（FDCAN1，复用功能 AF9）
 *   PB9  = FDCAN1_TX
 *   PA11 = FDCAN1_RX
 * 注意：PA11/PA12 同时是 USB_DM/DP，本工程不使用 USB。
 *
 * 引脚掩码用 LL 的 LL_GPIO_PIN_x（HAL 的 GPIO_PIN_x 已随 HAL 移出）。
 */
#define CAN_TX_GPIO_PORT              (GPIOB)
#define CAN_TX_GPIO_PIN               (LL_GPIO_PIN_9)
#define CAN_RX_GPIO_PORT              (GPIOA)
#define CAN_RX_GPIO_PIN               (LL_GPIO_PIN_11)

/* 复用功能号：FDCAN1 固定在 AF9（RM0440 的 GPIO 复用表中 FDCAN1_TX/RX = AF9）。
 *
 * ⚠️ 这里直接给 AF 编号，而不是 HAL 的 GPIO_AF9_FDCAN1 —— 后者定义在
 *    stm32g4xx_hal_gpio_ex.h，已随 HAL 一起移出本工程。
 *    LL 库用 LL_GPIO_AF_9（其值同样是 9），mcan.c 通过
 *    LL_GPIO_SetAFPin_8_15() 写入，两者数值一致。
 */
#define CAN_GPIO_AF                   (9U)

/* CAN 收发器 S（Standby / 使能）控制引脚 —— PC11。
 *
 * ⚠️ 该引脚必须显式驱动，不能悬空。
 * 收发器（TJA1042/1051/1044、SN65HVD230 等）的 S 引脚悬空时，内部上拉
 * 会把器件锁在 **Standby** 模式，收发通路关闭。此时 MCU 侧一切正常
 * （回环自检全过、TX 请求被接受、TXBTO 置位），但：
 *   - TXD 的支配位传不到总线 -> PSR.LEC = 5 (Bit0Error)，TEC 涨到 248+，
 *     最终 Bus Off；
 *   - RXD 收不到数据 -> 接收帧数恒为 0。
 * 现象是"总线上完全没有本机的任何报文"，且极易误判为 CAN 外设或时钟问题。
 *
 * 极性：CAN_STB_ACTIVE_HIGH = 0（S 低 = Normal，TJA1042/1051/1044 等常见）。
 *       若所用器件极性相反（S 高 = Normal），把它改为 1 即可。
 */
#define CAN_STB_GPIO_PORT             (GPIOC)
#define CAN_STB_GPIO_PIN              (LL_GPIO_PIN_11)
#define CAN_STB_ACTIVE_HIGH           (0)

/* ------------------- FDCAN 位时序（内核时钟 = HSE 8 MHz） -------------------
 * 位时序参数不再以宏形式定义，而是集中在 source/mcan.c 的
 * s_astcBaudTable 表中，由波特率查表得到（与 BOOT 共用同一张表）。
 *
 *   波特率    tq 总数   采样点    SJW
 *   125 k        64     87.5 %     8
 *   250 k        32     87.5 %     4
 *   500 k        16     87.5 %     2
 *   800 k        10     80.0 %     2   <- 默认（CAN_DEFAULT_BAUDRATE）
 *     1 M         8     87.5 %     1
 */

/* ========================== 五、状态指示灯 ==========================
 * PC6 推挽输出，高电平有效。
 *
 * 闪烁语义：
 *   极快闪（60 ms）   -> 时钟配置失败（HSE/PLL 未起振），系统无法正常工作
 *                       注意：此时主频为 HSI 16 MHz，实际闪烁会比标称慢，
 *                             但\"极快闪\"仍能与正常心跳明确区分。
 *   慢闪（500 ms）    -> App 正常运行心跳
 */
#define LED_GPIO_PORT                 (GPIOC)
#define LED_GPIO_PIN                  (LL_GPIO_PIN_6)
#define LED_ACTIVE_HIGH               (1)

/* 时钟故障：极快闪，明确区别于正常心跳 */
#define LED_FAULT_BLINK_MS            (60U)

/* App 心跳指示周期（ms） */
#define LED_BLINK_PERIOD_MS           (500U)

/* ========================= 六、应用标识与运行参数 ========================= */

/* 语义版本（人工维护；同时用于 fw_info 与 0xF195 应答） */
#define APP_VERSION_MAJOR             (1U)
#define APP_VERSION_MINOR             (0U)
#define APP_VERSION_PATCH             (0U)

/* UDS 会话超时参数（与 Bootloader 保持一致） */
#define UDS_P2_TIMEOUT_MS             (50U)
#define UDS_P2_STAR_TIMEOUT_MS        (5000U)
#define UDS_S3_TIMEOUT_MS             (5000U)

#ifdef __cplusplus
}
#endif

#endif /* __BOARD_H__ */
