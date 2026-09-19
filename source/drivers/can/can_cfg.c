/**
 *******************************************************************************
 * @file  can_cfg.c
 * @brief CAN 波特率配置的持久化实现（App 侧）
 *
 *   备份寄存器访问要求（缺一不可，否则写入不生效 / 读回为 0）：
 *     1. 使能 PWR 时钟（APB1）——DBP 写保护位在 PWR_CR1 内；
 *     2. 使能 RTCAPB 时钟（APB1）——BKP 寄存器挂在 RTC/TAMP 时钟域；
 *        注意 STM32G4 没有独立的 TAMPEN 位，用 RTCAPBEN 即可。
 *     3. 置 PWR_CR1.DBP = 1，解除备份域写保护。
 *
 *   这三点与 boot_req.c 完全一致，两处都做了使能，
 *   因为"谁先被调用"不确定，各自保证前置条件更稳妥
 *   （重复使能时钟无副作用）。
 *******************************************************************************
 * Copyright (C) 2026, all rights reserved.
 *
 * This software component is licensed under BSD 3-Clause license.
 *******************************************************************************
 */

/*******************************************************************************
 * 头文件
 ******************************************************************************/
#include "can_cfg.h"
#include "board.h"
#include "mcan.h"

/*******************************************************************************
 * 内部变量
 ******************************************************************************/
/* 当前生效的档位码（镜像备份寄存器的内容，便于快速读取） */
static uint8_t m_u8BaudCode = CAN_BAUD_CODE_INVALID;

/*******************************************************************************
 * 内部函数声明
 ******************************************************************************/
static void     CanCfg_BackupAccessEnable(void);
static int32_t  CanCfg_WriteBkp(uint32_t u32Value);

/*******************************************************************************
 * 函数实现
 ******************************************************************************/

/**
 * @brief  打开备份域访问（时钟 + 写使能）。
 * @param  无
 * @retval 无
 */
static void CanCfg_BackupAccessEnable(void)
{
    /* 1. PWR 提供 DBP 写保护开关 */
    __HAL_RCC_PWR_CLK_ENABLE();
    /* 2. 备份寄存器所在时钟域（STM32G4 无独立 TAMPEN 位） */
    __HAL_RCC_RTCAPB_CLK_ENABLE();
    /* 3. 解除备份域写保护 */
    HAL_PWR_EnableBkUpAccess();
}

/**
 * @brief  写备份寄存器并读回校验。
 *
 *   读回校验是必要的：备份域若未真正解锁（DBP 未生效），写入会被静默
 *   丢弃。不校验的话"配置成功"只是假象 —— 一复位就回默认，现象令人困惑。
 *
 * @param  [in] u32Value 待写入的值
 * @retval int32_t 0 = 成功；-1 = 无法持久化
 */
static int32_t CanCfg_WriteBkp(uint32_t u32Value)
{
    CanCfg_BackupAccessEnable();

    CAN_BAUD_BKP_REG = u32Value;

    if (CAN_BAUD_BKP_REG != u32Value) {
        /* 再试一次（少数情况下 DBP 生效需要数个 APB 周期） */
        CAN_BAUD_BKP_REG = u32Value;
        if (CAN_BAUD_BKP_REG != u32Value) {
            return -1;
        }
    }

    return 0;
}

/**
 * @brief  初始化：把持久化的档位码应用到 CAN 驱动（只记录，不改硬件）。
 * @param  无
 * @retval 无
 */
void CanCfg_Init(void)
{
    uint32_t u32Raw;
    uint8_t  u8Code;

    CanCfg_BackupAccessEnable();

    u32Raw = CAN_BAUD_BKP_REG;

    /* 判断“是否被用户配置过”要靠一个**显式标记**，不能靠“上电值”。
     *
     * 实测：STM32G4 的 TAMP 备份寄存器上电初值是 **0x00000000**，
     * 不是 0xFFFFFFFF（VBAT 域的复位值就是全 0）。
     * 因此若用 0xFFFFFFFF 当“未配置”哨兵值，一块全新板子上电后
     * 读回 0，会被当成“已配置、档位码 = 低8位 = 0x00 = 125 k” ——
     * 结果 App 用 125 k 初始化，而 BOOT 用 800 k，总线直接对不上。
     *
     * 改用 MAGIC 高位标记：只有高 16 位恰好等于 CAN_BAUD_BKP_MAGIC
     * 才算“配置过”，其余一律视为未配置（回落 CAN_DEFAULT_BAUDRATE）。
     * 这样上电值 0（不含 MAGIC）自然走默认路径，且与档位码 0x00 不冲突。 */
    if ((u32Raw & CAN_BAUD_BKP_MAGIC_MASK) != CAN_BAUD_BKP_MAGIC) {
        /* 未配置（或内容被破坏）：保持驱动默认值，并把寄存器规范化，
         * 免得以后每次都走到这个分支。 */
        (void)CanCfg_WriteBkp(CAN_BAUD_BKP_UNSET);
        m_u8BaudCode = Mcan_GetBaudrateCode();
        return;
    }

    /* 取出存档的档位码 */
    u8Code = (uint8_t)(u32Raw & CAN_BAUD_BKP_CODE_MASK);

    /* 未配置（哨兵）-> 用默认速率 */
    if (u8Code == CAN_BAUD_BKP_CODE_UNSET) {
        m_u8BaudCode = Mcan_GetBaudrateCode();
        return;
    }

    /* 校验合法性：备份寄存器内容可能因故损坏（如调试器误写），
     * 非法值一律回落默认，避免用错误速率点总线上电。 */
    if (Mcan_SetBaudrateByCode(u8Code) != MCAN_OK) {
        m_u8BaudCode = Mcan_GetBaudrateCode();
        /* 顺手把损坏的值清掉，下次上电即走默认路径 */
        (void)CanCfg_WriteBkp(CAN_BAUD_BKP_UNSET);
        return;
    }

    m_u8BaudCode = Mcan_GetBaudrateCode();
}

/**
 * @brief  把档位码写入备份寄存器并应用到 CAN 驱动（只记录，不改硬件）。
 * @param  [in] u8Code 档位码；0xFF 表示恢复默认
 * @retval int32_t 0 = 成功；-1 = 档位码不受支持（既有配置与硬件均不变）
 */
int32_t CanCfg_SetBaudCode(uint8_t u8Code)
{
    /* 先校验合法性再落盘：非法值不入备份寄存器，
     * 否则下次上电还得靠 CanCfg_Init() 的兜底逻辑去纠正。 */
    if (Mcan_SetBaudrateByCode(u8Code) != MCAN_OK) {
        return -1;
    }

    m_u8BaudCode = Mcan_GetBaudrateCode();

    /* 恢复默认时，把寄存器标回“未配置”哨兵（保留 MAGIC）。
     * 这样 CAN_DEFAULT_BAUDRATE 将来若被修改，恢复默认能自动跟随新值 ——
     * 若存成显式档位码，就会永远钉在改动前的那一档。 */
    if (u8Code == MCAN_BAUD_CODE_DEFAULT) {
        return CanCfg_WriteBkp(CAN_BAUD_BKP_UNSET);
    }

    return CanCfg_WriteBkp(CAN_BAUD_BKP_ENC(u8Code));
}

/**
 * @brief  查询当前生效的档位码。
 * @param  无
 * @retval uint8_t 档位码（MCAN_BAUD_CODE_*）
 */
uint8_t CanCfg_GetBaudCode(void)
{
    return m_u8BaudCode;
}

/**
 * @brief  查询当前生效的波特率（速率值，如 800000）。
 * @param  无
 * @retval uint32_t 波特率
 */
uint32_t CanCfg_GetBaudrate(void)
{
    return Mcan_GetBaudrate();
}

/**
 * @brief  查询备份寄存器里是否存有"用户显式配置过"的档位码。
 * @param  无
 * @retval uint8_t 1 = 曾配置；0 = 从未配置（正在用默认值）
 */
uint8_t CanCfg_IsConfigured(void)
{
    CanCfg_BackupAccessEnable();

    /* “配置过” = 含 MAGIC 且档位码不是哨兵 */
    if ((CAN_BAUD_BKP_REG & CAN_BAUD_BKP_MAGIC_MASK) != CAN_BAUD_BKP_MAGIC) {
        return 0U;
    }
    return ((CAN_BAUD_BKP_REG & CAN_BAUD_BKP_CODE_MASK) == CAN_BAUD_BKP_CODE_UNSET)
           ? 0U : 1U;
}

/*******************************************************************************
 * 文件结束
 ******************************************************************************/
