/**
 *******************************************************************************
 * @file  boot_req.c
 * @brief 应用侧诊断轮询与"进入 BOOT"请求执行（STM32G431 版）
 *
 *   本模块负责三件事：
 *     1. 轮询 FDCAN1，实现 ISO-TP 接收侧（SF / FF + CF），
 *        把完整请求连同**寻址类型**交给 app_uds.c 的 UDS 协议层；
 *     2. 维护跨复位的"请求进入 BOOT"标志（TAMP 备份寄存器）；
 *     3. 在收到已鉴权的固件更新请求后执行系统复位。
 *
 *   全系统只有本文件消费 FDCAN Rx FIFO 0 —— 这是必须守住的约定，
 *   否则两个消费者会互相抢帧（参见 README 的说明）。
 *******************************************************************************
 * Copyright (C) 2026, all rights reserved.
 *
 * This software component is licensed under BSD 3-Clause license.
 *******************************************************************************/
#include "boot_req.h"
#include "app_uds.h"
#include "mcan.h"
#include "board.h"
#include "clock.h"

#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_pwr.h"

#include <string.h>

/*******************************************************************************
 * 宏定义
 ******************************************************************************/
/* ISO-TP 帧类型（ISO 15765-2:2016），位于帧首字节高 4 位。
 *
 * 关键：CAN 帧里第一个字节是 ISO-TP 的 PCI，不是 UDS 的 SID。
 * 上位机把 0x22 FF00 这种 3 字节请求封成单帧后，线上字节是：
 *     [0x03, 0x22, 0xFF, 0x00, ...]
 *   即 byte0 = PCI(单帧,长度 3)，byte1 才是 SID。
 * 因此必须按帧类型定位 SID，不能直接看 byte0。
 */
#define BOOT_REQ_PCI_TYPE_SF        (0x00U)   /* SingleFrame       */
#define BOOT_REQ_PCI_TYPE_FF        (0x10U)   /* FirstFrame        */
#define BOOT_REQ_PCI_TYPE_CF        (0x20U)   /* ConsecutiveFrame  */
#define BOOT_REQ_PCI_TYPE_FC        (0x30U)   /* FlowControl       */
#define BOOT_REQ_MASK_FRAME_TYPE    (0xF0U)
#define BOOT_REQ_MASK_LOW_NIBBLE    (0x0FU)

/* 请求缓冲区：需容纳 0x22 一次连读多个 DID（49 个参数 DID 全读 = 1+2*49 = 99 字节），
 * 以及 0x2E 写入带较长数据块的配置类 DID，故取 128 字节。 */
#define BOOT_REQ_BUF_LEN            (128U)

/* CAN 单帧数据场长度 */
#define BOOT_REQ_MTU                (8U)

/* 复位前等待 CAN 响应真正发上总线的毫秒数。
 * 用 SysTick 而非 NOP 空转：与主频解耦，-O3 下也不会因优化而变短。 */
#define BOOT_REQ_TX_DRAIN_MS        (10U)

/*******************************************************************************
 * 内部函数声明
 ******************************************************************************/
static void BootReq_BackupAccessEnable(void);

/*******************************************************************************
 * 函数实现
 ******************************************************************************/

/**
 * @brief  打开备份域访问（时钟 + 写使能）。
 * @param  无
 * @retval 无
 */
static void BootReq_BackupAccessEnable(void)
{
    /* 1. PWR 提供 DBP 写保护开关（PWR 挂在 APB1） */
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
    /* 2. 备份寄存器所在时钟域（STM32G4 无独立 TAMPEN 位，用 RTCAPBEN） */
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_RTCAPB);
    /* 3. 解除备份域写保护 */
    LL_PWR_EnableBkUpAccess();
}

/**
 * @brief  初始化诊断模块。
 *
 *         验收滤波已由 Mcan_Init() 配置好（标准滤波器把 UDS_CAN_RX_ID 与
 *         UDS_CAN_FUNC_RX_ID 都路由到 Rx FIFO 0），本模块直接从该 FIFO
 *         消费请求，因此这里无需再安装滤波器。
 *
 * @param  无
 * @retval 无
 */
void BootReq_Init(void)
{
    AppUds_Init();
}

/**
 * @brief  轮询 CAN，并把完整请求交给 UDS 协议层处理。
 *
 *         实现 ISO-TP 接收侧的 SF / FF + CF，因此单帧与多帧请求都能处理。
 *         按 CAN ID 区分物理/功能寻址并透传，功能寻址下协议层不会响应。
 *
 * @param  [in] u32Tick 当前毫秒计数
 * @retval uint8_t 1 = 已收到并已鉴权的固件更新请求
 */
uint8_t BootReq_Poll(uint32_t u32Tick)
{
    static uint8_t  s_au8RxBuf[BOOT_REQ_BUF_LEN];
    static uint16_t s_u16RxLen;      /* 已收集字节数                    */
    static uint16_t s_u16RxExpect;   /* 首帧声明的总长度                */
    static uint8_t  s_u8RxNextSn;    /* 期望的下一个连续帧序号          */
    static uint8_t  s_u8RxActive;    /* 1 = 多帧接收进行中              */
    static uint8_t  s_u8RxAddrType;  /* 本次多帧组包的寻址类型          */

    uint32_t u32RxId;
    uint8_t  au8Data[BOOT_REQ_MTU];
    uint8_t  u8RxLen;
    uint8_t  u8Type;
    uint8_t  u8AddrType;

    /* 会话超时与安全访问延时均在此推进 */
    AppUds_Task(u32Tick);

    /* 先消费协议层在"等待流控"期间暂存的帧，再取 FIFO ——
     * 否则这些帧会一直压在暂存里直到下一轮，拖慢响应速度。
     * 注意循环条件：暂存优先，暂存空了才读硬件 FIFO。 */
    for (;;) {
        if (AppUds_PopStashFrame(&u32RxId, au8Data, &u8RxLen) == 0U) {
            if (Mcan_Receive(&u32RxId, au8Data, &u8RxLen) != MCAN_OK) {
                break;      /* 暂存与 FIFO 都空：本轮结束 */
            }
        }

        if (u8RxLen < 1U) {
            continue;
        }

        /* 寻址类型由 CAN ID 决定：0x7DF = 功能（广播），其余按物理处理 */
        u8AddrType = (u32RxId == UDS_CAN_FUNC_RX_ID) ? APP_ADDR_FUNCTIONAL
                                                     : APP_ADDR_PHYSICAL;

        u8Type = (uint8_t)(au8Data[0] & BOOT_REQ_MASK_FRAME_TYPE);

        switch (u8Type) {
            /* ---------------- 单帧 ---------------- */
            case BOOT_REQ_PCI_TYPE_SF: {
                uint8_t u8Len = (uint8_t)(au8Data[0] & BOOT_REQ_MASK_LOW_NIBBLE);

                if ((u8Len == 0U) || (u8Len > (uint8_t)(u8RxLen - 1U)) ||
                    (u8Len > BOOT_REQ_BUF_LEN)) {
                    break;      /* 长度非法：丢弃 */
                }
                memcpy(s_au8RxBuf, &au8Data[1], u8Len);
                s_u8RxActive = 0U;
                AppUds_HandleRequest(s_au8RxBuf, u8Len, u8AddrType);
                break;
            }

            /* ---------------- 首帧 ---------------- */
            case BOOT_REQ_PCI_TYPE_FF: {
                uint16_t u16Total;
                uint8_t  u8Copy;

                /* 功能寻址不支持多帧请求：ISO 15765-2 未定义广播多帧，
                 * 且回过流控帧会与总线上其它节点冲突。直接忽略。 */
                if (u8AddrType != APP_ADDR_PHYSICAL) {
                    break;
                }
                if (u8RxLen < 2U) {
                    break;
                }
                /* 总长度 = 低 4 位 << 8 | 第 2 字节（共 12 位） */
                u16Total = (uint16_t)(((uint16_t)(au8Data[0] &
                                                  BOOT_REQ_MASK_LOW_NIBBLE) << 8U) |
                                      au8Data[1]);
                if ((u16Total <= 7U) || (u16Total > BOOT_REQ_BUF_LEN)) {
                    s_u8RxActive = 0U;
                    break;      /* 长度非法或超出缓冲：放弃 */
                }

                u8Copy = (uint8_t)(u8RxLen - 2U);
                if (u8Copy > 6U) {
                    u8Copy = 6U;
                }
                memcpy(s_au8RxBuf, &au8Data[2], u8Copy);
                s_u16RxExpect = u16Total;
                s_u16RxLen    = u8Copy;
                s_u8RxNextSn  = 1U;
                s_u8RxActive  = 1U;
                s_u8RxAddrType = u8AddrType;

                /* 回送 CTS，允许对端继续发 */
                {
                    uint8_t au8Fc[3];

                    au8Fc[0] = (uint8_t)(BOOT_REQ_PCI_TYPE_FC | 0x00U); /* CTS */
                    au8Fc[1] = 0x00U;   /* BS = 0，不限块 */
                    au8Fc[2] = 0x00U;   /* STmin = 0      */
                    (void)Mcan_Send(UDS_CAN_TX_ID, au8Fc, 3U);
                }
                break;
            }

            /* ---------------- 连续帧 ---------------- */
            case BOOT_REQ_PCI_TYPE_CF: {
                uint8_t u8Sn = (uint8_t)(au8Data[0] & BOOT_REQ_MASK_LOW_NIBBLE);
                uint8_t u8Copy;

                if (s_u8RxActive == 0U) {
                    break;      /* 无进行中的接收：忽略 */
                }
                if (u8Sn != s_u8RxNextSn) {
                    s_u8RxActive = 0U;   /* 序号错：放弃本次 */
                    break;
                }

                u8Copy = (uint8_t)(u8RxLen - 1U);
                if ((uint16_t)(s_u16RxLen + u8Copy) > s_u16RxExpect) {
                    u8Copy = (uint8_t)(s_u16RxExpect - s_u16RxLen);
                }
                if ((uint16_t)(s_u16RxLen + u8Copy) > BOOT_REQ_BUF_LEN) {
                    s_u8RxActive = 0U;
                    break;
                }
                memcpy(&s_au8RxBuf[s_u16RxLen], &au8Data[1], u8Copy);
                s_u16RxLen += u8Copy;
                s_u8RxNextSn = (uint8_t)((s_u8RxNextSn + 1U) & 0x0FU);

                if (s_u16RxLen >= s_u16RxExpect) {
                    s_u8RxActive = 0U;
                    AppUds_HandleRequest(s_au8RxBuf, s_u16RxLen, s_u8RxAddrType);
                }
                break;
            }

            /* 流控帧本模块不处理（只在发送侧用） */
            default:
                break;
        }

        /* 每处理一帧就检查：一旦已接受刷写请求，立即返回让主循环复位 */
        if (AppUds_BootRequested() != 0U) {
            return 1U;
        }
    }

    return (AppUds_BootRequested() != 0U) ? 1U : 0U;
}

/**
 * @brief  写入跨复位"进入 BOOT"标志（TAMP->BKP0R，VBAT 域）。
 *
 *         读回校验：备份域若未真正解锁，写入会被静默丢弃，
 *         这里确保不带着无效标志去复位（否则会陷入复位循环）。
 *
 * @param  无
 * @retval 无
 */
void BootReq_Set(void)
{
    BootReq_BackupAccessEnable();

    TAMP->BKP0R = BOOT_REQ_MAGIC;

    if (TAMP->BKP0R != BOOT_REQ_MAGIC) {
        /* 再试一次（少数情况下 DBP 生效需要数个 APB 周期） */
        TAMP->BKP0R = BOOT_REQ_MAGIC;
    }
}

/**
 * @brief  读取跨复位"进入 BOOT"标志。
 * @param  无
 * @retval uint8_t 1 = 已置位，0 = 未置位
 */
uint8_t BootReq_IsSet(void)
{
    BootReq_BackupAccessEnable();

    return (TAMP->BKP0R == BOOT_REQ_MAGIC) ? 1U : 0U;
}

/**
 * @brief  清除跨复位"进入 BOOT"标志。
 * @param  无
 * @retval 无
 */
void BootReq_Clear(void)
{
    BootReq_BackupAccessEnable();

    TAMP->BKP0R = 0U;
}

/**
 * @brief  复位 MCU 使其进入 BOOT（不返回）。
 * @param  无
 * @retval 无
 */
void BootReq_Reboot(void)
{
    /* 先置跨复位标志：Bootloader 读到它才会驻留等待刷写，
     * 而不是在等待窗口结束后又跳回本 App（那将形成复位循环）。 */
    BootReq_Set();

    /* 给最后一次 CAN 响应留出送出的时间，避免复位打断总线传输 */
    SysTick_Delay(BOOT_REQ_TX_DRAIN_MS);

    NVIC_SystemReset();

    /* NVIC_SystemReset() 不会返回；此处仅为满足编译器 */
    for (;;) {
        __NOP();
    }
}

/**
 * @brief  复位 MCU 使其回到 App（不返回）。
 * @param  无
 * @retval 无
 */
void BootReq_RebootToApp(void)
{
    /* 清掉"进入 BOOT"标志：复位后 Bootloader 判定为普通上电，
     * 等待窗口结束即跳回 App。若不清，会驻留 BOOT 等待刷写。 */
    BootReq_Clear();

    SysTick_Delay(BOOT_REQ_TX_DRAIN_MS);

    NVIC_SystemReset();

    for (;;) {
        __NOP();
    }
}

/*******************************************************************************
 * 文件结束
 ******************************************************************************/
