/**
 *******************************************************************************
 * @file  version.c
 * @brief 固件信息块实例定义与工具函数
 *
 *   版本号维护说明：
 *     - u8VerMajor / u8VerMinor / u8VerPatch：**人工维护**，发布时手工修改。
 *       原因：代码改动不等于版本应变化，自动递增会破坏版本语义，
 *             且无法表达兼容性信息；
 *     - 编译日期 / 时刻 / Unix 时间戳 / git 信息：由 CMake 每次构建自动生成；
 *     - u32ImageSize / u32ImageCrc32 / u32InfoCrc32：由构建后脚本回填
 *       （tools/fill_fw_info.py）。
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
#include "version.h"

/* CMake 每次构建自动生成，包含编译时间与 git 信息 */
#include "build_info.h"

/*******************************************************************************
 * 版本号（人工维护）
 ******************************************************************************/
#define FW_VER_MAJOR            (1U)    /* 主版本号 */
#define FW_VER_MINOR            (0U)    /* 次版本号 */
#define FW_VER_PATCH            (0U)    /* 修订号   */
#define FW_BUILD_NO             (0U)    /* 构建号（可选） */

/*******************************************************************************
 * 角色与字符串（由 CMake 以 -D 传入，默认按 Bootloader 处理）
 ******************************************************************************/
#ifndef FW_ROLE_ID
#define FW_ROLE_ID              FW_ROLE_BOOT
#endif

#ifndef FW_SW_STRING
#define FW_SW_STRING            "BOOT_V1.00"
#endif

#ifndef FW_HW_STRING
#define FW_HW_STRING            "HW_A1"
#endif

/*******************************************************************************
 * 固件信息块实例
 *
 *   __attribute__((section(".fw_info"), used))：
 *     - 强制放入 .fw_info 段，链接脚本将该段排在镜像最末尾；
 *     - used 防止未被引用时被 --gc-sections 回收。
 ******************************************************************************/
const fw_info_t g_stcFwInfo __attribute__((section(".fw_info"), used)) = {
    .u32Magic       = FW_INFO_MAGIC,
    .u16StructVer   = FW_INFO_STRUCT_VER,
    .u16Role        = FW_ROLE_ID,

    .u8VerMajor     = FW_VER_MAJOR,
    .u8VerMinor     = FW_VER_MINOR,
    .u8VerPatch     = FW_VER_PATCH,
    .u8BuildNo      = FW_BUILD_NO,

    .u32BuildEpoch  = FW_BUILD_EPOCH,       /* CMake 自动生成 */
    .acBuildDate    = FW_BUILD_DATE,        /* CMake 自动生成 */
    .acBuildTime    = FW_BUILD_TIME,        /* CMake 自动生成 */

    .acSwVersion    = FW_SW_STRING,
    .acHwVersion    = FW_HW_STRING,

    .u32ImageSize   = 0U,                   /* 构建后由脚本回填 */
    .u32ImageCrc32  = 0U,                   /* 构建后由脚本回填 */
    .u32GitHash     = FW_GIT_HASH,          /* CMake 自动生成 */
    .u32GitDirty    = FW_GIT_DIRTY,         /* CMake 自动生成 */

    .au8Reserved    = {0},
    .u32InfoCrc32   = 0U                    /* 构建后由脚本回填 */
};

/*******************************************************************************
 * 函数实现
 ******************************************************************************/

/**
 * @brief  获取本镜像的固件信息结构体指针。
 * @param  无
 * @retval const fw_info_t* 固件信息指针
 */
const fw_info_t *FwInfo_Get(void)
{
    return &g_stcFwInfo;
}

/**
 * @brief  计算 CRC32（IEEE 802.3 多项式 0xEDB88320）。
 *         算法与 Python zlib.crc32()、PC 侧工具保持一致。
 * @param  [in] pu8Data 数据指针
 * @param  [in] u32Len  数据长度
 * @retval uint32_t CRC32 结果
 */
uint32_t FwInfo_Crc32(const uint8_t *pu8Data, uint32_t u32Len)
{
    uint32_t u32Crc = 0xFFFFFFFFUL;
    uint32_t i;
    uint8_t  u8Bit;

    for (i = 0U; i < u32Len; i++) {
        u32Crc ^= pu8Data[i];
        for (u8Bit = 0U; u8Bit < 8U; u8Bit++) {
            u32Crc = (u32Crc & 1UL) ? ((u32Crc >> 1) ^ 0xEDB88320UL)
                                    : (u32Crc >> 1);
        }
    }

    return ~u32Crc;
}

/**
 * @brief  校验固件信息块是否有效。
 * @param  [in] pInfo 待校验的结构体指针
 * @retval uint8_t 1 = 有效，0 = 无效
 */
uint8_t FwInfo_IsValid(const fw_info_t *pInfo)
{
    uint32_t u32CalcCrc;

    if (pInfo == NULL) {
        return 0U;
    }
    if (pInfo->u32Magic != FW_INFO_MAGIC) {
        return 0U;
    }

    u32CalcCrc = FwInfo_Crc32((const uint8_t *)pInfo,
                              (uint32_t)offsetof(fw_info_t, u32InfoCrc32));

    return (u32CalcCrc == pInfo->u32InfoCrc32) ? 1U : 0U;
}

/**
 * @brief  在给定 Flash 区间内按 magic 扫描固件信息块。
 * @param  [in] u32StartAddr 扫描起始地址
 * @param  [in] u32EndAddr   扫描结束地址（不含）
 * @retval const fw_info_t* 找到则返回指针，否则返回 NULL
 */
const fw_info_t *FwInfo_Find(uint32_t u32StartAddr, uint32_t u32EndAddr)
{
    uint32_t u32Addr;

    /* 从区间头部按 4 字节步进扫描（结构体 4 字节对齐） */
    for (u32Addr = u32StartAddr;
         (u32Addr + sizeof(fw_info_t)) <= u32EndAddr;
         u32Addr += 4U) {
        const fw_info_t *pInfo = (const fw_info_t *)u32Addr;

        if (pInfo->u32Magic != FW_INFO_MAGIC) {
            continue;
        }
        /* magic 命中后再用 CRC 复核，避免数据段中恰好出现相同数值 */
        if (FwInfo_IsValid(pInfo) != 0U) {
            return pInfo;
        }
    }

    return NULL;
}

/*******************************************************************************
 * 文件结束
 ******************************************************************************/
