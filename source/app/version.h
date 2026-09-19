/**
 *******************************************************************************
 * @file  version.h
 * @brief 固件信息块（Bootloader 与 App 共用），置于镜像 Flash 末尾
 *
 *   设计要点：
 *     1. 结构体固定布局（packed），PC 侧工具按同一布局解析；
 *     2. 内置 magic 与 CRC32，可在一个 Flash 区间内可靠定位并校验；
 *     3. 通过链接脚本放在 .fw_info 段，且该段始终排在镜像最末尾，
 *        因此固件信息紧跟在真实镜像数据之后（不会产生大段 0xFF 空洞）；
 *     4. 编译时间/git 信息由 CMake 每次构建自动生成；
 *     5. 语义版本号（major.minor.patch）由人工维护，避免"改代码即跳版本"。
 *******************************************************************************
 * Copyright (C) 2026, all rights reserved.
 *
 * This software component is licensed under BSD 3-Clause license.
 *******************************************************************************
 */
#ifndef __VERSION_H__
#define __VERSION_H__

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 固件信息块的魔数：'F''I''N''F'（小端字节序为 "FIN F"），用于扫描定位 */
#define FW_INFO_MAGIC           (0x464E4946UL)

/* 结构体版本号：字段布局变化时递增，便于工具向后兼容 */
#define FW_INFO_STRUCT_VER      (1U)

/* 镜像角色：区分 Bootloader 与 App，两者使用同一结构体 */
#define FW_ROLE_BOOT            (0xB007U)
#define FW_ROLE_APP             (0x0A99U)

/* 保留字段长度（填 26 字节使结构体总长为 116，4 字节对齐） */
#define FW_INFO_RESERVED_LEN    (26U)

/* ============================ 固件信息结构体 ============================
 * 注意：字段顺序即二进制布局，PC 侧工具依赖该顺序，勿随意调整。
 */
typedef struct __attribute__((packed)) {
    uint32_t u32Magic;                      /* 固定为 FW_INFO_MAGIC */
    uint16_t u16StructVer;                  /* 结构体版本（FW_INFO_STRUCT_VER） */
    uint16_t u16Role;                       /* 镜像角色（FW_ROLE_BOOT / FW_ROLE_APP） */

    uint8_t  u8VerMajor;                    /* 语义版本：主版本（人工维护） */
    uint8_t  u8VerMinor;                    /* 语义版本：次版本（人工维护） */
    uint8_t  u8VerPatch;                    /* 语义版本：修订号（人工维护） */
    uint8_t  u8BuildNo;                     /* 构建号（可选） */

    uint32_t u32BuildEpoch;                 /* 编译时间：Unix 时间戳（UTC，自动生成） */
    char     acBuildDate[12];               /* 编译日期 "YYYY-MM-DD"（自动生成） */
    char     acBuildTime[10];               /* 编译时刻 "HH:MM:SS"（自动生成） */

    char     acSwVersion[16];               /* 软件版本字符串，如 "BOOT_V1.00" */
    char     acHwVersion[16];               /* 硬件版本字符串，如 "HW_A1" */

    uint32_t u32ImageSize;                  /* 镜像有效字节数（不含本结构，构建后回填） */
    uint32_t u32ImageCrc32;                 /* 镜像数据 CRC32（构建后回填） */
    uint32_t u32GitHash;                    /* git 短散列（自动生成，无 git 时为 0） */
    uint32_t u32GitDirty;                   /* 1 = 构建时工作区有未提交改动 */

    uint8_t  au8Reserved[FW_INFO_RESERVED_LEN]; /* 保留字段，向后扩展用 */
    uint32_t u32InfoCrc32;                  /* 本结构自校验 CRC32（含本字段之前的内容） */
} fw_info_t;

/* 结构体长度必须是 4 的整数倍，便于按字对齐访问 */
#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
_Static_assert((sizeof(fw_info_t) % 4U) == 0U, "fw_info_t 长度需 4 字节对齐");
#endif

/* ============================== 对外接口 ============================== */

/**
 * @brief  获取本镜像的固件信息结构体指针（只读，位于 Flash 中）。
 * @param  无
 * @retval const fw_info_t* 固件信息指针
 */
const fw_info_t *FwInfo_Get(void);

/**
 * @brief  校验固件信息块是否有效（magic 匹配且 CRC32 正确）。
 * @param  [in] pInfo 待校验的结构体指针
 * @retval uint8_t 1 = 有效，0 = 无效
 */
uint8_t FwInfo_IsValid(const fw_info_t *pInfo);

/**
 * @brief  计算 CRC32（IEEE 802.3 多项式 0xEDB88320，与 Python zlib.crc32 一致）。
 * @param  [in] pu8Data 数据指针
 * @param  [in] u32Len  数据长度
 * @retval uint32_t CRC32 结果
 */
uint32_t FwInfo_Crc32(const uint8_t *pu8Data, uint32_t u32Len);

/**
 * @brief  在给定 Flash 区间内按 magic 扫描固件信息块（含 CRC 复核）。
 *         用于 Bootloader 读取 App 的版本信息。
 * @param  [in] u32StartAddr 扫描起始地址
 * @param  [in] u32EndAddr   扫描结束地址（不含）
 * @retval const fw_info_t* 找到则返回指针，否则返回 NULL
 */
const fw_info_t *FwInfo_Find(uint32_t u32StartAddr, uint32_t u32EndAddr);

#ifdef __cplusplus
}
#endif

#endif /* __VERSION_H__ */
