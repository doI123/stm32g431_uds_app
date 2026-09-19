#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
在固件镜像（.bin）末尾定位 fw_info 结构体，回填以下三个字段：
    u32ImageSize  : 固件信息块之前的镜像有效字节数
    u32ImageCrc32 : 镜像数据的 CRC32（IEEE 802.3，与 zlib.crc32 一致）
    u32InfoCrc32  : 固件信息结构体自身的 CRC32

用法:
    python fill_fw_info.py <firmware.bin> [基地址]

注意:
    结构体布局必须与 app/version.h 中的 fw_info_t 严格一致（packed）。
    若修改了 fw_info_t 的字段，请同步更新下方的偏移常量。
"""

import struct
import sys
import zlib

# 与 app/version.h 保持一致的结构体定义
FW_INFO_MAGIC = 0x464E4946          # 'FIN F'（小端）
FW_INFO_SIZE = 116                  # sizeof(fw_info_t)

# 关键字段在结构体内的偏移（严格对应 version.h 的 fw_info_t 字段顺序）
#   +0   u32Magic        (4)
#   +4   u16StructVer    (2)
#   +6   u16Role         (2)
#   +8   u8VerMajor..u8BuildNo (4)
#   +12  u32BuildEpoch   (4)
#   +16  acBuildDate[12]
#   +28  acBuildTime[10]
#   +38  acSwVersion[16]
#   +54  acHwVersion[16]
#   +70  u32ImageSize    (4)
#   +74  u32ImageCrc32   (4)
#   +78  u32GitHash      (4)
#   +82  u32GitDirty     (4)
#   +86  au8Reserved[26]
#   +112 u32InfoCrc32    (4)  <- 总计 116 字节
OFF_IMAGE_SIZE = 70
OFF_IMAGE_CRC = 74
OFF_INFO_CRC = 112                  # 结构体最后 4 字节


def main() -> int:
    if len(sys.argv) < 2:
        print("用法: fill_fw_info.py <firmware.bin> [基地址]")
        return 1

    path = sys.argv[1]
    base_addr = int(sys.argv[2], 16) if len(sys.argv) > 2 else 0x08000000

    with open(path, "rb") as fp:
        data = bytearray(fp.read())

    # 固件信息块位于镜像末尾，从尾部向前搜索 magic 更高效
    pos = data.rfind(struct.pack("<I", FW_INFO_MAGIC))
    if pos < 0:
        print(f"[fill_fw_info] 未找到固件信息魔数，跳过: {path}")
        return 0
    if pos + FW_INFO_SIZE > len(data):
        print(f"[fill_fw_info] 固件信息块不完整，跳过: {path}")
        return 0

    # 镜像有效数据 = 固件信息块之前的全部字节
    image_size = pos
    image_crc = zlib.crc32(bytes(data[:pos])) & 0xFFFFFFFF

    struct.pack_into("<I", data, pos + OFF_IMAGE_SIZE, image_size)
    struct.pack_into("<I", data, pos + OFF_IMAGE_CRC, image_crc)

    # 计算自校验 CRC 前先将该字段清零
    struct.pack_into("<I", data, pos + OFF_INFO_CRC, 0)
    info_crc = zlib.crc32(bytes(data[pos:pos + OFF_INFO_CRC])) & 0xFFFFFFFF
    struct.pack_into("<I", data, pos + OFF_INFO_CRC, info_crc)

    with open(path, "wb") as fp:
        fp.write(data)

    print(
        f"[fill_fw_info] 地址=0x{base_addr + pos:08X} "
        f"镜像大小={image_size} 镜像CRC=0x{image_crc:08X} "
        f"信息CRC=0x{info_crc:08X}"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
