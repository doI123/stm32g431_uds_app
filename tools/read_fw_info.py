#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
读取固件镜像（.bin）末尾的固件信息块并打印，同时复核 CRC。

用法:
    python read_fw_info.py <firmware.bin> [基地址]

说明:
    结构体布局必须与 app/version.h 中的 fw_info_t 严格一致（packed，116 字节）。
    若修改了 fw_info_t 的字段，请同步更新下方的 struct 格式与偏移。
"""

import struct
import sys
import zlib

FW_INFO_MAGIC = 0x464E4946          # 'FIN F'（小端）
FW_INFO_SIZE = 116                  # sizeof(fw_info_t)

# 结构体布局: magic, struct_ver, role, ver*4, build_epoch, date[12], time[10],
#             sw[16], hw[16], image_size, image_crc, git_hash, git_dirty,
#             reserved[26], info_crc
_FMT = "<IHHBBBB I 12s 10s 16s 16s I I I I 26s I"


def _cstr(raw: bytes) -> str:
    """按 C 字符串语义截断（到第一个 \\0）。"""
    return raw.split(b"\x00", 1)[0].decode("ascii", errors="replace")


def main() -> int:
    if len(sys.argv) < 2:
        print("用法: read_fw_info.py <firmware.bin> [基地址]")
        return 1

    path = sys.argv[1]
    base_addr = int(sys.argv[2], 16) if len(sys.argv) > 2 else 0x08000000

    with open(path, "rb") as fp:
        data = fp.read()

    # 固件信息块位于镜像末尾，从尾部向前搜索 magic
    pos = data.rfind(struct.pack("<I", FW_INFO_MAGIC))
    if pos < 0:
        print(f"[read_fw_info] 未找到固件信息魔数: {path}")
        return 2
    if pos + FW_INFO_SIZE > len(data):
        print(f"[read_fw_info] 固件信息块不完整: {path}")
        return 3

    blk = data[pos:pos + FW_INFO_SIZE]
    (magic, struct_ver, role, v_major, v_minor, v_patch, build_no,
     build_epoch, build_date, build_time, sw_ver, hw_ver,
     image_size, image_crc, git_hash, git_dirty, reserved, info_crc) = \
        struct.unpack(_FMT, blk)

    role_name = {0xB007: "BOOT", 0x0A99: "APP"}.get(role, f"0x{role:04X}")

    print("=" * 60)
    print(f"文件        : {path}")
    print(f"信息块地址  : 0x{base_addr + pos:08X} (镜像内偏移 0x{pos:X})")
    print(f"角色        : {role_name}")
    print(f"结构体版本  : {struct_ver}")
    print(f"语义版本    : {v_major}.{v_minor}.{v_patch} (build_no={build_no})")
    print(f"软件版本    : {_cstr(sw_ver)}")
    print(f"硬件版本    : {_cstr(hw_ver)}")
    print(f"编译时间    : {_cstr(build_date)} {_cstr(build_time)} "
          f"(Unix {build_epoch})")
    print(f"Git         : 0x{git_hash:08X} {'(有未提交改动)' if git_dirty else ''}")
    print(f"镜像大小    : {image_size} B")
    print(f"镜像 CRC32  : 0x{image_crc:08X}")
    print(f"信息 CRC32  : 0x{info_crc:08X}")

    # ---- CRC 复核 ----
    calc_image = zlib.crc32(data[:pos]) & 0xFFFFFFFF
    calc_info = zlib.crc32(blk[:112]) & 0xFFFFFFFF

    ok_image = "OK" if calc_image == image_crc else "MISMATCH"
    ok_info = "OK" if calc_info == info_crc else "MISMATCH"

    print("-" * 60)
    print(f"镜像 CRC 校验: 计算值 0x{calc_image:08X} -> {ok_image}")
    print(f"信息 CRC 校验: 计算值 0x{calc_info:08X} -> {ok_info}")
    print("=" * 60)

    return 0 if (ok_image == "OK" and ok_info == "OK") else 4


if __name__ == "__main__":
    sys.exit(main())
