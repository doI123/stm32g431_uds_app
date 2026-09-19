#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
固件分区与镜像有效性自检（上位机 / 离线校验用）。

校验内容：
  1. App 镜像起始地址是否等于 Bootloader 期望的 APP_START_ADDR；
  2. App 向量表首字（初始 MSP）是否落在合法 RAM 区间；
  3. 复位向量是否落在 App 区内；
  4. 两个镜像的 Flash 区间是否重叠。

用法:
    python check_layout.py <app.bin> [app_base]
"""

import struct
import sys

# 与 board.h 保持一致的分区参数
BOOT_REGION_START = 0x08000000
BOOT_REGION_SIZE = 0x00006000          # 24 KB
APP_START_ADDR = 0x08006000
APP_REGION_END = 0x0801FFFF

RAM_START = 0x20000000
RAM_END = 0x200057FF                   # STM32G431CB: 22 KB
MSP_MAX = RAM_END + 1                  # 合法栈顶上限（栈向下生长）


def main() -> int:
    if len(sys.argv) < 2:
        print("用法: check_layout.py <app.bin> [app_base]")
        return 1

    path = sys.argv[1]
    base = int(sys.argv[2], 16) if len(sys.argv) > 2 else APP_START_ADDR

    with open(path, "rb") as fp:
        data = fp.read()

    if len(data) < 8:
        print("[FAIL] 镜像过小，无法读取向量表")
        return 2

    msp, reset = struct.unpack("<II", data[:8])
    app_end_addr = base + len(data) - 1

    print("=" * 60)
    print(f"镜像        : {path}")
    print(f"大小        : {len(data)} B")
    print(f"起始地址    : 0x{base:08X}")
    print(f"结束地址    : 0x{app_end_addr:08X}")
    print(f"初始 MSP    : 0x{msp:08X}")
    print(f"复位向量    : 0x{reset:08X}")

    ok = True

    # 1. 起始地址必须等于 Bootloader 期望值
    if base != APP_START_ADDR:
        print(f"[FAIL] 起始地址应为 0x{APP_START_ADDR:08X}（上位机据此发 0x34）")
        ok = False
    else:
        print(f"[ OK ] 起始地址与 Bootloader 的 APP_START_ADDR 一致")

    # 2. MSP 必须落在合法 RAM 区间（含末端 +1）
    if not (RAM_START <= msp <= MSP_MAX):
        print(f"[FAIL] MSP 0x{msp:08X} 不在 [0x{RAM_START:08X}, 0x{MSP_MAX:08X}]")
        ok = False
    else:
        print(f"[ OK ] MSP 落在合法 RAM 区间")

    # 3. 复位向量必须落在 App 区内
    if not (APP_START_ADDR <= reset <= APP_REGION_END):
        print(f"[FAIL] 复位向量 0x{reset:08X} 不在 App 区")
        ok = False
    else:
        # 复位向量应按 Thumb 规则为奇数（bit0 = 1）
        thumb = "（Thumb 位已置位）" if (reset & 1) else "（警告：Thumb 位未置位）"
        print(f"[ OK ] 复位向量落在 App 区内 {thumb}")
        if not (reset & 1):
            ok = False

    # 4. 与 Boot 区不得重叠
    if base < (BOOT_REGION_START + BOOT_REGION_SIZE):
        print("[FAIL] App 镜像与 Bootloader 区重叠")
        ok = False
    else:
        print("[ OK ] 与 Bootloader 区（24 KB）无重叠")

    # 5. 不得越过 Flash 末尾
    if app_end_addr > APP_REGION_END:
        print(f"[FAIL] 镜像越过 App 区末端 0x{APP_REGION_END:08X}")
        ok = False
    else:
        free = APP_REGION_END - app_end_addr
        print(f"[ OK ] 未越过 App 区末端（余量 {free} B）")

    print("-" * 60)
    print("结论: " + ("全部通过" if ok else "存在问题，见上方 [FAIL]"))
    print("=" * 60)

    return 0 if ok else 3


if __name__ == "__main__":
    sys.exit(main())
