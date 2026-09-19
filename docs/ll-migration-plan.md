# LL 全量化改造方案（已确认范围）

> 基线：`v1.00-hal-baseline`（commit 1e3eb88，镜像 text=22944 data=32 bss=5032）
> 本文件记录**已确认的决策**与执行/验证步骤，便于回退与评审。

---

## 一、已确认的决策

| 项 | 决策 |
| --- | --- |
| 改造范围 | **100% LL 库**，App 工程内**不再保留任何 HAL 源码** |
| 涉及工程 | **仅本 App 工程**；`../STM32G431_UDS_Boot` 不改动 |
| 驱动放置 | 工程内 `drivers/STM32CubeG4_V1.6.0/`（CMSIS + LL 头） |
| 编译目标 | 保持 CMake + Ninja + arm-none-eabi 不变 |

---

## 二、连带影响（重要，需长期遵守）

1. **`mcan.c` 与 BOOT 侧"逐字节一致"的约定自本次改造起作废。**
   - App 侧 `mcan.c` 改为 FDCAN 寄存器级（LL）实现，BOOT 侧仍是 HAL 实现；
   - 两者**功能应等价**，但**不再字节相同**；
   - 今后 CAN 侧的任何修复都要**改两遍**，且一致性只能靠回归保证。
   - 因此 `README.md` 与 `board.h` / `mcan.c` 顶部关于"必须逐字节一致"的表述
     需同步修订，避免后人按旧约定误操作。

2. **HAL 时基依赖消失**：不再有 `uwTick` / `HAL_GetTick()`。
   `mcan.c` 内的超时（发送、Tx 排空、BusOff 限流）改用 `SysTick_GetTick()`
   （`source/bsp/clock.c` 已有，回绕安全）。

3. **`system_stm32g4xx.c` 必须原样拷贝**：本工程刻意不定义
   `USER_VECT_TAB_ADDRESS`，VTOR 由 `App_VectorTableInit()` 显式设置。

---

## 三、目录与构建改动

```
drivers/STM32CubeG4_V1.6.0/
├─ CMSIS/
│  ├─ Include/                            cmsis_compiler.h cmsis_version.h
│  │                                      cmsis_gcc.h core_cm4.h mpu_armv7.h
│  └─ Device/ST/STM32G4xx/
│     ├─ Include/                         stm32g4xx.h stm32g431xx.h system_stm32g4xx.h
│     └─ Source/Templates/
│        ├─ system_stm32g4xx.c            # 原样
│        └─ gcc/startup_stm32g431xx.s     # 原样
└─ Drivers/STM32G4xx_HAL_Driver/Inc/       stm32g4xx_ll_*.h（全量，无 .c）
```

`CMakeLists.txt`：`STM32CUBE_ROOT` 改指 `drivers/STM32CubeG4_V1.6.0`；
删除 `HAL_SOURCES` 定义及其在 `add_executable` 中的引用；
编译宏删除 `USE_HAL_DRIVER`（保留 `STM32G431xx`）。
`include_directories()` 与源文件顺序**保持不变**。

---

## 四、源文件改动清单

| 文件 | 改动 |
| --- | --- |
| `source/bsp/board.h` | `stm32g4xx_hal.h` → `stm32g4xx.h` |
| `source/app/main.h` | 同上 |
| `source/bsp/stm32g4xx_hal_conf.h` | 删除 |
| `source/bsp/led.c` | `LL_GPIO_*` + `LL_AHB2_GRP1_EnableClock` |
| `source/drivers/can/can_cfg.c` | `LL_PWR_EnableBkUpAccess()` |
| `source/app/main.c` | 去 `HAL_Init`/`HAL_IncTick`；改 `LL_APB1/APB2` 使能 PWR/SYSCFG |
| `source/bsp/clock.c` | `LL_RCC_*` + `LL_PWR_*` + `LL_SetFlashLatency`；删 `HAL_MspInit` |
| `source/drivers/can/mcan.c` / `mcan.h` | FDCAN 寄存器级重写（消息 RAM 布局须按手册核实） |
| `source/uds/*`、`boot_req.c`、`version.c` | 无需改动 |

---

## 五、执行顺序

1. 拷贝 `drivers/`（含自检清单）
2. 改 `CMakeLists.txt`
3. `board.h` / `main.h` / 删 `hal_conf.h` / `led.c` / `can_cfg.c`
4. `clock.c` + `main.c`
5. `mcan.c` / `mcan.h`（先核实 FDCAN 消息 RAM 基址与偏移）
6. 删 `build/` 全量重建；`arm-none-eabi-size` 与基线对比
7. 上板回归

---

## 六、验证清单

- [ ] 全量重建通过，无 warning 新增
- [ ] `text` 不增（预期明显变小：HAL FDCAN/RCC/GPIO/PWR 全部移除）
- [ ] `22 F184` 读镜像 CRC32 与 `tools/read_fw_info.py` 离线值一致
- [ ] **FDCAN 专项回归**：发送、接收、`2E F1A0` 波特率切换、Bus Off 恢复
- [ ] 刷写全链路：`10 02` → `27 01/02` → `22 FF00` → BOOT → `34/36/37/31`
- [ ] PC6 心跳正常；`22 F1 84` / `22 F4 01` 等 DID 读取正常
