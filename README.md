# STM32G431CBU6 UDS Application

Bootloader 的跳转目标，同时是一个**完整的 UDS 诊断服务端**：
除实现「请求进入 BOOT」交棒外，还提供 DID 读写、DTC 故障诊断、例程控制、
功能寻址（0x7DF 广播）等标准诊断能力，供上位机在 App 运行期间直接使用。

配套工程：`../STM32G431_UDS_Boot`（Bootloader）。

---

## 1. 为什么需要这个工程

刷写流程的痛点是「App 与 Boot 之间如何交棒」：

- App 运行期间**不实现 Flash 擦写**（那是 BOOT 的职责），无法被直接刷写；
- App 虽能自行复位，但复位后 Bootloader 无法区分
  「普通上电（应跳 App）」与「上位机要求刷写（应留在 BOOT）」，
  会在等待窗口结束后又跳回 App，形成**复位循环**。

因此需要两道门禁 + 一个跨复位标志：

```
上位机                         App                        Bootloader
  |                             |                             |
  |-- 0x10 02 编程会话 -------->|                             |
  |-- 0x27 01 求种子 ---------->|                             |
  |<-- 0x67 01 <seed> ----------|                             |
  |-- 0x27 02 <key> ----------->|  校验 key = seed ^ 0x5A5A5A5A
  |<-- 0x67 02 -----------------|                             |
  |-- 0x22 FF00 --------------->|  置 TAMP->BKP0R = "BOOT"     |
  |                             |-- NVIC_SystemReset() ------>|
  |                             |                             | 读到标志
  |<-- 0x22 FF01 返回 0x01 -----|                             | 驻留等待刷写
  |-- 0x34 / 0x36 / 0x37 / 0x31 刷写流程 ------------------->|
```

**为什么要在 App 里放 0x27**：App 要跳 BOOT 才能刷写，那么「跳 BOOT」这个
决定必须由 App 自己鉴权。若只靠 Bootloader 的 0x27，App 早就已经跳过去了
—— 门禁形同虚设。因此 App 必须自带同一套 seed/key 校验。

**为什么不做成「读版本号顺带触发」**：读版本必须保持纯只读、无副作用，
否则任何监测/日志轮询都会把运行中的 App 打掉；而且刷写意图应当能在 CAN
报文里明确看到。

---

## 1.1 软件分层

```
┌──────────────────────────────────────────────────────────────┐
│ 应用层 · UDS 诊断业务                                          │
│  ├─ app_uds_cfg.h    DID/RID/会话/安全/功能寻址 参数定义         │
│  ├─ app_uds_did.c    DID 业务表（0x22 读 / 0x2E 写的数据源）     │
│  ├─ app_uds_param.c  参数区 0xF900 ~ 0xF93F（14 项，u16/f32）  │
│  └─ app_uds_dtc.c    DTC 表与 0x19 / 0x14 / 0x85               │
├──────────────────────────────────────────────────────────────┤
│ UDS 协议层    app_uds.c  会话状态机 / S3 超时 / 安全访问 /       │
│                          NRC 生成 / 服务分发 / ISO-TP 发送侧     │
├──────────────────────────────────────────────────────────────┤
│ ISO-TP 传输层 boot_req.c  接收侧（SF/FF+CF，唯一 Rx FIFO 消费者）│
├──────────────────────────────────────────────────────────────┤
│ CAN 驱动层    mcan.c  FDCAN1 经典 CAN，波特率查表，双滤波器      │
└──────────────────────────────────────────────────────────────┘
```

**分层铁律**：`boot_req.c` 是全系统**唯一**的 FDCAN Rx FIFO 消费者。
若后续 FOC 也要收 CAN 报文，必须在该文件的分发处把非诊断帧转交应用层，
不能另外再起一个轮询点 —— 两个消费者会互相抢帧。

---

## 2. 硬件 / 配置

与 Bootloader 完全一致：

| 项目 | 取值 |
| --- | --- |
| MCU | STM32G431CBU6（Cortex-M4F，48 引脚，UFQFPN48） |
| Flash | 128 KB @ `0x08000000`（页大小 2 KB，共 64 页） |
| SRAM | **22 KB** @ `0x20000000`（SRAM1 16K + SRAM2 6K） |
| 外部晶振 | **8 MHz**（HSE） |
| 系统主频 | **170 MHz**（PLL：M=/2，N=85，R=/2） |
| FDCAN 时钟 | **8 MHz**（**HSE 直接驱动**，CKDIV=/1） |
| CAN | FDCAN1 @ **800 kbps 默认**（**可运行时切 125k/250k/500k/800k/1M**） |
| CAN 引脚 | PB9 = TX，PA11 = RX（AF9）；**PC11 = 收发器 S 脚（需拉低）** |
| App 基址 | **`0x08006000`** |
| 状态灯 | **PC6**，高电平有效 |

---

## 3. 行为

### 向量表重定向

厂商 `system_stm32g4xx.c` 只在定义了 `USER_VECT_TAB_ADDRESS` 时才写
`SCB->VTOR`，且写的是 `FLASH_BASE`（即 **Bootloader 的向量表**）。

本工程**刻意不定义**该宏，改由 `App_VectorTableInit()` 在 `main()` 开头
显式设置：

```c
SCB->VTOR = APP_START_ADDR;   /* 0x08006000 */
__DSB();
__ISB();
```

在使能任何外设中断之前调用即可 —— 复位后 NVIC 与 SysTick 都处于关闭状态，
`SystemInit()` 也只配置 FPU，因此从复位到该调用之间不会发生异常。

> 这比 GD32 那边更简单：不必 fork 厂商文件，也不必与
> `-DVECT_TAB_OFFSET` 的宏重定义问题斗争。

### 状态灯（PC6）

| 状态 | 闪烁周期 | 含义 |
| --- | --- | --- |
| **极快闪** | 60 ms | **时钟配置失败**（HSE 未起振 / PLL 未锁定 / 调压器不可控），主频停留在 HSI 16 MHz |
| 慢闪 | 500 ms | App 正常运行心跳 |

> **为什么需要\"极快闪\"这个独立状态**：时钟配置失败时主频只有 HSI 16 MHz，
> CAN 位时序与毫秒时基都是错的，但程序仍会\"正常\"跑下去。若不显式报错，
> 现象会是\"灯闪但 CAN 不通\"，很难定位到时钟问题。

> ⚠️ **`HAL_MspInit()` 必须实现**（见 `source/bsp/clock.c`）。
> HAL 库里它是 `__weak` 空函数；不实现的话 PWR 时钟不会使能，
> `HAL_PWREx_ControlVoltageScaling()` 会因写入被静默丢弃而超时失败，
> 170 MHz 就处在超规格状态。App 与 Bootloader 两侧都已实现。

### UDS 服务集

实现的全部服务（未实现的服务在**物理寻址**下回 NRC `0x11`，
使上位机快速失败而不是等到请求超时）：

| SID | 服务 | 说明 | 会话要求 | 需解锁 |
| --- | --- | --- | --- | --- |
| `0x10` | DiagnosticSessionControl | `01` 默认 / `02` 编程 / `03` 扩展 | — | — |
| `0x11` | ECUReset | `01`/`02`/`03` → 复位回 App | — | — |
| `0x14` | ClearDiagnosticInformation | `FFFFFF` = 清全部 | — | ✔ |
| `0x19` | ReadDTCInformation | 子功能 `01`/`02`/`03`/`04`/`06`/`0A`/`14` | — | — |
| `0x22` | ReadDataByIdentifier | 支持**一次读多个 DID** | 见 DID 表 | 见 DID 表 |
| `0x27` | SecurityAccess | `01` 求种子 / `02` 发密钥 | — | — |
| `0x28` | CommunicationControl | 只应答，不真正关闭收发 | — | ✔ |
| `0x2E` | WriteDataByIdentifier | 见 DID 表 | 见 DID 表 | ✔ |
| `0x31` | RoutineControl | `01`/`02`/`03`（RID 见下） | — | 部分 |
| `0x3E` | TesterPresent | `00` | — | — |
| `0x85` | ControlDTCSetting | `01` ON / `02` OFF | — | ✔ |

**会话与安全的总规则**：

- 服务在**当前会话**不允许时回 NRC `0x7F`（ServiceNotSupportedInActiveSession），
  而不是 `0x11` —— 两者语义不同，前者能让上位机知道"换个会话就能用"；
- 非默认会话下超过 `UDS_S3_TIMEOUT_MS`（5 s）没收到 `0x3E`，自动回落默认会话
  并清掉解锁状态；
- `0x27` 密钥连续错 3 次后进入 10 s 禁止期，期间请求种子回 NRC `0x37`。

### 0x27 安全访问的三个门禁

密钥算法固定为 `key = seed ^ 0x5A5A5A5A`（4 字节大端），必须与 BOOT 侧和
上位机的 `ac310_xor` 三方一致。在此之上还有三道门禁：

| 门禁 | 违反时 | 作用 |
| --- | --- | --- |
| **必须先 `27 01` 求种子** | NRC `0x24`（RSE） | 🔴 **关键**：seed 初值为 0，而密钥是固定异或，所以 `ComputeKey(0) = 0x5A5A5A5A` —— 不检查顺序的话，刚上电直接发 `27 02 5A 5A 5A 5A`（跳过 `27 01`）就能解锁。这是真实的鉴权绕过，已修 |
| **种子一次性** | NRC `0x24` | 验证（成功或失败）后立即作废，防止同一密钥重放 |
| **连续错 3 次** | NRC `0x35` → `0x36` | 第 3 次错误回 `0x36`（ENOA）并进入 10 s 禁止期，期间 `27 01` 回 `0x37` |

> **零种子语义**（ISO 14229-1 §10.4.4）：ECU 已解锁时 `27 01` 回全 0 种子，
> 表示"无需再解锁"。此时若上位机仍补发 `27 02`，ECU 直接回肯定响应
> （按"已解锁即成功"处理），避免被误判为解锁失败。
>
> ⚠️ **已知局限**：种子是 `SysTick ^ 0x5A5A5A5A | 1`，属于**可预测**的伪随机
> —— 在总线上抓几帧即可推算规律。生产环境建议改用 STM32G4 的硬件 RNG
> （`RNG->DR`），密钥算法保持不变，上位机无需改动。

交棒门禁（两道，缺一不可）：

1. 必须已通过 `0x27` 解锁（密钥正确）；
2. 必须发送 `0xFF00` 固件更新请求 DID。

未解锁时对 `0xFF00` 回 NRC `0x33`（SecurityAccessDenied），**不跳转**。

### 功能寻址（0x7DF）

App 额外装了一个验收滤波器放行功能寻址 ID `0x7DF`（`board.h` 的
`UDS_CAN_FUNC_RX_ID` + `MCAN_EXTRA_FILTER_ID`），与物理寻址共用一个 Rx FIFO，
上层按收到的 CAN ID 区分寻址类型。

| 规则 | 说明 |
| --- | --- |
| **一律不应答** | 肯定与否定响应都不发。这是功能寻址最容易做错的地方 —— 若回 NRC，总线上多个节点会同时应答而互相干扰 |
| **白名单外静默丢弃** | 不回 NRC `0x11` |
| **仅支持单帧** | ISO 15765-2 不定义广播多帧，收到首帧直接忽略（不回流控帧，否则会与其它节点冲突） |
| **禁止敏感服务** | 安全访问 / 写 DID / 例程控制 / 固件更新请求不在白名单内，否则一条广播就能解锁或改动总线上所有节点 |

白名单（见 `app_uds_cfg.h` 的 `APP_UDS_FUNC_SID_*`）：
`0x10` `0x11` `0x14` `0x19` `0x22` `0x28` `0x3E` `0x85`。

> **"抑制肯定响应"位（子功能 bit7）与功能寻址是两回事**：
> 前者只压制**肯定**响应（物理寻址下使用，否定响应照发）；
> 后者彻底不响应。代码里用 `m_u8SuppressPos` 与 `m_u8IsFuncAddr`
> 两个独立标志区分。

### DID 一览

| DID | 名称 | 长度 | 会话 | 读 | 写 |
| --- | --- | --- | --- | --- | --- |
| `0xF181` | ApplicationSoftwareIdentification | 变长 | 任意 | ✔ | — |
| `0xF184` | ApplicationSoftwareFingerprint（镜像 CRC32） | 4 | 任意 | ✔ | — |
| `0xF186` | ActiveDiagnosticSession | 1 | 任意 | ✔ | — |
| `0xF18C` | ECUSerialNumber（本工程用 git 短散列） | 8 | 任意 | ✔ | — |
| `0xF193` | SystemSupplierECUHardwareVersionNumber | 变长 | 任意 | ✔ | — |
| `0xF195` | SystemSupplierECUSoftwareVersionNumber | ≤12 | 任意 | ✔ | — |
| `0xF197` | SystemNameOrEngineType（`FOC_G431`） | 8 | 任意 | ✔ | — |
| `0xF199` | ProgrammingDate | 19 | 任意 | ✔ | — |
| `0xF1A0` | CAN 波特率档位 | 1 | 任意 | ✔ | ✔ |
| `0xF1B0` | 节点地址 | 1 | 任意 | ✔ | ✔ |
| `0xF401` | 电机转速 rpm | 2 | 任意 | ✔ | — |
| `0xF402` | 电机温度 °C | 1 | 任意 | ✔ | — |
| `0xF403` | 母线电压 mV | 2 | 任意 | ✔ | — |
| `0xF404` | 三相电流 | 6 | 任意 | ✔ | — |
| `0xF405` | 状态字 | 4 | 任意 | ✔ | — |
| `0xF406` | 故障字 | 4 | 任意 | ✔ | — |
| `0xF407` | 电角度（0.1°） | 2 | 任意 | ✔ | — |
| `0xF900`~`0xF93F` | **参数区（14 项，2 或 4 字节）** | 2/4 | 扩展/编程 | ✔ | ✔ |
| `0xFF00` | FirmwareUpdateRequest（交棒） | 0 | 任意 | ✔（需解锁） | — |

**一次读多个 DID**（ISO 14229-1 §14.4.2）：

```
请求：22 F9 00 F9 01 F9 02                      (DID 数量不限，只要响应放得下)
响应：62 F9 00 00 32 F9 01 0B B8 F9 02 00 07    (u16 类型，各 2 字节)
```

> 响应长度会自动累加：`APP_UDS_TX_BUF_LEN`（256 字节）决定一次最多能连读
> 多少项。混读 `u16` 与 `f32` 也支持（各自按类型长度拼接）。
> 若累加超过缓冲，回复 NRC `0x14`（ResponseTooLong）而不是截断 ——
> 截断会让上位机拿到错位的数据。

### 可读写参数区（DID 0xF900 ~ 0xF93F）

**14 个参数，与上位机 `params.items` 逐项对应**（DID、类型、scale、min/max、
可写性全部一致），实现见 `app_uds_param.c`。改任一侧都要同步另一侧。

| DID | 分组 | 名称 | 类型 | 长度 | scale | 单位 | 范围（物理值） | 可写 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `F900` | 电机参数 | 额定电流 | `u16` | 2 | 0.1 | A | 0.0 ~ 100.0 | ✔ |
| `F901` | 电机参数 | 额定转速 | `u16` | 2 | 1.0 | rpm | 0.0 ~ 6000.0 | ✔ |
| `F902` | 电机参数 | 极对数 | `u16` | 2 | 1.0 | — | 1.0 ~ 64.0 | ✔ |
| `F903` | 电机参数 | 反电势常数 | **`f32`** | 4 | 1.0 | V/krpm | — | ✘ 只读 |
| `F910` | 电流环参数 | Kp | **`f32`** | 4 | 1.0 | — | 0.0 ~ 100.0 | ✔ |
| `F911` | 电流环参数 | Ki | **`f32`** | 4 | 1.0 | — | 0.0 ~ 100.0 | ✔ |
| `F912` | 电流环参数 | 带宽 | `u16` | 2 | 1.0 | Hz | 100.0 ~ 5000.0 | ✔ |
| `F920` | 速度环参数 | Kp | **`f32`** | 4 | 1.0 | — | 0.0 ~ 100.0 | ✔ |
| `F921` | 速度环参数 | Ki | **`f32`** | 4 | 1.0 | — | 0.0 ~ 100.0 | ✔ |
| `F922` | 速度环参数 | 速度限幅 | `u16` | 2 | 1.0 | rpm | 0.0 ~ 6000.0 | ✔ |
| `F930` | 保护参数 | 过流阈值 | `u16` | 2 | 0.1 | A | 1.0 ~ 200.0 | ✔ |
| `F931` | 保护参数 | 过压阈值 | `u16` | 2 | 0.1 | V | 10.0 ~ 600.0 | ✔ |
| `F932` | 保护参数 | 欠压阈值 | `u16` | 2 | 0.1 | V | 10.0 ~ 600.0 | ✔ |
| `F933` | 保护参数 | 过温阈值 | `i16` | 2 | 1.0 | ℃ | 0.0 ~ 150.0 | ✔ |

> 上位机配置的 `params.did_hi` 是 `0xFA00`，但实际只定义到 `F933`。
> `F904`~`F90F`、`F913`~`F91F`、`F923`~`F92F`、`F934` 以上**全部未定义**，
> 读写一律回 NRC `0x31`。这是有意保留的扩展空间。

**四种数据类型**（大端）：`i16`/`u16` = 2 字节，`i32`/`u32`/`f32` = 4 字节。
长度校验**按类型**进行 —— 给 `F910`（f32）发 2 字节会回 NRC `0x13`。

> ⚠️ **`f32` 参数的存在意味着长度不再统一**。上位机 `ty` 字段必须与 ECU
> 描述表一致，否则要么写入被拒（0x13），要么读到错位的数据。

**两条设计原则**：

1. **RAM 里统一存"物理值"（float）**，读写时按该 DID 的类型转换。
   好处是"类型 / scale / 量程"三件事只在描述表里写一次 ——
   协议层与 FOC 应用层都不必知道某条参数是 `u16` 还是 `f32`：

   ```c
   /* FOC 侧只管物理值，不关心总线格式 */
   float f32Kp;
   (void)AppUdsParam_GetPhysical(0xF910U, &f32Kp);   /* 拿到的是 1.0 这样的数 */
   (void)AppUdsParam_SetPhysical(0xF920U, 0.8f);     /* 写入前自动量程校验 */
   ```

2. **逐条量程**（min/max 是物理值），越界回 NRC `0x31`。
   上位机配置里明确写着这是"防止手误写坏标定的第一道闸门"，
   因此不采用统一量程 —— 例如 `F912` 带宽下限 100 Hz，
   若沿用"任何值都收"的策略，误发 0 会让电流环直接失效。

**报文示例**：

```
读 22 F9 00               -> 62 F9 00 00 32          (0x0032=50 -> ×0.1 = 5.0 A)
写 2E F9 00 00 64         -> 6E F9 00                (0x0064=100 -> 10.0 A)
写 2E F9 00 03 E8         -> 7F 2E 31                (1000 -> 100.0 A，==上限，收)
写 2E F9 00 03 E9         -> 7F 2E 31                (1001 -> 100.1 A，越界被拒)
读 22 F9 10               -> 62 F9 10 3F 80 00 00    (f32 1.0 的位模式)
写 2E F9 10 41 20 00 00   -> 6E F9 10                (f32 10.0)
写 2E F9 10 00 0A         -> 7F 2E 13                (f32 参数只收 4 字节，长度错)
写 22 F9 03               -> 62 F9 03 3F 80 00 00    (只读参数，读正常)
写 2E F9 03 41 20 00 00   -> 7F 2E 31                (只读参数拒绝写入)
读 22 F9 04               -> 7F 22 31                (未定义的 DID)
```

3. **读需扩展/编程会话，写额外需 0x27 解锁**：参数会影响控制环，
   默认会话下不接受读写，避免误连的诊断仪在上电默认状态就把参数改掉。

**本工程无掉电存储**，参数只在本上电周期有效。若后续要持久化，
实现 `AppUdsParam_Save()`（`app_uds_param.c` 里已留 `__attribute__((weak))`
空实现钩子），无需改动本模块的其它代码。

参数区相关例程（`0x31`）：

| RID | 子功能 | 作用 |
| --- | --- | --- |
| `0x0201` | `01` | 应用层自检（占位，需解锁） |
| `0x0202` | `01` | **全部参数恢复默认值**（需解锁） |
| `0x0203` | `01` | 参数一致性检查（越界/不合法项自动钳回默认值） |
| `0x0204` | `01` | 保存配置（调用持久化钩子；本工程为空操作） |
| `0xFF01` | `01` | 检查编程依赖（App 在跑即视为通过） |

### DTC 故障诊断

纯 RAM 表（16 条上限，无掉电保持 —— 刷写后 DTC 清零本就是正确语义）。
每条记录含 3 字节 DTC、状态字节、老化计数、发生次数与 4 字节扩展数据。

`0x19` 各子功能的响应格式：

| 子功能 | 请求 | 响应 |
| --- | --- | --- |
| `01` | `19 01 <mask>` | `59 01 <availMask> <format> <count(2)>` |
| `02` | `19 02 <mask>` | `59 02 <availMask> [<DTC(3)><status>]*` |
| `03` | `19 03` | `59 03 [<DTC(3)><status>]*`（快照标识） |
| `04` | `19 04 <DTC(3)> <recNo>` | `59 04 <DTC(3)><status> <recNo> <snapshot[]>` |
| `06` | `19 06 <DTC(3)> <recNo>` | `59 06 <DTC(3)><status> <recNo> <extData[]>` |
| `0A` | `19 0A` | `59 0A <availMask> [<DTC(3)><status>]*` |
| `14` | `19 14 <DTC(3)>` | `59 14 <DTC(3)><status> <FDC>` |

**FDC（故障检测计数器）约定**：`0x00`~`0x7F` 未失败，`0x80`~`0xFF` 已失败
（与 ISO 14229-1 的有符号 −128..+127 等价，用无符号字节读起来更直观）。

**应用层接入方式**（FOC 侧）：

```c
AppUdsDtc_Report(APP_DTC_OVERVOLTAGE, 1, ext, 4);  /* 检测到故障 -> 置位 */
AppUdsDtc_Report(APP_DTC_OVERVOLTAGE, 0, NULL, 0); /* 故障消失   -> 清 testFailed */
AppUdsDtc_OperationCycle();                        /* 每个操作循环调一次（老化） */
```

老化语义：连续 20 个操作循环无故障则自动清 `confirmed` 位。

### CAN 波特率运行时切换（0xF1A0）

App 可以在线把 CAN 波特率切到 **125k / 250k / 500k / 800k / 1M**，
**BOOT 则固定 800 k**（刷写期间切速率会让上位机立刻失联，因此不在 BOOT 暴露）。

| 操作 | 请求 | 响应 |
| --- | --- | --- |
| 读当前档位 | `22 F1 A0` | `62 F1 A0 <档位码>` |
| 写档位 | `2E F1 A0 <档位码>` | `6E F1 A0` |

档位码：`00`=125k `01`=250k `02`=500k `03`=800k `04`=1M `FF`=恢复默认。

**两个关键设计**：

1. **写操作需要先 `0x27` 解锁** —— 波特率直接决定能否通信，
   未鉴权时回 NRC `0x33`，不允许探测帧把设备切到其它速率（那等价于远程失联）。
2. **响应先发、切换后做**：`0x6E` 肯定响应在**旧**速率上发完后，
   才在下一个主循环里重新初始化 FDCAN。否则响应用新速率发出，
   而上位机此刻还停在旧速率，必然收不到而报超时。
   切换前还会等 Tx FIFO 排空（含 `Mcan_WaitTxDrain()`），否则 `HAL_FDCAN_Stop()`
   会把未发出的应答直接丢掉。

持久化在 **TAMP->BKP1R**（VBAT 域，复位与掉电后保持），因此**配置一次长期生效**：

- 未配置过时用默认 800 k —— 与 BOOT 一致，
  因此“从未配过”的板子从 BOOT 跳到 App 不会因速率变化而失联；
- **用显式 MAGIC 高位标记判定“已配置”**（`[31:16] = 0x4244` `"BD"`），
  不能用 `0xFFFFFFFF`，也不能用 0 —— 实测 STM32G4 的备份寄存器
  **上电初值是 0x00000000**（不是全 1），用 0 当哨兵会把全新板判成
  “已配置为 125 k”，导致 App 跑 125 k 而 BOOT 跑 800 k，总线直接对不上
  （本项目踩过这个坑）；
- 档位码 `0xFF` 为“未配置”哨兵，先查 MAGIC 再查哨兵，任一不满足即回落默认；
- 写入后读回校验，失败则报错（不谎报成功）；
- 恢复默认（`FF`）时把寄存器标回“未配置”，
  这样将来改 `CAN_DEFAULT_BAUDRATE` 能自动跟随。

> 上位机 `configs/*.json` 的 `can.bitrate` 必须与设备一致。
> 若设备已被切到非 800 k，先把上位机改成对应速率再连接。

### 跨复位标志

介质是 **TAMP 备份寄存器 `TAMP->BKP0R`**（VBAT 域，系统复位与掉电后仍保持）。

⚠️ **注意 STM32G4 与 GD32F4 的差异**：STM32G4 的备份寄存器挂在
**TAMP 外设**（不是 RTC），且其时钟由 `RCC_APB1ENR1.RTCAPBEN` 提供
（本芯片**没有**独立的 `TAMPEN` 位）：

```c
__HAL_RCC_PWR_CLK_ENABLE();
__HAL_RCC_RTCAPB_CLK_ENABLE();   /* 不是 TAMPEN */
HAL_PWR_EnableBkUpAccess();      /* 置 PWR_CR1.DBP */
TAMP->BKP0R = BOOT_REQ_MAGIC;
```

写入后会**读回校验**，确保不带着无效标志去复位（否则会陷入复位循环）。

---

## 4. 构建

前置条件与 Bootloader 相同（GNU Arm Embedded Toolchain、CMake ≥ 3.16、
Ninja、STM32CubeG4 固件包、Python 3）。

```bat
build.bat
```

优化等级为 **`-O3`**（`CMakeLists.txt`）。本工程无硬实时中断服务，
对指令级时序不敏感，用最高优化换取代码体积余量。

> ⚠️ 注意 `Mcan_Send()` 的超时是"循环计数"而非毫秒计时，
> `-O3` 会让等效超时变短。该文件与 BOOT 侧保持逐字节一致，
> 若实测偶发 TX 失败，改为按 `SysTick` 计时（两个工程需同步修改）。

产物位于 `build/`：

| 文件 | 说明 |
| --- | --- |
| `stm32g431_uds_app.elf` | 供调试 |
| `stm32g431_uds_app.hex` | **起始地址 0x08006000** —— 这就是要刷写的文件 |
| `stm32g431_uds_app.bin` | 原始二进制（已回填固件信息块） |
| `stm32g431_uds_app.map` | 链接映射表 |

当前占用：**约 22 KB 代码**（App 区 104 KB，余量约 84 KB），
**bss 约 5.1 KB / 22 KB RAM**。

RAM 的主要开销：`m_au8Resp` 256 B（发送缓冲）+ `m_astcDtc` 256 B（DTC 表）
+ `s_au8RxBuf` 128 B（接收缓冲）+ 参数值数组 98 B + 暂存队列 32 B。

### 构建后自检

```bat
python tools\check_layout.py build\stm32g431_uds_app.bin 0x08006000
```

输出示例：

```
起始地址    : 0x08006000
结束地址    : 0x0800B64F
初始 MSP    : 0x20005800
复位向量    : 0x080099F9
[ OK ] 起始地址与 Bootloader 的 APP_START_ADDR 一致
[ OK ] MSP 落在合法 RAM 区间
[ OK ] 复位向量落在 App 区内 （Thumb 位已置位）
[ OK ] 与 Bootloader 区（24 KB）无重叠
[ OK ] 未越过 App 区末端（余量 13880 B）
```

还可离线读回固件信息块，确认镜像大小与 CRC 已正确回填：

```bat
python tools\read_fw_info.py build\stm32g431_uds_app.bin
```

也可用 Python 复现上位机的解析逻辑：

```python
from intelhex import IntelHex
ih = IntelHex("build/stm32g431_uds_app.hex")
print(hex(min(s for s, _ in ih.segments())))   # -> 0x8006000
```

> 该地址决定上位机在 `0x34 RequestDownload` 中发送的地址，必须等于
> Bootloader 的 `APP_START_ADDR`，否则会被回 NRC `0x31`。

---

## 5. 刷写

### 方式 A —— 直接烧写（bring-up 最快）

```bat
openocd -f interface/cmsis-dap.cfg -f target/stm32g4x.cfg ^
        -c "program build/stm32g431_uds_app.hex verify reset exit"
```

HEX 带绝对地址，会落到 `0x08006000`，`0x08000000` 处的 Bootloader 不受影响。

### 方式 B —— 通过 UDS（真正的测试）

在 `uds_iap` 上位机里选择 `build/stm32g431_uds_app.hex` 后开始刷写。

- 若 App 正在运行：上位机会先发 `0x27` 解锁，再发 `0x22 FF00`；
  App 置标志并复位，Bootloader 接手。
- 若已在 Bootloader 中：直接走
  `0x10 02` → `0x27` → `0x31 01 FF00` 擦除 → `0x34` / `0x36` / `0x37` → `0x31 01 FF01`。

#### 刷写耗时与注意事项

本映像约 21.6 KB，会被拆成约 6 个 `0x36 TransferData` 请求
（固件侧 `UDS_BLOCK_SIZE = 4088`，上位机的 4096 会被 `0x34` 协商下调）。

- **耗时**：800 kbps 下约 21.6 KB ≈ 450 个 CAN 数据帧，加上每块应答与
  每帧的 ISO-TP 流控等待，整个过程通常 **1~2 秒**。
- **不要中途断电**：App 区被整片擦除后若写入中断，App 会失效
  （此时灯仍闪，但会因镜像校验失败而停在某种状态）。恢复办法是让
  Bootloader 重新刷一次 —— Boot 区独立，永远不会被这一步破坏。

---

## 6. ⚠️ Boot 窗口只有 200 ms

App 一旦烧写成功，`Boot_AppPresent()` 就返回 1，于是**每次复位**：

1. Bootloader 只监听 `BOOT_WAIT_JUMP_MS`（**当前 200 ms**）；
2. 窗口内无请求 → 直接跳转 App；
3. 进入 App 后，Bootloader 不再应答 `0x22` / `0x34` 等。

因此**必须在复位后 200 ms 内开始通信**。三种做法：

- **推荐**：App 运行时由上位机走 `0x27` + `0x22 FF00` 交棒（本工程已实现），
  不依赖复位时序；
- 按复位键后立即在上位机点「开始刷写」；
- 调整 `BOOT_WAIT_JUMP_MS`（`../STM32G431_UDS_Boot/drivers/board.h`），
  但缩短会更难抓住窗口，加长会拖慢正常启动。

---

## 6.1 手工验证清单

用 TSMaster / `uds_iap` 的原始帧发送功能逐一验证（会话 = 默认，除注明外）：

| 请求 | 期望响应 | 验证点 |
| --- | --- | --- |
| `10 03` | `50 03` | 进扩展会话（只回 2 字节） |
| `22 F1 86` | `62 F1 86 03` | 会话 DID 跟随 |
| `22 F1 97` | `62 F1 97 46 4F 43 5F 47 34 33 31` | `"FOC_G431"` |
| `22 F4 01` | `62 F4 01 00 00` | 实时数据（未接 FOC 时为 0） |
| `27 01` → `27 02` | `67 01 <seed>` → `67 02` | 解锁 |
| `27 02`（错误密钥 ×3） | `7F 27 35` → `7F 27 36` | 第三次回 ENOA |
| `27 01`（超限期内） | `7F 27 37` | 禁止解锁延时 |
| `3E 00` | `7E 00` | S3 保活 |
| `3E 80` | **无响应** | 抑制肯定响应位 |
| `19 02 FF` @ `7DF` | **无响应** | 功能寻址静默 |
| `2E F1 B0 02` @ `7DF` | **无响应** | 敏感服务禁止功能寻址 |
| `22 F9 00`（默认会话） | `7F 22 7F` | 参数区会话检查 |
| `10 03` → `22 F9 00` | `62 F9 00 00 32` | 进扩展后可读（0x32=50 → 5.0 A） |
| `22 F9 00 F9 01 F9 02` | `62 F9 00 00 32 F9 01 0B B8 F9 02 00 07` | 多 DID 连读（2 字节类型） |
| `22 F9 10` | `62 F9 10 3F 80 00 00` | **f32** 读（4 字节，1.0） |
| `2E F9 00 00 64`（未解锁） | `7F 2E 33` | 参数写需解锁 |
| `2E F9 00 00 64`（已解锁） | `6E F9 00` | 写入 100 → 10.0 A |
| `2E F9 00 03 E8`（已解锁） | `6E F9 00` | 1000 → 100.0 A（== 上限，收） |
| `2E F9 00 03 E9`（已解锁） | `7F 2E 31` | 1001 → 100.1 A 越界被拒 |
| `2E F9 10 41 20 00 00`（已解锁） | `6E F9 10` | **f32** 写 10.0 |
| `2E F9 10 00 0A`（已解锁） | `7F 2E 13` | f32 参数只收 4 字节，长度错 |
| `2E F9 03 41 20 00 00`（已解锁） | `7F 2E 31` | `F903` 只读，拒写 |
| `22 F9 04` | `7F 22 31` | 未定义的 DID |
| `27 02 5A 5A 5A 5A`（未先请求种子） | `7F 27 24` | **鉴权顺序门禁（RSE）** |
| `27 01` → `27 02` | `67 01 <非零seed>` → `67 02` | 正常解锁 |
| `27 01`（解锁后） | `67 01 00 00 00 00` | 零种子 = 已解锁 |
| `27 02 <同一密钥>`（重放） | `7F 27 24` | 种子一次性 |
| `31 01 02 03` | `71 01 02 03` | 参数一致性检查 |
| `31 01 02 02`（未解锁） | `7F 31 33` | 恢复默认需解锁 |
| `31 01 02 02`（已解锁） | `71 01 02 02` | 恢复默认成功 |
| `19 0A` | `59 0A FF`（表空时） | 全部 DTC 列表 |
| `19 01 FF` | `59 01 FF 01 00 00` | DTC 数量 |
| `19 14 00 01 01`（无此码） | `7F 19 31` | 未知 DTC |
| `14 FF FF FF`（未解锁） | `7F 14 33` | 清 DTC 需解锁 |
| `14 FF FF FF`（已解锁） | `54 FF FF FF` | 清全部 DTC |
| `85 02`（已解锁） | `C5 02` | DTC 记录关闭 |
| `2E F1 A0 02`（已解锁） | `6E F1 A0` → 总线切 500 k | 波特率切换在应答之后 |
| 进扩展会话后停 5 s | `22 F1 86` → `01` | S3 超时回落 + 清解锁 |
| `22 FF 00`（未解锁） | `7F 22 33` | 交棒门禁 |
| `22 FF 00`（已解锁） | `62 FF 00` → 复位 → BOOT 应答 | 交棒成功 |
| `11 01` | `51 01` → 复位回 App | ECUReset（灯重新开始心跳） |

---

## 7. 布局

```
STM32G431_UDS_App/
├─ build.bat
├─ CMakeLists.txt
├─ cmake/
│  ├─ arm-none-eabi.cmake
│  └─ gen_build_info.cmake
├─ startup/stm32g431_app.ld         # ORIGIN = 0x08006000；.fw_info 必须是最后一个 Flash 段
├─ tools/
│  ├─ fill_fw_info.py               # 回填 image_size / CRC32
│  ├─ read_fw_info.py               # 离线读回固件信息块
│  └─ check_layout.py               # 镜像起始地址 / 向量表自检
└─ source/                            # 源码根：按功能分层
   ├─ app/                            # 应用层：启动与版本标识
   │  ├─ main.c / main.h             # VTOR 修正、初始化、主循环
   │  └─ version.c / version.h       # 固件信息块
   ├─ bsp/                            # 板级支持层：板级配置、时基、指示、HAL 裁剪
   │  ├─ board.h                     # 存储器映射、CAN 引脚（含 PC11 S 脚）、LED、功能寻址 ID
   │  ├─ clock.c / clock.h           # 8 MHz HSE -> 170 MHz + ms 时基
   │  ├─ led.c / led.h               # PC6 状态灯
   │  └─ stm32g4xx_hal_conf.h        # HAL 裁剪配置（★ 必须由最先的 -I 目录提供）
   ├─ core/                           # 核心服务层（预留）：事件与调度
   │  └─ （event.c / event.h 规划放此）
   ├─ drivers/
   │  └─ can/                         # CAN 外设驱动
   │     ├─ mcan.c / mcan.h          # FDCAN1 驱动（经典 CAN，波特率查表，双滤波器）
   │     └─ can_cfg.c / can_cfg.h    # CAN 波特率配置持久化（TAMP->BKP1R）
   └─ uds/                            # 诊断协议栈（ISO-TP + UDS 服务）
      ├─ boot_req.c / boot_req.h     # 跨复位标志 + ISO-TP 接收侧（唯一 Rx FIFO 消费者）
      ├─ app_uds.c / app_uds.h       # UDS 协议层（会话/安全/分发/NRC/ISO-TP 发送）
      ├─ app_uds_cfg.h               # UDS 参数集中定义（DID/RID/会话/安全/功能寻址白名单）
      ├─ app_uds_did.c / .h          # DID 业务表（0x22 数据源 / 0x2E 落地）
      ├─ app_uds_param.c / .h        # 参数区（0xF900 ~ 0xF93F，与上位机 params.items 对齐）
      └─ app_uds_dtc.c / .h          # DTC 表与 0x19 / 0x14 / 0x85
```

**分层与 include 机制**：

- `CMakeLists.txt` 把 `bsp/`、`app/`、`uds/`、`drivers/can/` 全部加入
  `include_directories()`，并**排在 `HAL_INC` 之前**（`stm32g4xx_hal_conf.h`
  必须命中本工程 `bsp/` 下的裁剪版，否则编译直接报错）；
- 因此所有源文件里的 `#include "xxx.h"` 都是**裸文件名**、不含相对路径 ——
  移动文件到新的层目录后，`#include` 一行都不需要改；
- `bsp/` 必须排在最前，因为 HAL 裁剪配置放在该目录。

`clock.*`、`mcan.*`、`led.*` 与 `stm32g4xx_hal_conf.h` 与 Bootloader 版本
**完全一致**，使两个镜像共享同一套已验证配置。修改其一时请同步另一方。

> ⚠️ 两个工程的**目录结构不同**（本工程已按功能分层，BOOT 侧仍是平铺结构），
> 因此同步时请**按文件名比对内容**，不要按路径比对。建议用哈希确认：
> `certutil -hashfile <文件> SHA256`。
> App 侧对应路径：`source/bsp/clock.c`、`source/bsp/led.c`、
> `source/bsp/stm32g4xx_hal_conf.h`、`source/drivers/can/mcan.c`。

> 两侧的 `mcan.c` 通过 `board.h` 的 `MCAN_STD_FILTER_COUNT` /
> `MCAN_EXTRA_FILTER_ID` 宏驱动滤波器数量：BOOT 不定义 → 只用 1 个滤波器
> 收 `0x7E0`；App 定义 → 额外放行 `0x7DF`。**文件本身逐字节相同**。

---

## 8. 调参点

| 项目 | 位置 |
| --- | --- |
| 语义版本号 | `FW_VER_*` 宏，`source/app/version.c` |
| 软件版本字符串 | `CMakeLists.txt` 的 `FW_SW_VER` |
| App 基址 | `APP_START_ADDR`（`source/bsp/board.h`）+ `ORIGIN`（链接脚本）+ `--change-addresses`（CMake）——**三处必须一致** |
| CAN 引脚 | `source/bsp/board.h` 的 `CAN_TX_GPIO_*` / `CAN_RX_GPIO_*` / `CAN_GPIO_AF` |
| CAN 默认波特率 | `source/bsp/board.h` 的 `CAN_DEFAULT_BAUDRATE` —— **必须与 BOOT 一致** |
| CAN 支持的速率 / 位时序 | `source/drivers/can/mcan.c` 的 `s_astcBaudTable`（唯一真值来源）—— **必须与 BOOT 一致** |
| 功能寻址 ID | `source/bsp/board.h` 的 `UDS_CAN_FUNC_RX_ID`（`0x7DF`）+ `MCAN_EXTRA_FILTER_ID` + `MCAN_STD_FILTER_COUNT` |
| CAN 波特率配置 DID | `source/bsp/board.h` 的 `CAN_BAUD_DID`（默认 `0xF1A0`） |
| 波特率持久化寄存器 | `source/bsp/board.h` 的 `CAN_BAUD_BKP_REG`（默认 `TAMP->BKP1R`） |
| 状态灯引脚 / 周期 | `source/bsp/board.h` 的 `LED_*` 宏 |
| CAN 收发器 S（Standby）引脚 | `source/bsp/board.h` 的 `CAN_STB_*` 宏（PC11） |
| **DID / RID 编号、会话掩码、功能寻址白名单** | `source/uds/app_uds_cfg.h` |
| **新增一条 DID** | `source/uds/app_uds_did.c` 的 `s_astcDidTable[]` 加一行 + 写一个读写回调 |
| **参数区（DID/类型/scale/量程）** | `source/uds/app_uds_param.c` 的 `s_astcParamDesc[]` —— **必须与上位机 config 的 `params.items` 逐项一致**（DID、`ty`、`scale`、`min`/`max`、`writable`）。改任一侧都要同步另一侧 |
| **DTC 表容量与老化阈值** | `app_uds_cfg.h` 的 `APP_UDS_DTC_MAX`、`app_uds_dtc.h` 的 `APP_DTC_AGING_LIMIT` |
| **发送缓冲大小** | `app_uds_cfg.h` 的 `APP_UDS_TX_BUF_LEN`（决定 0x22 一次能连读几个 DID） |
| 是否强制 0x27 鉴权 | `app_uds_cfg.h` 的 `APP_UDS_REQUIRE_SECURITY` |
| 0x27 超限禁止时长 | `app_uds_cfg.h` 的 `APP_SA_DELAY_MS`（默认 10 s） |
| 是否启用功能寻址 / 是否开放读写内存 | `app_uds_cfg.h` 的 `APP_UDS_ENABLE_FUNC_ADDR` / `APP_UDS_ENABLE_MEM_ACCESS` |
| Boot 窗口长度 | `../STM32G431_UDS_Boot/drivers/board.h` 的 `BOOT_WAIT_JUMP_MS` |

---

## 9. 接入 FOC 电机控制

UDS 诊断与电机控制通过**弱函数**解耦：在应用层实现下列同名函数即可覆盖
`app_uds_did.c` 里的空实现，无需改动诊断模块的任何代码。

| 弱函数 | 对应 DID |
| --- | --- |
| `AppUdsData_GetMotorSpeedRpm()` | `0xF401` |
| `AppUdsData_GetMotorTemp()` | `0xF402` |
| `AppUdsData_GetBusVoltageMv()` | `0xF403` |
| `AppUdsData_GetPhaseCurrentMa()` | `0xF404` |
| `AppUdsData_GetMotorStateWord()` | `0xF405` |
| `AppUdsData_GetMotorFaultWord()` | `0xF406` |
| `AppUdsData_GetMotorAngleDeciDeg()` | `0xF407` |
| `AppUdsDtc_Report()` / `AppUdsDtc_OperationCycle()` | DTC 置位与老化 |
| `AppUdsParam_Save()` | 持久化钩子（本工程为空操作） |
| `AppUdsParam_GetPhysical/SetPhysical()` | 参数区读写（**物理值**，float；自动处理 u16/f32 与 scale） |

扩展时注意：

1. 把 FOC 算法放在新的层目录里（建议新建 `source/foc/`，并在
   `CMakeLists.txt` 的源文件列表**末尾**追加、在 `include_directories()`
   中登记该目录），在 `main()` 主循环中调用；
2. **不要**在 App 侧另起一个 FDCAN Rx FIFO 轮询点 ——
   已经由 `BootReq_Poll()` 独占消费。若 FOC 也要收 CAN 报文，
   应改 `boot_req.c`，在它的分发处把非诊断帧转交应用层；
   两个消费者会互相抢帧，这是最容易埋下的隐性 bug；
3. 参数区改动时，`source/uds/app_uds_param.c` 的 `s_astcParamDesc[]` 与上位机
   config 的 `params.items` **必须逐项一致**（DID、类型长度、scale、量程）；
   只改一侧会出现"写入被拒"或"读出来是垃圾"这类很难定位的问题；
4. 需要给某条参数**收紧**范围，直接改描述表里该行的 `f32Min`/`f32Max`；
4. 若某条参数需要掉电保持，实现 `AppUdsParam_Save()` 并在
   `AppUdsParam_Set()` 调用点补齐触发逻辑（当前只在 `0x0204` 例程中调用）。
