cmake_minimum_required(VERSION 3.16)

project(stm32g431_uds_app ASM C)

# ---------------------------------------------------------------------------
# 工具链（arm-none-eabi 需在 PATH 中；交叉编译时测试编译改为静态库）
# ---------------------------------------------------------------------------
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# ---------------------------------------------------------------------------
# 路径：驱动已内置到工程内（100% LL，不依赖工作区外的 stm32cubeg4 包）
#   drivers/STM32CubeG4_V1.6.0/ 下只含 CMSIS + stm32g4xx_ll_*.h，
#   **没有任何 HAL 源码**，因此本工程不含 HAL 驱动。
# ---------------------------------------------------------------------------
if(NOT DEFINED STM32CUBE_ROOT)
    set(STM32CUBE_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/drivers/STM32CubeG4_V1.6.0)
endif()

# 目录名沿用厂商包的 Drivers/STM32G4xx_HAL_Driver —— 其 Inc/ 同时装着
# stm32g4xx_ll_*.h（本工程实际使用的头文件）与 stm32g4xx_hal_*.h（未内置、
# 也不参与编译，仅因厂商目录结构如此）。
set(CUBE_DRV_DIR   ${STM32CUBE_ROOT}/Drivers/STM32G4xx_HAL_Driver)
set(LL_INC         ${CUBE_DRV_DIR}/Inc)
set(CMSIS_INC      ${STM32CUBE_ROOT}/Drivers/CMSIS/Include)
set(CMSIS_DEV_INC  ${STM32CUBE_ROOT}/Drivers/CMSIS/Device/ST/STM32G4xx/Include)
set(CMSIS_DEV_SRC  ${STM32CUBE_ROOT}/Drivers/CMSIS/Device/ST/STM32G4xx/Source/Templates)
set(STARTUP_SRC    ${CMSIS_DEV_SRC}/gcc)

set(PROJ_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/source)

# 按功能分层的源码子目录（新增模块时在此登记）
set(APP_DIR   ${PROJ_SOURCE_DIR}/app)            # 应用层：启动与版本标识
set(BSP_DIR   ${PROJ_SOURCE_DIR}/bsp)            # 板级支持：board/clock/led
set(CORE_DIR  ${PROJ_SOURCE_DIR}/core)           # 核心服务：事件与调度（预留）
set(UDS_DIR   ${PROJ_SOURCE_DIR}/uds)            # 诊断协议栈（ISO-TP + UDS 服务）
set(CAN_DIR   ${PROJ_SOURCE_DIR}/drivers/can)    # CAN 外设驱动

set(LINKER_DIR      ${CMAKE_CURRENT_SOURCE_DIR}/startup)
set(TOOLS_DIR       ${CMAKE_CURRENT_SOURCE_DIR}/tools)
set(GEN_DIR         ${CMAKE_BINARY_DIR}/generated)

# ---------------------------------------------------------------------------
# 固件信息块（fw_info）身份参数
# ---------------------------------------------------------------------------
set(FW_ROLE_NAME   "FW_ROLE_APP")
set(FW_SW_VER      "APP_V1.00")
set(FW_HW_VER      "HW_A1")

# ---------------------------------------------------------------------------
# 头文件搜索路径
#   LL 是**纯头文件**实现（全部 __STATIC_INLINE），头文件位于厂商包的
#   Drivers/STM32G4xx_HAL_Driver/Inc（见上面的 LL_INC）。
#
#   ⚠️ 全部工程目录都写在 LL_INC 之前，好让本工程同名头文件优先于固件包被
#      命中；HAL 已完全移出本工程（无 stm32g4xx_hal_conf.h、无 HAL 源码），
#      因此此处不再需要为 HAL 裁剪配置预留优先级。
#      新增层目录请一并加在此处。
# ---------------------------------------------------------------------------
include_directories(
    ${BSP_DIR}
    ${APP_DIR}
    ${UDS_DIR}
    ${CAN_DIR}
    ${GEN_DIR}
    ${LL_INC}
    ${CMSIS_DEV_INC}
    ${CMSIS_INC}
)

# ---------------------------------------------------------------------------
# 编译宏定义与编译选项
# ---------------------------------------------------------------------------
add_compile_definitions(
    STM32G431xx
    HSE_VALUE=8000000UL
    HSE_STARTUP_TIMEOUT=100U
    FW_ROLE_ID=${FW_ROLE_NAME}
    FW_SW_STRING="${FW_SW_VER}"
    FW_HW_STRING="${FW_HW_VER}"
    # 本工程链接在 0x08006000，不是 Flash 基址，因此**不**定义
    # USER_VECT_TAB_ADDRESS —— 让厂商的 SystemInit() 不去设置 VTOR，
    # 由 App_VectorTableInit() 在 main() 里显式指向 APP_START_ADDR。
    #
    # 为什么不像 GD32 那样用 -DVECT_TAB_OFFSET=0x6000 覆盖：
    #   system_stm32g4xx.c 里 VECT_TAB_OFFSET 是无条件 #define，
    #   命令行覆盖会触发宏重定义警告且不生效。
)

# 注意：不能把这些标志放进变量再用 ${VAR} 展开 —— CMAKE_C_FLAGS 是字符串，
#       而 CMake 变量里的分号在字符串上下文中会被原样保留，导致 GCC 收到
#       形如 "-mfpu=fpv4-sp-d16;-mfloat-abi=hard" 的单个非法参数。
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard")
# 优化等级 -O3：本工程无硬实时中断服务，对指令级时序不敏感，
# 加入完整 UDS 后用最高优化换取代码体积余量。
# ⚠️ 注意 Mcan_Send() 的超时是"循环计数"而非毫秒计时，-O3 会让等效超时略变短。
#    该文件必须与 BOOT 侧保持逐字节一致，故不在此处改动；
#    若实测偶发 TX 失败，改为按 SysTick 计时（两个工程要同步改）。
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu11 -O3 -g3 -Wall -Wno-main -ffunction-sections -fdata-sections -fno-common")
set(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS} -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard -x assembler-with-cpp")

add_link_options(-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard)
add_link_options(-T ${LINKER_DIR}/stm32g431_app.ld)
add_link_options(--specs=nano.specs --specs=nosys.specs)
add_link_options(-Wl,--gc-sections -Wl,-Map=${CMAKE_BINARY_DIR}/stm32g431_uds_app.map)
add_link_options(-lm)

# ---------------------------------------------------------------------------
# 源文件列表
#
#   ⚠️ 顺序与原工程保持一致（未重组前的排列）：
#      链接顺序会影响节区对齐，从而让 .text 产生数字节差异；
#      而镜像 CRC32 会被上位机通过 DID 0xF184 读取，故刻意不调整顺序。
#      新增文件请追加在列表末尾，不要插入中间。
# ---------------------------------------------------------------------------
set(APP_SOURCES
    ${APP_DIR}/main.c           # 应用层
    ${BSP_DIR}/clock.c          # 板级支持层
    ${CAN_DIR}/mcan.c           # CAN 驱动层
    ${BSP_DIR}/led.c            # 板级支持层
    ${UDS_DIR}/boot_req.c       # 诊断：ISO-TP 接收侧 + 交棒
    ${CAN_DIR}/can_cfg.c        # CAN 驱动层：波特率持久化
    ${UDS_DIR}/app_uds.c        # 诊断：UDS 协议层
    ${UDS_DIR}/app_uds_did.c    # 诊断：DID 业务表
    ${UDS_DIR}/app_uds_param.c  # 诊断：参数区
    ${UDS_DIR}/app_uds_dtc.c    # 诊断：DTC 表
    ${APP_DIR}/version.c        # 应用层：固件信息块
)

set(DEVICE_SOURCES
    ${CMSIS_DEV_SRC}/system_stm32g4xx.c
    ${STARTUP_SRC}/startup_stm32g431xx.s
)

# 说明：本工程 100% 使用 LL 库。LL 是**纯头文件**实现（全部 __STATIC_INLINE），
#       因此这里没有 HAL_SOURCES —— 不存在任何 stm32g4xx_hal_*.c 参与编译，
#       也不需要 stm32g4xx_hal_conf.h。

find_package(Python3 COMPONENTS Interpreter QUIET)

add_executable(stm32g431_uds_app.elf
    ${APP_SOURCES}
    ${DEVICE_SOURCES}
)

# ---------------------------------------------------------------------------
# 生成 build_info.h（编译时间 / Git 信息）
# ---------------------------------------------------------------------------
file(MAKE_DIRECTORY ${GEN_DIR})
set(BUILD_INFO_HDR ${GEN_DIR}/build_info.h)

execute_process(
    COMMAND ${CMAKE_COMMAND}
            -DOUT=${BUILD_INFO_HDR}
            -DSOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}
            -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/gen_build_info.cmake
)

set_source_files_properties(${APP_DIR}/version.c
    PROPERTIES OBJECT_DEPENDS ${BUILD_INFO_HDR})

# ---------------------------------------------------------------------------
# 产物输出：ELF / BIN / HEX，并回填固件信息块
# ---------------------------------------------------------------------------
find_program(CMAKE_OBJCOPY arm-none-eabi-objcopy REQUIRED)
find_program(CMAKE_SIZE    arm-none-eabi-size    REQUIRED)

add_custom_command(TARGET stm32g431_uds_app.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:stm32g431_uds_app.elf>
            ${CMAKE_BINARY_DIR}/stm32g431_uds_app.bin
    COMMENT "Generating raw binary"
)

# 回填 image_size / image_crc32 / info_crc32（需要 Python3）
if(Python3_Interpreter_FOUND)
    add_custom_command(TARGET stm32g431_uds_app.elf POST_BUILD
        COMMAND ${Python3_EXECUTABLE} ${TOOLS_DIR}/fill_fw_info.py
                ${CMAKE_BINARY_DIR}/stm32g431_uds_app.bin
                0x08006000
        COMMENT "Filling fw_info (image size / CRC32)"
    )
else()
    message(WARNING "Python3 not found - skipping fw_info backfill")
endif()

# Intel HEX 由已回填的 .bin 生成，起始地址 = APP_START_ADDR。
# 上位机就是按十六进制文件的起始地址决定 0x34 请求下载的地址，
# 因此这里必须是 0x08006000，否则会被 Bootloader 以 NRC 0x31 拒绝。
add_custom_command(TARGET stm32g431_uds_app.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -I binary -O ihex --change-addresses 0x08006000
            ${CMAKE_BINARY_DIR}/stm32g431_uds_app.bin
            ${CMAKE_BINARY_DIR}/stm32g431_uds_app.hex
    COMMENT "Generating Intel HEX from filled binary"
)

add_custom_command(TARGET stm32g431_uds_app.elf POST_BUILD
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:stm32g431_uds_app.elf>
)
