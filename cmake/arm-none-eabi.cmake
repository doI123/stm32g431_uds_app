# ---------------------------------------------------------------------------
# ARM GCC 裸机工具链文件（Cortex-M4F，硬件单精度浮点）
# ---------------------------------------------------------------------------
set(CMAKE_SYSTEM_NAME       Generic)
set(CMAKE_SYSTEM_PROCESSOR  arm)

# 定位 arm-none-eabi 工具链（需在 PATH 中）
find_program(CMAKE_C_COMPILER   arm-none-eabi-gcc)
find_program(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
find_program(CMAKE_OBJCOPY      arm-none-eabi-objcopy)
find_program(CMAKE_SIZE         arm-none-eabi-size)

if(NOT CMAKE_C_COMPILER)
    message(FATAL_ERROR "arm-none-eabi-gcc not found. Install the GNU Arm Embedded Toolchain and add it to PATH.")
endif()

set(CMAKE_C_COMPILER_WORKS   1)
set(CMAKE_ASM_COMPILER_WORKS 1)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
