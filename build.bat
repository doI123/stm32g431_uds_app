@echo off
REM ============================================================================
REM  STM32G431CBU6 UDS Bootloader 构建脚本 (GCC + CMake + Ninja)
REM
REM  说明: 每次都先执行 cmake 配置, 以便刷新 build_info.h 的时间戳。
REM ============================================================================
setlocal

REM --- ARM 工具链路径 (如已加入 PATH 可删除本行) ---
set "TOOLCHAIN_PATH=C:\Program Files (x86)\GNU Arm Embedded Toolchain\10 2021.10\bin"
set "PATH=%TOOLCHAIN_PATH%;%PATH%"

REM --- 构建目录 (位置不敏感) ---
if not exist build mkdir build
cd build

REM --- 配置: 指定 Ninja 生成器, 交叉编译工具链文件 ---
cmake .. -G "Ninja" -DCMAKE_TOOLCHAIN_FILE=../cmake/arm-none-eabi.cmake
if errorlevel 1 exit /b 1

REM --- 编译 ---
cmake --build .
if errorlevel 1 exit /b 1

endlocal
