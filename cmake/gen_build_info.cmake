# =============================================================================
# 生成 build_info.h —— 每次构建刷新编译时间与 Git 信息
#
# 调用方式（由 CMakeLists.txt 调用）：
#   cmake -DOUT=<输出头文件路径> -DSOURCE_DIR=<工程源码根> -P gen_build_info.cmake
#
# 说明：使用 __DATE__ / __TIME__ 不可靠——只有当该 .c 文件被重新编译时才会
#       刷新，而增量构建常常不会重编。这里改由 CMake 每次构建都重新生成头文件。
# =============================================================================

# ---------- 编译时间（本地时间 + UTC 时间戳） ----------
string(TIMESTAMP _date  "%Y-%m-%d")
string(TIMESTAMP _time  "%H:%M:%S")
string(TIMESTAMP _epoch "%s" UTC)

# ---------- Git 信息（无仓库时优雅降级） ----------
execute_process(
    COMMAND git rev-parse --short=8 HEAD
    WORKING_DIRECTORY "${SOURCE_DIR}"
    OUTPUT_VARIABLE _git_hash
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
    RESULT_VARIABLE _git_rc)

execute_process(
    COMMAND git status --porcelain
    WORKING_DIRECTORY "${SOURCE_DIR}"
    OUTPUT_VARIABLE _git_dirty
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET)

if(NOT _git_rc EQUAL 0 OR _git_hash STREQUAL "")
    set(_git_hash "00000000")
endif()

if(_git_dirty STREQUAL "")
    set(_git_dirty_val 0)
else()
    set(_git_dirty_val 1)
endif()

# ---------- 写出头文件 ----------
file(WRITE "${OUT}"
"/* 本文件由 CMake 自动生成，请勿手工修改。 */
#ifndef __BUILD_INFO_H__
#define __BUILD_INFO_H__

/* 编译日期，格式 YYYY-MM-DD */
#define FW_BUILD_DATE   \"${_date}\"
/* 编译时刻，格式 HH:MM:SS */
#define FW_BUILD_TIME   \"${_time}\"
/* 编译时间（Unix 时间戳，UTC） */
#define FW_BUILD_EPOCH   ${_epoch}UL
/* Git 短散列（数值形式，无仓库时为 0） */
#define FW_GIT_HASH      0x${_git_hash}UL
/* 构建时工作区是否有未提交改动：1 = 有，0 = 无 */
#define FW_GIT_DIRTY     ${_git_dirty_val}U

#endif /* __BUILD_INFO_H__ */
")

message(STATUS "build_info.h: ${_date} ${_time} (UTC ${_epoch}), git=${_git_hash}, dirty=${_git_dirty_val}")
