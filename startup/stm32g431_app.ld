/******************************************************************************
 * STM32G431CBU6 UDS Application 链接脚本
 *
 *   Flash 映射（与 Bootloader 同一分区）：
 *     Boot : 0x08000000 - 0x08005FFF   24 KB（第 0 ~ 11 页）
 *     App  : 0x08006000 - 0x0801FFFF  104 KB（第 12 ~ 63 页）  <-- 本镜像
 *
 *   ORIGIN 决定了 uds_iap 上位机在 0x34 RequestDownload 中发送的地址
 *   （它取十六进制文件的起始地址），因此必须与 Bootloader 的
 *   APP_START_ADDR 完全一致，否则会被回 NRC 0x31。
 *
 *   RAM（共 22 KB，连续）：
 *     SRAM1 : 0x20000000 - 0x20003FFF  16 KB
 *     SRAM2 : 0x20004000 - 0x200057FF   6 KB
 *     CCM   : 0x10000000 - 0x100027FF  10 KB（本工程未使用）
 *
 *   注意：STM32G431**CB**（48 脚）的 SRAM 是 22 KB，不是 RB 型号的 32 KB。
 *****************************************************************************/

MEMORY
{
    FLASH (rx) : ORIGIN = 0x08006000, LENGTH = 104K
    RAM   (rwx): ORIGIN = 0x20000000, LENGTH = 22K
}

/* 栈顶：SRAM1 + SRAM2 的末端（0x20000000 + 22 KB） */
_estack = 0x20005800;

ENTRY(Reset_Handler)

SECTIONS
{
    __stack_size = DEFINED(__stack_size) ? __stack_size : 4K;
    __heap_size  = DEFINED(__heap_size)  ? __heap_size  : 0;

    /* 中断向量表 —— 必须位于镜像最开头，Bootloader 才能从 APP_START_ADDR
       读取 MSP 与复位向量。STM32 CMSIS 启动文件使用的段名是 .isr_vector。 */
    .isr_vector :
    {
        . = ALIGN(4);
        KEEP(*(.isr_vector))
        . = ALIGN(4);
    } >FLASH

    /* 程序代码 */
    .text :
    {
        . = ALIGN(4);
        *(.text)
        *(.text*)
        *(.glue_7)
        *(.glue_7t)
        *(.eh_frame)
        KEEP (*(.init))
        KEEP (*(.fini))
        . = ALIGN(4);
        _etext = .;
    } >FLASH

    /* 只读常量 */
    .rodata :
    {
        . = ALIGN(4);
        *(.rodata)
        *(.rodata*)
        . = ALIGN(4);
    } >FLASH

    /* 异常处理表及其索引 */
    .ARM.extab :
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } >FLASH

    .ARM :
    {
        __exidx_start = .;
        *(.ARM.exidx*)
        __exidx_end = .;
    } >FLASH

    .ARM.attributes :
    {
        *(.ARM.attributes)
    } >FLASH

    /* C 运行时初始化数组 */
    .preinit_array :
    {
        PROVIDE_HIDDEN (__preinit_array_start = .);
        KEEP (*(.preinit_array*))
        PROVIDE_HIDDEN (__preinit_array_end = .);
    } >FLASH

    .init_array :
    {
        PROVIDE_HIDDEN (__init_array_start = .);
        KEEP (*(SORT(.init_array.*)))
        KEEP (*(.init_array*))
        PROVIDE_HIDDEN (__init_array_end = .);
    } >FLASH

    .fini_array :
    {
        PROVIDE_HIDDEN (__fini_array_start = .);
        KEEP (*(.fini_array*))
        KEEP (*(SORT(.fini_array.*)))
        PROVIDE_HIDDEN (__fini_array_end = .);
    } >FLASH

    /* ===================== 已初始化数据 =====================
     * 加载地址在 Flash，运行地址在 RAM。 */
    _sidata = LOADADDR(.data);
    .data :
    {
        . = ALIGN(4);
        _sdata = .;
        *(.data)
        *(.data*)
        . = ALIGN(4);
        _edata = .;
    } >RAM AT> FLASH

    /* 未初始化数据 */
    .bss :
    {
        . = ALIGN(4);
        _sbss = .;
        __bss_start__ = _sbss;
        *(.bss)
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
        __bss_end__ = _ebss;
    } >RAM

    /* 堆栈空间 */
    .heap_stack :
    {
        . = ALIGN(8);
        PROVIDE ( end  = _ebss );
        PROVIDE ( _end = _ebss );
        . = . + __heap_size;
        PROVIDE ( _heap_end = . );
        . = . + __stack_size;
        PROVIDE ( _sp = . );
        . = ALIGN(8);
    } >RAM

    /* ===================== 固件信息块（必须是最后一个 Flash 段） =====================
     * 本段必须排在**所有** Flash 段之后 —— 包括 .data 的加载映像
     * （.data 的运行地址在 RAM，但初值要放在 Flash，属于本镜像的一部分）。
     *
     * 若把 .fw_info 放在 .data 之前，.data 的初值会被排到 _image_end 之后：
     *   1) u32ImageSize 少算这段字节数，与实际镜像长度不符；
     *   2) 这段初值游离于镜像 CRC32 之外 —— 漏写 / 写错都不会被 CRC 检出。
     *
     * Bootloader 与上位机通过扫描 FW_INFO_MAGIC 定位并做 CRC 校验。
     */
    .fw_info :
    {
        . = ALIGN(4);
        KEEP(*(.fw_info))
        . = ALIGN(4);
    } >FLASH

    PROVIDE(_fw_info_start = ADDR(.fw_info));
    PROVIDE(_fw_info_end   = ADDR(.fw_info) + SIZEOF(.fw_info));
    PROVIDE(_image_end     = _fw_info_end);
}

/* 启动文件默认引用的库 */
GROUP(libgcc.a libc.a libm.a libnosys.a)
