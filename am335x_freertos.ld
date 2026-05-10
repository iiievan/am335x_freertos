/* am335x_app.ld - GCC Linker Script for DDR-based application */

MEMORY
{
    /* DDR память - основная область */
    DDR (rwx) : ORIGIN = 0x80000000, LENGTH = 0x10000000  /* 256 Mb */

    /* RTT буфер в отдельной некешируемой области */
    /* Можно разместить в верхней части OCMC RAM или выделить область в DDR */
    RTT_RAM (rw) : ORIGIN = 0x40300000, LENGTH = 0x10000  /* 64KB OCMC RAM for RTT - noncashable */
}

OUTPUT_FORMAT("elf32-littlearm")
OUTPUT_ARCH(arm)
ENTRY(Entry)

CSTACK_SIZE     = 0x8000;   /* 32KB для System/User режима */
SVC_STACK_SIZE  = 0x1000;   /* 4KB для Supervisor */
IRQ_STACK_SIZE  = 0x2000;   /* 8KB для IRQ (нужно больше для вложенных) */
FIQ_STACK_SIZE  = 0x8;      /* Минимальный (не используется) */
UND_STACK_SIZE  = 0x100;    /* 256 байт для Undefined */
ABT_STACK_SIZE  = 0x100;    /* 256 байт для Abort */
HEAP_SIZE       = 0x80000;  /* 512KB для кучи FreeRTOS */

__DDR_START = ORIGIN(DDR);
__DDR_END = ORIGIN(DDR) + LENGTH(DDR);

SECTIONS
{
    /* ============================================================ */
    /* Код в DDR */
    /* ============================================================ */
    . = __DDR_START;

    .text.startup : ALIGN(4) {
        KEEP(*(.text.startup))
    } > DDR

    .text : ALIGN(4) {
        *(.text)
        *(.text.*)
        *(.rodata)
        *(.rodata.*)
    } > DDR

    /* Таблицы для таблиц функций (как в TI-RTOS) */
    .services : ALIGN(4) {
        __service_start = .;
        KEEP(*(.init_fptr*))
        __services_end = .;
    } > DDR

    /* ARM exception unwinding & index */
    .ARM.extab : ALIGN(4) {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > DDR

    .ARM.exidx : ALIGN(4) {
        __exidx_start = .;
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
        __exidx_end = .;
    } > DDR

    /* Инициализированные данные */
    .data : ALIGN(4) {
        __data_start = .;
        *(.data)
        *(.data.*)
        . = ALIGN(4);
        __data_end = .;
    } > DDR

/* BSS (неинициализированные данные) */
    .bss : ALIGN(4) {
        __bss_start = .;
        *(.bss)
        *(.bss.*)
        *(COMMON)
        __bss_end = .;
    } > DDR

    /* C++ конструкторы/деструкторы */
    .preinit_array : ALIGN(4) {
        PROVIDE_HIDDEN(__preinit_array_start = .);
        KEEP(*(.preinit_array*))
        PROVIDE_HIDDEN(__preinit_array_end = .);
    } > DDR

    .init_array : ALIGN(4) {
        PROVIDE_HIDDEN(__init_array_start = .);
        KEEP(*(SORT(.init_array.*)))
        KEEP(*(.init_array*))
        PROVIDE_HIDDEN(__init_array_end = .);
    } > DDR

    .fini_array : ALIGN(4) {
        PROVIDE_HIDDEN(__fini_array_start = .);
        KEEP(*(.fini_array*))
        KEEP(*(SORT(.fini_array.*)))
        PROVIDE_HIDDEN(__fini_array_end = .);
    } > DDR

    /* ============================================================ */
    /* Stacks */
    /* ============================================================ */
    .stack_svc (NOLOAD) : ALIGN(8) {
        __svc_stack_start = .;
        . += SVC_STACK_SIZE;
        . = ALIGN(8);
        __svc_stack_end = .;
    } > DDR

    .stack_abt (NOLOAD) : ALIGN(8) {
        __abt_stack_start = .;
        . += ABT_STACK_SIZE;
        . = ALIGN(8);
        __abt_stack_end = .;
    } > DDR

    .stack_und (NOLOAD) : ALIGN(8) {
        __und_stack_start = .;
        . += UND_STACK_SIZE;
        . = ALIGN(8);
        __und_stack_end = .;
    } > DDR

    .stack_fiq (NOLOAD) : ALIGN(8) {
        __fiq_stack_start = .;
        . += FIQ_STACK_SIZE;
        . = ALIGN(8);
        __fiq_stack_end = .;
    } > DDR

    .stack_irq (NOLOAD) : ALIGN(8) {
        __irq_stack_start = .;
        . += IRQ_STACK_SIZE;
        . = ALIGN(8);
        __irq_stack_end = .;
    } > DDR

    .stack_main (NOLOAD) : ALIGN(8) {
        __stack_start = .;
        . += CSTACK_SIZE;
        . = ALIGN(8);
        __stack_end = .;
    } > DDR

    /* ============================================================ */
    /* HEAP (heap_4 или heap_5) */
    /* ============================================================ */
    .heap (NOLOAD) : ALIGN(8) {
        __heap_start = .;
        . += HEAP_SIZE;
        . = ALIGN(8);
        __heap_end = .;
    } > DDR

    __end_of_memory = .;

    /* Символы для newlib/gcc */
    PROVIDE(end = __heap_end);
    PROVIDE(_end = __heap_end);
    PROVIDE(__end = __heap_end);

    /* ============================================================ */
    /* RTT буфер в некешируемой памяти */
    /* ============================================================ */
    .rtt (NOLOAD) : ALIGN(64) {
        __rtt_start = .;
        KEEP(*(.rtt))
        KEEP(*(.rtt.*))
        . = ALIGN(64);
        __rtt_end = .;
    } > RTT_RAM


    /* Экспорт символов для TI-RTOS совместимости */
    __INIT_FPTR_START = __service_start;
    __INIT_FPTR_END = __services_end;


    /* Проверка переполнения DDR */
    ASSERT(__end_of_memory <= __DDR_END, "Error: DDR overflow!")
}