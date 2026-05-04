@******************************************************************************
@ startup.s - Startup code for GCC
@******************************************************************************

.global Entry
.global __libc_init_array
.global main

.global __svc_stack_end
.global __abt_stack_end
.global __und_stack_end
.global __fiq_stack_end
.global __irq_stack_end
.global __stack_end

.global __bss_start
.global __bss_end

@ FreeRTOS переменные (должны быть определены в C коде)
.extern ulPortInterruptNesting
.extern ulCriticalNesting
.extern ulPortTaskHasFPUContext
.extern ulPortYieldRequired

.equ MODE_MSK, 0x1F
.equ MODE_USR, 0x10
.equ MODE_FIQ, 0x11
.equ MODE_IRQ, 0x12
.equ MODE_SVC, 0x13
.equ MODE_ABT, 0x17
.equ MODE_UND, 0x1B
.equ MODE_SYS, 0x1F
.equ I_BIT, 0x80
.equ F_BIT, 0x40

.section .text.startup, "ax"
.code 32

Entry:
    @ Запрещаем все прерывания
    MRS r0, cpsr
    ORR r0, r0, #(I_BIT | F_BIT)
    MSR cpsr_c, r0

    @ === Инициализация стеков для всех режимов ===
    @ Порядок важен: от меньшего адреса к большему

    @ SVC режим
    MRS r0, cpsr
    BIC r0, r0, #MODE_MSK
    ORR r0, r0, #(MODE_SVC | I_BIT | F_BIT)
    MSR cpsr_c, r0
    LDR sp, =__svc_stack_end

    @ ABT режим
    MRS r0, cpsr
    BIC r0, r0, #MODE_MSK
    ORR r0, r0, #(MODE_ABT | I_BIT | F_BIT)
    MSR cpsr_c, r0
    LDR sp, =__abt_stack_end

    @ UND режим
    MRS r0, cpsr
    BIC r0, r0, #MODE_MSK
    ORR r0, r0, #(MODE_UND | I_BIT | F_BIT)
    MSR cpsr_c, r0
    LDR sp, =__und_stack_end

    @ FIQ режим
    MRS r0, cpsr
    BIC r0, r0, #MODE_MSK
    ORR r0, r0, #(MODE_FIQ | I_BIT | F_BIT)
    MSR cpsr_c, r0
    LDR sp, =__fiq_stack_end

    @ IRQ режим
    MRS r0, cpsr
    BIC r0, r0, #MODE_MSK
    ORR r0, r0, #(MODE_IRQ | I_BIT | F_BIT)
    MSR cpsr_c, r0
    LDR sp, =__irq_stack_end

    @ System/User режим (используется для main и задач FreeRTOS)
    MRS r0, cpsr
    BIC r0, r0, #MODE_MSK
    ORR r0, r0, #(MODE_SYS | I_BIT | F_BIT)
    MSR cpsr_c, r0
    LDR sp, =__stack_end

    @ === Инициализация кэшей ===

    @ Invalidate Branch Prediction (BTAC)
    MOV r0, #0
    MCR p15, 0, r0, c7, c5, #6
    ISB

    @ Enable Branch Prediction
    MRC p15, 0, r0, c1, c0, #0
    ORR r0, r0, #0x00000800
    MCR p15, 0, r0, c1, c0, #0

    @ Invalidate instruction cache
    MOV r0, #0
    MCR p15, 0, r0, c7, c5, #0

    @ Invalidate Data Cache(?)
    @ BL BSP_DCacheInvalidateAll

    @ Set domain access (полный доступ для всех доменов)
    LDR r0, =0x55555555
    MCR p15, 0, r0, c3, c0, #0

    @ === Enable VFP/NEON ===
    MRC p15, 0, r1, c1, c0, #2       @ Read Coprocessor Access Control Register
    ORR r1, r1, #(0xF << 20)         @ Enable full access for CP10 and CP11
    MCR p15, 0, r1, c1, c0, #2       @ Write Coprocessor Access Control Register
    MOV r1, #0
    MCR p15, 0, r1, c7, c5, #4       @ Flush prefetch buffer
    MOV r0, #0x40000000              @ Enable VFP/NEON
    VMSR FPEXC, r0

    @ === Инициализация переменных FreeRTOS ===
    LDR r0, =ulPortInterruptNesting
    MOV r1, #0
    STR r1, [r0]

    LDR r0, =ulCriticalNesting
    MOV r1, #0
    STR r1, [r0]

    LDR r0, =ulPortTaskHasFPUContext
    MOV r1, #0
    STR r1, [r0]

    LDR r0, =ulPortYieldRequired
    MOV r1, #0
    STR r1, [r0]

    @ === Очистка BSS ===
    LDR r0, =__bss_start
    LDR r1, =__bss_end
    MOV r2, #0
clear_bss:
    STR r2, [r0], #4
    CMP r0, r1
    BLO clear_bss

    @ === Вызов C++ конструкторов ===
    LDR r0, =__libc_init_array
    BLX r0

    @ === Переход в main ===
    BL main

    @ Бесконечный цикл если main вернулся
loop:
    WFI
    B loop

@******************************************************************************
@ BSP_DCacheInvalidateAll - Инвалидация всего data cache
@ Важно: не вызывать после инициализации MMU для кэшируемых регионов!
@******************************************************************************
BSP_DCacheInvalidateAll:
    MOV r0, #0x1FF
dcache_loop1:
    MOV r1, #0x00000003
dcache_loop2:
    MOV r2, r1, LSL #30
    ADD r2, r2, r0, LSL #5
    MCR p15, 0, r2, c7, c6, #2
    SUBS r1, r1, #1
    BGE dcache_loop2
    SUBS r0, r0, #1
    BGE dcache_loop1
    DSB
    BX lr

.end