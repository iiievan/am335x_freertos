@******************************************************************************
@ exceptionhandlers.s - Exception handlers for FreeRTOS and GCC 
@******************************************************************************

	 @ Variables and functions. 
	.extern ulMaxAPIPriorityMask
	.extern _freertos_vector_table
	.extern pxCurrentTCB
	.extern vTaskSwitchContext
	.extern vApplicationIRQHandler
	.extern ulPortInterruptNesting
	.extern ulPortTaskHasFPUContext

	.global FreeRTOS_IRQ_Handler
	.global FreeRTOS_SWI_Handler
	.global vPortRestoreTaskContext

	@************************** Global symbols for TI ISR ************************************

	.global IRQHandler
	.global FIQHandler
	.global PrefetchAbortHandler
    .global DataAbortHandler
	.global SVC_Handler
	.global UndefInstHandler
	.global CPUAbortHandler

.section .text, "ax"
.code 32

	.set SYS_MODE,	0x1f
	.set SVC_MODE,	0x13
	.set IRQ_MODE,	0x12

	.equ MASK_SVC_NUM,      0xFF000000  @ Маска для выделения номера SVC
    .equ MODE_SYS,          0x1F        @ Режим System
    .equ MODE_IRQ,          0x12        @ Режим IRQ
    .equ I_BIT,             0x80        @ Бит маскирования IRQ в CPSR
    .equ F_BIT,             0x40        @ Бит маскирования FIQ в CPSR

	.equ SOC_AINTC_REGS,    0x48200000  @ Базовый адрес AINTC
    .equ INTC_SIR_IRQ,      0x0040      @ Смещение регистра SIR_IRQ
    .equ INTC_SIR_FIQ,      0x0044      @ Смещение регистра SIR_FIQ
    .equ INTC_CONTROL,      0x0048      @ Смещение регистра CONTROL
    .equ INTC_THRESHOLD,    0x0068      @ Смещение регистра THRESHOLD
    .equ INTC_IRQ_PRIORITY, 0x0070      @ Смещение регистра IRQ_PRIORITY

    .equ ADDR_SIR_IRQ,      (SOC_AINTC_REGS + INTC_SIR_IRQ)
    .equ ADDR_SIR_FIQ,      (SOC_AINTC_REGS + INTC_SIR_FIQ)
    .equ ADDR_CONTROL,      (SOC_AINTC_REGS + INTC_CONTROL)
    .equ ADDR_THRESHOLD,    (SOC_AINTC_REGS + INTC_THRESHOLD)
    .equ ADDR_IRQ_PRIORITY, (SOC_AINTC_REGS + INTC_IRQ_PRIORITY)

    .equ MASK_ACTIVE_IRQ,   0x0000007F  @ Маска номера активного IRQ
    .equ NEWIRQAGR,         0x00000001  @ Бит разрешения новых IRQ
    .equ NEWFIQAGR,         0x00000002  @ Бит разрешения новых FIQ

.macro portSAVE_CONTEXT

	@ Save the LR and SPSR onto the system mode stack before switching to
	@ system mode to save the remaining system mode registers.
	SRSDB	sp!, #SYS_MODE
	CPS		#SYS_MODE
	PUSH	{R0-R12, R14}

	@ Push the critical nesting count.
	LDR		R2, ulCriticalNestingConst
	LDR		R1, [R2]
	PUSH	{R1}

	@ Does the task have a floating point context that needs saving?  If
	@ ulPortTaskHasFPUContext is 0 then no.
	LDR		R2, ulPortTaskHasFPUContextConst
	LDR		R3, [R2]
	CMP		R3, #0

	@ Save the floating point context, if any. @
	VMRSNE  R1,  FPSCR
	VPUSHNE {D0-D7}
	@ VPUSHNE	{D16-D31}
	PUSHNE	{R1}

	@ Save ulPortTaskHasFPUContext itself. @
	PUSH	{R3}

	@ Save the stack pointer in the TCB. @
	LDR		R0, pxCurrentTCBConst
	LDR		R1, [R0]
	STR		SP, [R1]

	.endm

; /**********************************************************************/

.macro portRESTORE_CONTEXT

	@ Set the SP to point to the stack of the task being restored. @
	LDR		R0, pxCurrentTCBConst
	LDR		R1, [R0]
	LDR		SP, [R1]

	@ Is there a floating point context to restore?  If the restored
	@ ulPortTaskHasFPUContext is zero then no. @
	LDR		R0, ulPortTaskHasFPUContextConst
	POP		{R1}
	STR		R1, [R0]
	CMP		R1, #0

	@ Restore the floating point context, if any. @
	POPNE 	{R0}
	@ VPOPNE	{D16-D31}
	VPOPNE	{D0-D7}
	VMSRNE  FPSCR, R0

	@ Restore the critical section nesting depth. @
	LDR		R0, ulCriticalNestingConst
	POP		{R1}
	STR		R1, [R0]

	@ Ensure the priority mask is correct for the critical nesting depth. @
	LDR		R0, =ADDR_THRESHOLD
	LDR		R2, [R0]
	CMP		R1, #0
	MOVEQ	R4, #255
	LDRNE	R4, ulMaxAPIPriorityMaskConst
	LDRNE	R4, [R4]
	STR		R4, [R0]

	@ Restore all system mode registers other than the SP (which is already
	@ being used). @
	POP		{R0-R12, R14}

	@ Return to the task code, loading CPSR on the way. @
	RFEIA	sp!

	.endm

/******************************************************************************
 * SVC handler is used to start the scheduler.
 *****************************************************************************/
.align 4
.type FreeRTOS_SWI_Handler, %function
FreeRTOS_SWI_Handler:
	@ Save the context of the current task and select a new task to run. @
	portSAVE_CONTEXT
	LDR R0, vTaskSwitchContextConst
	BLX	R0
	portRESTORE_CONTEXT


/******************************************************************************
 * vPortRestoreTaskContext is used to start the scheduler.
 *****************************************************************************/
.type vPortRestoreTaskContext, %function
vPortRestoreTaskContext:
	@ Switch to system mode. @
	CPS		#SYS_MODE
	portRESTORE_CONTEXT

.align 4
.type FreeRTOS_IRQ_Handler, %function
FreeRTOS_IRQ_Handler:
	@ Return to the interrupted instruction. @
	SUB		lr, lr, #4

	@ Push the return address and SPSR. @
	PUSH	{lr}
	MRS		lr, SPSR
	PUSH	{lr}

	@ Change to supervisor mode to allow reentry. @
	CPS		#SVC_MODE

	@ Push used registers. @
	PUSH	{r0-r8, r12}

	@ Increment nesting count.  r3 holds the address of ulPortInterruptNesting
	@ for future use.  r1 holds the original ulPortInterruptNesting value for
	@ future use. @
	LDR		r3, ulPortInterruptNestingConst
	LDR		r1, [r3]
	ADD		r4, r1, #1
	STR		r4, [r3]

	@ Ensure bit 2 of the stack pointer is clear.  r2 holds the bit 2 value for
	@ future use.  _RB_ Does this ever actually need to be done provided the start
	@ of the stack is 8-byte aligned? @
	MOV		r2, sp
	AND		r2, r2, #4
	SUB		sp, sp, r2

	@ Call the interrupt handler.  r4 pushed to maintain alignment. @
	PUSH	{r0-r8, lr}
	LDR		r1, vApplicationIRQHandlerConst
	BLX		r1
	POP		{r0-r8, lr}
	ADD		sp, sp, r2

	CPSID	i
	DSB
	ISB

	MOV      r7, #NEWIRQAGR           @ Acknowledge the IRQ & Enable new IRQ Generation
	LDR      r6, =ADDR_CONTROL
	STR      r7, [r6]
	DSB
	ISB

	@ Restore the old nesting count. @
	STR		r1, [r3]

	@ A context switch is never performed if the nesting count is not 0. @
	CMP		r1, #0
	BNE		exit_without_switch

	@ Did the interrupt request a context switch?  r1 holds the address of
	@ ulPortYieldRequired and r0 the value of ulPortYieldRequired for future
	@ use. @
	LDR		r1, =ulPortYieldRequired
	LDR		r0, [r1]
	CMP		r0, #0
	BNE		switch_before_exit

exit_without_switch:
	@ No context switch.  Restore used registers, LR_irq and SPSR before
	@ returning. @
	POP		{r0-r8, r12}
	CPS		#IRQ_MODE
	POP		{LR}
	MSR		SPSR_cxsf, LR
	POP		{LR}
	MOVS	PC, LR

switch_before_exit:
	@ A context swtich is to be performed.  Clear the context switch pending
	@ flag. @
	MOV		r0, #0
	STR		r0, [r1]

	@ Restore used registers, LR-irq and SPSR before saving the context
	@ to the task stack. @
	POP		{r0-r8, r12}
	CPS		#IRQ_MODE
	POP		{LR}
	MSR		SPSR_cxsf, LR
	POP		{LR}
	portSAVE_CONTEXT

	@ Call the function that selects the new task to execute.
	@ vTaskSwitchContext() if vTaskSwitchContext() uses LDRD or STRD
	@ instructions, or 8 byte aligned stack allocated data.  LR does not need
	@ saving as a new LR will be loaded by portRESTORE_CONTEXT anyway. @
	LDR		R0, vTaskSwitchContextConst
	BLX		R0

	@ Restore the context of, and branch to, the task selected to execute
	@next. @
	portRESTORE_CONTEXT



@*****************************************************************************
@ If the application provides an implementation of vApplicationIRQHandler(),
@ then it will get called directly without saving the FPU registers on
@ interrupt entry, and this weak implementation of
@ vApplicationIRQHandler() will not get called.
@
@ If the application provides its own implementation of
@ vApplicationFPUSafeIRQHandler() then this implementation of
@ vApplicationIRQHandler() will be called, save the FPU registers, and then
@ call vApplicationFPUSafeIRQHandler().
@
@ Therefore, if the application writer wants FPU registers to be saved on
@ interrupt entry their IRQ handler must be called
@ vApplicationFPUSafeIRQHandler(), and if the application writer does not want
@ FPU registers to be saved on interrupt entry their IRQ handler must be
@ called vApplicationIRQHandler().
@****************************************************************************/

.align 4
.weak vApplicationIRQHandler
.type vApplicationIRQHandler, %function
vApplicationIRQHandler:
	PUSH	{LR}
	VMRS	R1,  FPSCR
	VPUSH	{D0-D7}
	PUSH	{R1}

	LDR		r1, vApplicationFPUSafeIRQHandlerConst
	BLX		r1

	POP		{R0}
	VPOP	{D0-D7}
	VMSR	FPSCR, R0

	POP {PC}

@******************************************************************************
@*                  Function Definition of FIQ Handler
@******************************************************************************
@
@ FIQ is not supported for this SoC.
@
FIQHandler:
    subs pc, lr, #4

.type PrefetchAbortHandler, %function
.align 5                            @ Выравнивание по строке кэша (32 байта)
PrefetchAbortHandler:
    SUB     lr, lr, #4              @ Корректируем LR для Prefetch Abort (адрес упавшей инструкции)

    @ Читаем системные регистры CP15 (Instruction faults)
    MRS     r0, spsr
    MRC     p15, 0, r1, c5, c0, 1   @ IFSR
    MRC     p15, 0, r2, c6, c0, 2   @ IFAR

    @ Сохраняем все на стек в единую структуру FaultContext
    STMFD   sp!, {r0-r2}            @ Сохраняем IFAR, IFSR, SPSR
    STMFD   sp!, {r0-r12, lr}       @ Сохраняем R0-R12 и fault_pc

    MOV     r0, sp                  @ arg0 (R0) = const FaultContext* ctx

    @ Выравниваем стек по 8 байт для безопасного вызова C++ / RTT
    MOV     r4, sp
    AND     r5, sp, #4
    SUB     sp, sp, r5

    BL      c_prefetch_abort_handler

    MOV     sp, r4                  @ Восстанавливаем SP
    ADD     sp, sp, #64             @ Снимаем со стека FaultContext (16 * 4 байта)
    MOVS    pc, lr                  @ Возврат из исключения


.type DataAbortHandler, %function
.align 5                            @ Выравнивание по строке кэша (32 байта)
DataAbortHandler:
    SUB     lr, lr, #8              @ Корректируем LR (fault_pc)

    @ Читаем системные регистры CP15
    MRS     r0, spsr
    MRC     p15, 0, r1, c5, c0, 0   @ DFSR
    MRC     p15, 0, r2, c6, c0, 0   @ DFAR

    @ Сохраняем все на стек в виде структуры FaultContext
    STMFD   sp!, {r0-r2}            @ Сохраняем DFAR, DFSR, SPSR
    STMFD   sp!, {r0-r12, lr}       @ Сохраняем R0-R12 и fault_pc

    MOV     r0, sp                  @ arg0 (R0) = указатель на FaultContext (наш SP)

    @ Выравниваем стек по 8 байт для безопасного вызова C++ / RTT
    MOV     r4, sp
    AND     r5, sp, #4
    SUB     sp, sp, r5

    BL      c_data_abort_handler    @ Вызов C-обработчика

    MOV     sp, r4                  @ Восстанавливаем SP
    ADD     sp, sp, #64             @ Снимаем со стека всю структуру (16 регистров * 4)
    MOVS    pc, lr                  @ Возврат из исключения


@******************************************************************************
@ Undefined Instruction Handler
@******************************************************************************
UndefInstHandler:
    @ В FreeRTOS это фатальная ошибка
    STMFD   sp!, {r0-r1, lr}
    
    @ Здесь можно добавить отладочный вывод
    MRS     r0, cpsr
    MRS     r1, spsr
    
    @ Вызываем обработчик или входим в бесконечный цикл
    bkpt #0
    b infinite_loop

infinite_loop:
    WFI
    B infinite_loop

pxCurrentTCBConst: 					.word pxCurrentTCB
ulCriticalNestingConst: 			.word ulCriticalNesting
ulPortTaskHasFPUContextConst: 		.word ulPortTaskHasFPUContext
ulMaxAPIPriorityMaskConst: 			.word ulMaxAPIPriorityMask
vTaskSwitchContextConst: 			.word vTaskSwitchContext
vApplicationIRQHandlerConst: 		.word vApplicationIRQHandler
ulPortInterruptNestingConst: 		.word ulPortInterruptNesting
.weak vApplicationFPUSafeIRQHandler
vApplicationFPUSafeIRQHandlerConst: .word vApplicationFPUSafeIRQHandler

@*****************************************************************************
@
@ End of the file
@
.end





