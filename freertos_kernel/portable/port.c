/*
    FreeRTOS V9.0.0 - Copyright (C) 2016 Real Time Engineers Ltd.
    ... (лицензия остается без изменений) ...
*/

/* Standard includes. */
#include <stdlib.h>
#include <string.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"

/* AM335x INTC support */
#include "AM335x_INTC.h"

/* Some vendor specific files default configCLEAR_TICK_INTERRUPT() in
portmacro.h. */
#ifndef configCLEAR_TICK_INTERRUPT
	#define configCLEAR_TICK_INTERRUPT()
#endif

/* A critical section is exited when the critical section nesting count reaches
this value. */
#define portNO_CRITICAL_NESTING			( ( uint32_t ) 0 )

/* Tasks are not created with a floating point context, but can be given a
floating point context after they have been created.  A variable is stored as
part of the tasks context that holds portNO_FLOATING_POINT_CONTEXT if the task
does not have an FPU context, or any other value if the task does have an FPU
context. */
#define portNO_FLOATING_POINT_CONTEXT	( ( StackType_t ) 0 )

/* Constants required to setup the initial task context. */
#define portINITIAL_SPSR				( ( StackType_t ) 0x1f ) /* System mode, ARM mode, IRQ enabled FIQ enabled. */
#define portTHUMB_MODE_BIT				( ( StackType_t ) 0x20 )
#define portINTERRUPT_ENABLE_BIT		( 0x80UL )
#define portTHUMB_MODE_ADDRESS			( 0x01UL )

/* Masks all bits in the APSR other than the mode bits. */
#define portAPSR_MODE_BITS_MASK			( 0x1F )

/* The value of the mode bits in the APSR when the CPU is executing in user
mode. */
#define portAPSR_USER_MODE				( 0x10 )

/* The critical section macros only mask interrupts up to an application
determined priority level.  Sometimes it is necessary to turn interrupt off in
the CPU itself before modifying certain hardware registers. */
static inline void cpu_irq_disable(void)
{
	__asm volatile ( "CPSID i \n DSB \n ISB" );
}

static inline void cpu_irq_enable(void)
{
	__asm volatile ( "CPSIE i \n DSB \n ISB" );
}

/* Macro to unmask all interrupt priorities. */
static inline void portCLEAR_INTERRUPT_MASK(void)
{
	cpu_irq_disable();
	IntPriorityThresholdSet(portUNMASK_VALUE);
	__asm volatile ( "DSB \n ISB" );
	cpu_irq_enable();
}

/* Let the user override the pre-loading of the initial LR with the address of
prvTaskExitError() in case it messes up unwinding of the stack in the
debugger. */
#ifdef configTASK_RETURN_ADDRESS
	#define portTASK_RETURN_ADDRESS	configTASK_RETURN_ADDRESS
#else
	#define portTASK_RETURN_ADDRESS	prvTaskExitError
#endif

/* The space on the stack required to hold the FPU registers. */
#define portFPU_REGISTER_WORDS	( ( 8 * 2 ) + 1 ) // D0-D7 + FPSCR

/*-----------------------------------------------------------*/

/* Used in the asm file. */
__attribute__(( used )) const uint32_t ulMaxAPIPriorityMask = ( configMAX_API_CALL_INTERRUPT_PRIORITY );

/*-----------------------------------------------------------*/

/*
 * Starts the first task executing.
 */
extern void vPortRestoreTaskContext( void );

/*
 * Used to catch tasks that attempt to return from their implementing function.
 */
static void prvTaskExitError( void );

/*-----------------------------------------------------------*/

/* A variable is used to keep track of the critical section nesting. */
volatile uint32_t ulCriticalNesting = 0UL;

/* Saved as part of the task context. */
volatile uint32_t ulPortTaskHasFPUContext = pdFALSE;

/* Set to 1 to pend a context switch from an ISR. */
volatile uint32_t ulPortYieldRequired = pdFALSE;

/* Counts the interrupt nesting depth. */
volatile uint32_t ulPortInterruptNesting = 0UL;

/*-----------------------------------------------------------*/

StackType_t *pxPortInitialiseStack( StackType_t *pxTopOfStack, TaskFunction_t pxCode, void *pvParameters )
{
	/* Setup the initial stack of the task. */
	*pxTopOfStack = ( StackType_t ) NULL;
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) NULL;
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) NULL;
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) portINITIAL_SPSR;

	if( ( ( uint32_t ) pxCode & portTHUMB_MODE_ADDRESS ) != 0x00UL )
	{
		*pxTopOfStack |= portTHUMB_MODE_BIT;
	}

	pxTopOfStack--;

	*pxTopOfStack = ( StackType_t ) pxCode;
	pxTopOfStack--;

	*pxTopOfStack = ( StackType_t ) portTASK_RETURN_ADDRESS;	/* R14 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x12121212;	/* R12 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x11111111;	/* R11 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x10101010;	/* R10 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x09090909;	/* R9 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x08080808;	/* R8 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x07070707;	/* R7 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x06060606;	/* R6 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x05050505;	/* R5 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x04040404;	/* R4 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x03030303;	/* R3 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x02020202;	/* R2 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x01010101;	/* R1 */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) pvParameters; /* R0 */
	pxTopOfStack--;

	*pxTopOfStack = portNO_CRITICAL_NESTING;

	#if( configUSE_TASK_FPU_SUPPORT == 1 )
	{
		pxTopOfStack--;
		*pxTopOfStack = portNO_FLOATING_POINT_CONTEXT;
	}
	#elif( configUSE_TASK_FPU_SUPPORT == 2 )
	{
		pxTopOfStack -= portFPU_REGISTER_WORDS;
		memset( pxTopOfStack, 0x00, portFPU_REGISTER_WORDS * sizeof( StackType_t ) );
		pxTopOfStack--;
		*pxTopOfStack = pdTRUE;
		ulPortTaskHasFPUContext = pdTRUE;
	}
	#else
	{
		#error Invalid configUSE_TASK_FPU_SUPPORT setting
	}
	#endif

	return pxTopOfStack;
}
/*-----------------------------------------------------------*/

static void prvTaskExitError( void )
{
	configASSERT( ulPortInterruptNesting == ~0UL );
	portDISABLE_INTERRUPTS();
	for( ;; );
}
/*-----------------------------------------------------------*/

BaseType_t xPortStartScheduler( void )
{
uint32_t ulAPSR;

	__asm volatile ( "MRS %0, APSR" : "=r" ( ulAPSR ) );
	ulAPSR &= portAPSR_MODE_BITS_MASK;
	configASSERT( ulAPSR != portAPSR_USER_MODE );

	if( ulAPSR != portAPSR_USER_MODE )
	{
		/* Disable interrupts in CPU */
		cpu_irq_disable();

		/* Start the timer that generates the tick ISR. */
		portCLEAR_INTERRUPT_MASK();
		configSETUP_TICK_INTERRUPT();

		/* Start the first task executing. */
		vPortRestoreTaskContext();
	}

	( void ) prvTaskExitError;
	return 0;
}
/*-----------------------------------------------------------*/

void vPortEndScheduler( void )
{
	configASSERT( ulCriticalNesting == 1000UL );
}
/*-----------------------------------------------------------*/

void vPortEnterCritical( void )
{
	/* Mask interrupts up to the max syscall interrupt priority. */
	ulPortSetInterruptMask();

	/* Increment ulCriticalNesting */
	ulCriticalNesting++;

	/* Assert if called from interrupt context */
	if( ulCriticalNesting == 1 )
	{
		configASSERT( ulPortInterruptNesting == 0 );
	}
}
/*-----------------------------------------------------------*/

void vPortExitCritical( void )
{
	if( ulCriticalNesting > portNO_CRITICAL_NESTING )
	{
		ulCriticalNesting--;

		if( ulCriticalNesting == portNO_CRITICAL_NESTING )
		{
			portCLEAR_INTERRUPT_MASK();
		}
	}
}
/*-----------------------------------------------------------*/

void FreeRTOS_Tick_Handler( void )
{
	/* Increment the RTOS tick. */
	if( xTaskIncrementTick() != pdFALSE )
	{
		ulPortYieldRequired = pdTRUE;
	}
}

/*-----------------------------------------------------------*/

#if( configUSE_TASK_FPU_SUPPORT != 2 )

	void vPortTaskUsesFPU( void )
	{
	uint32_t ulInitialFPSCR = 0;

		ulPortTaskHasFPUContext = pdTRUE;
		__asm volatile ( "VMSR FPSCR, %0" :: "r" (ulInitialFPSCR) );
	}

#endif /* configUSE_TASK_FPU_SUPPORT */
/*-----------------------------------------------------------*/

void vPortClearInterruptMask( uint32_t ulNewMaskValue )
{
	if((ulNewMaskValue == pdFALSE) && (ulPortInterruptNesting == 0))
	{
		portCLEAR_INTERRUPT_MASK();
	}
}
/*-----------------------------------------------------------*/

uint32_t ulPortSetInterruptMask( void )
{
	uint32_t ulReturn;

	/* Save current interrupt state */
	uint32_t current_threshold = IntPriorityThresholdGet();

	if ( (uint32_t) configMAX_API_CALL_INTERRUPT_PRIORITY == current_threshold )
	{
		/* Interrupts were already masked. */
		ulReturn = pdTRUE;
	}
	else
	{
		ulReturn = pdFALSE;

		/* Mask interrupts */
		cpu_irq_disable();
		IntPriorityThresholdSet( (uint32_t)(configMAX_API_CALL_INTERRUPT_PRIORITY) );
		__asm volatile ( "DSB \n ISB" );
		cpu_irq_enable();
	}

	return ulReturn;
}
/*-----------------------------------------------------------*/

#if( configASSERT_DEFINED == 1 )

	void vPortValidateInterruptPriority( void )
	{
        configASSERT(
            IntCurrIrqPriorityGet() >=
            (uint32_t)(configMAX_API_CALL_INTERRUPT_PRIORITY) );
	}

#endif /* configASSERT_DEFINED */


extern void vApplicationIRQHandler(void);

/*
 * If vApplicationFPUSafeIRQHandler is not provided by user,
 * this weak implementation will be used.
 * It simply calls vApplicationIRQHandler without FPU save/restore.
 */
__attribute__((weak)) void vApplicationFPUSafeIRQHandler(void)
{
	/*
	 * In a production system, this should save FPU context,
	 * then call vApplicationIRQHandler, then restore FPU context.
	 * For now, just call the IRQ handler directly.
	 */
	vApplicationIRQHandler();
}

/*
 * Stack overflow hook - required when configCHECK_FOR_STACK_OVERFLOW > 0
 */
#if (configCHECK_FOR_STACK_OVERFLOW > 0)
__attribute__((weak)) void vApplicationStackOverflowHook(void *xTask, char *pcTaskName)
{
	/*
	 * Critical error - stack overflow detected.
	 * In production code, log error and reset.
	 */
	(void)xTask;
	(void)pcTaskName;

	__asm volatile("CPSID i");  /* Disable interrupts */
	while(1)
	{
		/* Infinite loop for debugger */
		__asm volatile("BKPT #0");
	}
}
#endif

/*
 * If you have vApplicationIRQHandler defined somewhere else,
 * remove this stub.
 */
#if 0
__attribute__((weak)) void vApplicationIRQHandler(void)
{
	/* Default IRQ handler - should be overridden by application */
	volatile uint32_t *sir_irq = (volatile uint32_t *)0x48200040;
	uint32_t irq_num = *sir_irq & 0x7F;
	(void)irq_num;
}
#endif
/*-----------------------------------------------------------*/