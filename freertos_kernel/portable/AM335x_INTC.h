
#ifndef __AM335X_INTC_H
#define __AM335X_INTC_H

/*
 * AM335x_INTC.h - INTC interface for FreeRTOS on AM335x
 * Replaces GIC-specific functions with INTC equivalents
 */


#include <stdint.h>

/* AM335x INTC Register Offsets */
#define AM335X_INTC_BASE                0x48200000
#define AM335X_INTC_SIZE                0x00002000

/* INTC Registers */
#define INTC_SYSCONFIG_OFFSET           0x0010
#define INTC_IDLE_OFFSET                0x0014
#define INTC_THRESHOLD_OFFSET           0x0068
#define INTC_IRQ_PRIORITY_OFFSET        0x0070
#define INTC_CONTROL_OFFSET             0x0048
#define INTC_SIR_IRQ_OFFSET             0x0040

/* INTC Control bits */
#define INTC_NEWIRQAGR                  (1 << 0)
#define INTC_NEWFIQAGR                  (1 << 1)

/* Priority masking for AM335x INTC */
/* INTC doesn't have hardware priorities, so we implement software masking */
#define INTC_PRIORITY_THRESHOLD_MIN     0x00  /* All interrupts enabled */
#define INTC_PRIORITY_THRESHOLD_MAX     0xFF  /* All interrupts masked */

/*
 * INTC Threshold Register:
 * Bits 0-7: Priority threshold
 * Bits 8-15: Reserved
 *
 * Writing a value N masks all interrupts with priority <= N
 * On AM335x, priority is determined by interrupt number
 * Lower number = higher priority (inverted from typical priority scheme)
 */

/* Static inline functions for INTC operations */
static inline uint32_t IntPriorityThresholdGet(void)
{
    volatile uint32_t *threshold_reg = (volatile uint32_t *)(AM335X_INTC_BASE + INTC_THRESHOLD_OFFSET);
    return *threshold_reg & 0xFF;
}

static inline void IntPriorityThresholdSet(uint32_t threshold)
{
    volatile uint32_t *threshold_reg = (volatile uint32_t *)(AM335X_INTC_BASE + INTC_THRESHOLD_OFFSET);
    *threshold_reg = (threshold & 0xFF);
}

static inline uint32_t IntCurrIrqPriorityGet(void)
{
    volatile uint32_t *irq_priority_reg = (volatile uint32_t *)(AM335X_INTC_BASE + INTC_IRQ_PRIORITY_OFFSET);
    uint32_t priority = *irq_priority_reg;

    /* Extract priority from IRQ priority register */
    /* Bits 0-6: Active IRQ number */
    /* Bits 7-9: Priority? Check TRM for exact format */
    /* For AM335x, return the IRQ number as priority indicator */
    return (priority & 0x7F);
}

/*
 * Acknowledge IRQ and enable new IRQ generation
 */
static inline void IntAcknowledgeIRQ(void)
{
    volatile uint32_t *control_reg = (volatile uint32_t *)(AM335X_INTC_BASE + INTC_CONTROL_OFFSET);
    *control_reg = INTC_NEWIRQAGR;
}

#endif //__AM335X_INTC_H