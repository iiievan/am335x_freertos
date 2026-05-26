#include "edma_test.h"
#include "regs/EDMA.hpp"
#include "hal/EDMA.hpp"
#include "hal/INTC.hpp"
#include "rtt/rtt_log.h"

#define TAG "edma_test"

void (*EDMA_app_callbacks[REGS::EDMA::AM335X_DMACH_MAX])(unsigned int status);
volatile char m_src_buff[EDMAAPP_MAX_BUFFER_SIZE] __attribute__((aligned(AM335X_CACHELINE_SIZE_MAX)));
volatile char m_dst_buff[EDMAAPP_MAX_BUFFER_SIZE] __attribute__((aligned(AM335X_CACHELINE_SIZE_MAX)));

volatile int irq_raised;
void EDMA_app_callback(const unsigned int status)
{
    using namespace REGS::EDMA;

    if(XFER_COMPLETE == status)
        irq_raised = EDMAAPP_IRQ_STATUS_XFER_COMP;       // Transfer completed successfully
    else
    if(CC_DMA_EVT_MISS == status)
        irq_raised = EDMAAPP_IRQ_STATUS_DMA_EVT_MISS;    // Transfer resulted in DMA event miss error.
    else
    if(CC_QDMA_EVT_MISS == status)
        irq_raised = EDMAAPP_IRQ_STATUS_QDMA_EVT_MISS;   // Transfer resulted in QDMA event miss error.
}

void edma_setup()
{
    HAL::EDMA::module_clock_config();
    HAL::EDMA::init(REGS::EDMA::EVENT_Q0);

    HAL::INTC::register_handler(REGS::INTC::EDMACOMPINT, edma_cc_compl_isr);
    HAL::INTC::priority_set(REGS::INTC::EDMACOMPINT,0,REGS::INTC::HOSTINT_ROUTE_IRQ);
    HAL::INTC::unmask_interrupt(REGS::INTC::EDMACOMPINT);

    HAL::INTC::register_handler(REGS::INTC::EDMAERRINT, edma_cc_err_isr);
    HAL::INTC::priority_set(REGS::INTC::EDMAERRINT,0,REGS::INTC::HOSTINT_ROUTE_IRQ);
    HAL::INTC::unmask_interrupt(REGS::INTC::EDMAERRINT);
}

void app_qdma_test()
{
    using namespace HAL::EDMA;

    volatile uint32_t index = 0u;
    volatile uint32_t count = 0u;
    REGS::EDMA::paRAM_entry_t param_set;
    unsigned char data = 0u;
    uint32_t ret_value = 0u;
    uint32_t is_test_passed = false;

    uint32_t a_count = EDMAAPP_MAX_ACOUNT;
    uint32_t b_count = EDMAAPP_MAX_BCOUNT;
    uint32_t c_count = EDMAAPP_MAX_CCOUNT;

    volatile uint32_t opt = 0u;
    uint32_t paramId = 32u;

    // Initalize source and destination buffers
    for(count = 0u; count < (a_count * b_count * c_count); count++)
    {
        m_src_buff[count] = data++;
        // No need to initialize the destination buffer
        // as it is being invalidated.
    }

    // Request DMA channel and TCC
    ret_value = request_channel(EDMAAPP_DMA_CH_TYPE,
                     EDMAAPP_DMA_CH_NUM,
                     EDMAAPP_DMA_TCC_NUM,
                      static_cast<REGS::EDMA::e_EVENT_QUEUE>(EDMAAPP_DMA_EVTQ));
    map_QDMA_ch_to_paRAM(EDMAAPP_DMA_CH_NUM, &paramId);
    set_QDMA_trig_word(EDMAAPP_DMA_CH_NUM, REGS::EDMA::PARAM_ENTRY_DST);

    // Registering Callback Function
    EDMA_app_callbacks[EDMAAPP_DMA_TCC_NUM] = &EDMA_app_callback;

    if(true == ret_value)
    {
        // Fill the PaRAM Set with transfer specific information
        param_set.SRC = (uint32_t)(m_src_buff);
        param_set.DST = (uint32_t)(m_dst_buff);
        param_set.ACNT = (unsigned short)a_count;
        param_set.BCNT = (unsigned short)b_count;
        param_set.CCNT = (unsigned short)c_count;

        // Setting up the SRC/DES Index
        param_set.SRCBIDX = (short)a_count;
        param_set.DSTBIDX = (short)a_count;

        if(REGS::EDMA::SYNC_A == EDMAAPP_DMA_SYNC_TYPE)
        {
            // A Sync Transfer Mode
            param_set.SRCCIDX = (short)a_count;
            param_set.DSTCIDX = (short)a_count;
        }
        else
        {
            // AB Sync Transfer Mode
            param_set.SRCCIDX = ((short)a_count * (short)b_count);
            param_set.DSTCIDX = ((short)a_count * (short)b_count);
        }

        // Configure the param_set with NULL link
        param_set.LINK = (unsigned short)0xFFFFu;

        param_set.BCNTRLD = (unsigned short)0u;
        param_set.OPT.reg = 0u;

        // Src & Dest are in INCR modes
        param_set.OPT.reg &= ~(REGS::EDMA::OPT_SAM | REGS::EDMA::OPT_DAM);

        // Program the TCC
        param_set.OPT.reg |= ((EDMAAPP_DMA_TCC_NUM << REGS::EDMA::OPT_TCC_SHIFT) & REGS::EDMA::OPT_TCC);

        // Enable Intermediate & Final transfer completion interrupt
        param_set.OPT.reg |= (1u << REGS::EDMA::OPT_ITCINTEN_SHIFT);
        param_set.OPT.reg |= (1u << REGS::EDMA::OPT_ITCINTEN_SHIFT);

        if(REGS::EDMA::SYNC_A == EDMAAPP_DMA_SYNC_TYPE)
            param_set.OPT.reg &= ~REGS::EDMA::OPT_SYNCDIM;
        else
            param_set.OPT.reg |= (1u << REGS::EDMA::OPT_SYNCDIM_SHIFT); // AB Sync Transfer Mode

        opt = param_set.OPT.reg;

        // Now, write the PaRAM Set.
        QDMA_set_paRAM(EDMAAPP_DMA_CH_NUM, &param_set);
    }

    ret_value = enable_transfer(EDMAAPP_DMA_CH_NUM, EDMAAPP_DMA_TRIG_MODE);


    // Since the transfer is going to happen in Manual mode of EDMA3
    // operation, we have to 'Enable the Transfer' multiple times.
    // Number of times depends upon the Mode (A/AB Sync)
    // and the different counts.
    uint32_t num_enabled = 0u;
    uint32_t dst_buf_addr = 0u;

    if(true == ret_value)
    {
        // Need to activate next param
        if(REGS::EDMA::SYNC_A == EDMAAPP_DMA_SYNC_TYPE)
            num_enabled = b_count * c_count;
        else
            num_enabled = c_count; // AB Sync Transfer Mode

        for(index = 0u; index < num_enabled; index++)
        {
            irq_raised = EDMAAPP_IRQ_STATUS_XFER_INPROG;

            if(index == (num_enabled - 1u))
            {
                // Since OPT.STATIC field should be SET for isolated QDMA
                // transfers or for the final transfer in a linked list of QDMA
                // transfers, do the needful for the last request.
                opt |= REGS::EDMA::OPT_STATIC;
                QDMA_set_paRAM_entry(paramId, REGS::EDMA::PARAM_ENTRY_OPT, opt);
            }

            opt |= REGS::EDMA::OPT_FWID_8BIT;
            QDMA_set_paRAM_entry(paramId, REGS::EDMA::PARAM_ENTRY_OPT, opt);

            // Now trigger the QDMA channel by writing to the Trigger
            // Word which is set as Destination Address.
            dst_buf_addr = QDMA_get_paRAM_entry(paramId, REGS::EDMA::PARAM_ENTRY_DST);
            QDMA_set_paRAM_entry(paramId, REGS::EDMA::PARAM_ENTRY_DST, dst_buf_addr);

            // Wait for the Completion ISR.
            while(EDMAAPP_IRQ_STATUS_XFER_INPROG == irq_raised)
            {
                // Wait for the Completion ISR on Master Channel.
                // You can insert your code here to do something
                // meaningful.
            }

            // Check the status of the completed transfer
            if(irq_raised < (int)EDMAAPP_IRQ_STATUS_XFER_INPROG)
            {
                RTT_LOG_E(TAG,"QDMA3Test: Event Miss Occured!!!"); // Some error occured, break from the FOR loop.
                clear_error_bits(EDMAAPP_DMA_CH_NUM, static_cast<REGS::EDMA::e_EVENT_QUEUE>(EDMAAPP_DMA_EVTQ)); // Clear the error bits first
                break;
            }
        }
    }

    // Match the Source and Destination Buffers.
    if(true == ret_value)
    {
        for(index = 0u; index < (a_count * b_count * c_count); index++)
        {
            if(m_src_buff[index] != m_dst_buff[index])
            {
                is_test_passed = false;
                RTT_LOG_E(TAG,"QDMA3Test: Data write-read matching FAILED.");
                RTT_LOG_E(TAG,"The mismatch happened at index : %d", (static_cast<int>(index) + 1u));
                break;
            }
        }

        if(index == (a_count * b_count * c_count))
        {
            is_test_passed = true;
            RTT_LOG_I(TAG,"QDMA3Test: Data write-read matching PASSED.\r\n");
        }

        // Free the previously allocated channel.
        ret_value = free_channel(EDMAAPP_DMA_CH_TYPE,
                                  EDMAAPP_DMA_CH_NUM,
                                  EDMAAPP_DMA_TRIG_MODE,
                                  EDMAAPP_DMA_TCC_NUM,
                                  static_cast<REGS::EDMA::e_EVENT_QUEUE>(EDMAAPP_DMA_EVTQ));

        // Unregister Callback Function
        EDMA_app_callbacks[EDMAAPP_DMA_TCC_NUM] = nullptr;

        if(true != ret_value)
            RTT_LOG_E(TAG,"QDMA3Test: EDMA3_DRV_freeChannel() FAILED.");
    }

    if(true == is_test_passed)
        RTT_LOG_E(TAG,"QDMA3Test PASSED.");
    else
        RTT_LOG_I(TAG,"QDMA3Test FAILED.");
}

/**
 * ISR for successful transfer completion.
 *
 * Note: This function first disables its own interrupt to make it non-entrant.
 */
void edma_cc_compl_isr(void * param)
{
    using namespace HAL::EDMA;
    volatile uint32_t pendingIrqs;
    volatile uint32_t isIntrPending = 0u;
    volatile uint32_t isHighIntrPending = 0u;
    uint32_t count = 0u;
    uint32_t index = 1u;
    constexpr uint32_t EDMA3_XFER_COMPLETE = 0u;

    (void)param;
    isIntrPending  = get_intr_status();
    isHighIntrPending = intr_status_high_get();

    if(isIntrPending | isHighIntrPending)
    {
        while ((count < REGS::EDMA::COMPL_HANDLER_RETRY_COUNT)&& (index != 0u))
        {
            index = 0u;

            if(isIntrPending)
                pendingIrqs = get_intr_status();
            else
                pendingIrqs = intr_status_high_get();

            while(pendingIrqs)
            {
                if(true == (pendingIrqs & 1u))
                {
                    // If the user has not given any Callback function
                    // while requesting the TCC, its TCC specific bit
                    // in the IPR register will NOT be cleared.
                    if(isIntrPending)
                    {
                        // Here write to ICR to clear the corresponding
                        // IPR bits
                        clr_intr(index);
                        (*EDMA_app_callbacks[index])(EDMA3_XFER_COMPLETE);
                    }
                    else
                    {

                        // Here write to ICR to clear the corresponding
                        // IPR bits
                        clr_intr(index + 32u);
                        (*EDMA_app_callbacks[index + 32u])(EDMA3_XFER_COMPLETE);
                    }

                }
                ++index;
                pendingIrqs >>= 1u;
            }
            count++;
        }
    }
}

/**
 * Interrupt ISR for Channel controller error.
 *
 * Note: This function first disables its own interrupt to make it non-entrant.
 */
void edma_cc_err_isr(void * param)
{
    using namespace HAL::EDMA;

    volatile uint32_t evtQueNum = 0u; /* Event Queue Num */
    volatile uint32_t isHighIntrPending = 0u;
    volatile uint32_t isIntrPending = 0u;
    volatile uint32_t count = 0u;
    volatile uint32_t pendingIrqs = 0u;
    volatile uint32_t index = 1u;
    constexpr uint32_t EDMA3CC_CCERR_TCCERR_SHIFT = 0x00000010u;

    (void)param;

    isIntrPending  = get_intr_status();
    isHighIntrPending = intr_status_high_get();

    if((isIntrPending | isHighIntrPending ) ||
       (QDMA_get_Err_intr_status() != 0u) ||
       (get_CC_Err_status() != 0u))
    {
        // Loop for ERR_HANDLER_RETRY_COUNT number of time,
        // breaks when no pending interrupt is found
        while ((count < REGS::EDMA::ERR_HANDLER_RETRY_COUNT) && (index != 0u))
        {
            index = 0u;

            if(isIntrPending)
            {
                pendingIrqs = get_Err_intr_status();
            }
            else
            {
                pendingIrqs = Err_intr_high_status_get();
            }

            while(pendingIrqs)
            {
                // Process all the pending interrupts
                if(true == (pendingIrqs & 1u))
                {
                    // Write to EMCR to clear the corresponding EMR bits.
                    // Clear any SER
                    if(isIntrPending)
                        clr_miss_evt(index);
                    else
                        clr_miss_evt(index + 32u);
                }
                ++index;
                pendingIrqs >>= 1u;
            }
            index = 0u;
            pendingIrqs = QDMA_get_Err_intr_status();

            while(pendingIrqs)
            {
                // Process all the pending interrupts
                if(true == (pendingIrqs & 1u))
                {
                    // Here write to QEMCR to clear the corresponding QEMR bits
                    // Clear any QSER
                    QDMA_clr_miss_evt(index);
                }
                ++index;
                pendingIrqs >>= 1u;
            }
            index = 0u;

            pendingIrqs = get_CC_Err_status();
            if(pendingIrqs != 0u)
            {
                // Process all the pending CC error interrupts.
                // Queue threshold error for different event queues.
                for(evtQueNum = 0u; evtQueNum < REGS::EDMA::AM335x_EVQUEUE_MAX; evtQueNum++)
                {
                    if((pendingIrqs & (1u << evtQueNum)) != 0u)
                    {
                        // Clear the error interrupt.
                        clr_CC_Err(1u << evtQueNum);
                    }
                }

                // Transfer completion code error.
                if((pendingIrqs & (1u << EDMA3CC_CCERR_TCCERR_SHIFT)) != 0u)
                {
                    clr_CC_Err(1u << EDMA3CC_CCERR_TCCERR_SHIFT);
                }
                ++index;
            }
            count++;
        }
    }
}