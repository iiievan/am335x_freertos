#include "edma_test.h"
#include "hal/EDMA/EDMA.hpp"
#include "hal/EDMA/DmaChannel.hpp"
#include "hal/EDMA/QdmaChannel.hpp"
#include "hal/EDMA/ParamBuilder.hpp"
#include "hal/EDMA/EDMA_diagnostics.hpp"
#include "hal/INTC.hpp"
#include "rtt/rtt_log.h"
#include "startup/cp15.h"

#include <atomic>

#define TAG "EDMA_TEST"

namespace
{
    static HAL::EDMA::EDMA_DiagnosticSnapshot snapshot;

    constexpr size_t BUFFER_SIZE = 64;
    alignas(64) uint8_t src_buf[BUFFER_SIZE];
    alignas(64) uint8_t dst_buf[BUFFER_SIZE];

    std::atomic<bool> transfer_complete{false};

    void on_edma_complete(void* context)
    {
        auto* flag = static_cast<std::atomic<bool>*>(context);
        if (flag) {
            flag->store(true, std::memory_order_release);
        }
    }

    void on_edma_error(HAL::EDMA::ErrorType err, void* context)
    {
        (void)err;
        auto* flag = static_cast<std::atomic<bool>*>(context);
        if (flag) {
            flag->store(true, std::memory_order_release);
        }
    }

    void prepare_buffers()
    {
        for (size_t i = 0; i < BUFFER_SIZE; ++i) {
            dst_buf[i] = 0x00;
        }
        cp15_D_cache_clean_buff(reinterpret_cast<unsigned int>(src_buf), BUFFER_SIZE);
        cp15_D_cache_flush_buff(reinterpret_cast<unsigned int>(dst_buf), BUFFER_SIZE);
        cp15_DSB_barrier();
    }

    bool verify_buffers(const char* who, const uint8_t ch)
    {
        cp15_DSB_barrier();
        cp15_D_cache_flush_buff(reinterpret_cast<unsigned int>(dst_buf), BUFFER_SIZE);

        for (size_t i = 0; i < BUFFER_SIZE; ++i) {
            if (src_buf[i] != dst_buf[i]) {
                RTT_LOG_E(TAG, "%s ch%u mismatch @%u: src=0x%02X dst=0x%02X",
                          who, ch, static_cast<unsigned>(i), src_buf[i], dst_buf[i]);
                return false;
            }
        }
        return true;
    }
}

// ---------------------------------------------------------------------------
// DMA-канал (manual trigger)
// ---------------------------------------------------------------------------
bool test_dma_channel(uint8_t channel)
{
    using namespace HAL::EDMA;

    prepare_buffers();

    DmaChannel dma(channel, REGS::EDMA::EVENT_Q0);
    if (!dma.init())
    {
        RTT_LOG_E(TAG, "DMA ch%u: request failed", channel);
        EDMA_Diagnostics::dump_full_diagnostics(snapshot, channel, false, "INIT_FAILED");
        return false;
    }

    dma.setCallback(on_edma_complete, on_edma_error,
                    const_cast<std::atomic<bool>*>(&transfer_complete));

    auto param = ParamBuilder()
                     .setSource(reinterpret_cast<uintptr_t>(src_buf),
                                static_cast<int16_t>(BUFFER_SIZE),
                                static_cast<int16_t>(BUFFER_SIZE))
                     .setDest(reinterpret_cast<uintptr_t>(dst_buf),
                              static_cast<int16_t>(BUFFER_SIZE),
                              static_cast<int16_t>(BUFFER_SIZE))
                     .setTransferParams(static_cast<uint16_t>(BUFFER_SIZE), 1, 1)
                     .setSyncType(false)                    // A-Sync
                     .enableCompletionInterrupt(channel)
                     .build();

    dma.configure(param);

    transfer_complete.store(false, std::memory_order_release);
    dma.start(TriggerMode::TRIG_MODE_MANUAL);

    uint32_t timeout = 5'000'000;
    while (!transfer_complete.load(std::memory_order_acquire) && --timeout) {
        __asm volatile("nop");
    }

    if (timeout == 0)
    {
        RTT_LOG_E(TAG, "DMA ch%u: TIMEOUT", channel);
        EDMA_Diagnostics::dump_full_diagnostics(snapshot, channel, false, "TIMEOUT");
        return false;
    }

    if (!verify_buffers("DMA", channel))
    {
        EDMA_Diagnostics::dump_full_diagnostics(snapshot, channel, false, "DATA_MISMATCH");
        return false;
    }

    RTT_LOG_I(TAG, "DMA ch%u: PASSED", channel);
    return true;
}

// ---------------------------------------------------------------------------
// QDMA-канал (trigger word = DST)
// ---------------------------------------------------------------------------
bool test_qdma_channel(uint8_t qch)
{
    using namespace HAL::EDMA;

    prepare_buffers();

    const uint8_t tcc = qch;

    QdmaChannel qdma(qch, tcc, REGS::EDMA::e_paRAM_entry_field::DST);

    if (!qdma.init())
    {
        RTT_LOG_E(TAG, "QDMA ch%u: init failed", qch);
        EDMA_Diagnostics::dump_full_diagnostics(snapshot, qch, true, "INIT_FAILED");
        return false;
    }

    qdma.setCallback(on_edma_complete, on_edma_error,
                     const_cast<std::atomic<bool>*>(&transfer_complete));

    auto param = ParamBuilder()
                     .setSource(reinterpret_cast<uintptr_t>(src_buf),
                                static_cast<int16_t>(BUFFER_SIZE),
                                static_cast<int16_t>(BUFFER_SIZE))
                     .setDest(reinterpret_cast<uintptr_t>(dst_buf),
                              static_cast<int16_t>(BUFFER_SIZE),
                              static_cast<int16_t>(BUFFER_SIZE))
                     .setTransferParams(static_cast<uint16_t>(BUFFER_SIZE), 1, 1)
                     .setSyncType(false)                    // A-Sync
                     .enableCompletionInterrupt(tcc)
                     .setStatic()
                     .setSrcDstDestinationMode(false, false)
                     .build();

    transfer_complete.store(false, std::memory_order_release);

    qdma.configure(param);   // пишет PaRAM + enable QEER
    qdma.start();            // повторная запись trigger word → старт

    uint32_t timeout = 5'000'000;
    while (!transfer_complete.load(std::memory_order_acquire) && --timeout) {
        __asm volatile("nop");
    }

    if (timeout == 0)
    {
        RTT_LOG_E(TAG, "QDMA ch%u: TIMEOUT", qch);
        EDMA_Diagnostics::dump_full_diagnostics(snapshot, qch, true, "TIMEOUT");
        return false;
    }

    if (!verify_buffers("QDMA", qch))
    {
        EDMA_Diagnostics::dump_full_diagnostics(snapshot, qch, true, "DATA_MISMATCH");
        return false;
    }


    RTT_LOG_I(TAG, "QDMA ch%u: PASSED", qch);
    disable_QDMA_event(qch);

    return true;
}

extern "C" void edma_test(void)
{
    using namespace HAL::INTC;

    // Одноразовая инициализация модуля и прерываний
    HAL::EDMA::module_clock_config();
    HAL::EDMA::init(REGS::EDMA::EVENT_Q0);

    register_handler(REGS::INTC::EDMACOMPINT, reinterpret_cast<isr_handler_t>(EDMA_Completion_ISR));
    priority_set(REGS::INTC::EDMACOMPINT, 0, REGS::INTC::HOSTINT_ROUTE_IRQ);
    unmask_interrupt(REGS::INTC::EDMACOMPINT);

    register_handler(REGS::INTC::EDMAERRINT, reinterpret_cast<isr_handler_t>(EDMA_Error_ISR));
    priority_set(REGS::INTC::EDMAERRINT, 0, REGS::INTC::HOSTINT_ROUTE_IRQ);
    unmask_interrupt(REGS::INTC::EDMAERRINT);

    // Исходный буфер заполняем один раз
    for (size_t i = 0; i < BUFFER_SIZE; ++i) {
        src_buf[i] = static_cast<uint8_t>(i + 0xA5);
    }

    // --- Все 64 DMA-канала ---
    RTT_LOG_I(TAG, "=== DMA channels test (0..63) ===");
    uint32_t dma_ok = 0;
    for (uint8_t ch = 0; ch < 64; ++ch) {
        if (test_dma_channel(ch)) {
            ++dma_ok;
        }
    }
    RTT_LOG_I(TAG, "DMA: %u/64 passed", static_cast<unsigned>(dma_ok));

    // --- Все 8 QDMA-каналов ---
    RTT_LOG_I(TAG, "=== QDMA channels test (0..7) ===");
    REGS::EDMA::AM335X_EDMA3CC->QDMAQNUM.reg = 0x00000000;

    // Разрешаем Shadow Region access для QDMA (если не было включено)
    // Восстанавливаем сброшенные тестом DMA регистры доступа:
    const auto region = HAL::EDMA::get_region_id();
    REGS::EDMA::AM335X_EDMA3CC->DRAE(region).reg  = 0xFFFFFFFF;
    REGS::EDMA::AM335X_EDMA3CC->DRAEH(region).reg = 0xFFFFFFFF;
    REGS::EDMA::AM335X_EDMA3CC->QRAE[region].reg  = 0xFF;

    uint32_t qdma_ok = 0;
    for (uint8_t qch = 0; qch < 8; ++qch) {
        if (test_qdma_channel(qch)) {
            ++qdma_ok;
        }
    }
    RTT_LOG_I(TAG, "QDMA: %u/8 passed", static_cast<unsigned>(qdma_ok));

    RTT_LOG_I(TAG, "=== EDMA test finished ===");
}