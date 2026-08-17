#include "edma_test.h"
#include "hal/EDMA/EDMA.hpp"
#include "hal/EDMA/DmaChannel.hpp"
#include "hal/EDMA/QdmaChannel.hpp"
#include "hal/EDMA/ParamBuilder.hpp"
#include "hal/EDMA/EDMA_diagnostics.hpp"
#include "hal/INTC.hpp"
#include "rtt/rtt_log.h"
#include "startup/cp15.h"

#define TAG "EDMA_TEST"

namespace
{
    static HAL::EDMA::EDMA_DiagnosticSnapshot snapshot;

    constexpr size_t BUFFER_SIZE = 64;
    alignas(64) uint8_t src_buf[BUFFER_SIZE];
    alignas(64) uint8_t dst_buf[BUFFER_SIZE];

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
    dma.trigger(TriggerMode::TRIG_MODE_MANUAL);

    if (!dma.wait_completion())
    {
        RTT_LOG_E(TAG, "DMA ch%u: %s", channel, dma.has_error() ? "ERROR" : "TIMEOUT");
        EDMA_Diagnostics::dump_full_diagnostics(snapshot, channel, false,
                                                 dma.has_error() ? "ERROR" : "TIMEOUT");
        return false;
    }

    if (!verify_buffers("DMA", channel))
    {
        EDMA_Diagnostics::dump_full_diagnostics(snapshot, channel, false, "DATA_MISMATCH");
        return false;
    }

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

    qdma.configure(param);   // пишет PaRAM + разрешает QER
    qdma.trigger();          // запись в trigger word запускает передачу

    if (!qdma.wait_completion())
    {
        RTT_LOG_E(TAG, "QDMA ch%u: %s", qch, qdma.has_error() ? "ERROR" : "TIMEOUT");
        EDMA_Diagnostics::dump_full_diagnostics(snapshot, qch, true,
                                                 qdma.has_error() ? "ERROR" : "TIMEOUT");
        return false;
    }

    if (!verify_buffers("QDMA", qch))
    {
        EDMA_Diagnostics::dump_full_diagnostics(snapshot, qch, true, "DATA_MISMATCH");
        return false;
    }

    return true;
}

extern "C" void edma_test(void)
{
    // Одноразовая инициализация периферии EDMA3 и векторов прерываний INTC
    HAL::EDMA::module_clock_config();
    HAL::EDMA::init(REGS::EDMA::EVENT_Q0);

    for (size_t i = 0; i < BUFFER_SIZE; ++i) {
        src_buf[i] = static_cast<uint8_t>(i + 0xA5);
    }

    // --- DMA каналы (0..63) ---
    RTT_LOG_I(TAG, "=== DMA channels A-transfer test (0..63) ===");
    uint32_t dma_ok = 0;
    for (uint8_t ch = 0; ch < 64; ++ch) {
        if (test_dma_channel(ch)) {
            ++dma_ok;
        }
    }
    RTT_LOG_I(TAG, "DMA: %u/64 passed", static_cast<unsigned>(dma_ok));

    // --- QDMA каналы (0..7) ---
    RTT_LOG_I(TAG, "=== QDMA channels A-transfer test (0..7) ===");
    uint32_t qdma_ok = 0;
    for (uint8_t qch = 0; qch < 8; ++qch) {
        if (test_qdma_channel(qch)) {
            ++qdma_ok;
        }
    }
    RTT_LOG_I(TAG, "QDMA: %u/8 passed", static_cast<unsigned>(qdma_ok));

    RTT_LOG_I(TAG, "=== EDMA test finished ===");
}