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
bool test_dma_channel_a(const uint8_t channel)
{
    using namespace HAL::EDMA;
    constexpr char who[] = "DMA-At";

    prepare_buffers();

    DmaChannel dma(channel, REGS::EDMA::EVENT_Q0);
    if (!dma.init())
    {
        RTT_LOG_E(TAG, "%s ch%u: request failed",who, channel);
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
        RTT_LOG_E(TAG, "%s ch%u: %s", who, channel, dma.has_error() ? "ERROR" : "TIMEOUT");
        EDMA_Diagnostics::dump_full_diagnostics(snapshot, channel, false,
                                                 dma.has_error() ? "ERROR" : "TIMEOUT");
        return false;
    }

    if (!verify_buffers(who, channel))
    {
        EDMA_Diagnostics::dump_full_diagnostics(snapshot, channel, false, "DATA_MISMATCH");
        return false;
    }

    return true;
}

bool test_dma_channel_ab(const uint8_t channel)
{
    using namespace HAL::EDMA;
    constexpr char who[] = "DMA-ABt";

    prepare_buffers();

    DmaChannel dma(channel, REGS::EDMA::EVENT_Q0);
    if (!dma.init())
    {
        RTT_LOG_E(TAG, "%s ch%u: request failed", who, channel);
        EDMA_Diagnostics::dump_full_diagnostics(snapshot, channel, false, "INIT_FAILED");
        return false;
    }

    constexpr uint16_t ACNT = 16;
    constexpr uint16_t BCNT = 4;
    static_assert(ACNT * BCNT == BUFFER_SIZE);

    auto param = ParamBuilder()
                     .setSource(reinterpret_cast<uintptr_t>(src_buf),
                                static_cast<int16_t>(ACNT),   // SRCBIDX = ACNT
                                0)                            // SRCCIDX не нужен (CCNT=1)
                     .setDest(reinterpret_cast<uintptr_t>(dst_buf),
                              static_cast<int16_t>(ACNT),     // DSTBIDX = ACNT
                              0)
                     .setTransferParams(ACNT, BCNT, 1)
                     .setSyncType(true)                       // ← AB-Sync
                     .enableCompletionInterrupt(channel)
                     .build();

    dma.configure(param);
    dma.trigger(TriggerMode::TRIG_MODE_MANUAL);

    if (!dma.wait_completion())
    {
        RTT_LOG_E(TAG, "%s ch%u: %s", who, channel, dma.has_error() ? "ERROR" : "TIMEOUT");
        EDMA_Diagnostics::dump_full_diagnostics(snapshot, channel, false,
                                                 dma.has_error() ? "ERROR" : "TIMEOUT");
        return false;
    }

    if (!verify_buffers(who, channel))
    {
        EDMA_Diagnostics::dump_full_diagnostics(snapshot, channel, false, "DATA_MISMATCH");
        return false;
    }

    return true;
}

bool test_dma_channel_chain(const uint8_t channel)
{
    using namespace HAL::EDMA;
    constexpr char who[] = "DMA-CHAIN";

    prepare_buffers();

    // Используем два PaRAM: основной (channel) и следующий (channel+1)
    // На AM335x первые 64 PaRAM жёстко привязаны к каналам 0..63,
    // поэтому для простоты берём канал N и PaRAM N+1 (убедитесь, что N+1 свободен).
    const uint8_t param0 = channel;
    const uint8_t param1 = (channel + 1)%64;          // следующий набор
    constexpr uint16_t HALF = BUFFER_SIZE / 2;   // 32

    DmaChannel dma(channel, REGS::EDMA::EVENT_Q0);
    if (!dma.init())
    {
        RTT_LOG_E(TAG, "%s ch%u: request failed", who,  channel);
        return false;
    }

    // Второй PaRAM (конечный)
    auto param_last = ParamBuilder()
                          .setSource(reinterpret_cast<uintptr_t>(src_buf + HALF),
                                     static_cast<int16_t>(HALF), 0)
                          .setDest(reinterpret_cast<uintptr_t>(dst_buf + HALF),
                                   static_cast<int16_t>(HALF), 0)
                          .setTransferParams(HALF, 1, 1)
                          .setSyncType(false)
                          .enableCompletionInterrupt(channel)   // прерывание только в конце
                          .setStatic(true)
                          .setLink(0xFFFF)                      // конец цепочки
                          .build();

    // Первый PaRAM (линкуется на второй)
    auto param_first = ParamBuilder()
                           .setSource(reinterpret_cast<uintptr_t>(src_buf),
                                      static_cast<int16_t>(HALF), 0)
                           .setDest(reinterpret_cast<uintptr_t>(dst_buf),
                                    static_cast<int16_t>(HALF), 0)
                           .setTransferParams(HALF, 1, 1)
                           .setSyncType(false)                          // ← A-Sync
                           .enableTransferCompleteChaining(channel,true)
                           .setLink(static_cast<uint16_t>(param1 * 0x20))  // адрес следующего PaRAM
                           .setStatic(false)            // for chaining static must be set 0
                           .build();

    // Программируем оба набора
    HAL::EDMA::set_paRAM(param0, param_first);
    HAL::EDMA::set_paRAM(param1, param_last);

    dma.trigger(TriggerMode::TRIG_MODE_MANUAL);

    if (!dma.wait_completion())
    {
        RTT_LOG_E(TAG, "%s ch%u: %s", who, channel, dma.has_error() ? "ERROR" : "TIMEOUT");
        EDMA_Diagnostics::dump_full_diagnostics(snapshot, channel, false,
                                                 dma.has_error() ? "ERROR" : "TIMEOUT");
        return false;
    }

    if (!verify_buffers(who, channel))
    {
        EDMA_Diagnostics::dump_full_diagnostics(snapshot, channel, false, "DATA_MISMATCH");
        return false;
    }
    return true;
}

// ---------------------------------------------------------------------------
// QDMA-канал (trigger word = DST)
// ---------------------------------------------------------------------------
bool test_qdma_channel_a(const uint8_t qch)
{
    using namespace HAL::EDMA;
    constexpr char who[] = "QDMA-At";

    prepare_buffers();

    const uint8_t tcc = qch;

    QdmaChannel qdma(qch, tcc, REGS::EDMA::e_paRAM_entry_field::DST);

    if (!qdma.init())
    {
        RTT_LOG_E(TAG, "%s ch%u: init failed", who, qch);
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
        RTT_LOG_E(TAG, "%s ch%u: %s", who, qch, qdma.has_error() ? "ERROR" : "TIMEOUT");
        EDMA_Diagnostics::dump_full_diagnostics(snapshot, qch, true,
                                                 qdma.has_error() ? "ERROR" : "TIMEOUT");
        return false;
    }

    if (!verify_buffers(who, qch))
    {
        EDMA_Diagnostics::dump_full_diagnostics(snapshot, qch, true, "DATA_MISMATCH");
        return false;
    }

    return true;
}

bool test_qdma_channel_ab(const uint8_t qch)
{
    using namespace HAL::EDMA;
    constexpr char who[] = "QDMA-ABt";

    prepare_buffers();

    const uint8_t tcc = qch;

    QdmaChannel qdma(qch, tcc, REGS::EDMA::e_paRAM_entry_field::DST);

    if (!qdma.init())
    {
        RTT_LOG_E(TAG, "%s ch%u: init failed", who, qch);
        EDMA_Diagnostics::dump_full_diagnostics(snapshot, qch, true, "INIT_FAILED");
        return false;
    }

    constexpr uint16_t ACNT = 16;
    constexpr uint16_t BCNT = 4;
    static_assert(ACNT * BCNT == BUFFER_SIZE);

    auto param = ParamBuilder()
                     .setSource(reinterpret_cast<uintptr_t>(src_buf),
                                static_cast<int16_t>(ACNT), 0)
                     .setDest(reinterpret_cast<uintptr_t>(dst_buf),
                              static_cast<int16_t>(ACNT), 0)
                     .setTransferParams(ACNT, BCNT, 1)
                     .setSyncType(true)                       // ← AB-Sync
                     .enableCompletionInterrupt(tcc)
                     .setStatic()
                     .setSrcDstDestinationMode(false, false)
                     .build();

    qdma.configure(param);   // пишет PaRAM + разрешает QER
    qdma.trigger();          // запись в trigger word запускает передачу

    if (!qdma.wait_completion())
    {
        RTT_LOG_E(TAG, "%s ch%u: %s", who, qch, qdma.has_error() ? "ERROR" : "TIMEOUT");
        EDMA_Diagnostics::dump_full_diagnostics(snapshot, qch, true,
                                                 qdma.has_error() ? "ERROR" : "TIMEOUT");
        return false;
    }

    if (!verify_buffers(who, qch))
    {
        EDMA_Diagnostics::dump_full_diagnostics(snapshot, qch, true, "DATA_MISMATCH");
        return false;
    }

    return true;
}

bool test_qdma_channel_chain(const uint8_t qch)
{
    using namespace HAL::EDMA;
    constexpr char who[] = "QDMA-CHAIN";

    prepare_buffers();

    const uint8_t tcc     = qch;
    const uint32_t param0 = (32 + qch)%64;          // основной (как в QdmaChannel)
    const uint32_t param1 = (33 + qch)%64;          // свободный набор для цепочки
    constexpr uint16_t HALF = BUFFER_SIZE / 2;

    QdmaChannel qdma(qch, tcc, REGS::EDMA::e_paRAM_entry_field::DST);

    if (!qdma.init())
    {
        RTT_LOG_E(TAG, "%s ch%u: init failed", who, qch);
        EDMA_Diagnostics::dump_full_diagnostics(snapshot, qch, true, "INIT_FAILED");
        return false;
    }

    map_QDMA_ch_to_paRAM(qch, const_cast<uint32_t&>(param0));

    auto param_last = ParamBuilder()
                              .setSource(reinterpret_cast<uintptr_t>(src_buf + HALF),
                                         static_cast<int16_t>(HALF), 0)
                              .setDest(reinterpret_cast<uintptr_t>(dst_buf + HALF),
                                       static_cast<int16_t>(HALF), 0)
                              .setTransferParams(HALF, 1, 1)
                              .setSyncType(false)
                              .enableCompletionInterrupt(tcc)
                              .setStatic(true)
                              .setLink(0xFFFF)
                              .build();

    // Первый PaRAM
    auto param_first = ParamBuilder()
                           .setSource(reinterpret_cast<uintptr_t>(src_buf),
                                      static_cast<int16_t>(HALF), 0)
                           .setDest(reinterpret_cast<uintptr_t>(dst_buf),
                                    static_cast<int16_t>(HALF), 0)
                           .setTransferParams(HALF, 1, 1)
                           .setSyncType(false)
                           .setStatic(false)
                           .setLink(static_cast<uint16_t>(param1 * 0x20))
                           .build();

    disable_QDMA_event(qch);
    QDMA_clr_miss_evt(qch);
    QDMA_set_paRAM(param0, param_first);
    QDMA_set_paRAM(param1, param_last);
    enable_QDMA_event(qch);

    qdma.trigger();          // запись в trigger word запускает передачу

    if (!qdma.wait_completion())
    {
        RTT_LOG_E(TAG, "%s ch%u: %s", who, qch, qdma.has_error() ? "ERROR" : "TIMEOUT");
        EDMA_Diagnostics::dump_full_diagnostics(snapshot, qch, true,
                                                 qdma.has_error() ? "ERROR" : "TIMEOUT");
        return false;
    }

    if (!verify_buffers(who, qch))
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

    RTT_LOG_I(TAG, "=== DMA channels A-transfer test (0..63) ===");
    uint32_t dma_ok = 0;
    for (uint8_t ch = 0; ch < 64; ++ch) {
        if (test_dma_channel_a(ch)) {
            ++dma_ok;
        }
    }
    RTT_LOG_I(TAG, "DMA: %u/64 passed", static_cast<unsigned>(dma_ok));

    RTT_LOG_I(TAG, "=== QDMA channels A-transfer test (0..7) ===");
    uint32_t qdma_ok = 0;
    for (uint8_t qch = 0; qch < 8; ++qch) {
        if (test_qdma_channel_a(qch)) {
            ++qdma_ok;
        }
    }
    RTT_LOG_I(TAG, "QDMA: %u/8 passed", static_cast<unsigned>(qdma_ok));

    RTT_LOG_I(TAG, "=== DMA channels AB-transfer test (0..63) ===");
    dma_ok = 0;
    for (uint8_t ch = 0; ch < 64; ++ch) {
        if (test_dma_channel_ab(ch)) {
            ++dma_ok;
        }
    }
    RTT_LOG_I(TAG, "DMA: %u/64 passed", static_cast<unsigned>(dma_ok));

    RTT_LOG_I(TAG, "=== QDMA channels AB-transfer test (0..7) ===");
    qdma_ok = 0;
    for (uint8_t qch = 0; qch < 8; ++qch) {
        if (test_qdma_channel_ab(qch)) {
            ++qdma_ok;
        }
    }
    RTT_LOG_I(TAG, "QDMA: %u/8 passed", static_cast<unsigned>(qdma_ok));

    RTT_LOG_I(TAG, "=== DMA channels Chain-transfer test (0..63) ===");
    dma_ok = 0;
    for (uint8_t ch = 0; ch < 64; ++ch) {
        if (test_dma_channel_chain(ch)) {
            ++dma_ok;
        }
    }
    RTT_LOG_I(TAG, "DMA: %u/64 passed", static_cast<unsigned>(dma_ok));

    RTT_LOG_I(TAG, "=== QDMA channels Chain-transfer test (0..7) ===");
    qdma_ok = 0;
    for (uint8_t qch = 0; qch < 8; ++qch) {
        if (test_qdma_channel_chain(qch)) {
            ++qdma_ok;
        }
    }
    RTT_LOG_I(TAG, "QDMA: %u/8 passed", static_cast<unsigned>(qdma_ok));

    RTT_LOG_I(TAG, "=== EDMA test finished ===");
}