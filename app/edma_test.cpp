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

    // Контекст для передачи в ISR callback
    struct PingPongContext
    {
        uint8_t channel{0};
        volatile uint32_t completed_transfers{0};
        uint32_t target_transfers{0};
    };

    static PingPongContext g_pingpong_ctx;

    // Callback, вызываемый из InterruptDispatcher при каждом завершении (IPR)
    void on_pingpong_completion(void* context) noexcept
    {
        auto* ctx = static_cast<PingPongContext*>(context);
        if (!ctx) return;

        ctx->completed_transfers++;

        // Если достигли целевого количества итераций — отключаем событие EDMA
        if (ctx->completed_transfers >= ctx->target_transfers)
        {
            HAL::EDMA::disable_transfer(ctx->channel, REGS::EDMA::TRIG_MODE_EVENT);
        }
    }

    void on_pingpong_error(HAL::EDMA::ErrorType err, void* context) noexcept
    {
        (void)err;
        auto* ctx = static_cast<PingPongContext*>(context);
        if (ctx)
        {
            HAL::EDMA::disable_transfer(ctx->channel, REGS::EDMA::TRIG_MODE_EVENT);
        }
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

    const auto param = PaRAMFactory::makeASync(reinterpret_cast<uintptr_t>(src_buf),
                                                           reinterpret_cast<uintptr_t>(dst_buf),
                                                           BUFFER_SIZE,
                                                           channel);

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

    const auto param = PaRAMFactory::makeABSync(reinterpret_cast<uintptr_t>(src_buf),
                                                            reinterpret_cast<uintptr_t>(dst_buf),
                                                            ACNT,
                                                            BCNT,
                                                            channel);

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

    const uint8_t param0 = channel;
    const uint8_t param1 = (channel + 1) % REGS::EDMA::AM335X_DMACH_MAX;          // следующий набор
    constexpr uint16_t HALF = BUFFER_SIZE / 2;   // 32

    DmaChannel dma(channel, REGS::EDMA::EVENT_Q0);
    if (!dma.init())
    {
        RTT_LOG_E(TAG, "%s ch%u: request failed", who,  channel);
        return false;
    }

    const auto [param_first, param_last] = PaRAMFactory::makeChain(reinterpret_cast<uintptr_t>(src_buf),
                                                                                                   reinterpret_cast<uintptr_t>(dst_buf),
                                                                                                   HALF,
                                                                                                   param0,
                                                                                                   param1,
                                                                                                   channel);

    dma.configure({{param0,param_first},
                          {param1,param_last}});

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

bool test_dma_channel_pingpong(const uint8_t channel, const uint32_t repetitions)
{
    using namespace HAL::EDMA;
    constexpr uint16_t HALF = 32;
    constexpr char who[] = "DMA-PINGPONG";
    const uint8_t NUM_TRANSFERS = repetitions;

    alignas(64) static uint8_t bufA[HALF];
    alignas(64) static uint8_t bufB[HALF];

    // Готовим источник
    for (size_t i = 0; i < BUFFER_SIZE; ++i)
        src_buf[i] = static_cast<uint8_t>(0xA0 + i);

    // Чистим приёмники
    for (size_t i = 0; i < HALF; ++i) {
        bufA[i] = 0;
        bufB[i] = 0;
    }
    cp15_D_cache_clean_buff(reinterpret_cast<unsigned int>(src_buf), BUFFER_SIZE);
    cp15_D_cache_flush_buff(reinterpret_cast<unsigned int>(bufA), HALF);
    cp15_D_cache_flush_buff(reinterpret_cast<unsigned int>(bufB), HALF);
    cp15_DSB_barrier();

    DmaChannel dma(channel, REGS::EDMA::EVENT_Q0);

    const uint8_t ping_param_id = channel;
    const uint8_t pong_param_id = (channel + 1) % REGS::EDMA::AM335X_DMACH_MAX;

    g_pingpong_ctx.channel = channel;
    g_pingpong_ctx.completed_transfers = 0;
    g_pingpong_ctx.target_transfers = NUM_TRANSFERS;

    if (!dma.init(on_pingpong_completion, on_pingpong_error, &g_pingpong_ctx))
    {
        RTT_LOG_E(TAG, "%s ch%u: request failed", who, channel);
        EDMA_Diagnostics::dump_full_diagnostics(snapshot, channel, false, "INIT_FAILED");
        return false;
    }

    const auto [param_ping, param_pong] = PaRAMFactory::makePingPong(reinterpret_cast<uintptr_t>(src_buf),
                                                                                                     reinterpret_cast<uintptr_t>(bufA),
                                                                                                     reinterpret_cast<uintptr_t>(bufB),
                                                                                                     HALF,
                                                                                                     ping_param_id,
                                                                                                     pong_param_id,
                                                                                                     channel);

    dma.configure({{ ping_param_id, param_ping },
                          { pong_param_id, param_pong } });

    dma.trigger(TriggerMode::TRIG_MODE_MANUAL);

    uint32_t timeout_loops = 10'000'000;
    while (g_pingpong_ctx.completed_transfers < NUM_TRANSFERS && --timeout_loops)
    {
        // Если в реальном проекте Ping-Pong триггерится периферией (I2S/Timer/Event),
        // вызов dma.trigger() категорически НЕ НУЖЕН и даже вреден.
        if (g_pingpong_ctx.completed_transfers < NUM_TRANSFERS) {
            dma.trigger(TriggerMode::TRIG_MODE_MANUAL);
        }
        for (volatile int i = 0; i < 1000; ++i) { asm volatile("nop"); }
    }

    if (timeout_loops == 0) {
        RTT_LOG_E(TAG, "%s ch%u: TIMEOUT, done %u/%u transfers", who, channel,
                  (unsigned)g_pingpong_ctx.completed_transfers, NUM_TRANSFERS);
        return false;
    }

    cp15_DSB_barrier();
    cp15_D_cache_flush_buff(reinterpret_cast<unsigned int>(bufA), HALF);
    cp15_D_cache_flush_buff(reinterpret_cast<unsigned int>(bufB), HALF);

    bool ok = true;
    for (size_t i = 0; i < HALF; ++i)
    {
        if (bufA[i] != src_buf[i] && bufA[i] != src_buf[i + HALF])
        {
            RTT_LOG_E(TAG, "%s ch%u mismatch @%u: A=0x%02X B=0x%02X", who,
                      channel, static_cast<unsigned>(i), bufA[i], bufB[i]);
            ok = false;
            break;
        }
    }

    if (!ok) {
        EDMA_Diagnostics::dump_full_diagnostics(snapshot, channel, false, "DATA_MISMATCH");
        return false;
    }

    RTT_LOG_I(TAG, "Hardware Ping-Pong test (%u transfers) for ch%u PASSED.", NUM_TRANSFERS, channel);
    return true;
}

bool test_dma_channel_selflink(const uint8_t channel, const uint32_t repetitions)
{
    using namespace HAL::EDMA;
    constexpr char who[] = "DMA-SELFLINK";
    const uint32_t NUM_TRANSFERS = repetitions;

    prepare_buffers();

    DmaChannel dma(channel, REGS::EDMA::EVENT_Q0);

    g_pingpong_ctx.channel = channel;
    g_pingpong_ctx.completed_transfers = 0;
    g_pingpong_ctx.target_transfers = NUM_TRANSFERS;

    if (!dma.init(on_pingpong_completion, on_pingpong_error, &g_pingpong_ctx)) {
        RTT_LOG_E(TAG, "%s ch%u: request failed", who, channel);
        EDMA_Diagnostics::dump_full_diagnostics(snapshot, channel, false, "INIT_FAILED");
        return false;
    }

    const auto param = PaRAMFactory::makeSelfLink(reinterpret_cast<uintptr_t>(src_buf),
                                                              reinterpret_cast<uintptr_t>(dst_buf),
                                                              BUFFER_SIZE,
                                                              channel,          // param_id == channel
                                                              channel);

    dma.configure(param);

    dma.trigger(TriggerMode::TRIG_MODE_MANUAL);

    uint32_t timeout_loops = 10'000'000;
    while (g_pingpong_ctx.completed_transfers < NUM_TRANSFERS && --timeout_loops)
    {
        // Каждое событие Manual Trigger отрабатывает один перезагруженный кадр
        if (g_pingpong_ctx.completed_transfers < NUM_TRANSFERS) {
            dma.trigger(TriggerMode::TRIG_MODE_MANUAL);
        }
        for (volatile int i = 0; i < 1000; ++i) { asm volatile("nop"); }
    }

    if (timeout_loops == 0) {
        RTT_LOG_E(TAG, "%s ch%u: TIMEOUT, done %u/%u transfers", who, channel,
                  static_cast<unsigned>(g_pingpong_ctx.completed_transfers), (unsigned)NUM_TRANSFERS);
        return false;
    }

    if (!verify_buffers(who, channel)) {
        EDMA_Diagnostics::dump_full_diagnostics(snapshot, channel, false, "DATA_MISMATCH");
        return false;
    }

    RTT_LOG_I(TAG, "%s ch%u: PASSED (%u transfers)", who, channel, (unsigned)NUM_TRANSFERS);
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

    const auto param = PaRAMFactory::makeQdmaASync(reinterpret_cast<uintptr_t>(src_buf),
                                                               reinterpret_cast<uintptr_t>(dst_buf),
                                                               BUFFER_SIZE,
                                                               qch);

    qdma.configure(param);
    qdma.trigger();

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

    const auto param = PaRAMFactory::makeABSync(reinterpret_cast<uintptr_t>(src_buf),
                                                            reinterpret_cast<uintptr_t>(dst_buf),
                                                            ACNT,
                                                            BCNT,
                                                            qch);

    qdma.configure(param);
    qdma.trigger();

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
    const uint32_t param0 = (32 + qch) % REGS::EDMA::AM335X_DMACH_MAX;
    const uint32_t param1 = (33 + qch) % REGS::EDMA::AM335X_DMACH_MAX;
    constexpr uint16_t HALF = BUFFER_SIZE / 2;

    QdmaChannel qdma(qch, tcc, REGS::EDMA::e_paRAM_entry_field::DST);

    if (!qdma.init())
    {
        RTT_LOG_E(TAG, "%s ch%u: init failed", who, qch);
        EDMA_Diagnostics::dump_full_diagnostics(snapshot, qch, true, "INIT_FAILED");
        return false;
    }

    const auto [param_first, param_last] = PaRAMFactory::makeChain(reinterpret_cast<uintptr_t>(src_buf),
                                                                                                   reinterpret_cast<uintptr_t>(dst_buf),
                                                                                                   HALF,
                                                                                                   param0,
                                                                                                   param1,
                                                                                                   qch);

    qdma.configure({{ param0, param_first },
                           { param1, param_last } });
    qdma.trigger();

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
    RTT_LOG_I(TAG, "DMA: %u/64 PASSED", static_cast<unsigned>(dma_ok));

    RTT_LOG_I(TAG, "=== QDMA channels A-transfer test (0..7) ===");
    uint32_t qdma_ok = 0;
    for (uint8_t qch = 0; qch < 8; ++qch) {
        if (test_qdma_channel_a(qch)) {
            ++qdma_ok;
        }
    }
    RTT_LOG_I(TAG, "QDMA: %u/8 PASSED", static_cast<unsigned>(qdma_ok));

    RTT_LOG_I(TAG, "=== DMA channels AB-transfer test (0..63) ===");
    dma_ok = 0;
    for (uint8_t ch = 0; ch < 64; ++ch) {
        if (test_dma_channel_ab(ch)) {
            ++dma_ok;
        }
    }
    RTT_LOG_I(TAG, "DMA: %u/64 PASSED", static_cast<unsigned>(dma_ok));

    RTT_LOG_I(TAG, "=== QDMA channels AB-transfer test (0..7) ===");
    qdma_ok = 0;
    for (uint8_t qch = 0; qch < 8; ++qch) {
        if (test_qdma_channel_ab(qch)) {
            ++qdma_ok;
        }
    }
    RTT_LOG_I(TAG, "QDMA: %u/8 PASSED", static_cast<unsigned>(qdma_ok));

    RTT_LOG_I(TAG, "=== DMA channels Chain-transfer test (0..63) ===");
    dma_ok = 0;
    for (uint8_t ch = 0; ch < 64; ++ch) {
        if (test_dma_channel_chain(ch)) {
            ++dma_ok;
        }
    }
    RTT_LOG_I(TAG, "DMA: %u/64 PASSED", static_cast<unsigned>(dma_ok));

    RTT_LOG_I(TAG, "=== QDMA channels Chain-transfer test (0..7) ===");
    qdma_ok = 0;
    for (uint8_t qch = 0; qch < 8; ++qch) {
        if (test_qdma_channel_chain(qch)) {
            ++qdma_ok;
        }
    }
    RTT_LOG_I(TAG, "QDMA: %u/8 PASSED", static_cast<unsigned>(qdma_ok));

    RTT_LOG_I(TAG, "=== DMA channels Ping-Pong test ===");
    test_dma_channel_pingpong(0,8);
    test_dma_channel_pingpong(1,9);

    RTT_LOG_I(TAG, "=== DMA channels Selflink test ===");
    test_dma_channel_selflink(0, 5);
    test_dma_channel_selflink(1, 10);

    RTT_LOG_I(TAG, "=== EDMA test finished ===");
}