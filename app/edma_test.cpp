#include "edma_test.h"
#include "hal/EDMA/EDMA.hpp"
#include "hal/EDMA/DmaChannel.hpp"
#include "hal/EDMA/QdmaChannel.hpp"
#include "hal/EDMA/ParamBuilder.hpp"
#include "hal/INTC.hpp"
#include "rtt/rtt_log.h"
#include "startup/cp15.h"

#include <array>
#include <atomic>

#define TAG "EDMA_TEST"

namespace
{
    // Буферы выравниваем по кэш-линии Cortex-A8 (64 байта)
    constexpr size_t BUFFER_SIZE = 64;
    alignas(64) uint8_t src_buf[BUFFER_SIZE];
    alignas(64) uint8_t dst_buf[BUFFER_SIZE];

    // Синхронизация прерывания для bare-metal/setup фазы
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
}

extern "C" void edma_test(void)
{
    using namespace HAL::INTC;
    // Включаем тактирование и инициализируем модуль EDMA3
    HAL::EDMA::module_clock_config();
    HAL::EDMA::init(REGS::EDMA::EVENT_Q0);

    register_handler(REGS::INTC::EDMACOMPINT , reinterpret_cast<isr_handler_t>(EDMA_Completion_ISR));
    priority_set(REGS::INTC::EDMACOMPINT,0,REGS::INTC::HOSTINT_ROUTE_IRQ);
    unmask_interrupt(REGS::INTC::EDMACOMPINT);
    register_handler(REGS::INTC::EDMAERRINT, reinterpret_cast<isr_handler_t>(EDMA_Error_ISR));
    priority_set(REGS::INTC::EDMAERRINT,0,REGS::INTC::HOSTINT_ROUTE_IRQ);
    unmask_interrupt(REGS::INTC::EDMAERRINT);

    for (size_t i = 0; i < BUFFER_SIZE; ++i) {
        src_buf[i] = static_cast<uint8_t>(i + 0xA5);
        dst_buf[i] = 0x00;
    }

    // Выталкиваем исходный буфер из L1D кэша в RAM и Сбрасываем возможные dirty-строки целевого буфера
    cp15_D_cache_clean_buff(reinterpret_cast<unsigned int>(src_buf), BUFFER_SIZE);
    cp15_D_cache_flush_buff(reinterpret_cast<unsigned int>(dst_buf), BUFFER_SIZE);
    cp15_DSB_barrier(); // Гарантируем завершение всех операций записи перед стартом DMA

    constexpr uint8_t TEST_CHANNEL = 63;
    HAL::EDMA::DmaChannel dma(TEST_CHANNEL, REGS::EDMA::EVENT_Q0);

    if (!dma.init()) {
        RTT_LOG_E(TAG, "Failed to request EDMA channel %d", TEST_CHANNEL);
        return;
    }

    // Настраиваем Callback через InterruptDispatcher
    dma.setCallback(on_edma_complete, on_edma_error, const_cast<std::atomic<bool>*>(&transfer_complete));

    // Передаем A-Sync трансфер (1 фрейм из BUFFER_SIZE байт)
    auto param = HAL::EDMA::ParamBuilder()
                     .setSource(reinterpret_cast<uintptr_t>(src_buf), static_cast<int16_t>(BUFFER_SIZE), static_cast<int16_t>(BUFFER_SIZE))
                     .setDest(reinterpret_cast<uintptr_t>(dst_buf), static_cast<int16_t>(BUFFER_SIZE), static_cast<int16_t>(BUFFER_SIZE))
                     .setTransferParams(static_cast<uint16_t>(BUFFER_SIZE), 1, 1)
                     .setSyncType(false) // A-Sync
                     .enableCompletionInterrupt(TEST_CHANNEL)
                     .build();

    dma.configure(param);

    // Запускаем передачу (Manual Trigger)
    transfer_complete.store(false);
    dma.start(HAL::EDMA::TriggerMode::TRIG_MODE_MANUAL);

    uint32_t timeout = 10000000;
    while (!transfer_complete.load(std::memory_order_acquire) && --timeout > 0) {
        __asm volatile("nop");
    }

    if (timeout == 0) {
        RTT_LOG_E(TAG, "EDMA Transfer TIMEOUT!");
        return;
    }

    // Ждем завершения выгрузки шины DMA и синхронизируем память
    cp15_DSB_barrier();

    // Инвалидируем D-кэш приёмника, чтобы ЦП прочитал новые данные прямо из RAM
    cp15_D_cache_flush_buff(reinterpret_cast<unsigned int>(dst_buf), BUFFER_SIZE);

    bool match = true;
    for (size_t i = 0; i < BUFFER_SIZE; ++i) {
        if (src_buf[i] != dst_buf[i]) {
            match = false;
            RTT_LOG_E(TAG, "Data mismatch at index %d: src 0x%02X != dst 0x%02X", i, src_buf[i], dst_buf[i]);
            break;
        }
    }

    if (match) {
        RTT_LOG_I(TAG, "EDMA Loopback Test PASSED successfully!");
    }

    // -------------------------------------------------------------------------
    // Тестирование QDMA (Канал 0, TCC 10)
    // -------------------------------------------------------------------------
    constexpr uint8_t QDMA_CH = 0;
    constexpr uint8_t TCC_NUM = 0;

    // Сбрасываем приемный буфер в 0, чтобы убедиться, что QDMA действительно записал данные
    for (size_t i = 0; i < BUFFER_SIZE; ++i) {
        dst_buf[i] = 0x00;
    }

    // Подготавливаем кэш перед стартом QDMA
    cp15_D_cache_flush_buff(reinterpret_cast<unsigned int>(dst_buf), BUFFER_SIZE);
    cp15_DSB_barrier();

    HAL::EDMA::QdmaChannel qdma(QDMA_CH, TCC_NUM, REGS::EDMA::e_paRAM_entry_field::DST);

    if (!qdma.init()) {
        RTT_LOG_E(TAG, "Failed to init QDMA channel %d", QDMA_CH);
        return;
    }

    qdma.setCallback(on_edma_complete, on_edma_error, const_cast<std::atomic<bool>*>(&transfer_complete));

    param = HAL::EDMA::ParamBuilder()
                 .setSource(reinterpret_cast<uintptr_t>(src_buf), BUFFER_SIZE, BUFFER_SIZE)
                 .setDest(reinterpret_cast<uintptr_t>(dst_buf), BUFFER_SIZE, BUFFER_SIZE)
                 .setTransferParams(BUFFER_SIZE, 1, 1)
                 .setSyncType(false)
                 .enableCompletionInterrupt(TCC_NUM) // Прерывание по TCC_NUM
                 .enableIntermediateCompletionInterrupt()
                 .setStatic()
                 .setSrcDstDestinationMode(false,false)
                 .build();

    transfer_complete.store(false);

    qdma.configure(param);
    qdma.start();

    timeout = 10000000;
    while (!transfer_complete.load(std::memory_order_acquire) && --timeout > 0) {
        __asm volatile("nop");
    }

    if (timeout == 0) {
        RTT_LOG_E(TAG, "QDMA Transfer TIMEOUT!");
        return;
    }

    // Синхронизируем память и инвалидируем D-кэш для приёмника
    cp15_DSB_barrier();
    cp15_D_cache_flush_buff(reinterpret_cast<unsigned int>(dst_buf), BUFFER_SIZE);

    match = true;
    for (size_t i = 0; i < BUFFER_SIZE; ++i) {
        if (src_buf[i] != dst_buf[i]) {
            match = false;
            RTT_LOG_E(TAG, "QDMA Data mismatch at index %d: src 0x%02X != dst 0x%02X", i, src_buf[i], dst_buf[i]);
            break;
        }
    }

    if (match) {
        RTT_LOG_I(TAG, "QDMA Loopback Test PASSED successfully!");
    }
}