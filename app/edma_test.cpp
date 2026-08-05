#include "edma_test.h"

#include "edma_test.h"
#include "hal/EDMA/EDMA.hpp"
#include "hal/EDMA/DmaChannel.hpp"
#include "hal/EDMA/ParamBuilder.hpp"
#include "hal/INTC.hpp"
#include "rtt/rtt_log.h"

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
    std::atomic<bool> transfer_error{false};

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
    // 1. Включаем тактирование и инициализируем модуль EDMA3
    HAL::EDMA::module_clock_config();
    HAL::EDMA::init(REGS::EDMA::EVENT_Q0);

    // 2. Регистрируем обработчики EDMA3 в контроллере прерываний AINTC (AINTC_HOSTINT_ROUTE_IRQ)
    register_handler(REGS::INTC::EDMACOMPINT , reinterpret_cast<isr_handler_t>(EDMA_Completion_ISR));
    priority_set(REGS::INTC::EDMACOMPINT,0,REGS::INTC::HOSTINT_ROUTE_IRQ);
    unmask_interrupt(REGS::INTC::EDMACOMPINT);
    register_handler(REGS::INTC::EDMAERRINT, reinterpret_cast<isr_handler_t>(EDMA_Error_ISR));
    priority_set(REGS::INTC::EDMAERRINT,0,REGS::INTC::HOSTINT_ROUTE_IRQ);
    unmask_interrupt(REGS::INTC::EDMAERRINT);

    // 3. Заполняем тестовые данные
    for (size_t i = 0; i < BUFFER_SIZE; ++i) {
        src_buf[i] = static_cast<uint8_t>(i + 0xA5);
        dst_buf[i] = 0x00;
    }

    // 4. Создаем канал DMA (выбираем канал 63, как в примере StarterWare)
    constexpr uint8_t TEST_CHANNEL = 63;
    HAL::EDMA::DmaChannel dma(TEST_CHANNEL, REGS::EDMA::EVENT_Q0);

    if (!dma.init()) {
        RTT_LOG_E(TAG, "Failed to request EDMA channel %d", TEST_CHANNEL);
        return;
    }

    // 5. Настраиваем Callback через InterruptDispatcher
    dma.setCallback(on_edma_complete, on_edma_error, const_cast<std::atomic<bool>*>(&transfer_complete));

    // 6. Формируем PaRAM запись с помощью ParamBuilder
    // Передаем A-Sync трансфер (1 фрейм из BUFFER_SIZE байт)
    auto param = HAL::EDMA::ParamBuilder()
                     .setSource(reinterpret_cast<uintptr_t>(src_buf), static_cast<int16_t>(BUFFER_SIZE), static_cast<int16_t>(BUFFER_SIZE))
                     .setDest(reinterpret_cast<uintptr_t>(dst_buf), static_cast<int16_t>(BUFFER_SIZE), static_cast<int16_t>(BUFFER_SIZE))
                     .setTransferParams(static_cast<uint16_t>(BUFFER_SIZE), 1, 1)
                     .setSyncType(false) // A-Sync
                     .enableCompletionInterrupt(TEST_CHANNEL)
                     .build();

    dma.configure(param);

    // 7. Запускаем передачу (Manual Trigger)
    transfer_complete.store(false);
    dma.start(HAL::EDMA::TriggerMode::TRIG_MODE_MANUAL);

    // 8. Ожидаем завершения в прерывании
    uint32_t timeout = 100000000;
    while (!transfer_complete.load(std::memory_order_acquire) && --timeout > 0) {
        __asm volatile("nop");
    }

    if (timeout == 0) {
        RTT_LOG_E(TAG, "EDMA Transfer TIMEOUT!");
        return;
    }

    // 9. Проверяем целостность данных
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

    // При выходе из функции dma (DmaChannel) автоматически освободит ресурсы в своём деструкторе!
}