#include <stdint.h>
#include <stdbool.h>
#include "init.h"
#include "rtt/rtt_log.h"
#include "hal/boards/beaglebone_black.hpp"
#include "hal/sysTimer.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "hal/EDMA/EDMA.hpp"
#include "edma_test.h"
#include "hal/PERF.hpp"

#define TAG "main"

void delay_ms(const uint32_t ms)
{
    using namespace HAL::TIMERS;
    const volatile uint32_t start = sys_time.get_ms();
    while((sys_time.get_ms() - start) < ms);
}

void vTask1(void *pvParameters)
{
    (void)pvParameters;
    for(;;)
    {
        Board::USR0.toggle();
        vTaskDelay(1250);
        RTT_LOG_I(TAG, "USR2.LED toggle!");
    }
}

void vTask2(void *pvParameters)
{
    (void)pvParameters;
    for(;;)
    {
        Board::USR1.toggle();
        vTaskDelay(750);
        RTT_LOG_I(TAG, "USR1.LED toggle!");
    }
}

void vPerfBenchmarkTask(void *pvParameters)
{
    (void)pvParameters;

    // 1. Настройка PMU на отслеживание промахов Data TLB и сбоев предсказателя переходов
    HAL::PERF::configure_event(HAL::PERF::Counter::COUNTER_0, HAL::PERF::EventType::L1D_TLB_REFILL);
    HAL::PERF::configure_event(HAL::PERF::Counter::COUNTER_1, HAL::PERF::EventType::BRANCH_MISPRED);

    // 2. Запуск синтетического теста производительности оперативной памяти
    HAL::PERF::run_ddr_benchmark();

    for(;;)
    {
        {
            HAL::PERF::ScopedProfiler prof("EDMA_EXEC");

            edma_test();
        }

        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

int main ()
{
    bool init_sts = false;

    init_sts = init_board();

    if (!init_sts)
    {
        RTT_LOG_E(TAG, "Board initialization failed!");
        while (true){}
    }
    RTT_LOG_I(TAG, "Board initialization done!");

    xTaskCreate(vPerfBenchmarkTask, "PerfTask", 8192, NULL, 2, NULL);
    xTaskCreate(vTask1, "Task1", 512, NULL, 1, NULL);
    xTaskCreate(vTask2, "Task2", 512, NULL, 1, NULL);

    vTaskStartScheduler();

    for(;;){} // Should never reach here

    return(0);
}

