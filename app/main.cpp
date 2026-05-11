#include <stdint.h>
#include <stdbool.h>
#include "init.h"
#include "rtt/rtt_log.h"
#include "hal/boards/beaglebone_black.hpp"
#include "hal/sysTimer.hpp"
#include "FreeRTOS.h"
#include "task.h"

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
        vTaskDelay(250);
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

    xTaskCreate(vTask1, "Task1", 512, NULL, 1, NULL);
    xTaskCreate(vTask2, "Task2", 512, NULL, 1, NULL);

    vTaskStartScheduler();

    for(;;){} // Should never reach here

    return(0);
}