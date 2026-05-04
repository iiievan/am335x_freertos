#include <stdint.h>
#include <stdbool.h>
#include "init.h"
#include "rtt/rtt_log.h"
#include "hal/boards/beaglebone_black.hpp"
#include "hal/sysTimer.hpp"


#define TAG "main"

void delay_ms(const uint32_t ms)
{
    using namespace HAL::TIMERS;
    const volatile uint32_t start = sys_time.get_ms();
    while((sys_time.get_ms() - start) < ms);
}

int main ()
{
    bool init_sts = false;

    init_sts = init_board();

    if (!init_sts)
    {
        RTT_LOG_E(TAG, "Board initialization failed!");
        while (1);
    }

    RTT_LOG_I(TAG, "Board initialization seccess!");

    uint8_t counter = 0;
    while(true)
    {
        delay_ms(300);
        switch (counter)
        {
            case 0:
                Board::USR0.toggle();
                counter++;
                break;
            case 1:
                Board::USR1.toggle();
                counter++;
                break;
            case 2:
                Board::USR2.toggle();
                counter++;
                break;
            case 3:
                Board::USR3.toggle();
                counter++;
                break;
            default:
                counter = 0;
                break;
        }
    }

    return(0);
}