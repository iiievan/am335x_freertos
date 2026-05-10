/*=======================================================================*/
/*  Includes                                                             */
/*=======================================================================*/
#include "init.h"
#include "startup/cp15.h"
#include "regs/REGS.hpp"
#include "rtt/rtt_log.h"
#include "hal/INTC.hpp"
#include "hal/sysTimer.hpp"
#include "hal/boards/beaglebone_black.hpp"
#include "hal/MMU.hpp"


#define DDR_TEST_SIZE            (32 * 1024 * 1024)
#define TAG "brd_ini"

#define DLY_100US    (10160)  //11830

extern "C"
{
    void Entry(void);
    void UndefInstHandler(void);
    void FreeRTOS_SWI_Handler(void);
    void AbortHandler(void);
    void FreeRTOS_IRQ_Handler(void);
    void FIQHandler(void);
}

static uint32_t const vec_tbl[14] =
{
    0xE59FF018,    /* Opcode for loading PC with the contents of [PC + 0x18] */
    0xE59FF018,    /* Opcode for loading PC with the contents of [PC + 0x18] */
    0xE59FF018,    /* Opcode for loading PC with the contents of [PC + 0x18] */
    0xE59FF018,    /* Opcode for loading PC with the contents of [PC + 0x18] */
    0xE59FF014,    /* Opcode for loading PC with the contents of [PC + 0x14] */
    0xE24FF008,    /* Opcode for loading PC with (PC - 8) (eq. to while(1)) */
    0xE59FF010,    /* Opcode for loading PC with the contents of [PC + 0x10] */
    0xE59FF010,    /* Opcode for loading PC with the contents of [PC + 0x10] */
    (uint32_t)Entry,
    (uint32_t)UndefInstHandler,
    (uint32_t)FreeRTOS_SWI_Handler,
    (uint32_t)AbortHandler,
    (uint32_t)FreeRTOS_IRQ_Handler,
    (uint32_t)FIQHandler
};

extern HAL::TIMERS::sysTimer<SYST_t> sys_time;

extern "C" void vSetupTickInterrupt(void)
{
    // Системный таймер уже инициализирован в init_board()
    // Просто убеждаемся что он запущен
    // Таймер настроен на 1ms период (1000 Hz)

    // Если нужно переконфигурировать:
    // sys_time.set_period(1000 / configTICK_RATE_HZ); // для 1000Hz тика
}

/*
 * Обработчик тика FreeRTOS
 * Вызывается из IRQ обработчика когда срабатывает DMTIMER1ms
 */
extern "C" void vApplicationTickHook(void)
{
    // Этот хук вызывается из timer ISR если configUSE_TICK_HOOK = 1
}

extern "C" void FreeRTOS_Tick_Handler(void);
// В input_callback или новом IRQ обработчике:
extern "C" void vApplicationIRQHandler(void)
{
    // Получить номер активного IRQ из INTC
    volatile uint32_t *sir_irq = (volatile uint32_t *)0x48200040;
    uint32_t irq_num = *sir_irq & 0x7F;

    // Обработка разных прерываний
    switch(irq_num) {
    case 68: // DMTIMER1MS IRQ номер для AM335x
        // Очистить флаг прерывания таймера
        // Вызвать тик FreeRTOS
        FreeRTOS_Tick_Handler();
        break;

        // Другие прерывания...
    }

    // Очистить прерывание в INTC
    volatile uint32_t *control = (volatile uint32_t *)0x48200048;
    *control = 0x1; // NEWIRQAGR
}

static void copy_vector_table()
{
    auto *dest = reinterpret_cast<uint32_t*>(AM335X_VECTOR_BASE);
    auto *src  = const_cast<uint32_t*>(vec_tbl);

    cp15_vector_base_addr_set(AM335X_VECTOR_BASE);

    for(uint32_t count = 0; count < sizeof(vec_tbl)/sizeof(vec_tbl[0]); count++)
    {
        dest[count] = src[count];
    }
}

static void rtt_cache_clean()
{
    // Очищаем и инвалидируем кэш для RTT области
    // RTT область: 0x40300000 - 0x40310000 (64KB)
    cp15_D_cache_clean_flush_buff(0x40300000, 0x10000);
    cp15_I_cache_flush_buff(0x40300000, 0x10000);
    cp15_DSB_ISB_sync_barrier();
}

static void input_callback(char c);

bool init_board()
{
    copy_vector_table();

    init_memory();

    //rtt_log_init();
    //RTT_LOG_I(TAG, "=== AM335x Boot Loader Starting ===");
    //rtt_cache_clean();

    HAL::INTC::init();              //Initializing the ARM Interrupt Controller.
    HAL::TIMERS::sys_time.init();   // setup system timer for 1ms interrupt

    Board::init_user_leds();

    Board::get_uart0().init(input_callback);

    HAL::INTC::master_IRQ_enable();

    Board::get_uart0().put_string((char *)"\r\n Application started... \r\n");
    Board::get_uart0().put_string((char *)"UART0 initialized... \r\n");
    //RTT_LOG_I(TAG, "Application started successful!");

    return true;
}

void input_callback(char c)
{
    Board::get_uart0().put_char(c);
}

