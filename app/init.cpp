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

#define TAG "brd_ini"

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

static void mpu_pll_init();
static void core_pll_init();
static void per_pll_init();
static void interface_clocks_init();

extern "C" void vSetupTickInterrupt(void) { HAL::TIMERS::sys_time.init(); }

/*
 * Обработчик тика FreeRTOS
 * Вызывается из IRQ обработчика когда срабатывает DMTIMER1ms
 */
extern "C" void vApplicationTickHook(void)
{
    // Этот хук вызывается из timer ISR если configUSE_TICK_HOOK = 1
}

extern "C" void FreeRTOS_Tick_Handler(void);

extern "C" void vApplicationIRQHandler(void)
{
    using namespace REGS::INTC;
    using namespace REGS::DMTIMER;
    // Получить номер активного IRQ из INTC
    volatile uint32_t *sir_irq = (volatile uint32_t *)0x48200040;
    uint32_t irq_num = *sir_irq & 0x7F;

    switch(irq_num)
    {
    case TINT1_1MS:
        HAL::TIMERS::sys_time.IRQ_disable(IRQ_OVF); // Disable the DMTimer interrupts
        HAL::TIMERS::sys_time.IRQ_clear(IRQ_OVF);

        HAL::TIMERS::sys_time.increment();

        FreeRTOS_Tick_Handler();
        HAL::TIMERS::sys_time.IRQ_enable(IRQ_OVF);  // Enable the DM_Timer interrupts
        HAL::TIMERS::sys_time.sys_interrupt_enable();
        break;
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

    rtt_log_init();
    RTT_LOG_I(TAG, "=== AM335x FreeRTOS application starting ===");
    rtt_cache_clean();

    init_memory();

    mpu_pll_init();
    core_pll_init();
    per_pll_init();
    interface_clocks_init();

    HAL::INTC::init();              //Initializing the ARM Interrupt Controller.

    Board::init_user_leds();

    Board::get_uart0().init(input_callback);

    Board::get_uart0().put_string((char *)"\r\n Application started... \r\n");
    Board::get_uart0().put_string((char *)"UART0 initialized... \r\n");

    return true;
}

void input_callback(char c)
{
    Board::get_uart0().put_char(c);
}

static void mpu_pll_init()
{
    using namespace REGS::PRCM;
    auto& wkup = *AM335x_CM_WKUP;

    // Switch dpll mpu to bypass mode and  wait for bypass status
    wkup.CLKMODE_DPLL_MPU.reg = DPLL_MNBYPASS;
    while (wkup.IDLEST_DPLL_MPU.b.ST_MN_BYPASS == 0){}

    // configure divider and multipler
    // DPLL_MULT = 1000, DPLL_DIV = 23 (actual division factor is N+1)
    // 24MHz*1000/24 = 1GHz
    wkup.CLKSEL_DPLL_MPU.reg = (1000 << 8) | (23);

    wkup.DIV_M2_DPLL_MPU.b.DPLL_CLKOUT_DIV = 0x0;
    wkup.DIV_M2_DPLL_MPU.b.DPLL_CLKOUT_DIV |= 0x1;

    // Lock dpll mpu and  wait locking status
    wkup.CLKMODE_DPLL_MPU.reg = DPLL_LOCKMODE;
    while (wkup.IDLEST_DPLL_MPU.b.DPLL == 0){}
}

// Core PLL Configuration based on AM335x TRM 8.1.6.7.1
// All values based on AM335x TRM Table 8-22 Core PLL Typical Frequencies OPP100
// clock source is 24MHz crystal on OSC0-IN (BBB schematic page 3)
static void core_pll_init()
{
    using namespace REGS::PRCM;
    auto& wkup = *AM335x_CM_WKUP;

    // Switch dpll core to bypass mode and wait to baypass status
    wkup.CLKMODE_DPLL_CORE.b.DPLL_EN = DPLL_MNBYPASS;
    while (wkup.IDLEST_DPLL_CORE.b.ST_MN_BYPASS == 0){}

    // configure divider and multiplier
    // DPLL_MULT = 500, DPLL_DIV = 23 (actual division factor is N+1)
    // 24MHz*500/24 = 500 MHz
    wkup.CLKSEL_DPLL_CORE.reg = (500 << 8) | (23);

    // Set M4,M5,M6 dividers
    // Set M4,M5,M6 diveders
    wkup.DIV_M4_DPLL_CORE.b.HSDIVIDER_CLKOUT1_DIV = 0x0;
    wkup.DIV_M4_DPLL_CORE.b.HSDIVIDER_CLKOUT1_DIV |= 0x10;
    wkup.DIV_M5_DPLL_CORE.b.HSDIVIDER_CLKOUT2_DIV = 0x0;
    wkup.DIV_M5_DPLL_CORE.b.HSDIVIDER_CLKOUT2_DIV |= 0x8;
    wkup.DIV_M6_DPLL_CORE.b.HSDIVIDER_CLKOUT3_DIV = 0x0;
    wkup.DIV_M6_DPLL_CORE.b.HSDIVIDER_CLKOUT3_DIV |= 0x4;

    // Lock dpll core and wait locking status
    wkup.CLKMODE_DPLL_CORE.b.DPLL_EN = DPLL_LOCKMODE;
    while (wkup.IDLEST_DPLL_CORE.b.ST_DPLL_CLK == 0){}
}

// PER PLL Configuration based on AM335x TRM 8.1.6.8.1
// All values based on AM335x TRM Table 8-24 PER PLL Typical Frequencies OPP100
// clock source is 24MHz crystal on OSC0-IN (BBB schematic page 3)
static void per_pll_init()
{
    using namespace REGS::PRCM;
    auto& wkup = *AM335x_CM_WKUP;

    // Switch dpll per to bypas mode and wait bypass status
    wkup.CLKMODE_DPLL_PER.b.DPLL_EN = PER_MNBYPASS;
    while (wkup.IDLEST_DPLL_PER.b.ST_MN_BYPASS == 0){}

    // configure divider and multipler
    // DPLL_MULT = 960, DPLL_DIV = 23 (actual division factor is N+1)
    // 24MHz*960/24 = 960MHz
    wkup.CLKSEL_DPLL_PERIPH.reg = (960 << 8) | (23);

    wkup.DIV_M2_DPLL_PER.b.DPLL_CLKOUT_DIV = 0x0;
    wkup.DIV_M2_DPLL_PER.b.DPLL_CLKOUT_DIV |= 0x5;

    // Lock dpll per and wait locking status
    wkup.CLKMODE_DPLL_PER.b.DPLL_EN = PER_LOCKMODE;
    while (wkup.IDLEST_DPLL_PER.b.ST_DPLL_CLK == 0){}
}

static void interface_clocks_init()
{
    using namespace REGS::PRCM;
    auto& per = *AM335x_CM_PER;
    auto& wkup = *AM335x_CM_WKUP;

    wkup.CONTROL_CLKCTRL.b.MODULEMODE = MODULEMODE_ENABLE;
    per.L4LS_CLKCTRL.b.MODULEMODE = MODULEMODE_ENABLE;
    per.L3_CLKCTRL.b.MODULEMODE = MODULEMODE_ENABLE;
    wkup.CLKSTCTRL.b.CLKTRCTRL = SW_WKUP;
    per.L4LS_CLKSTCTRL.b.CLKTRCTRL = SW_WKUP;
    per.L3S_CLKSTCTRL.b.CLKTRCTRL = SW_WKUP;
}

