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


#define DDR_TEST_SIZE            (32 * 1024 * 1024)
#define TAG "brd_ini"

#define DLY_100US    (10160)  //11830

extern "C"
{
    void Entry(void);
    void UndefInstHandler(void);
    void SVC_Handler(void);
    void AbortHandler(void);
    void IRQHandler(void);
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
    (uint32_t)SVC_Handler,
    (uint32_t)AbortHandler,
    (uint32_t)IRQHandler,
    (uint32_t)FIQHandler
};

static void mpu_pll_init();
static void core_pll_init();
static void per_pll_init();
static void ddr_pll_init();
static void interface_clocks_init();
static void ddr_init();
static bool ddr_check();

extern HAL::TIMERS::sysTimer<SYST_t> sys_time;

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
    RTT_LOG_I(TAG, "=== AM335x Boot Loader Starting ===");
    cp15_MMU_disable();
    cp15_D_cache_disable();
    cp15_I_cache_disable();

    cp15_DSB_ISB_sync_barrier();

    rtt_cache_clean();

    //mpu_pll_init();
    //core_pll_init();
    //per_pll_init();
    //ddr_pll_init();
    //interface_clocks_init();

    HAL::INTC::init();              //Initializing the ARM Interrupt Controller.
    HAL::TIMERS::sys_time.init();   // setup system timer for 1ms interrupt

    Board::init_user_leds();

    Board::get_uart0().init(input_callback);

    HAL::INTC::master_IRQ_enable();

    Board::get_uart0().put_string((char *)"\r\n Application started... \r\n");
    Board::get_uart0().put_string((char *)"UART0 initialized... \r\n");
    RTT_LOG_I(TAG, "Application started successful!");

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

// DDR PLL Configuration based on AM335x TRM 8.1.6.11.1
// 400MHz based on Table 5-5 of AM335x datasheet DDR3L max frequency
// clock source is 24MHz crystal on OSC0-IN (BBB schematic page 3)
static void ddr_pll_init()
{
    using namespace REGS::PRCM;
    auto& wkup = *AM335x_CM_WKUP;

    // Switch dpll ddr to bypas mode and wait bypass status
    wkup.CLKMODE_DPLL_DDR.b.DPLL_EN = DPLL_MNBYPASS;
    while (wkup.IDLEST_DPLL_DDR.b.ST_MN_BYPASS == 0){}

    // configure divider and multipler
    // DPLL_MULT = 400, DPLL_DIV = 23 (actual division factor is N+1)
    // 24MHz*400/24 = 400MHz
    wkup.CLKSEL_DPLL_DDR.reg = (400 << 8) | (23);

    wkup.DIV_M2_DPLL_DDR.b.DPLL_CLKOUT_DIV = 0x0;
    wkup.DIV_M2_DPLL_DDR.b.DPLL_CLKOUT_DIV |= 0x1;

    // Lock dpll ddr and wait locking status
    wkup.CLKMODE_DPLL_DDR.b.DPLL_EN = DPLL_LOCKMODE;
    while (wkup.IDLEST_DPLL_DDR.b.ST_DPLL_CLK == 0){}
}

static void interface_clocks_init()
{
    using namespace REGS::PRCM;
    auto& per = *AM335x_CM_PER;
    auto& wkup = *AM335x_CM_WKUP;

    RTT_CHECK_MODULE_SIZE(AM335x_CM_PER_Type,0x150);
    RTT_CHECK_MODULE_SIZE(AM335x_CM_WKUP_Type,0xD8);

    wkup.CONTROL_CLKCTRL.b.MODULEMODE = MODULEMODE_ENABLE;
    per.L4LS_CLKCTRL.b.MODULEMODE = MODULEMODE_ENABLE;
    per.L3_CLKCTRL.b.MODULEMODE = MODULEMODE_ENABLE;
    wkup.CLKSTCTRL.b.CLKTRCTRL = SW_WKUP;
    per.L4LS_CLKSTCTRL.b.CLKTRCTRL = SW_WKUP;
    per.L3S_CLKSTCTRL.b.CLKTRCTRL = SW_WKUP;
}

static void ddr_init()
{
    using namespace REGS::CONTROL_MODULE;
    using namespace REGS::EMIF;
    using namespace REGS::PRCM;
    auto& per = *AM335x_CM_PER;
    auto& cm = *AM335x_CONTROL_MODULE;
    auto& emif = *AM335x_EMIF0;
    auto& phy = *AM335x_DDR23mPHY;

    RTT_CHECK_MODULE_SIZE(AM335x_CTRL_MODULE_Type, 0x1444);

    per.EMIF_CLKCTRL.reg = MODULEMODE_ENABLE;
    per.EMIF_FW_CLKCTRL.reg = MODULEMODE_ENABLE;

    while (!(per.L3_CLKSTCTRL.b.CLKACTIVITY_EMIF_GCLK == CLK_ACT &&
             per.L3_CLKSTCTRL.b.CLKACTIVITY_MMC_FCLK == CLK_ACT)) {}

    // Note beaglebone black does not have VTT termination
    // initialize virtual temperature process compensation
    cm.vtp_ctrl.reg |= 0x40;
    cm.vtp_ctrl.reg &= ~0x1;
    cm.vtp_ctrl.reg |= 0x1;

    while (cm.vtp_ctrl.b.ready != 0x1) {}

    phy.CMD0_CTRL_SLAVE_RATIO_0.reg = DDR3_CMD_SLAVE_RATIO;
    phy.CMD0_INVERT_CLKOUT_0.reg = DDR3_CMD_INVERT_CLKOUT; // Core clock not inverted
    phy.CMD1_CTRL_SLAVE_RATIO_0.reg = DDR3_CMD_SLAVE_RATIO;
    phy.CMD1_INVERT_CLKOUT_0.reg = DDR3_CMD_INVERT_CLKOUT; // Core clock not inverted
    phy.CMD2_CTRL_SLAVE_RATIO_0.reg = DDR3_CMD_SLAVE_RATIO;
    phy.CMD2_INVERT_CLKOUT_0.reg = DDR3_CMD_INVERT_CLKOUT; // Core clock not inverted

    phy.DATA0_RD_DQS_SLAVE_RATIO_0.reg   = DDR3_DATA0_RD_DQS_SLAVE_RATIO;
    phy.DATA0_WR_DQS_SLAVE_RATIO_0.reg   = DDR3_DATA0_WR_DQS_SLAVE_RATIO;
    phy.DATA0_FIFO_WE_SLAVE_RATIO_0.reg = DDR3_DATA0_FIFO_WE_SLAVE_RATIO;
    phy.DATA0_WR_DATA_SLAVE_RATIO_0.reg = DDR3_DATA0_WR_DATA_SLAVE_RATIO;
    phy.DATA1_RD_DQS_SLAVE_RATIO_0.reg  = DDR3_DATA0_RD_DQS_SLAVE_RATIO;
    phy.DATA1_WR_DQS_SLAVE_RATIO_0.reg   = DDR3_DATA0_WR_DQS_SLAVE_RATIO;
    phy.DATA1_FIFO_WE_SLAVE_RATIO_0.reg = DDR3_DATA0_FIFO_WE_SLAVE_RATIO;
    phy.DATA1_WR_DATA_SLAVE_RATIO_0.reg = DDR3_DATA0_WR_DATA_SLAVE_RATIO;

    cm.ddr_cmd0_ioctrl.reg = DDR3_IOCTRL_VALUE;
    cm.ddr_cmd1_ioctrl.reg = DDR3_IOCTRL_VALUE;
    cm.ddr_cmd2_ioctrl.reg = DDR3_IOCTRL_VALUE;
    cm.ddr_data0_ioctrl.reg = DDR3_IOCTRL_VALUE;
    cm.ddr_data1_ioctrl.reg = DDR3_IOCTRL_VALUE;

    cm.ddr_io_ctrl.reg &= ~0x10000000;;
    cm.ddr_cke_ctrl.reg |= 0x1;

    emif.DDR_PHY_CTRL_1.reg = DDR3_READ_LATENCY;
    emif.DDR_PHY_CTRL_1_SHDW.reg = DDR3_READ_LATENCY;
    emif.DDR_PHY_CTRL_2.reg = DDR3_READ_LATENCY;
    emif.SDRAM_TIM_1.reg = DDR3_SDRAM_TIMING1;
    emif.SDRAM_TIM_1_SHDW.reg = DDR3_SDRAM_TIMING1;
    emif.SDRAM_TIM_2.reg = DDR3_SDRAM_TIMING2;
    emif.SDRAM_TIM_2_SHDW.reg = DDR3_SDRAM_TIMING2;
    emif.SDRAM_TIM_3.reg = DDR3_SDRAM_TIMING3;
    emif.SDRAM_TIM_3_SHDW.reg = DDR3_SDRAM_TIMING3;
    emif.SDRAM_REF_CTRL.reg = DDR3_REF_CTRL;
    emif.SDRAM_REF_CTRL_SHDW.reg = DDR3_REF_CTRL;
    emif.ZQ_CONFIG.reg = DDR3_ZQ_CONFIG;
    emif.SDRAM_CONFIG.reg = DDR3_SDRAM_CONFIG;
}


// read and write to some addresses in DDR, returns 0 on sucess
static bool ddr_check()
{
    using namespace REGS::EMIF;

    cp15_D_cache_disable();
    cp15_I_cache_disable();
    cp15_TLB_invalidate();

    cp15_DSB_ISB_sync_barrier();
/*
    ddr_calib_values_t calib_values;

    if (ddr_calibrate(&calib_values))
    {
        RTT_LOG_I(TAG, "Calibration successful!");

        if (ddr_stress_test(100))
            RTT_LOG_I(TAG, "DDR fully initialized and stable!");
        else
        {
            RTT_LOG_W(TAG, "Stress test failed, but calibration values may still work");
            ddr_init();
        }
    }
    else
    {
        RTT_LOG_E(TAG, "Calibration failed! Using default values.");
        ddr_init();
    }
*/
    uint32_t i;
    volatile uint32_t* ddr = (uint32_t*)DDR_START;

    for (i = 0; i < DDR_TEST_SIZE / 4; i += 1024)
    {
        ddr[i] = 0x55555555;
    }

    cp15_DSB_barrier();
    for (i = 0; i < DDR_TEST_SIZE / 4; i += 1024)
    {
        if (ddr[i] != 0x55555555) return false;
    }


    for (i = 0; i < DDR_TEST_SIZE / 4; i += 1024)
    {
        ddr[i] = 0xAAAAAAAA;
    }

    cp15_DSB_barrier();
    for (i = 0; i < DDR_TEST_SIZE / 4; i += 1024)
    {
        if (ddr[i] != 0xAAAAAAAA) return false;
    }

    // Адресный тест
    for (i = 0; i < DDR_TEST_SIZE / 4; i += 1024)
    {
        ddr[i] = i;
    }

    cp15_DSB_barrier();
    for (i = 0; i < DDR_TEST_SIZE / 4; i += 1024)
    {
        if (ddr[i] != i) return false;
    }

    return true;
}