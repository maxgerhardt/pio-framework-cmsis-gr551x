#include "gr_soc.h"
#include "gr55xx.h"
#include <custom_config.h>

// NOTE: SVC #0 is reserved for freertos, DO NOT USE IT!
#define SVC_TABLE_NUM_MAX           4

#define REG_PL_WR(addr, value)      (*(volatile uint32_t *)(addr)) = (value)
#define REG_PL_RD(addr)             (*(volatile uint32_t *)(addr))

volatile uint32_t g_app_msp_addr;   /* record app msp address */
static uint32_t SVC_TABLE_USER_SPACE[SVC_TABLE_NUM_MAX] __attribute__((section("SVC_TABLE")));
void svc_table_register(uint32_t *p_svc_table);

#if (CFG_LCP_SUPPORT && (CHIP_TYPE <= 5))
extern uint16_t gdx_lcp_buf_init(uint32_t buf_addr);
static uint8_t lcp_buf[280] __attribute__((section (".ARM.__at_0x00820000"), zero_init));
#endif

SECTION_RAM_CODE void __attribute__((naked))SVC_Handler(void)
{
    __asm("TST R14,$4\n");
    __asm("IT NE\n");
    __asm("MRSNE   R12,PSP\n");
    __asm("IT EQ\n");
    __asm("MOVEQ   R12,SP\n");
    __asm("PUSH    {R0-R3,LR}\n");
    __asm("MOV  R0, R12\n");
    __asm("BL  SVC_handler_proc\n");
    __asm("MOV  R12, R0\n");
    __asm("POP {R0-R3,LR}\n");
    __asm("CMP R12,$0\n");
    __asm("IT NE\n");
    __asm("BLXNE     R12\n");
    __asm("BX      LR\n");
}

__ALIGNED(0x100) FuncVector_t FuncVector_table[MAX_NUMS_IRQn + NVIC_USER_IRQ_OFFSET] = {
    0,
    Reset_Handler,
    NMI_Handler,
    HardFault_Handler,
    MemManage_Handler,
    BusFault_Handler,
    UsageFault_Handler,
    0,
    0,
    0,
    0,
    SVC_Handler,
    DebugMon_Handler,
    0,
    PendSV_Handler,
    SysTick_Handler,
};

void soc_register_nvic(IRQn_Type indx, uint32_t func)
{
    FuncVector_table[indx + 16] = (FuncVector_t)func;
}

void sdk_init() {}

void platform_init(void)
{
    svc_table_register(SVC_TABLE_USER_SPACE);
}

void vector_table_init(void)
{
    __DMB(); // Data Memory Barrier
    FuncVector_table[0] = *(FuncVector_t *)(SCB->VTOR);
    SCB->VTOR = (uint32_t)FuncVector_table; // Set VTOR to the new vector table location
    __DSB(); // Data Synchronization Barrier to ensure all
}

void warm_boot_process(void)
{
    vector_table_init();
    //pwr_mgmt_warm_boot();
}

#define SOFTWARE_REG_WAKEUP_FLAG_POS   (8)
uint32_t get_wakeup_flag(void)
{
    return (AON->SOFTWARE_2 & (1 << SOFTWARE_REG_WAKEUP_FLAG_POS));
}

void soc_init(void)
{
    platform_init();

    /* record app msp */
    g_app_msp_addr = REG_PL_RD(APP_CODE_RUN_ADDR);

}

/*----------------------------------------------------------------------------
  System Core Clock Variable
 *----------------------------------------------------------------------------*/
static const uint32_t systemClock[CLK_TYPE_NUM] = {
                                        CLK_64M, /*CPLL_S64M_CLK*/
                                        CLK_48M, /*CPLL_F48M_CLK*/
                                        CLK_16M, /*XO_S16M_CLK*/
                                        CLK_24M, /*CPLL_T24M_CLK*/
                                        CLK_16M, /*CPLL_S16M_CLK*/
                                        CLK_32M, /*CPLL_T32M_CLK*/
                                        };

// xqspi clock table by sys_clk_type
const uint32_t mcu_clk_2_qspi_clk[CLK_TYPE_NUM] = {
                                        [CPLL_S64M_CLK] = QSPI_64M_CLK,
                                        [CPLL_F48M_CLK] = QSPI_48M_CLK,
                                        [CPLL_T32M_CLK] = QSPI_32M_CLK,
                                        [CPLL_T24M_CLK] = QSPI_24M_CLK,
                                        [CPLL_S16M_CLK] = QSPI_16M_CLK,
                                        [XO_S16M_CLK] = QSPI_16M_CLK,
                                        };

uint32_t SystemCoreClock = CLK_64M;  /* System Core Clock Frequency as 64Mhz     */

//lint -e{2,10,48,63}
//The previous line of comment is to inhibit PC-Lint errors for next code block.
void SystemCoreSetClock(mcu_clock_type_t clock_type)
{
    if (clock_type >= CLK_TYPE_NUM)
        return;        // input parameter is out of range

    if ((AON->PWR_RET01 & AON_PWR_REG01_SYS_CLK_SEL) != clock_type)
    {
        uint32_t temp = AON->PWR_RET01 & (~(AON_PWR_REG01_SYS_CLK_SEL | AON_PWR_REG01_XF_SCK_CLK_SEL));
        //When a 16M or 64M clock is switched to another clock, it needs to be switched to 32M first.
        AON->PWR_RET01 = (temp | (CPLL_T32M_CLK << AON_PWR_REG01_SYS_CLK_SEL_Pos) | (QSPI_32M_CLK << AON_PWR_REG01_XF_SCK_CLK_SEL_Pos));

        __asm ("nop;nop;nop;nop;");
        temp = AON->PWR_RET01 & (~(AON_PWR_REG01_SYS_CLK_SEL | AON_PWR_REG01_XF_SCK_CLK_SEL));
        AON->PWR_RET01 = (temp | (clock_type << AON_PWR_REG01_SYS_CLK_SEL_Pos) | (mcu_clk_2_qspi_clk[clock_type] << AON_PWR_REG01_XF_SCK_CLK_SEL_Pos));
    }

    SystemCoreClock = systemClock[clock_type];

    return;
}

void SystemCoreGetClock(mcu_clock_type_t *clock_type)
{
    *clock_type = (mcu_clock_type_t)(AON->PWR_RET01 & AON_PWR_REG01_SYS_CLK_SEL);
}

void SystemCoreUpdateClock(void)
{
    SystemCoreClock  = systemClock[AON->PWR_RET01 & AON_PWR_REG01_SYS_CLK_SEL];
}

void set_msp()
{
    #ifndef DRIVER_TEST
    #ifdef APP_CODE_RUN_ADDR
    __DMB();
     __set_MSP(REG_PL_RD(APP_CODE_RUN_ADDR));
    __DSB();
    #endif
    #endif
}

