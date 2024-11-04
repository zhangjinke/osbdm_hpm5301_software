/**
 * \file
 * \brief 板级配置
 *
 * \internal
 * \par Modification history
 * - 1.00 24-10-14  peace, first implementation
 * \endinternal
 */

/*******************************************************************************
  头文件包含
*******************************************************************************/

#include "board_custom.h"

#include "hpm_adc16_drv.h"
#include "hpm_clock_drv.h"
#include "hpm_gpio_drv.h"
#include "hpm_gpiom_drv.h"
#include "hpm_gptmr_drv.h"
#include "hpm_pcfg_drv.h"
#include "hpm_pllctlv2_drv.h"
#include "hpm_usb_drv.h"

#include <stdio.h>

/*******************************************************************************
  宏定义
*******************************************************************************/

/*******************************************************************************
  本地全局变量声明
*******************************************************************************/

/*******************************************************************************
  本地函数声明
*******************************************************************************/

/*******************************************************************************
  本地全局变量定义
*******************************************************************************/

/** \brief VIO PWM 重装载值 */
static uint32_t _g_vio_pwm_reload = 0;

/*******************************************************************************
  本地函数定义
*******************************************************************************/

/**
 * \brief USB 引脚初始化
 */
static void _usb_dp_dm_pins_init (void)
{
    uint8_t tmp;

    /* Disconnect usb dp/dm pins pull down 45ohm resistance */

    while (sysctl_resource_any_is_busy(HPM_SYSCTL)) {}

    if (pllctlv2_xtal_is_stable(HPM_PLLCTLV2) && pllctlv2_xtal_is_enabled(HPM_PLLCTLV2)) {
        if (clock_check_in_group(clock_usb0, 0)) {
            usb_phy_disable_dp_dm_pulldown(HPM_USB0);
        } else {
            clock_add_to_group(clock_usb0, 0);
            usb_phy_disable_dp_dm_pulldown(HPM_USB0);
            clock_remove_from_group(clock_usb0, 0);
        }
    } else {
        tmp = sysctl_resource_target_get_mode(HPM_SYSCTL, sysctl_resource_xtal);
        sysctl_resource_target_set_mode(HPM_SYSCTL, sysctl_resource_xtal, 0x03);
        clock_add_to_group(clock_usb0, 0);
        usb_phy_disable_dp_dm_pulldown(HPM_USB0);
        clock_remove_from_group(clock_usb0, 0);
        while (sysctl_resource_target_is_busy(HPM_SYSCTL, sysctl_resource_usb0)) {}
        sysctl_resource_target_set_mode(HPM_SYSCTL, sysctl_resource_xtal, tmp);
    }
}

/**
 * \brief 时钟初始化
 */
static void _clk_init (void)
{
    uint32_t cpu0_freq = clock_get_frequency(clock_cpu0);

    if (cpu0_freq == PLLCTL_SOC_PLL_REFCLK_FREQ) {
        /* Configure the External OSC ramp-up time: ~9ms */
        pllctlv2_xtal_set_rampup_time(HPM_PLLCTLV2, 32UL * 1000UL * 9U);

        /* Select clock setting preset1 */
        sysctl_clock_set_preset(HPM_SYSCTL, 2);
    }

    /* group0[0] */
    clock_add_to_group(clock_cpu0, 0);
    clock_add_to_group(clock_ahb, 0);
    clock_add_to_group(clock_lmm0, 0);
    clock_add_to_group(clock_mchtmr0, 0);
    clock_add_to_group(clock_rom, 0);
    clock_add_to_group(clock_gptmr0, 0);
    clock_add_to_group(clock_gptmr1, 0);
    clock_add_to_group(clock_i2c2, 0);
    clock_add_to_group(clock_spi1, 0);
    clock_add_to_group(clock_uart0, 0);
    clock_add_to_group(clock_uart3, 0);

    clock_add_to_group(clock_watchdog0, 0);
    clock_add_to_group(clock_watchdog1, 0);
    clock_add_to_group(clock_mbx0, 0);
    clock_add_to_group(clock_tsns, 0);
    clock_add_to_group(clock_crc0, 0);
    clock_add_to_group(clock_adc0, 0);
    clock_add_to_group(clock_acmp, 0);
    clock_add_to_group(clock_kman, 0);
    clock_add_to_group(clock_gpio, 0);
    clock_add_to_group(clock_hdma, 0);
    clock_add_to_group(clock_xpi0, 0);
    clock_add_to_group(clock_usb0, 0);

    /* Connect Group0 to CPU0 */
    clock_connect_group_to_cpu(0, 0);

    /* Bump up DCDC voltage to 1175mv */
    pcfg_dcdc_set_voltage(HPM_PCFG, 1175);

    /* Configure CPU to 360MHz, AXI/AHB to 120MHz */
    sysctl_config_cpu0_domain_clock(HPM_SYSCTL, clock_source_pll0_clk0, 2, 3);

    /* Configure PLL0 Post Divider */
    pllctlv2_set_postdiv(HPM_PLLCTLV2, 0, 0, 0); /* PLL0CLK0: 720MHz */
    pllctlv2_set_postdiv(HPM_PLLCTLV2, 0, 1, 3); /* PLL0CLK1: 450MHz */
    pllctlv2_set_postdiv(HPM_PLLCTLV2, 0, 2, 7); /* PLL0CLK2: 300MHz */

    /* Configure PLL0 Frequency to 720MHz */
    pllctlv2_init_pll_with_freq(HPM_PLLCTLV2, 0, 720000000);

    clock_update_core_clock();

    /* Configure mchtmr to 24MHz */
    clock_set_source_divider(clock_mchtmr0, clk_src_osc24m, 1);
}

/**
 * \brief 打印时钟频率
 */
static void _clk_freq_print (void)
{
    printf("==============================\n");
    printf(" %s clock summary\n", BOARD_NAME);
    printf("==============================\n");
    printf("cpu0:\t\t %luHz\n", clock_get_frequency(clock_cpu0));
    printf("ahb:\t\t %luHz\n", clock_get_frequency(clock_ahb));
    printf("mchtmr0:\t %luHz\n", clock_get_frequency(clock_mchtmr0));
    printf("xpi0:\t\t %luHz\n", clock_get_frequency(clock_xpi0));
    printf("==============================\n");
}

/**
 * \brief 引脚初始化
 */
static void _pin_init (void)
{
    /* PA00 PWR */
    HPM_IOC->PAD[IOC_PAD_PA00].FUNC_CTL = IOC_PA00_FUNC_CTL_GPIO_A_00;
    gpiom_set_pin_controller(HPM_GPIOM, PWR_GPIO_INDEX, PWR_GPIO_PIN, gpiom_core0_fast);
    gpio_write_pin(PWR_GPIO_CTRL, PWR_GPIO_INDEX, PWR_GPIO_PIN, 1);
    gpio_set_pin_output(PWR_GPIO_CTRL, PWR_GPIO_INDEX, PWR_GPIO_PIN);

    /* PA01 TRST_DIR */
    HPM_IOC->PAD[IOC_PAD_PA01].FUNC_CTL = IOC_PA01_FUNC_CTL_GPIO_A_01;
    gpiom_set_pin_controller(HPM_GPIOM, TRST_DIR_GPIO_INDEX, TRST_DIR_GPIO_PIN, gpiom_core0_fast);
    gpio_write_pin(TRST_DIR_GPIO_CTRL, TRST_DIR_GPIO_INDEX, TRST_DIR_GPIO_PIN, 1);
    gpio_set_pin_output(TRST_DIR_GPIO_CTRL, TRST_DIR_GPIO_INDEX, TRST_DIR_GPIO_PIN);

    /* PA02 TRST */
    HPM_IOC->PAD[IOC_PAD_PA02].FUNC_CTL = IOC_PA02_FUNC_CTL_GPIO_A_02;
    gpiom_set_pin_controller(HPM_GPIOM, TRST_GPIO_INDEX, TRST_GPIO_PIN, gpiom_core0_fast);
    gpio_write_pin(TRST_GPIO_CTRL, TRST_GPIO_INDEX, TRST_GPIO_PIN, 1);
    gpio_set_pin_output(TRST_GPIO_CTRL, TRST_GPIO_INDEX, TRST_GPIO_PIN);

    /* PA03 TDI_DIR */
    HPM_IOC->PAD[IOC_PAD_PA03].FUNC_CTL = IOC_PA03_FUNC_CTL_GPIO_A_03;
    gpiom_set_pin_controller(HPM_GPIOM, TDI_DIR_GPIO_INDEX, TDI_DIR_GPIO_PIN, gpiom_core0_fast);
    gpio_write_pin(TDI_DIR_GPIO_CTRL, TDI_DIR_GPIO_INDEX, TDI_DIR_GPIO_PIN, 1);
    gpio_set_pin_output(TDI_DIR_GPIO_CTRL, TDI_DIR_GPIO_INDEX, TDI_DIR_GPIO_PIN);

    /* PA08 LED_GREEN */
    HPM_IOC->PAD[IOC_PAD_PA08].FUNC_CTL = IOC_PA08_FUNC_CTL_GPIO_A_08;
    gpiom_set_pin_controller(HPM_GPIOM, LED_GREEN_GPIO_INDEX, LED_GREEN_GPIO_PIN, gpiom_core0_fast);
    gpio_write_pin(LED_GREEN_GPIO_CTRL, LED_GREEN_GPIO_INDEX, LED_GREEN_GPIO_PIN, 1);
    gpio_set_pin_output(LED_GREEN_GPIO_CTRL, LED_GREEN_GPIO_INDEX, LED_GREEN_GPIO_PIN);

    /* PA09 LED_RED */
    HPM_IOC->PAD[IOC_PAD_PA09].FUNC_CTL = IOC_PA09_FUNC_CTL_GPIO_A_09;
    gpiom_set_pin_controller(HPM_GPIOM, LED_RED_GPIO_INDEX, LED_RED_GPIO_PIN, gpiom_core0_fast);
    gpio_write_pin(LED_RED_GPIO_CTRL, LED_RED_GPIO_INDEX, LED_RED_GPIO_PIN, 1);
    gpio_set_pin_output(LED_RED_GPIO_CTRL, LED_RED_GPIO_INDEX, LED_RED_GPIO_PIN);

    /* PA10 VREF_PWM */
    HPM_IOC->PAD[IOC_PAD_PA10].FUNC_CTL = IOC_PA10_FUNC_CTL_GPTMR0_COMP_2;

    HPM_IOC->PAD[IOC_PAD_PA24].FUNC_CTL = IOC_PAD_FUNC_CTL_ANALOG_MASK;
    HPM_IOC->PAD[IOC_PAD_PA25].FUNC_CTL = IOC_PAD_FUNC_CTL_ANALOG_MASK;

    /* PA27 TCK */
    HPM_IOC->PAD[IOC_PAD_PA27].FUNC_CTL = IOC_PA27_FUNC_CTL_GPIO_A_27;
    gpiom_set_pin_controller(HPM_GPIOM, TCK_GPIO_INDEX, TCK_GPIO_PIN, gpiom_core0_fast);
    gpio_write_pin(TCK_GPIO_CTRL, TCK_GPIO_INDEX, TCK_GPIO_PIN, 0);
    gpio_set_pin_output(TCK_GPIO_CTRL, TCK_GPIO_INDEX, TCK_GPIO_PIN);

    /* PA28 TMS */
    HPM_IOC->PAD[IOC_PAD_PA28].FUNC_CTL = IOC_PA28_FUNC_CTL_GPIO_A_28;
    gpiom_set_pin_controller(HPM_GPIOM, TMS_GPIO_INDEX, TMS_GPIO_PIN, gpiom_core0_fast);
    gpio_write_pin(TMS_GPIO_CTRL, TMS_GPIO_INDEX, TMS_GPIO_PIN, 0);
    gpio_set_pin_output(TMS_GPIO_CTRL, TMS_GPIO_INDEX, TMS_GPIO_PIN);

    /* PA30 TMS_DIR */
    HPM_IOC->PAD[IOC_PAD_PA30].FUNC_CTL = IOC_PA30_FUNC_CTL_GPIO_A_30;
    gpiom_set_pin_controller(HPM_GPIOM, TMS_DIR_GPIO_INDEX, TMS_DIR_GPIO_PIN, gpiom_core0_fast);
    gpio_write_pin(TMS_DIR_GPIO_CTRL, TMS_DIR_GPIO_INDEX, TMS_DIR_GPIO_PIN, 1);
    gpio_set_pin_output(TMS_DIR_GPIO_CTRL, TMS_DIR_GPIO_INDEX, TMS_DIR_GPIO_PIN);

    /* PA31 SRST_OUT */
    HPM_IOC->PAD[IOC_PAD_PA31].FUNC_CTL = IOC_PA31_FUNC_CTL_GPIO_A_31;
    gpiom_set_pin_controller(HPM_GPIOM, SRST_OUT_GPIO_INDEX, SRST_OUT_GPIO_PIN, gpiom_core0_fast);
    gpio_write_pin(SRST_OUT_GPIO_CTRL, SRST_OUT_GPIO_INDEX, SRST_OUT_GPIO_PIN, 0);
    gpio_set_pin_output(SRST_OUT_GPIO_CTRL, SRST_OUT_GPIO_INDEX, SRST_OUT_GPIO_PIN);

    /* PB08 ADC_VREF(ADC0_IN11) */
    HPM_IOC->PAD[IOC_PAD_PB08].FUNC_CTL = IOC_PAD_FUNC_CTL_ANALOG_MASK;

    /* PB09 SRST_DIR */
    HPM_IOC->PAD[IOC_PAD_PB09].FUNC_CTL = IOC_PB09_FUNC_CTL_GPIO_B_09;
    gpiom_set_pin_controller(HPM_GPIOM, SRST_DIR_GPIO_INDEX, SRST_DIR_GPIO_PIN, gpiom_core0_fast);
    gpio_write_pin(SRST_DIR_GPIO_CTRL, SRST_DIR_GPIO_INDEX, SRST_DIR_GPIO_PIN, 0);
    gpio_set_pin_output(SRST_DIR_GPIO_CTRL, SRST_DIR_GPIO_INDEX, SRST_DIR_GPIO_PIN);

    /* PB12 TDO */
    HPM_IOC->PAD[IOC_PAD_PB12].FUNC_CTL = IOC_PB12_FUNC_CTL_GPIO_B_12;
    gpiom_set_pin_controller(HPM_GPIOM, TDO_GPIO_INDEX, TDO_GPIO_PIN, gpiom_core0_fast);
    gpio_set_pin_input(TDO_GPIO_CTRL, TDO_GPIO_INDEX, TDO_GPIO_PIN);

    /* PB13 TDI */
    HPM_IOC->PAD[IOC_PAD_PB13].FUNC_CTL = IOC_PB13_FUNC_CTL_GPIO_B_13;
    gpiom_set_pin_controller(HPM_GPIOM, TDI_GPIO_INDEX, TDI_GPIO_PIN, gpiom_core0_fast);
    gpio_write_pin(TDI_GPIO_CTRL, TDI_GPIO_INDEX, TDI_GPIO_PIN, 0);
    gpio_set_pin_output(TDI_GPIO_CTRL, TDI_GPIO_INDEX, TDI_GPIO_PIN);

    /* PY00 SRST_IN */
    HPM_PIOC->PAD[IOC_PAD_PY00].FUNC_CTL = PIOC_PY00_FUNC_CTL_SOC_GPIO_Y_00;
    HPM_IOC->PAD[IOC_PAD_PY00].FUNC_CTL  = IOC_PY00_FUNC_CTL_GPIO_Y_00;
    gpiom_set_pin_controller(HPM_GPIOM, SRST_IN_GPIO_INDEX, SRST_IN_GPIO_PIN, gpiom_core0_fast);
    gpio_set_pin_input(SRST_IN_GPIO_CTRL, SRST_IN_GPIO_INDEX, SRST_IN_GPIO_PIN);

    /* PY01 TCK_DIR */
    HPM_PIOC->PAD[IOC_PAD_PY01].FUNC_CTL = PIOC_PY01_FUNC_CTL_SOC_GPIO_Y_01;
    HPM_IOC->PAD[IOC_PAD_PY01].FUNC_CTL  = IOC_PY01_FUNC_CTL_GPIO_Y_01;
    gpiom_set_pin_controller(HPM_GPIOM, TCK_DIR_GPIO_INDEX, TCK_DIR_GPIO_PIN, gpiom_core0_fast);
    gpio_write_pin(TCK_DIR_GPIO_CTRL, TCK_DIR_GPIO_INDEX, TCK_DIR_GPIO_PIN, 1);
    gpio_set_pin_output(TCK_DIR_GPIO_CTRL, TCK_DIR_GPIO_INDEX, TCK_DIR_GPIO_PIN);
}

/**
 * \brief ADC 初始化
 */
static void _adc_init (void)
{
    adc16_config_t         cfg;
    adc16_channel_config_t ch_cfg;

    /* 外设时钟使能 */
    clock_set_adc_source(clock_adc0, clk_adc_src_ahb0);
    printf("adc0:\t\t %luHz\n", clock_get_frequency(clock_adc0));

    /* 外设初始化 */
    adc16_get_default_config(&cfg);
    cfg.res          = adc16_res_16_bits;
    cfg.conv_mode    = adc16_conv_mode_oneshot;
    cfg.adc_clk_div  = adc16_clock_divider_4;
    cfg.sel_sync_ahb = true;
    cfg.adc_ahb_en   = false;
    adc16_init(HPM_ADC0, &cfg);

    /* 通道初始化 */
    adc16_get_channel_default_config(&ch_cfg);
    ch_cfg.ch           = 11;
    ch_cfg.sample_cycle = 20;
    adc16_init_channel(HPM_ADC0, &ch_cfg);

    /* 配置为非阻塞读取 */
    adc16_set_nonblocking_read(HPM_ADC0);

    /* 使能单次转换模式 */
    adc16_enable_oneshot_mode(HPM_ADC0);
}

/**
 * \brief PWM 初始化
 */
static void _pwm_init (void)
{
    gptmr_channel_config_t config;
    uint32_t               gptmr_freq;

    /* 配置定时器频率 */
    gptmr_channel_get_default_config(HPM_GPTMR0, &config);
    gptmr_freq                       = clock_get_frequency(clock_gptmr0);
    _g_vio_pwm_reload                = gptmr_freq / 100000;
    config.reload                    = _g_vio_pwm_reload;
    config.cmp_initial_polarity_high = false;
    gptmr_stop_counter(HPM_GPTMR0, 2);
    gptmr_channel_config(HPM_GPTMR0, 2, &config, false);
    gptmr_channel_reset_count(HPM_GPTMR0, 2);
    gptmr_start_counter(HPM_GPTMR0, 2);

    /* 配置 PWM 占空比 */
    vio_pwm_duty_set(VIO_PWM_DUTY_GET(0));
}

/**
 * \brief USB 初始化
 */
static void _usb_init (void)
{
    intc_set_irq_priority(IRQn_USB0, 1);

    usb_hcd_set_power_ctrl_polarity(HPM_USB0, true);
    usb_phy_using_internal_vbus(HPM_USB0);
}

/*******************************************************************************
  外部函数定义
*******************************************************************************/

/**
 * \brief 获取 VREF 电压
 */
int32_t vref_voltage_get (void)
{
    uint16_t result;
    int32_t  voltage = -1;

    if (adc16_get_oneshot_result(HPM_ADC0, 11, &result) == status_success) {
        adc16_get_oneshot_result(HPM_ADC0, 11, &result);
        voltage = (int32_t)(result * 3300 * 2 / ((1 << 16) - 1));
        if (voltage > 5000) {
            voltage = 5000;
        }
    }

    return voltage;
}

/**
 * \brief 设置 VIO PWM 占空比
 */
void vio_pwm_duty_set (uint32_t duty)
{
    uint32_t cmp;

    if (duty > (100 * 1000)) {
        duty = (100 * 1000);
    }

    cmp = ((_g_vio_pwm_reload * duty) / (100 * 1000)) + 1;
    gptmr_update_cmp(HPM_GPTMR0, 2, 0, cmp);
}

/**
 * \brief 板级初始化
 */
int32_t board_custom_init (void)
{
    _usb_dp_dm_pins_init();
    _clk_init();
    _clk_freq_print();
    _pin_init();
    _adc_init();
    _pwm_init();
    _usb_init();

    return 0;
}

/* end of file */