/**
 * \file
 * \brief OSBDM
 *
 * \internal
 * \par Modification history
 * - 1.00 24-08-08  peace, first implementation
 * \endinternal
 */

/*******************************************************************************
  头文件包含
*******************************************************************************/

#include "board.h"
#include "board_id.h"
#include "cmd_processing.h"
#include "hpm_gpio_drv.h"
#include "hpm_gpiom_drv.h"
#include "jtag_io.h"
#include "targetAPI.h"
#include "usb_osbdm.h"

#include <stdio.h>

/*******************************************************************************
  宏定义
*******************************************************************************/

#define LOGT(format, ...) printf("%010d> " format, (uint32_t)hpm_csr_get_core_cycle() / 360, ##__VA_ARGS__)

/** \brief PWR 引脚定义 */
#define PWR_GPIO_INDEX GPIO_OE_GPIOY
#define PWR_GPIO_PIN   0

/** \brief LED_GREEN 引脚定义 */
#define LED_GREEN_GPIO_INDEX GPIO_OE_GPIOA
#define LED_GREEN_GPIO_PIN   30

/** \brief LED_RED 引脚定义 */
#define LED_RED_GPIO_INDEX GPIO_OE_GPIOA
#define LED_RED_GPIO_PIN   31

/*******************************************************************************
  本地全局变量声明
*******************************************************************************/

/*******************************************************************************
  本地函数声明
*******************************************************************************/

void t_debug_init (void);

/*******************************************************************************
  本地全局变量定义
*******************************************************************************/

/*******************************************************************************
  本地函数定义
*******************************************************************************/

/*******************************************************************************
  外部函数定义
*******************************************************************************/

int main (void)
{
    /* 板级初始化 */
    board_init();

    /* 初始化 USB */
    usb_osbdm_init();

    /* 初始化 LED */
    gpio_write_pin(HPM_GPIO0, LED_GREEN_GPIO_INDEX, LED_GREEN_GPIO_PIN, 1);
    gpio_set_pin_output(HPM_GPIO0, LED_GREEN_GPIO_INDEX, LED_GREEN_GPIO_PIN);

    gpio_write_pin(HPM_GPIO0, LED_RED_GPIO_INDEX, LED_RED_GPIO_PIN, 1);
    gpio_set_pin_output(HPM_GPIO0, LED_RED_GPIO_INDEX, LED_RED_GPIO_PIN);

    /* 点亮 LED_GREEN */
    gpio_write_pin(HPM_GPIO0, LED_GREEN_GPIO_INDEX, LED_GREEN_GPIO_PIN, 0);

    /* 初始化 PWR 引脚 */
    HPM_PIOC->PAD[IOC_PAD_PY00].FUNC_CTL = PIOC_PY00_FUNC_CTL_PGPIO_Y_00;
    gpio_write_pin(HPM_PGPIO, PWR_GPIO_INDEX, PWR_GPIO_PIN, 0);
    gpio_set_pin_output(HPM_PGPIO, PWR_GPIO_INDEX, PWR_GPIO_PIN);

    /* 初始化 JTAG 引脚 */
    gpiom_set_pin_controller(HPM_GPIOM, TMS_GPIO_INDEX, TMS_GPIO_PIN, gpiom_core0_fast);
    gpio_write_pin(HPM_FGPIO, TMS_GPIO_INDEX, TMS_GPIO_PIN, 0);
    gpio_set_pin_output(HPM_FGPIO, TMS_GPIO_INDEX, TMS_GPIO_PIN);

    gpiom_set_pin_controller(HPM_GPIOM, TCK_GPIO_INDEX, TCK_GPIO_PIN, gpiom_core0_fast);
    gpio_write_pin(HPM_FGPIO, TCK_GPIO_INDEX, TCK_GPIO_PIN, 0);
    gpio_set_pin_output(HPM_FGPIO, TCK_GPIO_INDEX, TCK_GPIO_PIN);

    gpiom_set_pin_controller(HPM_GPIOM, TDI_GPIO_INDEX, TDI_GPIO_PIN, gpiom_core0_fast);
    gpio_write_pin(HPM_FGPIO, TDI_GPIO_INDEX, TDI_GPIO_PIN, 0);
    gpio_set_pin_output(HPM_FGPIO, TDI_GPIO_INDEX, TDI_GPIO_PIN);

    gpiom_set_pin_controller(HPM_GPIOM, TDO_GPIO_INDEX, TDO_GPIO_PIN, gpiom_core0_fast);
    gpio_set_pin_input(HPM_FGPIO, TDO_GPIO_INDEX, TDO_GPIO_PIN);

    gpio_write_pin(HPM_GPIO0, JCOMP_GPIO_INDEX, JCOMP_GPIO_PIN, 1);
    gpio_set_pin_output(HPM_GPIO0, JCOMP_GPIO_INDEX, JCOMP_GPIO_PIN);

    gpio_write_pin(HPM_GPIO0, SRST_GPIO_INDEX, SRST_GPIO_PIN, 0);
    gpio_set_pin_output(HPM_GPIO0, SRST_GPIO_INDEX, SRST_GPIO_PIN);

    read_board_id();
    read_osbdm_id();
    t_debug_init();

    while (1) {
        if (debug_cmd_pending) { // if BDM command is ready,
            // LOGT("exec cmd %02x %02x %02x\n", debug_cmd_pending, g_usb_osbdm_rx_buf[1], g_usb_osbdm_rx_buf[2]);

            debug_command_exec(); // process BDM command

            // LOGT("done cmd %02x\n", debug_cmd_pending);
            debug_cmd_pending = 0x00; // clear pending flag
        }
    }

    return 0;
}

/* end of file */
