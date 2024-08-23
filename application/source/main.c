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

#include "USB_User_API.h"
#include "board.h"
#include "board_id.h"
#include "cmd_processing.h"
#include "hpm_gpio_drv.h"
#include "usb_osbdm.h"

#include <stdio.h>

/*******************************************************************************
  宏定义
*******************************************************************************/

/** \brief TMS 引脚定义 */
#define TMS_GPIO_INDEX GPIO_OE_GPIOA
#define TMS_GPIO_PIN   28

/** \brief TCK 引脚定义 */
#define TCK_GPIO_INDEX GPIO_OE_GPIOA
#define TCK_GPIO_PIN   27

/** \brief TDI 引脚定义 */
#define TDI_GPIO_INDEX GPIO_OE_GPIOB
#define TDI_GPIO_PIN   13

/** \brief TDO 引脚定义 */
#define TDO_GPIO_INDEX GPIO_OE_GPIOB
#define TDO_GPIO_PIN   12

/** \brief SRST 引脚定义 */
#define SRST_GPIO_INDEX GPIO_OE_GPIOB
#define SRST_GPIO_PIN   15

/** \brief JCOMP 引脚定义 */
#define JCOMP_GPIO_INDEX GPIO_OE_GPIOB
#define JCOMP_GPIO_PIN   14

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
    gpio_write_pin(HPM_GPIO0, PWR_GPIO_INDEX, PWR_GPIO_PIN, 0);
    gpio_set_pin_output(HPM_GPIO0, PWR_GPIO_INDEX, PWR_GPIO_PIN);

    /* 初始化 JTAG 引脚 */
    gpio_write_pin(HPM_GPIO0, TMS_GPIO_INDEX, TMS_GPIO_PIN, 0);
    gpio_set_pin_output(HPM_GPIO0, TMS_GPIO_INDEX, TMS_GPIO_PIN);

    gpio_write_pin(HPM_GPIO0, TCK_GPIO_INDEX, TCK_GPIO_PIN, 0);
    gpio_set_pin_output(HPM_GPIO0, TCK_GPIO_INDEX, TCK_GPIO_PIN);

    gpio_write_pin(HPM_GPIO0, TDI_GPIO_INDEX, TDI_GPIO_PIN, 0);
    gpio_set_pin_output(HPM_GPIO0, TDI_GPIO_INDEX, TDI_GPIO_PIN);

    gpio_set_pin_input(HPM_GPIO0, TDO_GPIO_INDEX, TDO_GPIO_PIN);

    gpio_write_pin(HPM_GPIO0, SRST_GPIO_INDEX, SRST_GPIO_PIN, 0);
    gpio_set_pin_output(HPM_GPIO0, SRST_GPIO_INDEX, SRST_GPIO_PIN);

    gpio_write_pin(HPM_GPIO0, JCOMP_GPIO_INDEX, JCOMP_GPIO_PIN, 1);
    gpio_set_pin_output(HPM_GPIO0, JCOMP_GPIO_INDEX, JCOMP_GPIO_PIN);

    read_board_id();
    read_osbdm_id();
    t_debug_init();

    while (1) {
        if (debug_cmd_pending) { // if BDM command is ready,
            gpio_write_pin(HPM_GPIO0, LED_GREEN_GPIO_INDEX, LED_GREEN_GPIO_PIN, 1);
            // printf("exec cmd %02x %02x\n", debug_cmd_pending, EP1_Buffer[1]);

            debug_command_exec(); // process BDM command
            // printf("done cmd %02x\n", debug_cmd_pending);
            debug_cmd_pending = 0x00; // clear pending flag

            gpio_write_pin(HPM_GPIO0, LED_GREEN_GPIO_INDEX, LED_GREEN_GPIO_PIN, 0);
        }
    }

    return 0;
}

/* end of file */
