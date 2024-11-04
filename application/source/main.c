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

#include "board_custom.h"
#include "board_id.h"
#include "cmd_processing.h"
#include "hpm_gpio_drv.h"
#include "hpm_gpiom_drv.h"
#include "targetAPI.h"
#include "usb_osbdm.h"

#include <stdio.h>

/*******************************************************************************
  宏定义
*******************************************************************************/

#define LOGT(format, ...) printf("%010d> " format, systick_get(), ##__VA_ARGS__)

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

/** \brief VREF 电压 */
static uint32_t _g_vref_voltage = 0;

/*******************************************************************************
  本地函数定义
*******************************************************************************/

/**
 * \brief 处理 VIO 电压任务
 */
static void _vio_process (void)
{
    int32_t voltage;

    voltage = vref_voltage_get();
    if (voltage >= 0) {
        _g_vref_voltage = voltage;
        vio_pwm_duty_set(VIO_PWM_DUTY_GET(voltage));
    }
}

/**
 * \brief 处理 LED ERROR 任务
 */
static void _led_error_process (void)
{
    if (_g_vref_voltage < 1000) {
        LED_RED_OFF();
    } else {
        if (SRST_OUT_GET() && (!SRST_IN_GET())) { /* 复位输出有效，但输入无效 */
            LED_RED_ON();
        } else {
            LED_RED_OFF();
        }
    }
}

/**
 * \brief 处理 LED STATUS 任务
 */
static void _led_status_process (void)
{
    static uint32_t s_tick  = 0;
    static uint32_t s_state = 0;

    switch (s_state) {
    case 0: /* 空闲状态 */
        if (debug_cmd_pending) {
            s_tick = systick_get();
            LED_GREEN_OFF(); /* 执行命令，熄灭 LED GREEN */
            s_state = 1;
            break;
        }
        break;

    case 1: /* LED 点亮状态 */
        if ((systick_get() - s_tick) >= 50) {
            s_tick = systick_get();
            LED_GREEN_ON();
            s_state = 2;
            break;
        }
        break;

    case 2: /* LED 熄灭状态 */
        if ((systick_get() - s_tick) >= 10) {
            s_state = 0;
            break;
        }
        break;
    }
}

/*******************************************************************************
  外部函数定义
*******************************************************************************/

/**
 * \brief 主函数
 */
int main (void)
{
    /* 板级初始化 */
    board_custom_init();

    /* 初始化 USB */
    usb_osbdm_init();

    /* 点亮 LED_GREEN */
    LED_GREEN_ON();

    /* 初始化 OSBDM */
    read_board_id();
    read_osbdm_id();
    t_debug_init();

    while (1) {
        /* 处理 VIO 电压任务 */
        _vio_process();

        /* 处理 LED ERROR 任务 */
        _led_error_process();

        /* 处理 LED STATUS 任务 */
        _led_status_process();

        /* 处理 OSBDM 命令 */
        if (debug_cmd_pending) {
            // LOGT("exec cmd %02x %02x %02x\n", debug_cmd_pending, g_usb_osbdm_rx_buf[1], g_usb_osbdm_rx_buf[2]);
            debug_command_exec();
            // LOGT("done cmd %02x\n", debug_cmd_pending);
            debug_cmd_pending = 0x00;
        }
    }

    return 0;
}

/* end of file */
