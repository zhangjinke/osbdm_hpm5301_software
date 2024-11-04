/**
 * \file
 * \brief 板级配置
 *
 * \internal
 * \par Modification history
 * - 1.00 24-10-14  peace, first implementation
 * \endinternal
 */

#ifndef __BOARD_CUSTOM_H__
#define __BOARD_CUSTOM_H__

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
  头文件包含
*******************************************************************************/

#include "hpm_gpio_drv.h"

#include <stdint.h>

/*******************************************************************************
  宏定义
*******************************************************************************/

#define BOARD_NAME "Embedded Tools"

/** \brief PWR 引脚定义 */
#define PWR_GPIO_CTRL  HPM_FGPIO
#define PWR_GPIO_INDEX GPIO_OE_GPIOA
#define PWR_GPIO_PIN   0

/** \brief TRST_DIR 引脚定义 */
#define TRST_DIR_GPIO_CTRL  HPM_FGPIO
#define TRST_DIR_GPIO_INDEX GPIO_OE_GPIOA
#define TRST_DIR_GPIO_PIN   1

/** \brief TRST 引脚定义 */
#define TRST_GPIO_CTRL  HPM_FGPIO
#define TRST_GPIO_INDEX GPIO_OE_GPIOA
#define TRST_GPIO_PIN   2

/** \brief TDI_DIR 引脚定义 */
#define TDI_DIR_GPIO_CTRL  HPM_FGPIO
#define TDI_DIR_GPIO_INDEX GPIO_OE_GPIOA
#define TDI_DIR_GPIO_PIN   3

/** \brief LED_GREEN 引脚定义 */
#define LED_GREEN_GPIO_CTRL  HPM_FGPIO
#define LED_GREEN_GPIO_INDEX GPIO_OE_GPIOA
#define LED_GREEN_GPIO_PIN   8

/** \brief LED_RED 引脚定义 */
#define LED_RED_GPIO_CTRL  HPM_FGPIO
#define LED_RED_GPIO_INDEX GPIO_OE_GPIOA
#define LED_RED_GPIO_PIN   9

/** \brief VREF_PWM 引脚定义 */
#define VREF_PWM_GPIO_CTRL  HPM_FGPIO
#define VREF_PWM_GPIO_INDEX GPIO_OE_GPIOA
#define VREF_PWM_GPIO_PIN   10

/** \brief TCK 引脚定义 */
#define TCK_GPIO_CTRL  HPM_FGPIO
#define TCK_GPIO_INDEX GPIO_OE_GPIOA
#define TCK_GPIO_PIN   27

/** \brief TMS 引脚定义 */
#define TMS_GPIO_CTRL  HPM_FGPIO
#define TMS_GPIO_INDEX GPIO_OE_GPIOA
#define TMS_GPIO_PIN   28

/** \brief TMS_DIR 引脚定义 */
#define TMS_DIR_GPIO_CTRL  HPM_FGPIO
#define TMS_DIR_GPIO_INDEX GPIO_OE_GPIOA
#define TMS_DIR_GPIO_PIN   30

/** \brief SRST_OUT 引脚定义 */
#define SRST_OUT_GPIO_CTRL  HPM_FGPIO
#define SRST_OUT_GPIO_INDEX GPIO_OE_GPIOA
#define SRST_OUT_GPIO_PIN   31

/** \brief ADC_VREF 引脚定义 */
#define ADC_VREF_GPIO_CTRL  HPM_FGPIO
#define ADC_VREF_GPIO_INDEX GPIO_OE_GPIOB
#define ADC_VREF_GPIO_PIN   8

/** \brief SRST_DIR 引脚定义 */
#define SRST_DIR_GPIO_CTRL  HPM_FGPIO
#define SRST_DIR_GPIO_INDEX GPIO_OE_GPIOB
#define SRST_DIR_GPIO_PIN   9

/** \brief TDO 引脚定义 */
#define TDO_GPIO_CTRL  HPM_FGPIO
#define TDO_GPIO_INDEX GPIO_OE_GPIOB
#define TDO_GPIO_PIN   12

/** \brief TDI 引脚定义 */
#define TDI_GPIO_CTRL  HPM_FGPIO
#define TDI_GPIO_INDEX GPIO_OE_GPIOB
#define TDI_GPIO_PIN   13

/** \brief SRST_IN 引脚定义 */
#define SRST_IN_GPIO_CTRL  HPM_FGPIO
#define SRST_IN_GPIO_INDEX GPIO_OE_GPIOY
#define SRST_IN_GPIO_PIN   0

/** \brief TCK_DIR 引脚定义 */
#define TCK_DIR_GPIO_CTRL  HPM_FGPIO
#define TCK_DIR_GPIO_INDEX GPIO_OE_GPIOY
#define TCK_DIR_GPIO_PIN   1

/******************************************************************************/

/** \brief 设置 PWR 开关 */
#define PWR_ON()  gpio_set_port_high_with_mask(PWR_GPIO_CTRL, PWR_GPIO_INDEX, 1 << PWR_GPIO_PIN)
#define PWR_OFF() gpio_set_port_low_with_mask(PWR_GPIO_CTRL, PWR_GPIO_INDEX, 1 << PWR_GPIO_PIN)

/** \brief 设置 TRST_DIR 方向 */
#define TRST_DIR_OUT() gpio_set_port_high_with_mask(TRST_DIR_GPIO_CTRL, TRST_DIR_GPIO_INDEX, 1 << TRST_DIR_GPIO_PIN)
#define TRST_DIR_IN()  gpio_set_port_low_with_mask(TRST_DIR_GPIO_CTRL, TRST_DIR_GPIO_INDEX, 1 << TRST_DIR_GPIO_PIN)

/** \brief 设置 TRST 是否有效 */
#define TRST_DEASSERT() gpio_set_port_high_with_mask(TRST_GPIO_CTRL, TRST_GPIO_INDEX, 1 << TRST_GPIO_PIN)
#define TRST_ASSERT()   gpio_set_port_low_with_mask(TRST_GPIO_CTRL, TRST_GPIO_INDEX, 1 << TRST_GPIO_PIN)

/** \brief 设置 TDI_DIR 方向 */
#define TDI_DIR_OUT() gpio_set_port_high_with_mask(TDI_DIR_GPIO_CTRL, TDI_DIR_GPIO_INDEX, 1 << TDI_DIR_GPIO_PIN)
#define TDI_DIR_IN()  gpio_set_port_low_with_mask(TDI_DIR_GPIO_CTRL, TDI_DIR_GPIO_INDEX, 1 << TDI_DIR_GPIO_PIN)

/** \brief 设置 LED_GREEN 开关 */
#define LED_GREEN_OFF() gpio_set_port_high_with_mask(LED_GREEN_GPIO_CTRL, LED_GREEN_GPIO_INDEX, 1 << LED_GREEN_GPIO_PIN)
#define LED_GREEN_ON()  gpio_set_port_low_with_mask(LED_GREEN_GPIO_CTRL, LED_GREEN_GPIO_INDEX, 1 << LED_GREEN_GPIO_PIN)

/** \brief 设置 LED_RED 开关 */
#define LED_RED_OFF() gpio_set_port_high_with_mask(LED_RED_GPIO_CTRL, LED_RED_GPIO_INDEX, 1 << LED_RED_GPIO_PIN)
#define LED_RED_ON()  gpio_set_port_low_with_mask(LED_RED_GPIO_CTRL, LED_RED_GPIO_INDEX, 1 << LED_RED_GPIO_PIN)

/** \brief 设置 TCK 输出电平 */
#define TCK_SET()   gpio_set_port_high_with_mask(TCK_GPIO_CTRL, TCK_GPIO_INDEX, 1 << TCK_GPIO_PIN)
#define TCK_RESET() gpio_set_port_low_with_mask(TCK_GPIO_CTRL, TCK_GPIO_INDEX, 1 << TCK_GPIO_PIN)

/** \brief 设置 TMS 输出电平 */
#define TMS_SET()   gpio_set_port_high_with_mask(TMS_GPIO_CTRL, TMS_GPIO_INDEX, 1 << TMS_GPIO_PIN)
#define TMS_RESET() gpio_set_port_low_with_mask(TMS_GPIO_CTRL, TMS_GPIO_INDEX, 1 << TMS_GPIO_PIN)

/** \brief 设置 TMS_DIR 方向 */
#define TMS_DIR_OUT() gpio_set_port_high_with_mask(TMS_DIR_GPIO_CTRL, TMS_DIR_GPIO_INDEX, 1 << TMS_DIR_GPIO_PIN)
#define TMS_DIR_IN()  gpio_set_port_low_with_mask(TMS_DIR_GPIO_CTRL, TMS_DIR_GPIO_INDEX, 1 << TMS_DIR_GPIO_PIN)

/** \brief 读取/设置 SRST_OUT 是否有效 */
#define SRST_OUT_GET()      gpio_get_pin_output_status(SRST_OUT_GPIO_CTRL, SRST_OUT_GPIO_INDEX, SRST_OUT_GPIO_PIN)
#define SRST_OUT_ASSERT()   gpio_set_port_high_with_mask(SRST_OUT_GPIO_CTRL, SRST_OUT_GPIO_INDEX, 1 << SRST_OUT_GPIO_PIN)
#define SRST_OUT_DEASSERT() gpio_set_port_low_with_mask(SRST_OUT_GPIO_CTRL, SRST_OUT_GPIO_INDEX, 1 << SRST_OUT_GPIO_PIN)

/** \brief 设置 SRST_DIR 方向 */
#define SRST_DIR_OUT() gpio_set_port_high_with_mask(SRST_DIR_GPIO_CTRL, SRST_DIR_GPIO_INDEX, 1 << SRST_DIR_GPIO_PIN)
#define SRST_DIR_IN()  gpio_set_port_low_with_mask(SRST_DIR_GPIO_CTRL, SRST_DIR_GPIO_INDEX, 1 << SRST_DIR_GPIO_PIN)

/** \brief 获取 TDO 输入电平 */
#define TDO_GET() (gpio_read_port(TDO_GPIO_CTRL, TDO_GPIO_INDEX) & (1 << TDO_GPIO_PIN))

/** \brief 设置 TDI 输出电平 */
#define TDI_SET()   gpio_set_port_high_with_mask(TDI_GPIO_CTRL, TDI_GPIO_INDEX, 1 << TDI_GPIO_PIN)
#define TDI_RESET() gpio_set_port_low_with_mask(TDI_GPIO_CTRL, TDI_GPIO_INDEX, 1 << TDI_GPIO_PIN)

/** \brief 获取 SRST_IN 是否有效 */
#define SRST_IN_GET() (!(gpio_read_port(SRST_IN_GPIO_CTRL, SRST_IN_GPIO_INDEX) & (1 << SRST_IN_GPIO_PIN)))

/** \brief 设置 TCK_DIR 方向 */
#define TCK_DIR_OUT() gpio_set_port_high_with_mask(TCK_DIR_GPIO_CTRL, TCK_DIR_GPIO_INDEX, 1 << TCK_DIR_GPIO_PIN)
#define TCK_DIR_IN()  gpio_set_port_low_with_mask(TCK_DIR_GPIO_CTRL, TCK_DIR_GPIO_INDEX, 1 << TCK_DIR_GPIO_PIN)

/******************************************************************************/

/**
 * \brief 通过目标电压，计算占空比
 *        duty = voltage / (3.08 * (100 / 110) * 2)，其中
 *            duty：   占空比，单位 %
 *            voltage：输出电压，单位 V
 *            3.08：   PWM 引脚高电平电压，单位 V
 *            100：    R5 阻值
 *            110：    R5 + R6 阻值
 *
 * \note 宏参数 voltage 为目标电压，单位 mV，计算结果为占空比，单位 %，精度 0.001
 */
#define VIO_PWM_DUTY_GET(voltage) ((voltage * (10 * 100)) / 56)

/*******************************************************************************
  本地全局变量声明
*******************************************************************************/

/*******************************************************************************
  本地函数声明
*******************************************************************************/

/**
 * \brief 获取时间戳
 *
 * \return 获取到的时间戳，单位 ms
 */
static inline uint32_t systick_get (void)
{
    return (uint32_t)hpm_csr_get_core_cycle() / 360000;
}

/**
 * \brief 获取 VREF 电压
 *
 * \return 转换未完成返回 -1，否则返回 VREF 电压，单位 mV，范围 0~5000
 */
int32_t vref_voltage_get (void);

/**
 * \brief 设置 VIO PWM 占空比
 *
 * \param duty 占空比，单位 %，精度 0.001
 */
void vio_pwm_duty_set (uint32_t duty);

/**
 * \brief 板级初始化
 *
 * \retval  0 成功
 * \retval -1 失败
 */
int32_t board_custom_init (void);

#ifdef __cplusplus
}
#endif

#endif /* __BOARD_CUSTOM_H__ */

/* end of file */
