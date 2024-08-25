#ifndef _JTAG_IO_H_
#define _JTAG_IO_H_

#include "hpm_gpio_drv.h"

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

#define TRST_SET()   gpio_set_port_low_with_mask(HPM_GPIO0, SRST_GPIO_INDEX, 1 << SRST_GPIO_PIN)
#define TRST_RESET() gpio_set_port_high_with_mask(HPM_GPIO0, SRST_GPIO_INDEX, 1 << SRST_GPIO_PIN)

#define TMS_SET()   gpio_set_port_high_with_mask(HPM_GPIO0, TMS_GPIO_INDEX, 1 << TMS_GPIO_PIN)
#define TMS_RESET() gpio_set_port_low_with_mask(HPM_GPIO0, TMS_GPIO_INDEX, 1 << TMS_GPIO_PIN)

#define TCLK_SET()   gpio_set_port_high_with_mask(HPM_GPIO0, TCK_GPIO_INDEX, 1 << TCK_GPIO_PIN)
#define TCLK_RESET() gpio_set_port_low_with_mask(HPM_GPIO0, TCK_GPIO_INDEX, 1 << TCK_GPIO_PIN)

#define TDI_OUT_SET()   gpio_set_port_high_with_mask(HPM_GPIO0, TDI_GPIO_INDEX, 1 << TDI_GPIO_PIN)
#define TDI_OUT_RESET() gpio_set_port_low_with_mask(HPM_GPIO0, TDI_GPIO_INDEX, 1 << TDI_GPIO_PIN)

#define TDO_IN_SET gpio_read_port(HPM_GPIO0, TDO_GPIO_INDEX) & (1 << TDO_GPIO_PIN)

#endif
