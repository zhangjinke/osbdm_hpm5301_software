/**
 * \file
 * \brief OSBDM USB 设备
 *
 * \internal
 * \par Modification history
 * - 1.00 24-08-08  peace, first implementation
 * \endinternal
 */

/*******************************************************************************
  头文件包含
*******************************************************************************/

#include "usb_osbdm.h"

#include "board.h"
#include "usbd_core.h"

// OSBDM
#include "USB_User_API.h"
#include "cmd_processing.h"

#include <string.h>

/*******************************************************************************
  宏定义
*******************************************************************************/

#define __BUSID 0 /**< \brief 总线号 */

#define __CONFIG_SIZE   (9 + 9 + 7 + 7) /**< \brief 配置描述符长度 */
#define __INTERFACE_NUM 1               /**< \brief 接口数量 */

#define __OSBDM_EP_ADDR_IN  0x82 /**< \brief OSBDM 输入端点地址 */
#define __OSBDM_EP_ADDR_OUT 0x01 /**< \brief OSBDM 输出端点地址 */

/*******************************************************************************
  本地全局变量声明
*******************************************************************************/

/*******************************************************************************
  本地函数声明
*******************************************************************************/

static const uint8_t *__device_descriptor_callback (uint8_t speed);
static const uint8_t *__config_descriptor_callback (uint8_t speed);
static const uint8_t *__g_device_quality_descriptor_callback (uint8_t speed);
static const uint8_t *__other_speed_config_descriptor_callback (uint8_t speed);
static const char    *__string_descriptor_callback (uint8_t speed, uint8_t index);

static void __osbdm_ep_in_cb (uint8_t busid, uint8_t ep, uint32_t nbytes);
static void __osbdm_ep_out_cb (uint8_t busid, uint8_t ep, uint32_t nbytes);

/*******************************************************************************
  本地全局变量定义
*******************************************************************************/

/** \brief 设备描述符 */
static const uint8_t __g_device_descriptor[] = {
    USB_DEVICE_DESCRIPTOR_INIT(USB_2_0,  /**< \brief bcdUSB*/
                               0x00,     /**< \brief bDeviceClass */
                               0x00,     /**< \brief bDeviceSubClass */
                               0x00,     /**< \brief bDeviceProtocol */
                               USBD_VID, /**< \brief idVendor */
                               USBD_PID, /**< \brief idProduct */
                               0x0000,   /**< \brief bcdDevice */
                               0x01),    /**< \brief bNumConfigurations */
};

/** \brief 配置描述符 (高速) */
static const uint8_t __g_config_descriptor_hs[] = {
    USB_CONFIG_DESCRIPTOR_INIT(__CONFIG_SIZE, __INTERFACE_NUM, 0x01, USB_CONFIG_BUS_POWERED, USBD_MAX_POWER),
    USB_INTERFACE_DESCRIPTOR_INIT(0x00,  /**< \brief bInterfaceNumber */
                                  0x00,  /**< \brief bAlternateSetting */
                                  0x02,  /**< \brief bNumEndpoints */
                                  0x00,  /**< \brief bInterfaceClass */
                                  0x00,  /**< \brief bInterfaceSubClass */
                                  0x00,  /**< \brief bInterfaceProtocol */
                                  0x02), /**< \brief iInterface */
    USB_ENDPOINT_DESCRIPTOR_INIT(__OSBDM_EP_ADDR_IN, USB_ENDPOINT_TYPE_BULK, USB_BULK_EP_MPS_HS, 0x00),
    USB_ENDPOINT_DESCRIPTOR_INIT(__OSBDM_EP_ADDR_OUT, USB_ENDPOINT_TYPE_BULK, USB_BULK_EP_MPS_HS, 0x00),
};

/** \brief 配置描述符 (全速) */
static const uint8_t __g_config_descriptor_fs[] = {
    USB_CONFIG_DESCRIPTOR_INIT(__CONFIG_SIZE, __INTERFACE_NUM, 0x01, USB_CONFIG_BUS_POWERED, USBD_MAX_POWER),
    USB_INTERFACE_DESCRIPTOR_INIT(0x00,  /**< \brief bInterfaceNumber */
                                  0x00,  /**< \brief bAlternateSetting */
                                  0x02,  /**< \brief bNumEndpoints */
                                  0x00,  /**< \brief bInterfaceClass */
                                  0x00,  /**< \brief bInterfaceSubClass */
                                  0x00,  /**< \brief bInterfaceProtocol */
                                  0x02), /**< \brief iInterface */
    USB_ENDPOINT_DESCRIPTOR_INIT(__OSBDM_EP_ADDR_IN, USB_ENDPOINT_TYPE_BULK, USB_BULK_EP_MPS_FS, 0x00),
    USB_ENDPOINT_DESCRIPTOR_INIT(__OSBDM_EP_ADDR_OUT, USB_ENDPOINT_TYPE_BULK, USB_BULK_EP_MPS_FS, 0x00),
};

/** \brief 设备限定描述符 */
static const uint8_t __g_device_quality_descriptor[] = {
    USB_DEVICE_QUALIFIER_DESCRIPTOR_INIT(USB_2_0, 0x00, 0x00, 0x00, 0x01),
};

/** \brief 其它速度配置描述符 (高速) */
static const uint8_t __g_other_speed_config_descriptor_hs[] = {
    USB_OTHER_SPEED_CONFIG_DESCRIPTOR_INIT(__CONFIG_SIZE,
                                           __INTERFACE_NUM,
                                           0x01,
                                           USB_CONFIG_BUS_POWERED,
                                           USBD_MAX_POWER),
    USB_INTERFACE_DESCRIPTOR_INIT(0x00,  /**< \brief bInterfaceNumber */
                                  0x00,  /**< \brief bAlternateSetting */
                                  0x02,  /**< \brief bNumEndpoints */
                                  0x00,  /**< \brief bInterfaceClass */
                                  0x00,  /**< \brief bInterfaceSubClass */
                                  0x00,  /**< \brief bInterfaceProtocol */
                                  0x02), /**< \brief iInterface */
    USB_ENDPOINT_DESCRIPTOR_INIT(__OSBDM_EP_ADDR_IN, USB_ENDPOINT_TYPE_BULK, USB_BULK_EP_MPS_FS, 0x00),
    USB_ENDPOINT_DESCRIPTOR_INIT(__OSBDM_EP_ADDR_OUT, USB_ENDPOINT_TYPE_BULK, USB_BULK_EP_MPS_FS, 0x00),
};

/** \brief 其它速度配置描述符 (全速) */
static const uint8_t __g_other_speed___g_config_descriptor_fs[] = {
    USB_OTHER_SPEED_CONFIG_DESCRIPTOR_INIT(__CONFIG_SIZE,
                                           __INTERFACE_NUM,
                                           0x01,
                                           USB_CONFIG_BUS_POWERED,
                                           USBD_MAX_POWER),
    USB_INTERFACE_DESCRIPTOR_INIT(0x00,  /**< \brief bInterfaceNumber */
                                  0x00,  /**< \brief bAlternateSetting */
                                  0x02,  /**< \brief bNumEndpoints */
                                  0x00,  /**< \brief bInterfaceClass */
                                  0x00,  /**< \brief bInterfaceSubClass */
                                  0x00,  /**< \brief bInterfaceProtocol */
                                  0x02), /**< \brief iInterface */
    USB_ENDPOINT_DESCRIPTOR_INIT(__OSBDM_EP_ADDR_IN, USB_ENDPOINT_TYPE_BULK, USB_BULK_EP_MPS_HS, 0x00),
    USB_ENDPOINT_DESCRIPTOR_INIT(__OSBDM_EP_ADDR_OUT, USB_ENDPOINT_TYPE_BULK, USB_BULK_EP_MPS_HS, 0x00),
};

/** \brief 字符串描述符 */
static const char *__g_string_descriptors[] = {
    "x09x04",                      /**< \brief Langid */
    "Freescale Semiconductor Inc", /**< \brief Manufacturer */
    "OSBDM - Debug Port",          /**< \brief Product */
    "OS0000",                      /**< \brief Serial Number */
};

/** \brief USB 描述符 */
const struct usb_descriptor __g_descriptor = {
    .device_descriptor_callback         = __device_descriptor_callback,
    .config_descriptor_callback         = __config_descriptor_callback,
    .device_quality_descriptor_callback = __g_device_quality_descriptor_callback,
    .other_speed_descriptor_callback    = __other_speed_config_descriptor_callback,
    .string_descriptor_callback         = __string_descriptor_callback,
};

/** \brief USB 接口 */
static struct usbd_interface __g_interface = {};

/** \brief OSBDM 输入端点 */
static struct usbd_endpoint __g_osbdm_ep_in = {.ep_addr = __OSBDM_EP_ADDR_IN, .ep_cb = __osbdm_ep_in_cb};

/** \brief OSBDM 输出端点 */
static struct usbd_endpoint __g_osbdm_ep_out = {.ep_addr = __OSBDM_EP_ADDR_OUT, .ep_cb = __osbdm_ep_out_cb};

/** \brief OSBDM 接收缓冲区 */
USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t _g_osbdm_rx_buf[2048];

/** \brief OSBDM 发送缓冲区 */
USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t _g_osbdm_tx_buf[2048];

/*******************************************************************************
  本地函数定义
*******************************************************************************/

/**
 * \brief 获取设备描述符回调函数
 */
static const uint8_t *__device_descriptor_callback (uint8_t speed)
{
    (void)speed;

    return __g_device_descriptor;
}

/**
 * \brief 获取配置描述符回调函数
 */
static const uint8_t *__config_descriptor_callback (uint8_t speed)
{
    if (speed == USB_SPEED_HIGH) {
        return __g_config_descriptor_hs;
    } else if (speed == USB_SPEED_FULL) {
        return __g_config_descriptor_fs;
    } else {
        return NULL;
    }
}

/**
 * \brief 获取设备限定描述符回调函数
 */
static const uint8_t *__g_device_quality_descriptor_callback (uint8_t speed)
{
    (void)speed;

    return __g_device_quality_descriptor;
}

/**
 * \brief 获取其它速度配置描述符回调函数
 */
static const uint8_t *__other_speed_config_descriptor_callback (uint8_t speed)
{
    if (speed == USB_SPEED_HIGH) {
        return __g_other_speed_config_descriptor_hs;
    } else if (speed == USB_SPEED_FULL) {
        return __g_other_speed___g_config_descriptor_fs;
    } else {
        return NULL;
    }
}

/**
 * \brief 获取字符串描述符回调函数
 */
static const char *__string_descriptor_callback (uint8_t speed, uint8_t index)
{
    (void)speed;

    if (index >= (sizeof(__g_string_descriptors) / sizeof(char *))) {
        return NULL;
    }
    return __g_string_descriptors[index];
}

/**
 * \brief OSBDM 输入端点回调函数
 */
static void __osbdm_ep_in_cb (uint8_t busid, uint8_t ep, uint32_t nbytes)
{
    if ((nbytes % usbd_get_ep_mps(busid, ep)) == 0 && nbytes) {
        usbd_ep_start_write(busid, ep, NULL, 0); /* 发送 zlp */
    }
}

/**
 * \brief OSBDM 输出端点回调函数
 */
static void __osbdm_ep_out_cb (uint8_t busid, uint8_t ep, uint32_t nbytes)
{
    debug_cmd_pending = _g_osbdm_rx_buf[0];
    memcpy(&EP1_Buffer[0], &_g_osbdm_rx_buf[0], nbytes);

    /* 启动接收输出端点 */
    usbd_ep_start_read(busid, ep, _g_osbdm_rx_buf, sizeof(_g_osbdm_rx_buf));
}

/**
 * \brief USB 事件回调函数
 */
static void __usbd_event_handler (uint8_t busid, uint8_t event)
{
    switch (event) {
    case USBD_EVENT_RESET:
        break;

    case USBD_EVENT_CONNECTED:
        break;

    case USBD_EVENT_DISCONNECTED:
        break;

    case USBD_EVENT_RESUME:
        break;

    case USBD_EVENT_SUSPEND:
        break;

    case USBD_EVENT_CONFIGURED:

        /* 启动接收输出端点 */
        usbd_ep_start_read(busid, __OSBDM_EP_ADDR_OUT, _g_osbdm_rx_buf, sizeof(_g_osbdm_rx_buf));
        break;

    case USBD_EVENT_SET_REMOTE_WAKEUP:
        break;

    case USBD_EVENT_CLR_REMOTE_WAKEUP:
        break;

    default:
        break;
    }
}

/*******************************************************************************
  外部函数定义
*******************************************************************************/

/**
 * \brief 初始化 OSBDM USB
 */
int32_t usb_osbdm_ep_in_send (uint8_t *p_data, uint32_t length)
{
    memcpy(&_g_osbdm_tx_buf[0], &p_data[0], length);

    return usbd_ep_start_write(__BUSID, __OSBDM_EP_ADDR_IN, _g_osbdm_tx_buf, length);
}

/**
 * \brief 初始化 OSBDM USB
 */
void usb_osbdm_init (void)
{
    intc_set_irq_priority(CONFIG_HPM_USBD_IRQn, 1);
    board_init_usb_pins();

    usbd_desc_register(__BUSID, &__g_descriptor);

    usbd_add_interface(__BUSID, &__g_interface);
    usbd_add_endpoint(__BUSID, &__g_osbdm_ep_in);
    usbd_add_endpoint(__BUSID, &__g_osbdm_ep_out);

    usbd_initialize(__BUSID, CONFIG_HPM_USBD_BASE, __usbd_event_handler);
}

/* end of file */
