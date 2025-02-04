/**
 * \file
 * \brief OSBDM USB 设备
 *
 * \internal
 * \par Modification history
 * - 1.00 24-08-08  peace, first implementation
 * \endinternal
 */

#ifndef __USB_OSBDM
#define __USB_OSBDM

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
  头文件包含
*******************************************************************************/

#include "usbd_core.h"

#include <stdint.h>

/*******************************************************************************
  宏定义
*******************************************************************************/

/*******************************************************************************
  本地全局变量声明
*******************************************************************************/

/** \brief OSBDM 接收缓冲区 */
extern uint8_t g_usb_osbdm_rx_buf[USB_BULK_EP_MPS_HS];

/** \brief OSBDM 发送缓冲区 */
extern uint8_t g_usb_osbdm_tx_buf[USB_BULK_EP_MPS_HS];

/*******************************************************************************
  本地函数声明
*******************************************************************************/

/**
 * \brief 输入端点发送数据
 *
 * \param[in] p_data 指向待发送数据的指针
 * \param[in] length 待发送数据长度，单位：字节
 *
 * \retval  0 成功
 * \retval -1 失败
 */
int32_t usb_osbdm_ep_in_send (uint8_t *p_data, uint32_t length);

/**
 * \brief 初始化 OSBDM USB
 */
void usb_osbdm_init (void);

#ifdef __cplusplus
}
#endif

#endif /* __USB_OSBDM */

/* end of file */
