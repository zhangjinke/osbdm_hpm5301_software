# Copyright (c) 2024 HPMicro
# SPDX-License-Identifier: BSD-3-Clause

cmake_minimum_required(VERSION 3.13)

# USB Device
set(CONFIG_USB_DEVICE 1)

# CherryUSB
set(CONFIG_CHERRYUSB 1)
set(CONFIG_CHERRYUSB_DEVICE 1)

# SEGGER RTT
set(CONFIG_NDEBUG_CONSOLE 1)
set(CONFIG_SEGGER_RTT 1)

find_package(hpm-sdk REQUIRED HINTS $ENV{HPM_SDK_BASE})
project(osbdm)

sdk_compile_options("-O3")

sdk_inc(application/include)

# Application
sdk_app_inc(application/include)
sdk_app_src(application/source/main.c)
sdk_app_src(application/source/usb_osbdm.c)

# EasyLogger
sdk_app_inc(middleware/easylogger/inc)
sdk_app_src(middleware/easylogger/port/elog_port.c)
sdk_app_src(middleware/easylogger/src/elog.c)
# sdk_app_src(middleware/easylogger/src/elog_async.c)
# sdk_app_src(middleware/easylogger/src/elog_buf.c)
sdk_app_src(middleware/easylogger/src/elog_utils.c)

# # SystemView
# sdk_compile_definitions(-DSEGGER_RTT_MAX_NUM_UP_BUFFERS=1)
# sdk_compile_definitions(-DSEGGER_RTT_MAX_NUM_DOWN_BUFFERS=1)
# sdk_app_inc(middleware/SystemView/Config)
# sdk_app_inc(middleware/SystemView/SEGGER)
# sdk_app_src(middleware/SystemView/SEGGER/SEGGER_RTT_printf.c)
# sdk_app_src(middleware/SystemView/SEGGER/SEGGER_RTT.c)
# # sdk_app_src(middleware/SystemView/SEGGER/SEGGER_SYSVIEW.c)

# SEGGER RTT
sdk_compile_definitions(-DCONFIG_NDEBUG_CONSOLE=1)
sdk_ses_opt_lib_io_type(RTT)
sdk_ses_opt_debug_connection(J-Link)

# OSBDM
sdk_app_inc(osbdm)
sdk_app_inc(osbdm/common)
sdk_app_src(osbdm/board_id.c)
sdk_app_src(osbdm/cmd_processing.c)
sdk_app_src(osbdm/jtag_eppc.c)
sdk_app_src(osbdm/util.c)

# generate_ide_projects()
