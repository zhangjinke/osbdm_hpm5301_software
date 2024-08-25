/* OSBDM-JM60 Target Interface Software Package
 * Copyright (C) 2009  Freescale
 *
 * This software package is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this software; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */


/*****************************************************************************\
*
*	Author:		Axiom Manufacturing
*
*	File:		targetAPI.h
*
*	Purpose: 	header file for the generic target module interface API
*
*
* 03/08/2011  : Modified by P&E Microcomputer Systems
*               http://www.pemicro.com/osbdm
*
*
*
\*****************************************************************************/

#ifndef _TARGETAPI_H_
#define _TARGETAPI_H_

//----------------------------------------------------------------------------
// Hardware and Firmware Version Numbers
//----------------------------------------------------------------------------
#define VERSION_SW 0x01 // Firmware Version 1.0
#define VERSION_HW 0x10 // Hardware Version 1.0

//---------------------------------------------------------------------------
// Firmware Build Info
//---------------------------------------------------------------------------
//
// Warning! New versions of Codewarrior will automatically update the firmware.
//          If you are building a custom firmware based on OSBDM, please set
//          the BUILD_VER version number to a minimum of 100
//
#define BUILD_VER     31
#define BUILD_VER_REV 26

//----------------------------------------------------------------------------
// Reset Type Definition
//----------------------------------------------------------------------------

typedef enum { // type of reset mode
    eSoftReset_to_DebugMode,
    eSoftReset_to_NormalMode,
    eHardReset_to_DebugMode,
    eHardReset_to_NormalMode,
    ePowerReset_to_DebugMode,
    ePowerReset_to_NormalMode // not implemented yet
} ResetT;

//----------------------------------------------------------------------------
// Core Type Definition
//----------------------------------------------------------------------------

typedef enum {
    eCFv234,
    eCFv1,
    eS08,
    eRS08,
    eS12,
    eDSC,
    eCoreTypeUnknown,
    eKinetis,
    ePPC,
    eS12Z,
} CoreT;

//----------------------------------------------------------------------------
// Firmware Type Definition
//----------------------------------------------------------------------------

typedef enum {
    eGeneric,
    eEmbeddedGeneric
} FirmwareT;

//----------------------------------------------------------------------------
// target implementation module API functions
//----------------------------------------------------------------------------

uint8_t  t_init (void);         // init BDM_CF and target
uint8_t  t_reset (ResetT mode); // resets target and leaves in selected mode;
uint16_t t_sync (void);         // Returns the sync value in 24 MHz. clocks;
uint8_t  t_resync (void);
int32_t  t_stat (uint8_t *pData); // Returns status
uint8_t  t_halt (void);           // halt program execution at next command
uint8_t  t_go (void);             // resume code execution from current PC
uint8_t  t_step (void);
int32_t  t_get_ver (uint8_t *pData); // Returns OSBDM-JM60 version info

int32_t t_config (uint8_t configType, uint8_t configParam, uint32_t paramVal);

uint32_t t_get_clock (void);
void     t_set_clock (uint32_t interval);

int32_t t_write_mem (uint8_t type, uint32_t addr, uint8_t width, uint32_t count, uint8_t *pData);
int32_t t_soft_reset_halt (uint8_t type, uint32_t addr, uint8_t width, uint32_t count, uint8_t *pData);
int32_t t_read_mem (uint8_t type, uint32_t addr, uint8_t width, uint32_t count, uint8_t *pData);
int32_t t_fill_mem (uint8_t type, uint32_t addr, uint8_t width, uint32_t count, uint8_t *pData);

int32_t t_write_ad (uint32_t addr, uint8_t *pData);
int32_t t_read_ad (uint32_t addr, uint8_t *pData);

int32_t t_write_creg (uint32_t addr, uint8_t *pData);
int32_t t_read_creg (uint32_t addr, uint8_t *pData);

int32_t t_write_dreg (uint32_t addr, uint8_t uWidth, uint8_t *pData);
int32_t t_read_dreg (uint32_t addr, uint8_t uWidth, uint8_t *pData);

uint8_t t_unsecure (uint8_t lockr, uint8_t lrcnt, uint8_t clkdiv);
void    t_assert_ta (uint16_t dly_cnt);

int32_t t_flash_power (uint8_t enable);
int32_t t_flash_enable (uint8_t enable);
int32_t t_flash_prog (uint8_t *pData);

void t_flash_stat (void);

int32_t t_enable_ack (uint8_t enable); // 1 to enable, 0 to disable

int32_t t_special_feature (uint8_t   sub_cmd_num,   // Special feature number (sub_cmd_num)
                           uint16_t *pInputLength,  // Length of Input Command
                           uint8_t  *pInputBuffer,  // Input Command Buffer
                           uint16_t *pOutputLength, // Length of Output Response
                           uint8_t  *pOutputBuffer); // Output Response Buffer

#endif // _TARGETAPI_H_
// --------------------------------- EOF -------------------------------------
