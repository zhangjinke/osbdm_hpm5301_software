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

/******************************************************************************
 *	File:		cmd_processing.c
 *
 *	Purpose: parse and process BDM commands from host
 *
 *	Output:	returns cmd in cmd_array[0] on success along with optional data
 *				returns error code in cmd_array[0] on error
 *
 *
 * 03/08/2011  : Modified by P&E Microcomputer Systems
 *               http://www.pemicro.com/osbdm
 *
 *
 *
 ******************************************************************************/

#include "cmd_processing.h" // command processing structures

#include "board_custom.h"
#include "commands.h" // BDM commands header file
#include "jtag_eppc.h"
#include "targetAPI.h" // target API include file
#include "usb_osbdm.h"
#include "util.h"


// Function Prototypes
void command_exec (void); // executes BDM commands

// Data Types
_tCable_Status tCable_Status; // BDM cable status

uint16_t spi_data;
uint16_t rdata = 0;

// Global variables parsed from the USB buffer stream
uint8_t  uCmd;   // command
uint8_t  uType;  // type
uint32_t uAddr;  // address
uint8_t  uWidth; // width (access size)
uint32_t uLen;   // length in bytes

volatile uint8_t debug_cmd_pending; // input BDM command pending

/*------------------------------------------------------------------------------------
  get_mem_header
  ----------------
  parse the usb header for memory commands into global variables

*/
void get_mem_header (uint8_t *buf)
{
    uType = *(buf + 1); // uType = 8-bit Memory Type
#ifdef __DSC__
    if (uType == MEM_RAM)
        uType = MEM_P;
#endif
    uAddr  = getbuf4(buf + 2); // uAddr = 32-bit address
    uWidth = *(buf + 6);       // uWidth = 8-bit Bit Width (8/16/32)
    uLen   = getbuf4(buf + 7); // uLen = 32-bit length (number to read or write)
}

// note:  g_usb_osbdm_tx_buf[1] should be preloaded with the number of data bytes to send back
//		  before calling this function
void debug_command_send_return (uint8_t rval, uint8_t count)
{
    g_usb_osbdm_tx_buf[0] = rval;
    g_usb_osbdm_tx_buf[1] = count;
    usb_osbdm_ep_in_send(&g_usb_osbdm_tx_buf[0], count);
}

/*------------------------------------------------------------------------------------
  debug_command_exec
  ------------
  handle bdm commands
*/
void debug_command_exec ()
{
    uint8_t  count, rcount;
    uint8_t *dptr;
    uint32_t rval32;

    uCmd   = g_usb_osbdm_rx_buf[0]; // store command, this will be returned to host if successful
    rcount = 2;                     // default number of bytes to return

    switch (uCmd) {
        // -- System Commands --------------------------------------------

    case CMD_GET_STATUS:
        if (t_stat(g_usb_osbdm_tx_buf + 2))
            uCmd = CMD_FAILED;      // get status bytes
        rcount = STATUS_BLOCK_SIZE; // number of bytes to return (including header bytes)
        break;

    case CMD_GET_VER:
        if (t_get_ver(g_usb_osbdm_tx_buf + 2))
            uCmd = CMD_FAILED;       // get firmware version info bytes
        rcount = VERSION_BLOCK_SIZE; // number of bytes to return (including header bytes)
        break;

    case CMD_BDM_INIT:
        if (t_init())
            uCmd = CMD_FAILED; // Initialize BDM
        break;

    case CMD_RESET_TARGET:              // command 0x1B
        t_reset(g_usb_osbdm_rx_buf[1]); // issue target reset - drives RSTO high for 50ms
        g_usb_osbdm_tx_buf[1] = CMD_OK; // return status good
        tCable_Status.reset   = 0x01;   // reset occurred
                                        //			Ep_In_Size = 0x02;			// return byte count
        break;

    case CMD_ASSERT_TA:
        t_assert_ta(getbuf2little(g_usb_osbdm_rx_buf + 1));
        break;

    case CMD_GO:
        t_go();
        break;

    case CMD_STEP:
        t_step();
        break;

    case CMD_HALT:
        t_halt();
        break;

    case CMD_CONFIG:

        // configparam_type, config_param, value
        t_config(g_usb_osbdm_rx_buf[1], g_usb_osbdm_rx_buf[2], getbuf4(g_usb_osbdm_rx_buf + 3));
        break;

    case CMD_SET_SPEED:
        // Set the BDM clock value of the target
        t_set_clock(getbuf4(g_usb_osbdm_rx_buf + 1));
        break;

    case CMD_GET_SPEED:
        rval32                                = t_get_clock(); // get target clock speed
        *(uint32_t *)(g_usb_osbdm_tx_buf + 2) = rval32;
        rcount                                = 6;
        break;

    case CMD_SPECIAL_FEATURE:
        t_special_feature(g_usb_osbdm_rx_buf[1],                // Special feature number (sub_cmd_num)
                          (uint16_t *)(g_usb_osbdm_rx_buf + 2), // Length of Input Command
                          g_usb_osbdm_rx_buf + 4,               // Input Command Buffer
                          (uint16_t *)(g_usb_osbdm_tx_buf + 2), // Length of Output Response
                          g_usb_osbdm_tx_buf + 4);              // Output Response Buffer
        rcount = 64;
        break;


    // -- Write Memory Commands --------------------------------------
    case CMD_WRITE_MEM:

        get_mem_header(g_usb_osbdm_rx_buf); // parse memory header into variables
        dptr = g_usb_osbdm_rx_buf + MEM_HEADER_SIZE;

        // determine number of bytes to write this time
        // no more than MAX_DATA_SIZE bytes can be sent at a time
        count = (MAX_DATA_SIZE - MEM_HEADER_SIZE); // default: maximum data size for first packet
        if (uLen < count)
            count = (uint8_t)uLen; // set to total if less than max packet size

        switch (uType) {
        case MEM_RAM:
            // write memory
            if (t_write_mem(uType, uAddr, uWidth, count, dptr))
                uCmd = CMD_FAILED;
            else
                uAddr += count; // update uAddr to point to the address following this block
            break;

        case MEM_P:
        case MEM_P_FLASH:
            // write memory
            if (t_write_mem(uType, uAddr, uWidth, count, dptr))
                uCmd = CMD_FAILED;
            else
                uAddr += count / 2; // update uAddr to point to the address following this block
            break;

        case MEM_X:
        case MEM_X_FLASH:
            // write memory
            if (t_write_mem(uType, uAddr, uWidth, count, dptr))
                uCmd = CMD_FAILED;
            else if (uWidth > 8)
                uAddr += count / 2; // update uAddr to point to the address following this block
            else
                uAddr += count; // update uAddr to point to the address following this block
            break;

        case MEM_REG:
            // write A/D register
            if (t_write_ad(uAddr, dptr))
                uCmd = CMD_FAILED;
            break;
        case MEM_CREG:
            // write Control register
            if (t_write_creg(uAddr, dptr))
                uCmd = CMD_FAILED;
            break;
        case MEM_DREG:
            // write Debug register
            if (t_write_dreg(uAddr, uWidth, dptr))
                uCmd = CMD_FAILED;
            break;
        }
        break;

    case CMD_WRITE_NEXT:
        count = g_usb_osbdm_rx_buf[1]; // get byte count for THIS BLOCK
        if (t_write_mem(uType, uAddr, uWidth, count, g_usb_osbdm_rx_buf + 2))
            uCmd = CMD_FAILED;
        else if (uType == MEM_P || uType == MEM_P_FLASH || ((uType == MEM_X || uType == MEM_X_FLASH) && uWidth > 8))
            uAddr += count / 2; // update uAddr to point to the address following this block
        else
            uAddr += count; // update uAddr to point to the address following this block
        break;

    case CMD_FILL:

        get_mem_header(g_usb_osbdm_rx_buf); // parse memory header into variables

        // issue fill command using these variables
        if (t_fill_mem(uType, uAddr, uWidth, (uint32_t)uLen, g_usb_osbdm_rx_buf + MEM_HEADER_SIZE))
            uCmd = CMD_FAILED;
        break;

    case CMD_FLASH_DLSTART:
        // turn ON flash programming voltage
        if (t_flash_power(1))
            uCmd = CMD_FAILED;
        break;

    case CMD_FLASH_DLEND:
        // turn OFF flash programming voltage
        if (t_flash_power(0))
            uCmd = CMD_FAILED;
        break;

    case CMD_FLASH_PROG:
        // execute the flash programming algorythem in target RAM
        if (t_flash_prog(g_usb_osbdm_rx_buf + 1))
            uCmd = CMD_FAILED;
        break;

    // -- Read Memory Commands --------------------------------------
    case CMD_READ_MEM:

        get_mem_header(g_usb_osbdm_rx_buf); // parse memory header into variables
        dptr = g_usb_osbdm_tx_buf + 2;

        dptr[0] = 0; // Preclear the return buffer for 8/16bit reads
        dptr[1] = 0;
        dptr[2] = 0;
        dptr[3] = 0;

        count = (uint8_t)uLen; // determine number of bytes to read

        switch (uType) {
        case MEM_RAM:
            // read memory into g_usb_osbdm_tx_buf[2]...
            if (t_read_mem(uType, uAddr, uWidth, count, dptr)) {
                uCmd = CMD_FAILED;
                break;
            }
            rcount  = count + 2; // number of bytes to return (including header bytes)
            uAddr  += count;     // update uAddr to point to the address following this block
            break;

        case MEM_P:
        case MEM_P_FLASH:
            // read memory into g_usb_osbdm_tx_buf[2]...
            if (t_read_mem(uType, uAddr, uWidth, count, dptr)) {
                uCmd = CMD_FAILED;
                break;
            }
            rcount  = count + 2; // number of bytes to return (including header bytes)
            uAddr  += count / 2; // update uAddr to point to the address following this block
            break;

        case MEM_X:
        case MEM_X_FLASH:
            // read memory into g_usb_osbdm_tx_buf[2]...
            if (t_read_mem(uType, uAddr, uWidth, count, dptr)) {
                uCmd = CMD_FAILED;
                break;
            }
            rcount = count + 2; // number of bytes to return (including header bytes)
            if (uWidth > 8)
                uAddr += count / 2; // update uAddr to point to the address following this block
            else
                uAddr += count; // update uAddr to point to the address following this block
            break;

        case MEM_REG:
            // Read A/D register
            if (t_read_ad(uAddr, dptr))
                uCmd = CMD_FAILED;
            rcount = count + 2; // number of bytes to return (including header bytes)
            break;
        case MEM_CREG:
            // Read Control register
            if (t_read_creg(uAddr, dptr))
                uCmd = CMD_FAILED;
            rcount = count + 2; // number of bytes to return (including header bytes)
            break;
        case MEM_DREG:
            // Read Debug register
            if (t_read_dreg(uAddr, uWidth, dptr))
                uCmd = CMD_FAILED;
            rcount = count + 2; // number of bytes to return (including header bytes)
            break;
        }

        break;

    case CMD_READ_NEXT:
        count = g_usb_osbdm_rx_buf[1];

        // read memory into g_usb_osbdm_tx_buf[2]...
        if (t_read_mem(uType, uAddr, uWidth, count, g_usb_osbdm_tx_buf + 2)) {
            uCmd = CMD_FAILED;
            break;
        }
        rcount = count + 2; // number of bytes to return (including header bytes)
        if (uType == MEM_P || uType == MEM_P_FLASH || ((uType == MEM_X || uType == MEM_X_FLASH) && uWidth > 8))
            uAddr += count / 2; // update uAddr to point to the address following this block
        else
            uAddr += count; // update uAddr to point to the address following this block
                            //			uLen -= count;	// update total byte count to send
        break;

    // -- Peripheral Commands  --------------------------------------
    case CMD_SCI_READ:
        // return count 0 if no data received
        break;

    case CMD_SCI_WRITE:

        break;

    case CMD_JTAG_UNSECURE:
        //			if(t_unsecure())		uCmd = CMD_FAILED;	// unsecure and erase flash
        if (t_unsecure(g_usb_osbdm_rx_buf[1], g_usb_osbdm_rx_buf[2], g_usb_osbdm_rx_buf[3]))
            uCmd = CMD_FAILED; // unsecure and erase flash
        break;

    default:
        uCmd = CMD_UNKNOWN; // command not recognized
        break;
    }

    debug_command_send_return(uCmd,
                              rcount); // send back return condition (command that was executed) along with data, if any
}

/*------------------------------------------------------------------------------------
  command_exec
  ------------
  handle bdm commands
*/

/*
void serial_command_exec(){

  uint8_t command_num;

  command_num =  EP3_Buffer[1];	// store command, this will be returned to host if successful

  // Execute command here

  switch (command_num) {
    case 0x02 :  // get serial port driver version
      serial_command_send_return(0xA3,2);	// 0xAn : A is format indicating response, n is the version of the serial
firmware break; case 0x03 :  // disable serial port SCI_Close(); break; case 0x04 :  // send serial string
      SCI_SendBuffer(EP3_Buffer[2], &(EP3_Buffer[3]));
      break;
    case 0x05 :  // enable serial port
      SCI_Init((EP3_Buffer[3] << 8) | EP3_Buffer[4], EP3_Buffer[5]);
      t_serial_init();
      break;
  }


}

void serial_send_data_if_pending(void){

  if (sci_virtual_serial_port_is_enabled==FALSE)
    return;

  if (
     (Bdtmap.ep4Bio.Stat.McuCtlBit.OWN == 0) &&    // Is USB transmit empty AND
     (SCI_CharReady()==TRUE)           // Serial Data Available
     ) {

       uint32_t numbytes = 0;
       do {
           EP4_Buffer[2+numbytes] = SCI_GetChar();
           numbytes++;
       }
       while ((SCI_CharReady()==TRUE)&&(numbytes+2<UEP4_SIZE));
       serial_command_send_return(SCI_GetOverflowStatus(),numbytes+2);
     }

}
*/
