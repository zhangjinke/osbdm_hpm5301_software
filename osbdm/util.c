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


// utility functions

#include "util.h"


uint16_t ByteSwap16 (uint16_t val)
{
    uint8_t h   = (val >> 8);   // shift high to low and save
    val         = (val & 0xFF); // mask high
    val       <<= 8;            // shift low to high
    return (val + h);           // return them merged
}
uint16_t ByteSwap32 (uint32_t val)
{
    uint32_t a, b, c, d;

    a = val << 24;
    b = (val << 8) & 0x00FF0000;
    c = (val >> 8) & 0x0000FF00;
    d = val >> 24;

    return (a + b + c + d);
}

/*------------------------------------------------------------------------------------
   return a 32-bit value taking 4 bytes from an 8-bit buffer -- LSB first
   so INTEL processors can send us data their way
*/
uint32_t getbuf4 (uint8_t *buf)
{
    uint32_t val = 0;
    uint32_t i;
    buf += 3; // point to last byte
    for (i = 0; i < 4; i++) {
        val <<= 8;
        val  += *(buf--);
    }

    return val;
}
/*------------------------------------------------------------------------------------
  return a 16-bit value taking 2 bytes from an 8-bit buffer
  data is Little-endian
*/
uint16_t getbuf2little (uint8_t *buf)
{
    uint16_t val;
    val   = *(buf + 1); // get high byte
    val <<= 8;
    val  += *buf; // add low byte
    return val;
}
/*------------------------------------------------------------------------------------
    return a 16-bit value taking 2 bytes from an 8-bit buffer
  data is Big-endian
*/
uint16_t getbuf2big (uint8_t *buf)
{
    uint16_t val;
    val   = *(buf); // get high byte
    val <<= 8;
    val  += *(buf + 1); // add low byte
    return val;
}

// todo: use these instead, I think they're more efficient:

uint32_t bigendian4_to_ulong (uint8_t *uc)
{
    return ((((uint32_t)((uc)[0])) << 24) | (((uint32_t)((uc)[1])) << 16) | (((uint32_t)((uc)[2])) << 8) |
            (((uint32_t)((uc)[3]))));
}

uint32_t bigendian2_to_ulong (uint8_t *uc)
{
    return ((((uint32_t)((uc)[0])) << 8) | (((uint32_t)((uc)[1]))));
}
uint32_t littleendian2_to_ulong (uint8_t *uc)
{
    return ((((uint32_t)((uc)[1])) << 8) | (((uint32_t)((uc)[0]))));
}

void ulong_to_bigendian4 (uint32_t ul, uint8_t *uc)
{
    (uc)[0] = (uint8_t)((ul) >> 24);
    (uc)[1] = (uint8_t)((ul) >> 16);
    (uc)[2] = (uint8_t)((ul) >> 8);
    (uc)[3] = (uint8_t)(ul);
}
void ulong_to_littleendian4 (uint32_t ul, uint8_t *uc)
{
    (uc)[3] = (uint8_t)((ul) >> 24);
    (uc)[2] = (uint8_t)((ul) >> 16);
    (uc)[1] = (uint8_t)((ul) >> 8);
    (uc)[0] = (uint8_t)(ul);
}
void ulong_to_bigendian2 (uint32_t ul, uint8_t *uc)
{
    (uc)[0] = (uint8_t)((ul) >> 8);
    (uc)[1] = (uint8_t)(ul);
}
void ulong_to_littleendian2 (uint32_t ul, uint8_t *uc)
{
    (uc)[1] = (uint8_t)((ul) >> 8);
    (uc)[0] = (uint8_t)(ul);
}
