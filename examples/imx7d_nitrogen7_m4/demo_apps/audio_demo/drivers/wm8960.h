/*
 * Copyright (c) 2016, Timesys Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __WM8960_H__
#define __WM8960_H__

/*! @brief Init wm8960 driver */
void wm8960_init(void);
/*! @brief Read wm8960 register */
uint16_t wm8960_read(uint8_t addr);
/*! @brief Write wm8960 register */
bool wm8960_write_blocking(uint8_t addr, uint16_t data);
/*! @brief Update wm8960 register */
bool wm8960_update_blocking(uint8_t addr, uint16_t mask, uint16_t data);

/* WM8960 register space */
#define WM8960_LINVOL       0x0
#define WM8960_RINVOL       0x1
#define WM8960_LOUT1        0x2
#define WM8960_ROUT1        0x3
#define WM8960_CLOCK1       0x4
#define WM8960_DACCTL1      0x5
#define WM8960_DACCTL2      0x6
#define WM8960_IFACE1       0x7
#define WM8960_CLOCK2       0x8
#define WM8960_IFACE2       0x9
#define WM8960_LDAC         0xa
#define WM8960_RDAC         0xb

#define WM8960_RESET        0xf
#define WM8960_3D           0x10
#define WM8960_ALC1         0x11
#define WM8960_ALC2         0x12
#define WM8960_ALC3         0x13
#define WM8960_NOISEG       0x14
#define WM8960_LADC         0x15
#define WM8960_RADC         0x16
#define WM8960_ADDCTL1      0x17
#define WM8960_ADDCTL2      0x18
#define WM8960_POWER1       0x19
#define WM8960_POWER2       0x1a
#define WM8960_ADDCTL3      0x1b
#define WM8960_APOP1        0x1c
#define WM8960_APOP2        0x1d

#define WM8960_LINPATH      0x20
#define WM8960_RINPATH      0x21
#define WM8960_LOUTMIX      0x22

#define WM8960_ROUTMIX      0x25
#define WM8960_MONOMIX1     0x26
#define WM8960_MONOMIX2     0x27
#define WM8960_LOUT2        0x28
#define WM8960_ROUT2        0x29
#define WM8960_MONO         0x2a
#define WM8960_INBMIX1      0x2b
#define WM8960_INBMIX2      0x2c
#define WM8960_BYPASS1      0x2d
#define WM8960_BYPASS2      0x2e
#define WM8960_POWER3       0x2f
#define WM8960_ADDCTL4      0x30
#define WM8960_CLASSD1      0x31

#define WM8960_CLASSD3      0x33
#define WM8960_PLL1         0x34
#define WM8960_PLL2         0x35
#define WM8960_PLL3         0x36
#define WM8960_PLL4         0x37

#endif /* __WM8960_H__ */
/*******************************************************************************
 * EOF
 ******************************************************************************/
