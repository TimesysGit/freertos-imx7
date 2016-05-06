/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
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

#include "board.h"
#include "pin_mux.h"
#include "ccm_analog_imx7d.h"
#include "wm8960.h"

void hardware_init(void)
{
    /* Board specific RDC settings */
    BOARD_RdcInit();

    /* Board specific clock settings */
    BOARD_ClockInit();

    /* initialize debug uart */
    dbg_uart_init();

    /* Take Audio PLL out of bypass */
    CCM_ANALOG_EnableAudioPll(CCM_ANALOG, 36, 0, 0xd2f00, 0xf4240, 0);

    /* Take exclusive access of SAI */
    RDC_SetPdapAccess(RDC, BOARD_SAI_RDC_PDAP, 3 << (BOARD_DOMAIN_ID * 2), false, false);

    /* Take exclusive access of I2C */
    RDC_SetPdapAccess(RDC, BOARD_I2C_RDC_PDAP, 3 << (BOARD_DOMAIN_ID * 2), false, false);

    /* Select SAI clock derived from Audio Pll */
    CCM_UpdateRoot(CCM, BOARD_SAI_CCM_ROOT, ccmRootmuxSaiAudioPll, 0, 23);

    /* Enable SAI clock */
    CCM_EnableRoot(CCM, BOARD_SAI_CCM_ROOT);
    CCM_ControlGate(CCM, BOARD_SAI_CCM_CCGR, ccmClockNeededRunWait);

    /* Select I2C clock derived from OSC clock(24M) */
    CCM_UpdateRoot(CCM, BOARD_I2C_CCM_ROOT, ccmRootmuxI2cOsc24m, 0, 0);
    /* Enable I2C clock */
    CCM_EnableRoot(CCM, BOARD_I2C_CCM_ROOT);
    CCM_ControlGate(CCM, BOARD_I2C_CCM_CCGR, ccmClockNeededRunWait);

    /* Enable Audio MCLK */
    CCM_UpdateRoot(CCM, ccmRootAudio, ccmRootmuxAudioAudioPll, 7, 8);
    CCM_EnableRoot(CCM, ccmRootAudio);

    configure_sai_pins((I2S_Type *)I2S1_BASE);
    configure_i2c_pins(BOARD_I2C_BASEADDR);

    wm8960_init();
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
