/*
 * Copyright (c) 2016, Timesys Corporation
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

#include "sai_imx7.h"

/*******************************************************************************
 * Code
 ******************************************************************************/

/*******************************************************************************
 * SAI Initialization and Configuration functions
 ******************************************************************************/
/*FUNCTION**********************************************************************
 *
 * Function Name : SAI_Init
 * Description   : Initialize SAI module with given initialize structure.
 *
 *END**************************************************************************/
void SAI_Init(I2S_Type* base, sai_init_config_t* initConfig)
{
    assert(initConfig);


    /* If the transmitter bit clock and frame sync are to be used by
     * both the transmitter and receiver:
     * - The transmitter must be configured for asynchronous operation and the
     *   receiver for synchronous operation.
     * - In synchronous mode, the receiver is enabled only when both the
     *   transmitter and receiver are enabled.
     * - It is recommended that the transmitter is the last enabled and the
     *   first disabled.
     */

    /* Clock configuration
     *
     * Bus Clock = 36.864 MHz
     * MCLK = 12.288 MHz (Generated by Audio PLL)
     * FS = Sample Rate = 48 KHz
     * BCLK = 64 * FS = 3.072 MHz
     */

    /* Put the TX FIFO into a reset state */
    I2S_TCSR_REG(base) = I2S_TCSR_FR_MASK |  /* Reset the FIFO */
                         I2S_TCSR_SR_MASK;   /* Software Reset */
    I2S_TCSR_REG(base) = 0;                  /* Take out of Reset */

    /* Put the RX FIFO into a reset state */
    I2S_RCSR_REG(base) = I2S_RCSR_FR_MASK |  /* Reset the FIFO */
                         I2S_RCSR_SR_MASK;   /* Software Reset */
    I2S_RCSR_REG(base) = 0;                  /* Take out of Reset */

    /* Set up the BCLK */
    I2S_TCR2_REG(base) = I2S_TCR2_SYNC(0) |  /* Asynchronous mode */
                         I2S_TCR2_MSEL(0) |  /* MCLK 0 option */
                         I2S_TCR2_BCP_MASK | /* Sample inputs on rising edge */
                         I2S_TCR2_BCD_MASK | /* Bit clock generated internally */
                         I2S_TCR2_DIV(5);    /* Divide by 12 (Bus Clock / ((DIV + 1) * 2)) */

    /* Set up the FIFO */
    I2S_TCR3_REG(base) = I2S_TCR3_TCE_MASK |  /* Enable channel */
                         I2S_TCR3_WDFL(0);    /* Set first word as start word */

    /* Configure the Frame Sync for I2S operation
     * The frame sync is 64 bits long, divided into two regions for each channel
     */
    I2S_TCR4_REG(base) = I2S_TCR4_FRSZ(1) |   /* 2 words per frame */
                         I2S_TCR4_SYWD(31) |  /* 32 BCLKs per word */
                         I2S_TCR4_MF_MASK |   /* MSB First */
                         I2S_TCR4_FSE_MASK |  /* 1 BCLK delay for first word */
                         I2S_TCR4_FSD_MASK;   /* Frame sync master */

    /* Configure for 32 bit words */
    I2S_TCR5_REG(base) = I2S_TCR5_WNW(31) |   /* Subsequent words are 32 bits */
                         I2S_TCR5_W0W(31) |   /* First word is 32 bits */
                         I2S_TCR5_FBT(31);    /* Bits are left justified in I2S */

    I2S_RCR1_REG(base) = I2S_RCR1_RFW(1);    /* Generate interrupt after 2nd word received */

    /* Set up the bit clock */
    I2S_RCR2_REG(base) = I2S_RCR2_SYNC(1) |  /* Synchronous with Transmitter */
                         I2S_RCR2_MSEL(0) |  /* MCLK 0 option */
                         I2S_RCR2_BCP_MASK | /* Sample inputs on rising edge */
                         I2S_RCR2_BCD_MASK | /* Bit clock generated internally */
                         I2S_RCR2_DIV(5);    /* Divide by 12 (Bus Clock / ((DIV + 1) * 2)) */


    /* Set up the FIFO */
    I2S_RCR3_REG(base) = I2S_RCR3_RCE_MASK |  /* Enable channel */
                         I2S_RCR3_WDFL(0);    /* Set first word as start word */

    /* Configure the Frame Sync for I2S operation
     * The frame sync is 64 bits long, divided into two regions for each channel
     */
    I2S_RCR4_REG(base) = I2S_RCR4_FRSZ(1) |   /* 2 words per frame */
                         I2S_RCR4_SYWD(31) |  /* 32 BCLKs per word */
                         I2S_RCR4_MF_MASK |   /* MSB First */
                         I2S_RCR4_FSE_MASK |  /* 1 BCLK delay for first word */
                         I2S_RCR4_FSD_MASK;   /* Frame sync master */

    /* Configure for 32 bit words */
    I2S_RCR5_REG(base) = I2S_RCR5_WNW(31) |   /* Subsequent words are 32 bits */
                         I2S_RCR5_W0W(31) |   /* First word is 32 bits */
                         I2S_RCR5_FBT(31);    /* Bits are left justified in I2S */

}

/*FUNCTION**********************************************************************
 *
 * Function Name : SAI_Deinit
 * Description   : This function reset SAI module register content to 
 *                 its default value.
 *
 *END**************************************************************************/
void SAI_Deinit(I2S_Type* base)
{
}
