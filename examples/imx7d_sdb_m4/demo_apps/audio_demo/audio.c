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

///////////////////////////////////////////////////////////////////////////////
//  Includes
///////////////////////////////////////////////////////////////////////////////
#include <string.h>
#include <stdlib.h>
#include "FreeRTOS.h"
#include "board.h"
#include "debug_console_imx.h"
#include "sai_imx7.h"
#include "audio.h"

/* Note: Nothing is done with this yet */
static sai_init_config_t saiInitConfig = {
    .clockRate = 12288000,
};

enum channel {
    RIGHT = 0,
    LEFT = 1,
};

static volatile int32_t samp_in[2] = {0, 0};
static volatile int32_t samp_out[2] = {0, 0};

#define DELAY_NUM 1

static int32_t delay[DELAY_NUM];
static int32_t coeffs[COEFF_T_NUM];

enum coeff_cols {
    COEFF_DEFAULT,
    COEFF_MIN,
    COEFF_MAX,
};

#define CIRCBUF_SIZE (0x10000)
#define CIRCBUF_MASK (CIRCBUF_SIZE - 1)
int16_t __attribute__((section(".buffer"))) circbuf[CIRCBUF_SIZE];

static const int32_t coeffs_default[COEFF_T_NUM][3] = {
    /* Default            MIN         MAX */
    {fixedpt_rconst(1.0), 0,          fixedpt_rconst(1.0)},  /* C_GAIN */
    {0,                   0,          1},                    /* C_MUTE */
    {1,                   1,          CIRCBUF_MASK},         /* C_DELAY */
    {fixedpt_rconst(1.0), 0,          fixedpt_rconst(1.0)},  /* C_DECAY */
    {fixedpt_rconst(0.00025), 0,          fixedpt_rconst(1.0)},  /* C_SMOOTH */
};

static uint32_t cb_in = 0;
static uint32_t cb_off = 0;

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

static inline uint32_t cb_offset(uint32_t index, uint32_t inc)
{
    return (index + inc) & CIRCBUF_MASK;
}

static inline void cb_set(int16_t *buffer, uint32_t index, int32_t value)
{
    buffer[index] = value >> 16;
}

static inline int32_t cb_get(int16_t *buffer, uint32_t index)
{
    return (int32_t)buffer[index] << 16;
}

/* All audio processing should occur in this function */
static void ProcessAudio()
{
    int32_t signal;
    int32_t cb_out;

    signal = samp_in[LEFT];

    /* Smooth out the delay offset value */
    if (abs(cb_off - coeffs[C_DELAY]) < 10){
        cb_off = coeffs[C_DELAY];
    } else {
        cb_off = fixedpt_xmul(cb_off, fixedpt_rconst(1.0) - coeffs[C_SMOOTH]);
        cb_off += fixedpt_xmul(coeffs[C_DELAY], coeffs[C_SMOOTH]);
    }

    /* Get the delayed sample */
    cb_out = cb_offset(cb_in, -cb_off);

    /* Mix in the delayed sample */
    signal += fixedpt_xmul(cb_get(circbuf, cb_out), fixedpt_rconst(1.0) - coeffs[C_DECAY]);

    /* Put the new sample into the buffer */
    cb_set(circbuf, cb_in, signal);

    /* Increment the circular buffer for the next sample */
    cb_in = cb_offset(cb_in, 1);

    /* Output gain */
    signal = fixedpt_xmul(signal, coeffs[C_GAIN]);

    /* Output mute */
    if (coeffs[C_MUTE]) {
        samp_out[LEFT] = 0;
        samp_out[RIGHT] = 0;
    } else {
        samp_out[LEFT] = signal;
        samp_out[RIGHT] = signal;
    }
}

#ifdef __DEBUG
static void print_qvalue(int32_t val)
{
        char buf[20];

        fixedpt_str((fixedpt)val, buf, -2);
        PRINTF("%s", buf);
}

void audio_dump_reg()
{
        int32_t printsamp[4];

        printsamp[0] = samp_in[LEFT];
        printsamp[1] = samp_in[RIGHT];
        printsamp[2] = samp_out[LEFT];
        printsamp[3] = samp_out[RIGHT];

        PRINTF("TCSR = %08x\n\r", I2S_TCSR_REG(BOARD_I2S_BASEADDR));
        PRINTF("TCR1 = %08x\n\r", I2S_TCR1_REG(BOARD_I2S_BASEADDR));
        PRINTF("TCR2 = %08x\n\r", I2S_TCR2_REG(BOARD_I2S_BASEADDR));
        PRINTF("TCR3 = %08x\n\r", I2S_TCR3_REG(BOARD_I2S_BASEADDR));
        PRINTF("TCR4 = %08x\n\r", I2S_TCR4_REG(BOARD_I2S_BASEADDR));
        PRINTF("TCR5 = %08x\n\r", I2S_TCR5_REG(BOARD_I2S_BASEADDR));
        PRINTF("RCSR = %08x\n\r", I2S_RCSR_REG(BOARD_I2S_BASEADDR));
        PRINTF("RCR1 = %08x\n\r", I2S_RCR1_REG(BOARD_I2S_BASEADDR));
        PRINTF("RCR2 = %08x\n\r", I2S_RCR2_REG(BOARD_I2S_BASEADDR));
        PRINTF("RCR3 = %08x\n\r", I2S_RCR3_REG(BOARD_I2S_BASEADDR));
        PRINTF("RCR4 = %08x\n\r", I2S_RCR4_REG(BOARD_I2S_BASEADDR));
        PRINTF("RCR5 = %08x\n\r", I2S_RCR5_REG(BOARD_I2S_BASEADDR));
        PRINTF("RFR  = %08x\n\r", I2S_RFR_REG(BOARD_I2S_BASEADDR,0));
        PRINTF("RMR  = %08x\n\r", I2S_RMR_REG(BOARD_I2S_BASEADDR));

        PRINTF("samp_in[LEFT] = 0x%08x\n\r", printsamp[0]);
        PRINTF("samp_in[RIGHT] = 0x%08x\n\r", printsamp[1]);
        PRINTF("samp_out[LEFT] = 0x%08x\n\r", printsamp[2]);
        PRINTF("samp_out[RIGHT] = 0x%08x\n\r", printsamp[3]);
        PRINTF("\n\r");
}
#else
void audio_dump_reg() {};
#endif

void audio_init()
{
    int i;

    /* Initialize coefficients */
    for (i = 0; i < COEFF_T_NUM; i++)
        coeffs[i] = coeffs_default[i][COEFF_DEFAULT];

    /* Clear the buffers */
    memset(delay, 0, DELAY_NUM * sizeof(int32_t));
    memset(circbuf, 0, CIRCBUF_SIZE * sizeof(int16_t));

    SAI_Init(BOARD_I2S_BASEADDR, &saiInitConfig);

    /* Set SAI Interrupt priority */
    NVIC_SetPriority(BOARD_I2S_IRQ_NUM, 3);

    /* Call core API to enable the IRQ. */
    NVIC_EnableIRQ(BOARD_I2S_IRQ_NUM);

    SAI_Enable(BOARD_I2S_BASEADDR);
}

int audio_update_coeff(coeff_t index, int32_t val)
{
    if (index >= COEFF_T_NUM || index < 0)
        return -1;

    if ((val < coeffs_default[index][COEFF_MIN]) ||
        (val > coeffs_default[index][COEFF_MAX]))
            return -1;

    coeffs[index] = val;

    return 0;
}

void BOARD_I2S_HANDLER(void)
{

    SAI_ClearIRQ(BOARD_I2S_BASEADDR);

    /* Write samples */
    SAI_TxData(BOARD_I2S_BASEADDR, (uint32_t)samp_out[LEFT]);
    SAI_TxData(BOARD_I2S_BASEADDR, (uint32_t)samp_out[RIGHT]);

    /* Read samples */
    samp_in[LEFT] = (int32_t)SAI_RxData(BOARD_I2S_BASEADDR);
    samp_in[RIGHT] = (int32_t)SAI_RxData(BOARD_I2S_BASEADDR);

    ProcessAudio();
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
