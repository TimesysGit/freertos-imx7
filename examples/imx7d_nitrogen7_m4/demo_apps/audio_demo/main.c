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

///////////////////////////////////////////////////////////////////////////////
//  Includes
///////////////////////////////////////////////////////////////////////////////
#include "rpmsg/rpmsg_rtos.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "string.h"
#include "board.h"
#include "mu_imx.h"
#include "debug_console_imx.h"
#include "audio.h"
#include "ts_audio_msg.h"

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////
#define APP_TASK_STACK_SIZE 256

/*
 * APP decided interrupt priority
 */
#define APP_MU_IRQ_PRIORITY 3

/* Globals */
static char app_buf[512]; /* Each RPMSG buffer can carry less than 512 payload */

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

static int ParseMsg(ts_audio_msg_t *msg)
{
    return audio_update_coeff(msg->key, msg->value);
}

static void PrintMsg(ts_audio_msg_t *msg)
{
    PRINTF("Message : 0x%04x 0x%08x\n\r", msg->key, msg->value);
}

/*!
 * @brief A basic RPMSG task
 */
static void AudioCtrlTask(void *pvParameters)
{
    int result;
    struct remote_device *rdev = NULL;
    struct rpmsg_channel *app_chnl = NULL;
    void *rx_buf;
    int len;
    unsigned long src;
    void *tx_buf;
    unsigned long size;
    ts_audio_msg_t msg;

    /* Print the initial banner */
    PRINTF("\r\nTimesys AMP Audio Demo for i.MX7\n\n\r");
    PRINTF("SAI Clk = %d Hz\n\r", get_sai_clock_freq(BOARD_I2S_BASEADDR));

    /* RPMSG Init as REMOTE */
    PRINTF("RPMSG Init as Remote\r\n");
    result = rpmsg_rtos_init(0 /*REMOTE_CPU_ID*/, &rdev, RPMSG_MASTER, &app_chnl);
    assert(result == 0);

    PRINTF("Name service handshake is done, M4 has setup a rpmsg channel [%d ---> %d]\r\n", app_chnl->src, app_chnl->dst);

    /*
     * str_echo demo loop
     */
    for (;;)
    {
        /* Get RPMsg rx buffer with message */
        result = rpmsg_rtos_recv_nocopy(app_chnl->rp_ept, &rx_buf, &len, &src, 0xFFFFFFFF);
        assert(result == 0);

        /* Copy string from RPMsg rx buffer */
        assert(len < sizeof(app_buf));
        memcpy(app_buf, rx_buf, len);

        /* Receive message */
        if(!StreamToMsg(app_buf, len, &msg)) {
            PrintMsg(&msg);
            if(ParseMsg(&msg)) {
                PRINTF("Invalid key\n\r");
                /* TODO: Generate failure response */
            }
            /* TODO: Generate success response */
        } else {
            PRINTF("Malformed message!\n\r");
            /* TODO: Generate failure response */
        }

        /* Get tx buffer from RPMsg */
        tx_buf = rpmsg_rtos_alloc_tx_buffer(app_chnl->rp_ept, &size);
        assert(tx_buf);
        /* Copy string to RPMsg tx buffer */
        memcpy(tx_buf, app_buf, len);
        /* Echo back received message with nocopy send */
        result = rpmsg_rtos_send_nocopy(app_chnl->rp_ept, tx_buf, len, src);
        assert(result == 0);

        /* Release held RPMsg rx buffer */
        result = rpmsg_rtos_recv_nocopy_free(app_chnl->rp_ept, rx_buf);
        assert(result == 0);
    }
}

/*
 * MU Interrrupt ISR
 */
void BOARD_MU_HANDLER(void)
{
    /*
     * calls into rpmsg_handler provided by middleware
     */
    rpmsg_handler();
}

/*!
 * @brief Main function
 */
int main(void)
{
    // Initialize demo application pins setting and clock setting.
    hardware_init();

    /*
     * Prepare for the MU Interrupt
     *  MU must be initialized before rpmsg init is called
     */
    MU_Init(BOARD_MU_BASE_ADDR);
    NVIC_SetPriority(BOARD_MU_IRQ_NUM, APP_MU_IRQ_PRIORITY);
    NVIC_EnableIRQ(BOARD_MU_IRQ_NUM);

    /* Create a demo task. */
    xTaskCreate(AudioCtrlTask, "Audio Effects Control Task", APP_TASK_STACK_SIZE,
                NULL, tskIDLE_PRIORITY+1, NULL);

    // Start FreeRTOS scheduler.
    vTaskStartScheduler();

    // Should never reach this point.
    while (true);
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
