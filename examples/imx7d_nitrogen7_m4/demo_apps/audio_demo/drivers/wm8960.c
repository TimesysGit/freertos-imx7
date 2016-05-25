/*
 * Copyright (c) 2015, Timesys Inc.
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

#include <stdint.h>
#include <stdbool.h>
#include "board.h"
#include "debug_console_imx.h"
#include "i2c_imx.h"
#include "wm8960.h"

typedef struct _i2c_state {
    const uint8_t*    cmdBuff;         /*!< The buffer of I2C command. */
    const uint8_t*    txBuff;          /*!< The buffer of data being sent.*/
    uint8_t*          rxBuff;          /*!< The buffer of received data. */
    volatile uint32_t cmdSize;         /*!< The remaining number of commands to be transmitted. */
    volatile uint32_t txSize;          /*!< The remaining number of bytes to be transmitted. */
    volatile uint32_t rxSize;          /*!< The remaining number of bytes to be received. */
    volatile bool     isBusy;          /*!< True if there is an active transmission. */
    volatile uint32_t operateDir;      /*!< Overall I2C bus operating direction. */
    volatile uint32_t currentDir;      /*!< Current Data transfer direction. */
    volatile uint32_t currentMode;     /*!< Current I2C Bus role of this module. */
} i2c_state_t;

/* I2C runtime state structure */
static i2c_state_t i2cState;

static uint8_t txBuffer[2];
static uint8_t cmdBuffer[3];

/* Default/reset values of wm8960 registers */
static uint16_t wm8960_reg_cache[] = {
    0x00a7,
    0x00a7,
    0x0000,
    0x0000,
    0x0000,
    0x0008,
    0x0000,
    0x000a,
    0x01c0,
    0x0000,
    0x00ff,
    0x00ff,
    0x0000,
    0x0000,
    0x0000,
    0x0000,
    0x0000,
    0x007b,
    0x0100,
    0x0032,
    0x0000,
    0x00c3,
    0x00c3,
    0x01c0,
    0x0000,
    0x0000,
    0x0000,
    0x0000,
    0x0000,
    0x0000,
    0x0000,
    0x0000,
    0x0100,
    0x0100,
    0x0050,
    0x0050,
    0x0050,
    0x0050,
    0x0000,
    0x0000,
    0x0000,
    0x0000,
    0x0040,
    0x0000,
    0x0000,
    0x0050,
    0x0050,
    0x0000,
    0x0002,
    0x0037,
    0x004d,
    0x0080,
    0x0008,
    0x0031,
    0x0026,
    0x00e9,
};

static void I2C_XFER_Config(i2c_init_config_t* initConfig);
static bool I2C_XFER_Write(const uint8_t* cmdBuff, uint32_t cmdSize, const uint8_t* txBuffer, uint32_t txSize);
static bool I2C_XFER_IsBusy(void);

void wm8960_init(void)
{
    uint32_t delay;
    /* Setup I2C init structure. */
    i2c_init_config_t i2cInitConfig = {
        .baudRate     = 100000u,
        .slaveAddress = 0x00
    };

    /* Get current module clock frequency. */
    i2cInitConfig.clockRate = get_i2c_clock_freq(BOARD_I2C_BASEADDR);

    I2C_XFER_Config(&i2cInitConfig);

    /* Finally, enable the I2C module */
    I2C_Enable(BOARD_I2C_BASEADDR);

    /* Enable Vmid and Vref voltages */
    wm8960_update_blocking(0x19, 0x180, 0x080); //Vmid 2x50k divider
    /* Wait 100ms for stable Vmid. TODO: The cycle number should be changed
     * according to M4 Core clock frequncy.
     */
    for (delay = 0 ; delay < 1000000; delay++)
    {
        __NOP();
    }

    wm8960_update_blocking(0x19, 0x40, 0x40); //VREF enable
    for (delay = 0 ; delay < 100000; delay++)
    {
        __NOP();
    }

    wm8960_update_blocking(0x07, 0xc, 0x8); //24 bits
    wm8960_update_blocking(0x07, 0x3, 0x2); //I2S Mode
    wm8960_update_blocking(0x09, 0x40, 0x40); //ALRCGPIO=1 (Use DACLR for both)

    /* Enable left microphone path */
    wm8960_update_blocking(0x19, 0x20, 0x20); //AINL = 1
    wm8960_update_blocking(0x2F, 0x20, 0x20); //LMIC =1
    wm8960_update_blocking(0x00, 0x80, 0x00); //LINMUTE = 0
    wm8960_update_blocking(0x20, 0x8, 0x8); //LMIC2B = 1
    //wm8960_update_blocking(0x20, 0x30, 0x30); //LMICBOOST[1:0] = +29dB
    //wm8960_update_blocking(0x00, 0x3F, 0x3F); //LINVOL[5:0]  = +30dB
    wm8960_update_blocking(0x00, 0x100, 0x100); //IPVU = 1

    /* Left output setup */
    wm8960_update_blocking(0x2F, 0x8, 0x8); //LOMIX = 1
    wm8960_update_blocking(0x02, 0x7F, 0x79); //LOUT1VOL[6:0] = 0dB
    wm8960_update_blocking(0x1A, 0x40, 0x40); //LOUT1 = 1
    wm8960_update_blocking(0x02, 0x100, 0x100); //Left OUT1VU = 1

    /* Right output setup */
    wm8960_update_blocking(0x2F, 0x4, 0x4); //ROMIX = 1
    wm8960_update_blocking(0x03, 0x100, 0x100); //Right OUT1VU = 1
    wm8960_update_blocking(0x03, 0x7F, 0x79); //ROUT1VOL[6:0] = 0dB
    wm8960_update_blocking(0x1A, 0x20, 0x20); //ROUT1 = 1

#ifdef MIC_BYPASS_TO_MIXER
    /* Test code to bypass ADC/DAC and connect mic to output mixer */
    wm8960_update_blocking(0x2D, 0x80, 0x80); //LB2LO = 1
    wm8960_update_blocking(0x2D, 0x70, 0x00); //LB2LOVOL[2:0] = 0dB
#else
    /* Disable bypass */
    wm8960_update_blocking(0x2D, 0x80, 0x00); //LB2LO = 0

    /* ADC setup */
    wm8960_update_blocking(0x19, 0xC, 0xC); //ADCL = 1, ADCR = 1
    wm8960_update_blocking(0x17, 0xC, 0x4); //DATSEL[1:0] left, right data = left ADC
    wm8960_update_blocking(0x15, 0x100, 0x100); //Left ADCVU = 1
    wm8960_update_blocking(0x16, 0x100, 0x100); //Right ADCVU = 1
    /* DAC setup */
    wm8960_update_blocking(0x1A, 0x180, 0x180); //DACL = 1, DACR = 1
    wm8960_update_blocking(0x0A, 0xFF, 0xFF); //Left DAC Volume = 0db
    wm8960_update_blocking(0x0B, 0xFF, 0xFF); //Right DAC Volume = 0db
    wm8960_update_blocking(0x0A, 0x100, 0x100); //Left DACVU = 1
    wm8960_update_blocking(0x0B, 0x100, 0x100); //Right DACVU = 1
    wm8960_update_blocking(0x05, 0x8, 0x0); //DACMU = 0
    /* Output mixer source */
    wm8960_update_blocking(0x22, 0x100, 0x100); //LD2LO = 1
    wm8960_update_blocking(0x25, 0x100, 0x100); //RD2RO = 1
#endif

    PRINTF("WM8960 codec initialized\n\r");
}

uint16_t wm8960_read(uint8_t addr)
{
    return wm8960_reg_cache[addr];
}

static bool wm8960_write(uint8_t addr, uint16_t data, bool blocking)
{
    cmdBuffer[0] = BOARD_I2C_WM8960_ADDR << 1;
    cmdBuffer[1] = (addr << 1) | (uint8_t)((data >> 8) & 0x1);
    txBuffer[0]  = (uint8_t)(data);

    if (I2C_XFER_Write(cmdBuffer, 2, txBuffer, 1)) {
        if (blocking)
            while(I2C_XFER_IsBusy());
        wm8960_reg_cache[addr] = data;
        PRINTF("WM8960 codec: Set register 0x%02x to 0x%04x\n\r",addr, data);
        return true;
    } else {
        return false;
    }
}

bool wm8960_write_blocking(uint8_t addr, uint16_t data)
{
    return wm8960_write(addr, data, true);
}

static bool wm8960_update(uint8_t addr, uint16_t mask, uint16_t data, bool blocking)
{
    uint16_t temp = wm8960_reg_cache[addr];
    temp &= ~mask;
    temp |= (mask & data);
    return wm8960_write(addr, temp, blocking);
}

bool wm8960_update_blocking(uint8_t addr, uint16_t mask, uint16_t data)
{
    return wm8960_update(addr, mask, data, true);
}

static void I2C_XFER_Config(i2c_init_config_t* initConfig)
{
    /* Initialize I2C state structure content. */
    i2cState.cmdBuff = 0;
    i2cState.txBuff = 0;
    i2cState.rxBuff = 0;
    i2cState.cmdSize = 0;
    i2cState.txSize = 0;
    i2cState.rxSize = 0;
    i2cState.isBusy = false;
    i2cState.operateDir = i2cDirectionReceive;
    i2cState.currentDir = i2cDirectionReceive;
    i2cState.currentMode = i2cModeSlave;

    /* Initialize I2C baud rate, mode, transfer direction and slave address. */
    I2C_Init(BOARD_I2C_BASEADDR, initConfig);

    /* Set I2C Interrupt priority */
    NVIC_SetPriority(BOARD_I2C_IRQ_NUM, 3);

    /* Call core API to enable the IRQ. */
    NVIC_EnableIRQ(BOARD_I2C_IRQ_NUM);

    /* Finally, enable the I2C module */
    I2C_Enable(BOARD_I2C_BASEADDR);
}

static bool I2C_XFER_Write(const uint8_t* cmdBuff, uint32_t cmdSize,
                      const uint8_t* txBuffer, uint32_t txSize)
{
    if ((i2cState.isBusy) || (0 == txSize))
        return false;

    /* Initialize i2c transfer struct */
    i2cState.cmdBuff = cmdBuff;
    i2cState.cmdSize = cmdSize;
    i2cState.txBuff = txBuffer;
    i2cState.txSize = txSize;
    i2cState.isBusy = true;
    i2cState.operateDir = i2cDirectionTransmit;

    /* Clear I2C interrupt flag to avoid spurious interrupt */
    I2C_ClearStatusFlag(BOARD_I2C_BASEADDR, i2cStatusInterrupt);

    if (I2C_GetStatusFlag(BOARD_I2C_BASEADDR, i2cStatusBusBusy))
    {
        /* Reset i2c transfer state. */
        i2cState.operateDir = i2cDirectionReceive;
        i2cState.isBusy = false;
        return false;
    }

    /* Set I2C work under Tx mode */
    I2C_SetDirMode(BOARD_I2C_BASEADDR, i2cDirectionTransmit);
    i2cState.currentDir = i2cDirectionTransmit;

    /* Switch to Master Mode and Send Start Signal. */
    I2C_SetWorkMode(BOARD_I2C_BASEADDR, i2cModeMaster);
    i2cState.currentMode = i2cModeMaster;

    if (0 != cmdSize)
    {
        I2C_WriteByte(BOARD_I2C_BASEADDR, *i2cState.cmdBuff);
        i2cState.cmdBuff++;
        i2cState.cmdSize--;
    }
    else
    {
        I2C_WriteByte(BOARD_I2C_BASEADDR, *i2cState.txBuff);
        i2cState.txBuff++;
        i2cState.txSize--;
    }

    /* Enable I2C interrupt, subsequent data transfer will be handled in ISR. */
    I2C_SetIntCmd(BOARD_I2C_BASEADDR, true);

    return true;
}

static bool I2C_XFER_IsBusy(void)
{
    return i2cState.isBusy;
}

void BOARD_I2C_HANDLER(void)
{
    /* Clear interrupt flag. */
    I2C_ClearStatusFlag(BOARD_I2C_BASEADDR, i2cStatusInterrupt);

    /* Exit the ISR if no transfer is happening for this instance. */
    if (!i2cState.isBusy)
        return;

    if (i2cModeMaster == i2cState.currentMode)
    {
        if (i2cDirectionTransmit == i2cState.currentDir)
        {
            if ((I2C_GetStatusFlag(BOARD_I2C_BASEADDR, i2cStatusReceivedAck)) ||
                ((0 == i2cState.txSize) && (0 == i2cState.cmdSize)))
            {
                if ((i2cDirectionTransmit == i2cState.operateDir) ||
                    (I2C_GetStatusFlag(BOARD_I2C_BASEADDR, i2cStatusReceivedAck)))
                {
                    /* Switch to Slave mode and Generate a Stop Signal. */
                    I2C_SetWorkMode(BOARD_I2C_BASEADDR, i2cModeSlave);
                    i2cState.currentMode = i2cModeSlave;

                    /* Switch back to Rx direction. */
                    I2C_SetDirMode(BOARD_I2C_BASEADDR, i2cDirectionReceive);
                    i2cState.currentDir = i2cDirectionReceive;

                    /* Close I2C interrupt. */
                    I2C_SetIntCmd(BOARD_I2C_BASEADDR, false);
                    /* Release I2C Bus. */
                    i2cState.isBusy = false;
                }
                else
                {
                    /* Switch back to Rx direction. */
                    I2C_SetDirMode(BOARD_I2C_BASEADDR, i2cDirectionReceive);
                    i2cState.currentDir = i2cDirectionReceive;

                    if (1 == i2cState.rxSize)
                        /* Send Nack */
                        I2C_SetAckBit(BOARD_I2C_BASEADDR, false);
                    else
                        /* Send Ack */
                        I2C_SetAckBit(BOARD_I2C_BASEADDR, true);
                    /* dummy read to clock in 1st byte */
                    *i2cState.rxBuff = I2C_ReadByte(BOARD_I2C_BASEADDR);
                }
            }
            else
            {
                if (0 != i2cState.cmdSize)
                {
                    if ((1 == i2cState.cmdSize) && (i2cDirectionReceive == i2cState.operateDir))
                        I2C_SendRepeatStart(BOARD_I2C_BASEADDR);
                    I2C_WriteByte(BOARD_I2C_BASEADDR, *i2cState.cmdBuff);
                    i2cState.cmdBuff++;
                    i2cState.cmdSize--;
                }
                else
                {
                    I2C_WriteByte(BOARD_I2C_BASEADDR, *i2cState.txBuff);
                    i2cState.txBuff++;
                    i2cState.txSize--;
                }
            }
        }
        else
        {
            /* Normal read operation. */
            if (2 == i2cState.rxSize)
                /* Send Nack */
                I2C_SetAckBit(BOARD_I2C_BASEADDR, false);
            else
                /* Send Nack */
                I2C_SetAckBit(BOARD_I2C_BASEADDR, true);

            if (1 == i2cState.rxSize)
            {
                /* Switch back to Tx direction to avoid additional I2C bus read. */
                I2C_SetDirMode(BOARD_I2C_BASEADDR, i2cDirectionTransmit);
                i2cState.currentDir = i2cDirectionTransmit;
            }
            *i2cState.rxBuff = I2C_ReadByte(BOARD_I2C_BASEADDR);
            i2cState.rxBuff++;
            i2cState.rxSize--;

            /* receive finished. */
            if (0 == i2cState.rxSize)
            {
                /* Switch to Slave mode and Generate a Stop Signal. */
                I2C_SetWorkMode(BOARD_I2C_BASEADDR, i2cModeSlave);
                i2cState.currentMode = i2cModeSlave;

                /* Switch back to Rx direction. */
                I2C_SetDirMode(BOARD_I2C_BASEADDR, i2cDirectionReceive);
                i2cState.currentDir = i2cDirectionReceive;

                /* Close I2C interrupt. */
                I2C_SetIntCmd(BOARD_I2C_BASEADDR, false);
                /* Release I2C Bus. */
                i2cState.isBusy = false;
            }
        }
    }
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
