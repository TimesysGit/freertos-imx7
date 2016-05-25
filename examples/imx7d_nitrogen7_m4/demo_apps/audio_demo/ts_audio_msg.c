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
#include "ts_audio_msg.h"

#define TS_AUDIO_MSG_LEN 9
#define TS_AUDIO_MSG_PREAMBLE_OFF 0
#define TS_AUDIO_MSG_PREAMBLE_LEN 2
#define TS_AUDIO_MSG_KEY_OFF 2
#define TS_AUDIO_MSG_VAL_OFF 4
#define TS_AUDIO_MSG_TERM_OFF (TS_AUDIO_MSG_LEN - 1)

#define TS_AUDIO_MSG_TERM '\0'

const char ts_audio_preamble[2] = {'T', 'S'};

int StreamToMsg(char *buf, int len, ts_audio_msg_t *msg)
{
    /* Make sure the string is terminated properly */
    if ((buf[TS_AUDIO_MSG_TERM_OFF] != TS_AUDIO_MSG_TERM) ||
        (buf[0] != ts_audio_preamble[0]) ||
        (buf[1] != ts_audio_preamble[1]) ||
        (len < TS_AUDIO_MSG_LEN)) {
        return 1;
    }

    msg->key =   *(int16_t *)(buf + TS_AUDIO_MSG_KEY_OFF);
    msg->value = *(int32_t *)(buf + TS_AUDIO_MSG_VAL_OFF);
   
    return 0;
}

int MsgToStream(char *buf, int len, ts_audio_msg_t *msg)
{
    if (len < TS_AUDIO_MSG_LEN)
        return 0;

    buf[0] = ts_audio_preamble[0];
    buf[1] = ts_audio_preamble[1];

    *(int16_t *)(buf + TS_AUDIO_MSG_KEY_OFF) = msg->key;
    *(int32_t *)(buf + TS_AUDIO_MSG_VAL_OFF) = msg->value;

    buf[TS_AUDIO_MSG_TERM_OFF] = TS_AUDIO_MSG_TERM;

    return TS_AUDIO_MSG_LEN;
}

