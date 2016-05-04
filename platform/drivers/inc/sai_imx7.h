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

#ifndef __SAI_IMX7_H__
#define __SAI_IMX7_H__

#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include "device_imx.h"

/*!
 * @addtogroup sai_imx7_driver
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @brief SAI module initialize structure. */
typedef struct _sai_init_config
{
    uint32_t clockRate;       /*!< Current SAI module clock freq. */
} sai_init_config_t;



/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name SAI Initialization and Configuration functions
 * @{
 */

/*!
 * @brief Initialize SAI module with given initialize structure.
 *
 * @param base SAI base pointer.
 * @param initConfig SAI initialize structure(see sai_init_config_t above).
 */
void SAI_Init(I2S_Type* base, sai_init_config_t* initConfig);

/*!
 * @brief This function reset SAI module register content to its default value.
 *
 * @param base SAI base pointer.
 */
void SAI_Deinit(I2S_Type* base);

/*!
 * @brief This function is used to Enable the SAI Module.
 *
 * @param base SAI base pointer.
 */
static inline void SAI_Enable(I2S_Type* base)
{
}

/*!
 * @brief This function is used to Disable the SAI Module.
 *
 * @param base SAI base pointer.
 */
static inline void SAI_Disable(I2S_Type* base)
{
}


#ifdef __cplusplus
}
#endif

/*! @}*/

#endif /* __SAI_IMX7_H__ */
/*******************************************************************************
 * EOF
 ******************************************************************************/
