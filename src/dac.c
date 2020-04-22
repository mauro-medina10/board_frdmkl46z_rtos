/* Copyright 2019, DSI FCEIA UNR
 *    DSI: http://www.dsi.fceia.unr.edu.ar/
 * Copyright 2019, Gustavo Muro
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*==================[inclusions]=============================================*/

// Standard C Included Files
#include <board_dsi.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

// Project Included Files
#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_dac.h"
#include "fsl_port.h"


//FreeRTOS includes
#include "FreeRTOS.h"
#include "semphr.h"
#include "queue.h"
#include "task.h"
#include "timers.h"

/*==================[macros and definitions]=================================*/


/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

void dac_init(){
	dac_config_t dac_config;
	//dac_buffer_config_t dac_buff_config;

	DAC_GetDefaultConfig(&dac_config);
	dac_config.referenceVoltageSource = kDAC_ReferenceVoltageSourceVref1;

	//DAC_GetDefaultBufferConfig(&dac_buff_config);

	DAC_Init(DAC0, &dac_config);

	((*(volatile uint16_t *)DAC0_BASE) = (2045)); //con el buffer inhabilitado el dac saca el calor de DAT0

	DAC_Enable(DAC0, true);

	//DAC_SetBufferConfig(DAC0, &dac_buff_config);

	//DAC_EnableBuffer(DAC0, true);

	//DAC_SetBufferValue(DAC0, 1, 1861);

	//DAC_DoSoftwareTriggerBuffer(DAC0);
}


/*==================[end of file]============================================*/
