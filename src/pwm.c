/* Copyright 2018, DSI FCEIA UNR - Sistemas Digitales 2
 *    DSI: http://www.dsi.fceia.unr.edu.ar/
 * Copyright 2018, Gustavo Muro
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

#include "pwm.h"
#include "FreeRTOS.h"
#include "task.h"

/*==================[macros and definitions]=================================*/
#define PWM_FREQ 43000U

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/


/*==================[external functions definition]==========================*/

void pwm_init(void)
{
    uint32_t sourceClock;

    const tpm_config_t TPM_0_config = {
      .prescale = kTPM_Prescale_Divide_1,
      .useGlobalTimeBase = false,
      .triggerSelect = kTPM_Trigger_Select_0,
      .enableDoze = false,
      .enableDebugMode = true,
      .enableReloadOnTrigger = false,
      .enableStopOnOverflow = false,
      .enableStartOnTrigger = false,
    };

    tpm_chnl_pwm_signal_param_t pwm_chnl_config;

    pwm_chnl_config.chnlNumber = kTPM_Chnl_4;
    pwm_chnl_config.dutyCyclePercent = 0;
    pwm_chnl_config.level = kTPM_HighTrue;

    TPM_Init(TPM0, &TPM_0_config);

    sourceClock = CLOCK_GetFreq(kCLOCK_Osc0ErClk);

    if(TPM_SetupPwm(TPM0, &pwm_chnl_config, 1, kTPM_EdgeAlignedPwm, PWM_FREQ, sourceClock) == kStatus_Fail){
        printf("Error pwm config");
        while(1);
    }

    TPM_StartTimer(TPM0, kTPM_SystemClock);

}

void pwm_updateDutycycle(uint8_t dutyCyclePercent){
    TPM_UpdatePwmDutycycle(TPM0, kTPM_Chnl_4, kTPM_EdgeAlignedPwm, dutyCyclePercent);
}

void pwm_rtos_init()
{

}

/*==================[end of file]============================================*/
