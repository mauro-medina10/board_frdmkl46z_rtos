/* Copyright 2017, DSI FCEIA UNR - Sistemas Digitales 2
 *    DSI: http://www.dsi.fceia.unr.edu.ar/
 * Copyright 2017, Diego Alegrechi
 * Copyright 2017, Gustavo Muro
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
#include "key.h"
#include "FreeRTOS.h"
#include "task.h"

/*==================[macros and definitions]=================================*/

typedef enum
{
    ESPERANDO_ACTIVACION = 0,
    ESPERANDO_DESACTIVACION,
}estPul_enum;

typedef struct
{
    TaskHandle_t taskHandle;
    board_swId_enum swId;
}taskWaitData_type;

#define TASK_WAIT_DATA_ARRAY_LENGTH     5

/*==================[internal data declaration]==============================*/

static estPul_enum estSW[BOARD_SW_ID_TOTAL];
static bool eventSW[BOARD_SW_ID_TOTAL];
static taskWaitData_type taskWaitData[TASK_WAIT_DATA_ARRAY_LENGTH];

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

static void notificarTarea(board_swId_enum id, BaseType_t *pxHigherPriorityTaskWoken)
{
    int i;

    for (i = 0 ; i < TASK_WAIT_DATA_ARRAY_LENGTH ; i++)
    {
        if (taskWaitData[i].taskHandle != NULL && taskWaitData[i].swId == id)
        {
            vTaskNotifyGiveFromISR(taskWaitData[i].taskHandle, pxHigherPriorityTaskWoken);
            taskWaitData[i].taskHandle = NULL;
        }
    }
}

/*==================[external functions definition]==========================*/

void key_init(void)
{
    int32_t i;

    for (i = 0 ; i < BOARD_SW_ID_TOTAL ; i++)
    {
        estSW[i] = ESPERANDO_ACTIVACION;
        eventSW[i] = 0;
    }

    for (i = 0 ; i < TASK_WAIT_DATA_ARRAY_LENGTH ; i++)
    {
        taskWaitData[i].taskHandle = NULL;
    }
}

bool key_getPressEv(board_swId_enum id)
{
    bool ret = false;

    if (eventSW[id])
    {
        eventSW[id] = 0;
        ret = true;
    }

    return ret;
}

bool key_waitForPressEv(board_swId_enum id, TickType_t waitTime)
{
    int i;
    bool ret = false;

    portENTER_CRITICAL();

    /* busca un elemento sin usar dentro del array */
    for (i = 0 ; i < TASK_WAIT_DATA_ARRAY_LENGTH ; i++)
    {
        if (taskWaitData[i].taskHandle == NULL)
            break;
    }

    /* si se encontro un elemento disponible lo usa */
    if (i < TASK_WAIT_DATA_ARRAY_LENGTH)
    {
        taskWaitData[i].taskHandle = xTaskGetCurrentTaskHandle();
        taskWaitData[i].swId = id;
    }
    else
    {
        /* ERROR: NO HAY ELEMENTO DISPONIBLE */
        while (1);
    }

    portEXIT_CRITICAL();

    if (ulTaskNotifyTake(pdTRUE, waitTime))
    {
        /* se obtuvo la notificación */
        ret = true;
    }
    else
    {
        /* no se obtuvo la notificación -> libera elemento */
        taskWaitData[i].taskHandle = NULL;
    }

    return ret;
}

void key_periodicTask1ms(void)
{
    BaseType_t xHigherPriorityTaskWoken;

    xHigherPriorityTaskWoken = pdFALSE;

    int32_t i;

    for (i = 0 ; i < BOARD_SW_ID_TOTAL ; i++)
    {
        switch (estSW[i])
        {
            case ESPERANDO_ACTIVACION:
                if (board_getSw(i))
                {
                    eventSW[i] = 1;
                    notificarTarea(i, &xHigherPriorityTaskWoken);
                    estSW[i] = ESPERANDO_DESACTIVACION;
                }
                break;

            case ESPERANDO_DESACTIVACION:
                if (!board_getSw(i))
                {
                    estSW[i] = ESPERANDO_ACTIVACION;
                }
                break;

            default:
                estSW[i] = ESPERANDO_ACTIVACION;
                break;
        }
    }

    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}


/*==================[end of file]============================================*/
