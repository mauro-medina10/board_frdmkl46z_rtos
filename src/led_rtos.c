/* Copyright 2019, DSI FCEIA UNR - Sistemas Digitales 2
 *    DSI: http://www.dsi.fceia.unr.edu.ar/
 * Copyright 2019, 2018, Gustavo Muro
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
#include <led_rtos.h>

#include "fsl_port.h"
#include "fsl_gpio.h"
#include "fsl_clock.h"
#include "pin_mux.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

/*==================[macros and definitions]=================================*/

/*==================[internal data declaration]==============================*/

static const board_gpioInfo_type board_gpioLeds[] =
{
    {PORTE, GPIOE, 29},     /* LED ROJO */
    {PORTD, GPIOD, 5},      /* LED VERDE */
};

static uint32_t countLed[BOARD_LED_ID_TOTAL];
static led_conf_enum ledConf[BOARD_LED_ID_TOTAL];

uint8_t n[BOARD_LED_ID_TOTAL];
uint8_t semiPeriod[BOARD_LED_ID_TOTAL];

/*==================[internal functions declaration]=========================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/

void led_Init(void)
{
    uint8_t i;
    for(i = 0; i < BOARD_LED_ID_TOTAL ; i++)
    {
        ledConf[i].msgLed = LED_MSG_OFF;
        countLed[i] = 0;
        semiPeriod[i] = 1;
        n[i] = 0;
    }
}

void led_setConf(led_conf_enum* conf)
{
    ledConf[conf->idLed] = *conf;
    switch (conf->msgLed)
    {
    case LED_MSG_OFF:
        GPIO_PortSet(board_gpioLeds[conf->idLed].gpio, 1<<board_gpioLeds[conf->idLed].pin);
        break;

    case LED_MSG_ON:
        GPIO_PortClear(board_gpioLeds[conf->idLed].gpio, 1<<board_gpioLeds[conf->idLed].pin);
        break;

    case LED_MSG_TOGGLE:
        GPIO_PortToggle(board_gpioLeds[conf->idLed].gpio, 1<<board_gpioLeds[conf->idLed].pin);
        break;

    case LED_MSG_BLINK:
        countLed[conf->idLed] = conf->semiPeriodo;
        break;

    case LED_MSG_HEARTBEAT:
        countLed[conf->idLed] = conf->semiPeriodo;
        semiPeriod[conf->idLed] = 2;
        break;

    case LED_MSG_PULSE_TRAIN:
        countLed[conf->idLed] = conf->semiPeriodo;
        semiPeriod[conf->idLed] = 1;
        n[conf->idLed] = 0;
        break;

    default:
        break;
    }
}

void led_periodicTask1ms(void)
{
    uint8_t i;

    for(i = 0 ; i < BOARD_LED_ID_TOTAL ; i++)
    {
        switch(ledConf[i].msgLed)
        {
        case LED_MSG_BLINK:
            countLed[i]--;
            if(countLed[i] == 0)
            {
                GPIO_PortToggle(board_gpioLeds[ledConf[i].idLed].gpio, 1<<board_gpioLeds[ledConf[i].idLed].pin);
                countLed[i] = ledConf[i].semiPeriodo;
            }
            break;

        case LED_MSG_HEARTBEAT:
            countLed[i]--;
            switch(semiPeriod[i])
            {
            case 1:
                if(countLed[i] == (2*ledConf[i].semiPeriodo/3))
                {
                    GPIO_PortToggle(board_gpioLeds[ledConf[i].idLed].gpio, 1<<board_gpioLeds[ledConf[i].idLed].pin); //aca apaga
                }
                else if(countLed[i] == (ledConf[i].semiPeriodo/3))
                {
                    GPIO_PortToggle(board_gpioLeds[ledConf[i].idLed].gpio, 1<<board_gpioLeds[ledConf[i].idLed].pin); //prende
                }
                else if(countLed[i] == 0)
                {
                    GPIO_PortToggle(board_gpioLeds[ledConf[i].idLed].gpio, 1<<board_gpioLeds[ledConf[i].idLed].pin); //aca apaga
                    semiPeriod[i] = 2;
                    countLed[i] = ledConf[i].semiPeriodo;
                }
                break;
            case 2:
                if(countLed[i] == 0)
                {
                    GPIO_PortToggle(board_gpioLeds[ledConf[i].idLed].gpio, 1<<board_gpioLeds[ledConf[i].idLed].pin); //aca prende
                    semiPeriod[i] = 1;
                    countLed[i] = ledConf[i].semiPeriodo;
                }
                break;

            }
            break;

            case LED_MSG_PULSE_TRAIN:
                countLed[i]--;
                switch(semiPeriod[i])
                {
                case 1: //pulsos
                    if(countLed[i] == 0 && n[i] < (ledConf[i].trainLength*2-1))
                    {
                        GPIO_PortToggle(board_gpioLeds[ledConf[i].idLed].gpio, 1<<board_gpioLeds[ledConf[i].idLed].pin);
                        countLed[i] = ledConf[i].semiPeriodo / (ledConf[i].trainLength*2-1);
                        n[i]++;
                    }
                    if(countLed[i] == 0 && n[i] == (ledConf[i].trainLength*2-1))
                    {
                        GPIO_PortSet(board_gpioLeds[ledConf[i].idLed].gpio, 1<<board_gpioLeds[ledConf[i].idLed].pin);
                        countLed[i] = ledConf[i].semiPeriodo;
                        n[i] = 0;
                        semiPeriod[i] = 2;
                    }
                    break;
                case 2:
                    if(countLed[i] == 0)
                    {
                        GPIO_PortToggle(board_gpioLeds[ledConf[i].idLed].gpio, 1<<board_gpioLeds[ledConf[i].idLed].pin);
                        countLed[i] = ledConf[i].semiPeriodo / ledConf[i].trainLength;
                        semiPeriod[i] = 1;
                    }
                    break;
                }
                break;

                default:
                    break;
        }
    }
}

/*==================[end of file]============================================*/

