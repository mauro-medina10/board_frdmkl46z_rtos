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
#include <board_dsi.h>
#include <i2c.h>
#include <mma8451.h>

#include "fsl_port.h"
#include "fsl_gpio.h"
#include "fsl_clock.h"
#include "pin_mux.h"
#include "board.h"
#include "fsl_debug_console.h"

/*==================[macros and definitions]=================================*/

/*==================[internal data declaration]==============================*/
static const board_gpioInfo_type board_gpioLeds[] =
{
    {PORTE, GPIOE, 29},     /* LED ROJO */
    {PORTD, GPIOD, 5},      /* LED VERDE */
};

static const board_gpioInfo_type board_gpioSw[] =
{
    {PORTC, GPIOC, 3},      /* SW1 */
    {PORTC, GPIOC, 12},     /* SW3 */
};

static uint32_t countLed[BOARD_LED_ID_TOTAL];
static board_ledConf_enum ledConf[BOARD_LED_ID_TOTAL];
/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/
void board_init(void)
{
	int32_t i;
	gpio_pin_config_t gpio_led_config =
	{
		.outputLogic = 1,
		.pinDirection = kGPIO_DigitalOutput,
	};
	gpio_pin_config_t gpio_sw_config = {
		.pinDirection = kGPIO_DigitalInput,
		.outputLogic = 0U
	};

	const port_pin_config_t port_led_config = {
			/* Internal pull-up/down resistor is disabled */
		.pullSelect = kPORT_PullDisable,
		/* Slow slew rate is configured */
		.slewRate = kPORT_SlowSlewRate,
		/* Passive filter is disabled */
		.passiveFilterEnable = kPORT_PassiveFilterDisable,
		/* Low drive strength is configured */
		.driveStrength = kPORT_LowDriveStrength,
		/* Pin is configured as PTC3 */
		.mux = kPORT_MuxAsGpio,
	};

	const port_pin_config_t port_sw_config = {
		/* Internal pull-up resistor is enabled */
		.pullSelect = kPORT_PullUp,
		/* Fast slew rate is configured */
		.slewRate = kPORT_FastSlewRate,
		/* Passive filter is disabled */
		.passiveFilterEnable = kPORT_PassiveFilterDisable,
		/* Low drive strength is configured */
		.driveStrength = kPORT_LowDriveStrength,
		/* Pin is configured as PTC3 */
		.mux = kPORT_MuxAsGpio,
	};

	CLOCK_EnableClock(kCLOCK_PortA);
	CLOCK_EnableClock(kCLOCK_PortC);
	CLOCK_EnableClock(kCLOCK_PortD);
	CLOCK_EnableClock(kCLOCK_PortE);

	/* inicialización de leds */
	for (i = 0 ; i < BOARD_LED_ID_TOTAL ; i++)
	{
		PORT_SetPinConfig(board_gpioLeds[i].port, board_gpioLeds[i].pin, &port_led_config);
		GPIO_PinInit(board_gpioLeds[i].gpio, board_gpioLeds[i].pin, &gpio_led_config);
	}

	board_ledInit();

	/* inicialización de SWs */
	for (i = 0 ; i < BOARD_SW_ID_TOTAL ; i++)
	{
		PORT_SetPinConfig(board_gpioSw[i].port, board_gpioSw[i].pin, &port_sw_config);
		GPIO_PinInit(board_gpioSw[i].gpio, board_gpioSw[i].pin, &gpio_sw_config);
	}
	/* ============ consola debug ===========
	 * -----------------------------------------
	 * Solo se recomienda usar cuando se esta en
	 * el main stack o cuando solo se necesita
	 * mandar un mensaje, ya que, utiliza demasiado stack.
	 * -----------------------------------------*/

	if(DbgConsole_Init(BOARD_DEBUG_UART_BASEADDR,BOARD_DEBUG_UART_BAUDRATE,BOARD_DEBUG_UART_TYPE,BOARD_DEBUG_UART_CLK_FREQ))
	{
	    while(1);
	}

	/* =========== I2C =================== */

	i2c_init();

	/* =========== MMA8451 ================ */
	mma8451_init();
}
void board_ledInit(void)
{
    uint8_t i;
    for(i = 0; i < BOARD_LED_ID_TOTAL ; i++)
    {
        ledConf[i].msgLed = BOARD_LED_MSG_OFF;
        countLed[i] = 0;
    }
}

void board_setLed(board_ledConf_enum* conf)
{
    ledConf[conf->idLed] = *conf;
    switch (conf->msgLed)
    {
        case BOARD_LED_MSG_OFF:
        	GPIO_PortSet(board_gpioLeds[conf->idLed].gpio, 1<<board_gpioLeds[conf->idLed].pin);
            break;

        case BOARD_LED_MSG_ON:
        	GPIO_PortClear(board_gpioLeds[conf->idLed].gpio, 1<<board_gpioLeds[conf->idLed].pin);
            break;

        case BOARD_LED_MSG_TOGGLE:
        	GPIO_PortToggle(board_gpioLeds[conf->idLed].gpio, 1<<board_gpioLeds[conf->idLed].pin);
            break;

        case BOARD_LED_MSG_BLINK:
            countLed[conf->idLed] = conf->semiPeriodo;
            break;

        case BOARD_LED_MSG_HEARTBEAT:
            countLed[conf->idLed] = conf->semiPeriodo;
            ledConf[conf->idLed].trainLength = 2;
            break;

        case BOARD_LED_MSG_PULSE_TRAIN:
            ledConf[conf->idLed].semiPeriodo = conf->semiPeriodo / conf->trainLength;
            countLed[conf->idLed] = ledConf[conf->idLed].semiPeriodo;
            break;

        default:
            break;
    }
}

void board_periodicTask1msLed(void)
{
    uint8_t i, n[] = {0,0};
    uint8_t semiPeriod[] = {1,1};
    for(i = 0 ; i < BOARD_LED_ID_TOTAL ; i++)
    {
        switch(ledConf[i].msgLed)
        {
        case BOARD_LED_MSG_BLINK:
            countLed[i]--;
            if(countLed[i] == 0)
            {
                GPIO_PortToggle(board_gpioLeds[ledConf[i].idLed].gpio, 1<<board_gpioLeds[ledConf[i].idLed].pin);
                countLed[i] = ledConf[i].semiPeriodo;
            }
            break;

        case BOARD_LED_MSG_HEARTBEAT:
            countLed[i]--;
            switch(ledConf[i].trainLength)
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
                    ledConf[i].trainLength = 2;
                    countLed[i] = ledConf[i].semiPeriodo;
                }
                break;
            case 2:
                if(countLed[i] == 0)
                {
                    GPIO_PortToggle(board_gpioLeds[ledConf[i].idLed].gpio, 1<<board_gpioLeds[ledConf[i].idLed].pin); //aca prende
                    ledConf[i].trainLength = 1;
                    countLed[i] = ledConf[i].semiPeriodo;
                }
                break;

            }
            break;

            case BOARD_LED_MSG_PULSE_TRAIN:
                countLed[i]--;
                switch(semiPeriod[i])
                {
                case 1: //pulsos
                    if(countLed[i] == 0)
                    {
                        GPIO_PortToggle(board_gpioLeds[ledConf[i].idLed].gpio, 1<<board_gpioLeds[ledConf[i].idLed].pin);
                        countLed[i] = ledConf[i].semiPeriodo;
                        n[i]++;
                    }
                    if(n[i] == (ledConf->trainLength*2-1))
                    {
                        ledConf[i].semiPeriodo = ledConf[i].semiPeriodo * ledConf[i].trainLength;
                        n[i] = 0;
                        semiPeriod[i] = 2;
                    }
                    break;
                case 2:
                    if(countLed[i] == 0)
                    {
                        GPIO_PortToggle(board_gpioLeds[ledConf[i].idLed].gpio, 1<<board_gpioLeds[ledConf[i].idLed].pin);
                        ledConf[i].semiPeriodo = ledConf[i].semiPeriodo / ledConf[i].trainLength;
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

bool board_getSw(board_swId_enum id)
{
    return !GPIO_ReadPinInput(board_gpioSw[id].gpio, board_gpioSw[id].pin);
}

/*==================[end of file]============================================*/
