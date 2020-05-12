/*
 * Copyright 2016-2020 NXP
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
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
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

/**
 * @file    LPC802_3215_Project.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "LPC802.h"
#include "fsl_debug_console.h"
#include <project.h>
#include <math.h>

void turnoff();
void turnoffDigit();
void digit1();
void digit2();
/* TODO: insert other include files here. */

/* TODO: insert other definitions and declarations here. */

/*
 * @brief   Application entry point.
 */

//array to save all the digit definitions
int digits[4] = { DIGIT_0, DIGIT_1, DIGIT_2, DIGIT_3 };

volatile uint8_t g_I2C0DataBuf[LPC_I2C0BUFFERSize];
volatile uint8_t g_I2C0DataCnt;
uint32_t g_WKT_RELOAD = WKT_RELOAD; // counter reload value for WKT

//LM75 variables
float g_LM75SensorValue = 0;
uint32_t g_LM75SensorValueInt = 0;
uint8_t g_LM75SensorValueChar[4] = { 0 };
uint8_t g_LM75SensorData[2] = { 0 };
volatile uint32_t g_SystemTicks = 0;

//wakeup timer handler
//void WKT_IRQHandler(void) {
//	WKT->CTRL |= WKT_CTRL_ALARMFLAG_MASK;
//	WKT->COUNT = WKT_RELOAD;
//
//	lm75_read(&g_LM75SensorValue, g_LM75SensorData);
//
//	//assign the numbers to be displayed on the seven segment
//	digit_0_num = g_LM75SensorValue / 10;
//	digit_1_num = g_LM75SensorValue; //cannot perform modulo operation of float variable so i split it
//	digit_1_num = digit_1_num % 10;
//

//
//	return;
//}

void SysTick_Handler(void) {
// System Tick ++
	g_SystemTicks++;
//interval display the value
	if (g_SystemTicks >= 5) {
// Read Temperature value from LM75
		lm75_read(&g_LM75SensorValue, g_LM75SensorData);
		digit_0_num = g_LM75SensorValue / 10;
		digit_1_num = g_LM75SensorValue; //cannot perform modulo operation of float variable so i split it
		digit_1_num = digit_1_num % 10;

		if (system_state == 1) {
			if (g_LM75SensorValue >= 30){
				CTIMER0->MR[0] = (CTIMER_PERIOD_TICKS*(1-0.20));
				digit_4_num = 3;
			} else if (g_LM75SensorValue >= 28){

				CTIMER0->MR[0] = (CTIMER_PERIOD_TICKS*(1-0.50));
				digit_4_num = 2;
			} else if (g_LM75SensorValue >= 26) {

				CTIMER0->MR[0] = (CTIMER_PERIOD_TICKS*(1-0.80));
				digit_4_num = 1;
			} else if (g_LM75SensorValue < 26) {


				digit_4_num = 0;
			}
		}

		g_SystemTicks = 0; // reset the number of ticks
	}
}

void PIN_INT0_IRQHandler(void) {
// was an IRQ requested for Channel 0 of GPIO INT?
	if (PINT->IST & (1 << 0)) {
// remove the any IRQ flag for Channel 0 of GPIO INT
		PINT->IST = (1 << 0);
		system_state++;

		if (system_state == 6) {
			system_state = 0;
		}

	} else {
		asm("NOP");
		// Place a breakpt here if debugging.
	}
	return;
}

int main(void) {

	/* Init board hardware. */

	GPIO_init();
	//system_state = (GPIO->B[0][BUTTON_USER1] & 1); //save the system state on or off
	//display_digit(system_state, DIGIT_4); //display the system state on single segment
	//system_state = 0;

	SysTick_init();
	SysTick_Config(2400000);
	lm75_i2c_init();
	SWM_init();
	//WKT_init();

	while (1) {

		/* 'Dummy' NOP to allow source level single stepping of
		 tight while() loop */
		if (system_state == 0) { //fan off and system off
			GPIO->CLR[0] = RELAY;
			display_digit(-1, DIGIT_0);
			display_digit(0, DIGIT_1);
			display_digit(-2, DIGIT_2);
			display_digit(-2, DIGIT_3);
			display_digit(-1, DIGIT_4);
		} else if (system_state == 1){
			if (g_LM75SensorValue >= 26){
				GPIO->SET[0] = RELAY;
			} else {
				GPIO->CLR[0] = RELAY;
			}
			display_digit(digit_0_num, DIGIT_0);
			display_digit(digit_1_num, DIGIT_1);
			display_digit(-1, DIGIT_2);
			display_digit(1, DIGIT_3);
			display_digit(digit_4_num, DIGIT_4);
		} else if (system_state == 2) { //30% PWM start
			GPIO->SET[0] = RELAY;
			display_digit(digit_0_num, DIGIT_0);
			display_digit(digit_1_num, DIGIT_1);
			display_digit(-1, DIGIT_2);
			display_digit(0, DIGIT_3);
			display_digit(1, DIGIT_4);
		} else if (system_state == 3) { //system on and fan speed at 2 50% pwm
			GPIO->SET[0] = RELAY;
			CTIMER0->MR[0] = (CTIMER_PERIOD_TICKS*(1-0.50));
			display_digit(digit_0_num, DIGIT_0);
			display_digit(digit_1_num, DIGIT_1);
			display_digit(-1, DIGIT_2);
			display_digit(0, DIGIT_3);
			display_digit(2, DIGIT_4);

		} else if (system_state == 4) { //system on and fan speed at 3 80% pwm
			GPIO->SET[0] = RELAY;
			CTIMER0->MR[0] = (CTIMER_PERIOD_TICKS*(1-0.20));
			display_digit(digit_0_num, DIGIT_0);
			display_digit(digit_1_num, DIGIT_1);
			display_digit(-1, DIGIT_2);
			display_digit(0, DIGIT_3);
			display_digit(3, DIGIT_4);
		} else { //fan off but system on to display temperature
			GPIO->CLR[0] = RELAY;
			CTIMER0->MR[0] = (CTIMER_PERIOD_TICKS*(1-0.80));

			display_digit(digit_0_num, DIGIT_0);
			display_digit(digit_1_num, DIGIT_1);
			display_digit(-1, DIGIT_2);
			display_digit(-1, DIGIT_3);
			display_digit(-1, DIGIT_4);
		}

		__asm("nop");

	}
	return 0;
}
