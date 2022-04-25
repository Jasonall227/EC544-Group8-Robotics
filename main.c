/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_ftm.h"
#include "fsl_pit.h"
#include "fsl_gpio.h"


// position
float pwid = 25;    // position from 15-90
float p1 = 50;
float p2 = 50;
float p3 = 15;
float p4 = 75;
float p5 = 50;
float p6 = 50;
/*
// rotating direction
int flag = 1;
int f1 = 1;
int f2 = 1;
int f3 = 1;
int f4 = 1;
int f5 = 1;
int f6 = 1;
*/

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define BOARD_LED_GPIO     BOARD_LED_RED_GPIO
#define BOARD_LED_GPIO_PIN BOARD_LED_RED_GPIO_PIN

#define DEMO_PIT_BASEADDR PIT
#define DEMO_PIT_CHANNEL  kPIT_Chnl_0
#define PIT_LED_HANDLER   PIT0_IRQHandler
#define PIT_IRQ_ID        PIT0_IRQn
/* Get source clock for PIT driver */
#define PIT_SOURCE_CLOCK CLOCK_GetFreq(kCLOCK_BusClk)
#define LED_INIT()       LED_RED_INIT(LOGIC_LED_ON)
#define LED_TOGGLE()     LED_RED_TOGGLE()

volatile bool pitIsrFlag = false;
volatile bool ExtInterrupt = false;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*!
 * @brief delay a while.
 */
void delay(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/
void delay(void)
{
    volatile uint32_t i = 0;
    for (i = 0; i < 800000; ++i)
    {
        __asm("NOP"); /* delay */
    }
}

int mydelay(float pwid) {
	int i = 0;
	float cycles = USEC_TO_COUNT(380U, PIT_SOURCE_CLOCK);
	cycles *= pwid;
	cycles /= 100;
	while(i < cycles){
		i++;
		__asm("NOP");
	}
}

void PIT_LED_HANDLER(void)
{
    /* Clear interrupt flag.*/
    PIT_ClearStatusFlags(DEMO_PIT_BASEADDR, DEMO_PIT_CHANNEL, kPIT_TimerFlag);
    pitIsrFlag = true;
    /* Added for, and affects, all PIT handlers. For CPU clock which is much larger than the IP bus clock,
     * CPU can run out of the interrupt handler before the interrupt flag being cleared, resulting in the
     * CPU's entering the handler again and again. Adding DSB can prevent the issue from happening.
     */
    __DSB();
}

/*!
 * @brief Main function
 */

void CustomInit() {
	gpio_pin_config_t out_config = {
	        kGPIO_DigitalOutput,
	        0,
	    };
	gpio_pin_config_t in_config = {
	    	kGPIO_DigitalInput,
			0,
	    };
	/* Port B Clock Gate Control: Clock enabled */
	CLOCK_EnableClock(kCLOCK_PortB);
	CLOCK_EnableClock(kCLOCK_PortC);
	/* PORTB22 (pin 68) is configured as PTB22 */

	PORT_SetPinMux(PORTB, 18U, kPORT_MuxAsGpio); // PTB18: Direction
	PORT_SetPinMux(PORTB, 19U, kPORT_MuxAsGpio); // PTB19: Enable

	// PORT_SetPinMux();

	PORT_SetPinMux(PORTC, 8U, kPORT_MuxAsGpio);  // PTC8  Sel 0
	PORT_SetPinMux(PORTC, 9U, kPORT_MuxAsGpio);  // PTC9: Sel 1
	PORT_SetPinMux(PORTC, 0U, kPORT_MuxAsGpio);  // PTC0: Sel 2

	PORT_SetPinMux(PORTB, 2U, kPORT_MuxAsGpio);  // PWM for Motor 1
	PORT_SetPinMux(PORTB, 3U, kPORT_MuxAsGpio);  // PWM for Motor 2
	PORT_SetPinMux(PORTB, 10U, kPORT_MuxAsGpio); // PWM for Motor 3
	PORT_SetPinMux(PORTB, 11U, kPORT_MuxAsGpio); // PWM for Motor 4
	PORT_SetPinMux(PORTC, 11U, kPORT_MuxAsGpio); // PWM for Motor 5
	PORT_SetPinMux(PORTC, 10U, kPORT_MuxAsGpio); // PWM for Motor 6
	/* ************ Motor Selection  ************ */
	/*          PTC8  PTC9  PTC0     Motor        */
	/*           0     0     0        N/A         */
	/*           0     0     1         1          */
	/*           0     1     0         2          */
	/*           0     1     1         3          */
	/*           1     0     0         4          */
	/*           1     0     1         5          */
	/*           1     1     0         6          */
	/*           1     1     1        N/A         */
	/* ****************************************** */

	GPIO_PinInit(GPIOB, 18, &in_config);
	GPIO_PinInit(GPIOB, 19, &in_config);
	GPIO_PinInit(GPIOC, 8, &in_config);
	GPIO_PinInit(GPIOC, 9, &in_config);
	GPIO_PinInit(GPIOC, 0, &in_config);

	GPIO_PinInit(GPIOB, 2, &out_config);
	GPIO_PinInit(GPIOB, 3, &out_config);
    GPIO_PinInit(GPIOB, 10, &out_config);
    GPIO_PinInit(GPIOB, 11, &out_config);
    GPIO_PinInit(GPIOC, 11, &out_config);
    GPIO_PinInit(GPIOC, 10, &out_config);
}


void InterruptConfig(){
	// Only port C19(Enable) is connected to an external interrupt source
	PORT_SetPinInterruptConfig(PORTB, 19, kPORT_InterruptFallingEdge);
	EnableIRQ(PORTC_IRQn); // interrupt at port C
}

/******* Motor Selection *******/
// MotorNum = Motor_Sel();
int Motor_Sel() {
	int num = 0;
	num += GPIO_PinRead(PORTC, 8) << 2;
	num += GPIO_PinRead(PORTC, 9) << 1;
	num += GPIO_PinRead(PORTC, 0);
	return num;
}

/******* Direction Selection *******/
// dir = Dir_Sel();
int Dir_Sel() {
	return GPIO_PinRead(PORTB, 18);
}

void M1(float p) {
	// Motor 1 rotates to the designated position
	float pwid = p;
	GPIO_PinWrite(GPIOB, 2, 1);
	mydelay(pwid);
	GPIO_PinWrite(GPIOB, 2, 0);
}
void M2(float p) {
	// Motor 2 rotates to the designated position
	float pwid = p;
	GPIO_PinWrite(GPIOB, 3, 1);
	mydelay(pwid);
	GPIO_PinWrite(GPIOB, 3, 0);
}
void M3(float p) {
	// Motor 3 rotates to the designated position
	float pwid = p;
	GPIO_PinWrite(GPIOB, 10, 1);
	mydelay(pwid);
	GPIO_PinWrite(GPIOB, 10, 0);
}
void M4(float p) {
	// Motor 4 rotates to the designated position
	float pwid = p;
	GPIO_PinWrite(GPIOB, 11, 1);
	mydelay(pwid);
	GPIO_PinWrite(GPIOB, 11, 0);
}
void M5(float p) {
	// Motor 5 rotates to the designated position
	float pwid = p;
	GPIO_PinWrite(GPIOC, 11, 1);
	mydelay(pwid);
	GPIO_PinWrite(GPIOC, 11, 0);
}
void M6(float p) {
	// Motor 6 rotates to the designated position
	float pwid = p;
	GPIO_PinWrite(GPIOC, 10, 1);
	mydelay(pwid);
	GPIO_PinWrite(GPIOC, 10, 0);
}


void M1R(float p, int dir, float spd) {
	// Motor 1 rotates to a certain direction
	float temp = p;

	if (dir == 0)
	    temp += spd;
	else if (dir == 1)
		temp -= spd;
	GPIO_PinWrite(GPIOB, 2, 1);
	mydelay(temp);
	GPIO_PinWrite(GPIOB, 2, 0);
	p1 = temp;
}

void M2R(float p, int dir, float spd) {
	float temp = p;

		if (dir == 0)
		    temp += spd;
		else if (dir == 1)
			temp -= spd;
		GPIO_PinWrite(GPIOB, 3, 1);
		mydelay(temp);
		GPIO_PinWrite(GPIOB, 3, 0);
		p2 = temp;
}

void M3R(float p, int dir, float spd) {
	float temp = p;

			if (dir == 0)
			    temp += spd;
			else if (dir == 1)
				temp -= spd;
			GPIO_PinWrite(GPIOB, 10, 1);
			mydelay(temp);
			GPIO_PinWrite(GPIOB, 10, 0);
			p3 = temp;
}

void M4R(float p, int dir, float spd) {
	float temp = p;

			if (dir == 0)
			    temp += spd;
			else if (dir == 1)
				temp -= spd;
			GPIO_PinWrite(GPIOB, 11, 1);
			mydelay(temp);
			GPIO_PinWrite(GPIOB, 11, 0);
			p4 = temp;
}

void M5R(float p, int dir, float spd) {
	float temp = p;

			if (dir == 0)
			    temp += spd;
			else if (dir == 1)
				temp -= spd;
			GPIO_PinWrite(GPIOC, 11, 1);
			mydelay(temp);
			GPIO_PinWrite(GPIOC, 11, 0);
			p5 = temp;
}

void M6R(float p, int dir, float spd) {
	float temp = p;

			if (dir == 0)
			    temp += spd;
			else if (dir == 1)
				temp -= spd;
			GPIO_PinWrite(GPIOC, 10, 1);
			mydelay(temp);
			GPIO_PinWrite(GPIOC, 10, 0);
			p6 = temp;
}
void Claw(int flag) {
	float des = 45;
	// clip
	if (flag == 0)   des = 30;
	// loose
	else if (flag == 1)   des = 60;

	if (des > p6)   M6R(p6, 0, 0.4);
	else if (des < p6)   M6R(p6, 1, 0.4);
}

/*
void Act_Rot(int motor, int dir, float spd, float pos) {
	int d = dir;
	float s = spd;
	float p = pos;

	switch (motor){
	  case 1: M1R(p1, d, s); break;
	  case 2: M2R(p2, d, s); break;
	  case 3: M3R(p3, d, s); break;
	  case 4: M4R(p4, d, s); break;
	  case 5: M5R(p5, d, s); break;
	  case 6: M6R(p6, d, s); break;
	  default: break;
	}
}  */

void Act_Pos(float pos1, float pos2, float pos3, float pos4, float pos5, float pos6) {
	p1 = pos1; p2 = pos2; p3 = pos3; p4 = pos4; p5 = pos5; p6 = pos6;
	M1(p1);
	M2(p2);
	M3(p3);
	M4(p4);
	M5(p5);
	M6(p6);
}

void Arm_Start() {
	p1 = 50; p2 = 50; p3 = 15; p4 = 75; p5 = 55; p6 = 55;
	M1(p1);
	M2(p2);
	M3(p3);
	M4(p4);
	M5(p5);
	M6(p6);
}


void Arm_GentlePos(float des1, float des2, float des3, float des4, float des5) {
	if (des1 > p1)   M1R(p1, 0, 0.1);
		else if (des1 < p1)   M1R(p1, 1, 0.1);
	if (des2 > p2)   M2R(p2, 0, 0.1);
		else if (des2 < p2)   M2R(p2, 1, 0.1);
	if (des3 > p3)   M3R(p3, 0, 0.1);
		else if (des3 < p3)   M3R(p3, 1, 0.1);
	if (des4 > p4)   M4R(p4, 0, 0.1);
		else if (des4 < p4)   M4R(p4, 1, 0.1);
	if (des5 > p5)   M5R(p5, 0, 0.1);
		else if (des5 < p5)   M5R(p5, 1, 0.1);
}

void Arm_Remain() {
	M1(p1);
	M2(p2);
	M3(p3);
	M4(p4);
	M5(p5);
}

int main(void)
{
    /***** Init Structures for Pins *****/
    gpio_pin_config_t out_config = {
        kGPIO_DigitalOutput,
        0,
    };
    gpio_pin_config_t in_config = {
    		kGPIO_DigitalInput,
			0,
    };

    int motor = 1;
    int dir = 0;      // dir = 0 -> rotate positive;   dir = 1 -> rotate negative
    float spd = 0.1;

    pit_config_t pitConfig;

    CustomInit();
    InterruptConfig();


    /* Board pin, clock, debug console init */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitDebugConsole();

    PIT_GetDefaultConfig(&pitConfig);

    	    /* Init pit module */
    	    PIT_Init(DEMO_PIT_BASEADDR, &pitConfig);

    	    /* Set timer period for channel 0 */
    	    // Set timer 5,000,000 us = 5s. Change 5000000U to vary the timer period
    	    PIT_SetTimerPeriod(DEMO_PIT_BASEADDR, DEMO_PIT_CHANNEL, USEC_TO_COUNT(20000U, PIT_SOURCE_CLOCK));

    	    /* Enable timer interrupts for channel 0 */
    	    PIT_EnableInterrupts(DEMO_PIT_BASEADDR, DEMO_PIT_CHANNEL, kPIT_TimerInterruptEnable);

    	    /* Enable at the NVIC */
    	    EnableIRQ(PIT_IRQ_ID);

    	    /* Start channel 0 */
    	    PRINTF("Part 3:\n\r");
    	    PIT_StartTimer(DEMO_PIT_BASEADDR, DEMO_PIT_CHANNEL);

    /* Init output LED GPIO. */
    GPIO_PinInit(BOARD_LED_GPIO, BOARD_LED_GPIO_PIN, &out_config);

    int Dft_Pos = 1;  // one-time flag, initializes the arm_start function
    float des1, des2, des3, des4, des5, des6;
    des1 = 40;
    des2 = 45;
    des3 = 10;
    des4 = 75;
    des5 = 50;

    p1 = 50;
    p2 = 30;
    p3 = 15;
    p4 = 90;
    p5 = 50;
    p6 = 50;

    int count = 0;
    int clawflag = 0;     // claw clip flag: 0 = clip, 1 = loose, 2 = default

    while (1)
    {
        /***** PWM Generation Zone *****/
    	if (true == pitIsrFlag) {
    		/*
    		if(Dft_Pos == 1) {
    			Arm_Start();
    			Arm_Remain();
    			Dft_Pos = 0;
    		} */

    		/***** test zone rotation 1  ******/
    		Arm_GentlePos(des1, des2, des3, des4, des5);
    	    Arm_Remain();
    	    Claw(clawflag);
    		/***** test zone rotation 1  ******/
    	    count ++;
    	    if (count > 100) {
    	    	clawflag = abs(clawflag % 2 - 1);
    	    	count = 0;
    	    	LED_TOGGLE();
    	    }


            pitIsrFlag = false;
    	}
    	/***** External Interrupt Zone *****/

    	if (true == ExtInterrupt) {
    		Claw(0);
    	}

    }

}
