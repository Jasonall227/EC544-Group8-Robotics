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
#include "peripherals.h"


// position
// float pwid = 25;    // position from 15-90
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
#define step 0.1
#define EnableTerminalDebug 0

#define V_max 3.300
#define Vdig_max 4096.0

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

void EXT_Interrupt_IRQ(void)
{
#if (defined(FSL_FEATURE_PORT_HAS_NO_INTERRUPT) && FSL_FEATURE_PORT_HAS_NO_INTERRUPT)
    /* Clear external interrupt flag. */
    GPIO_GpioClearInterruptFlags(BOARD_SW_GPIO, 1U << BOARD_SW_GPIO_PIN);
#else
    /* Clear external interrupt flag. */
    GPIO_PortClearInterruptFlags(GPIOB, 1U << 19U);
#endif
    /* Change state of button. */
    ExtInterrupt = true;
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

	PORT_SetPinMux(PORTC, 5U, kPORT_MuxAsGpio);  // ref out high
	PORT_SetPinMux(PORTC, 7U, kPORT_MuxAsGpio);  // ref out high

	PORT_SetPinMux(PORTB, 18U, kPORT_MuxAsGpio); // PTB18: Direction
	PORT_SetPinMux(PORTB, 19U, kPORT_MuxAsGpio); // PTB19: Enable

	// PORT_SetPinMux();

	PORT_SetPinMux(PORTC, 1U, kPORT_MuxAsGpio);  // PTC1  Sel 0
	PORT_SetPinMux(PORTC, 8U, kPORT_MuxAsGpio);  // PTC8: Sel 1
	PORT_SetPinMux(PORTC, 9U, kPORT_MuxAsGpio);  // PTC9: Sel 2
	PORT_SetPinMux(PORTC, 0U, kPORT_MuxAsGpio);  // PTC0  PWM in

	PORT_SetPinMux(PORTB, 2U, kPORT_MuxAsGpio);  // PWM for Motor 1
	PORT_SetPinMux(PORTB, 3U, kPORT_MuxAsGpio);  // PWM for Motor 2
	PORT_SetPinMux(PORTB, 10U, kPORT_MuxAsGpio); // PWM for Motor 3
	PORT_SetPinMux(PORTB, 11U, kPORT_MuxAsGpio); // PWM for Motor 4
	PORT_SetPinMux(PORTC, 11U, kPORT_MuxAsGpio); // PWM for Motor 5
	PORT_SetPinMux(PORTC, 10U, kPORT_MuxAsGpio); // PWM for Motor 6
	/* ************ Motor Selection  ************ */
	/*          PTC1  PTC8  PTC9     Motor        */
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
	GPIO_PinInit(GPIOC, 1, &in_config);

	GPIO_PinInit(GPIOB, 2, &out_config);
	GPIO_PinInit(GPIOB, 3, &out_config);
    GPIO_PinInit(GPIOB, 10, &out_config);
    GPIO_PinInit(GPIOB, 11, &out_config);
    GPIO_PinInit(GPIOC, 11, &out_config);
    GPIO_PinInit(GPIOC, 10, &out_config);

    GPIO_PinInit(GPIOC, 5, &out_config);  // high out ref
    GPIO_PinInit(GPIOC, 7, &out_config);
}


void InterruptConfig(){
	// Only port C19(Enable) is connected to an external interrupt source
	PORT_SetPinInterruptConfig(PORTB, 19, kPORT_InterruptFallingEdge);
	EnableIRQ(PORTC_IRQn); // interrupt at port C
	EnableIRQ(PORTB_IRQn); // interrupt at port B
}

/******* Motor Selection *******/
// MotorNum = Motor_Sel();
int Motor_Sel() {
	int num = 0;
	// num += GPIO_PinRead(PORTC, 1) << 2;
	// num += GPIO_PinRead(PORTC, 8) << 1;
	// num += GPIO_PinRead(PORTC, 9);
	num += GPIO_PinRead(GPIOC, 1) << 2;
	num += GPIO_PinRead(GPIOC, 8) << 1;
	num += GPIO_PinRead(GPIOC, 9);
	return num;
}

/******* Direction Selection *******/
// dir = Dir_Sel();
int Dir_Sel() {
	return GPIO_PinRead(GPIOB, 18);
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
void Claw(int flag) {   // need to be done consistantly. This function integrates M6 in it
	float des = 45;
	// loose
	if (flag == 0)   des = 35;
	// clip
	else if (flag == 1)   des = 68;
	else ;

	if (des > p6)   M6R(p6, 0, 0.1);
	else if (des < p6)   M6R(p6, 1, 0.1);
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
	p1 = pos1; p2 = pos2; p3 = pos3; p4 = pos4; p5 = pos5;
	int flag = 2;
	if (pos6 > 70)  flag = 0;  // clip
	if (pos6 < 30)  flag = 1;  // loose
	// else nop
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

    int motor = 0;
    int dir = 0;      // dir = 0 -> rotate positive;   dir = 1 -> rotate negative
    int enable = 0;
    float spd = 0.1;

    pit_config_t pitConfig;

    CustomInit();
    InterruptConfig();


    /* Board pin, clock, debug console init */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitDebugConsole();
    BOARD_InitPeripherals();

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

    	    EnableIRQ(PORTB_IRQn);
    	    EXT_Interrupt_IRQ();

    /* Init output LED GPIO. */
    GPIO_PinInit(BOARD_LED_GPIO, BOARD_LED_GPIO_PIN, &out_config);

    int Dft_Pos = 1;  // one-time flag, initializes the arm_start function
    float des1, des2, des3, des4, des5, des6;
    des1 = 40;
    des2 = 65;
    des3 = 10;
    des4 = 75;
    des5 = 50;
    des6 = 25;
    float Pos_input = 0;   // range: 0-4096    mapping to position 15-90

    p1 = 50;
    p2 = 60;
    p3 = 15;
    p4 = 90;
    p5 = 50;
    p6 = 40;

    int count = 0;
    int clawflag = 2;     // claw clip flag: 0 = clip, 1 = loose, 2 = default

    ADC16_SetChannelConfig(BOARD_ADC16_PERIPHERAL,
    		0U, &BOARD_ADC16_channelsConfig[0]);

    GPIO_PinWrite(GPIOC, 5, 1);
    GPIO_PinWrite(GPIOC, 7, 0);
    char msg;
    int Motor_selected = 2;
    int countdown = 0;


    while (1)
    {
        /***** PWM Generation Zone *****/
    	if (true == pitIsrFlag) {
    		/***************** Judge block version 1 *****************/
            /*
    		enable = GPIO_PinRead(GPIOB, 19);
    		if (enable == 1) {    // if enabled
    			motor = Motor_Sel();
    			dir = Dir_Sel();
    			switch (motor) {
    			case 1: if (dir == 1) des1 +=step; else des1 -=step; break;
    			case 2: if (dir == 1) des2 +=step; else des2 -=step; break;
    			case 3: if (dir == 1) des3 +=step; else des3 -=step; break;
    			case 4: if (dir == 1) des4 +=step; else des4 -=step; break;
    			case 5: if (dir == 1) des5 +=step; else des5 -=step; break;
    			case 6: if (dir == 1) clawflag = 0; break;  // select 6, dir = 1 then grab
    			case 7: if (dir == 1) clawflag = 1; break;  // select 7, dir = 1 then loose
    			default: break;
    			}
    		}
            */
    		/***************** Judge block version 1 ends *****************/

    		/***************** Judge block version 2 *****************/

    		enable = GPIO_PinRead(GPIOB, 19);
    		if (enable == 1) {
    			Pos_input = ADC16_GetChannelConversionValue(BOARD_ADC16_PERIPHERAL,
    				0U);
    			motor = Motor_Sel();
    			dir = Dir_Sel();
    			switch(motor) {
    			case 1: des1 = Pos_input * 75 / 4096 + 15; break;  // mapping 0-4096 to 15-90
    			case 2: des2 = Pos_input * 75 / 4096 + 15; break;
    			case 3: des3 = Pos_input * 75 / 4096 + 15; break;
    			case 4: des4 = Pos_input * 75 / 4096 + 15; break;
    			case 5: des5 = Pos_input * 75 / 4096 + 15; break;
    			case 6: des6 = Pos_input * 75 / 4096 + 15; break;
    			default: break;
    			}
    			if (des6 <= 30) clawflag = 0;  // clip
    			else if (des6 >= 70) clawflag = 1;  // loose
    			else clawflag = 2;
    		}

    		/***************** Judge block version 2 ends *****************/
    		/***** test zone rotation 1  ******/
    		Arm_GentlePos(des1, des2, des3, des4, des5);
    		if (des6 <= 30)  clawflag = 0;
    		else if (des6 >= 70) clawflag = 1;
    		else clawflag = 2;
    	    Claw(clawflag);
    		/***** test zone rotation 1  ******/

    	    count ++;
    	    if (count > 20) {
    	    	//clawflag = abs(clawflag % 2 - 1);
    	    	count = 0;
    	    	countdown ++;
    	    	volatile int Pos_input_2 = 1000;
    	    	Pos_input_2 = ADC16_GetChannelConversionValue(BOARD_ADC16_PERIPHERAL,
    	    	    				0U);
    	    	PRINTF("input value: %d/4096", Pos_input_2);
    	    	PRINTF("  motor: %d,   dir: %d,  enable: %d\n\r", motor, dir, enable);
    	    	LED_TOGGLE();
    	    }

    	    /***********  Demo: grab and present  ***********/
    	    /*
    	    if (countdown > 20) {
    	    	des1 = 30;
    	    	des2 = 85;
    	    	des3 = 10;
    	    	des4 = 75;
    	    	des5 = 50;
    	    	des6 = 25;
    	    }
    	    if (countdown > 30) {
    	    	des6 = 75;
    	    }
    	    if (countdown > 40) {
    	        	    	des1 = 30;
    	        	    	des2 = 60;
    	        	    	des3 = 10;
    	        	    	des4 = 75;
    	        	    	des5 = 50;
    	        	    	des6 = 75;
    	        	    }
    	    if (countdown > 55) {
    	        	        des1 = 56;
    	        	        des2 = 55;
    	        	        des3 = 10;
    	        	        des4 = 75;
    	        	        des5 = 50;
    	        	        des6 = 75;
    	        	    }
    	    if (countdown > 70) {
    	        des6 = 25;
    	    }
    	    */
    	    /***********  Demo: grab and present  ***********/

            /*
            if (EnableTerminalDebug == 1) {
              if (Motor_selected == 0) {
            	PRINTF("\rSelect Motor: \n");
            	char msg = ' ';
            	msg = GETCHAR();
            	switch (msg)
            	{
            	            case '1': motor = 1; break;
            	            case '2': motor = 2; break;
            	            case '3': motor = 3; break;
            	            case '4': motor = 4; break;
            	            case '5': motor = 5; break;
            	            default:
            	                PRINTF("\r\nPlease input a valid number: 1-5 \r\n");
            	                break;
            	}
            	Motor_selected = 1;
              }
              else {
            	  PRINTF("\rSelect Direction: \n");
            	msg = GETCHAR();
            	switch (msg) {
            	case'1': dir = 1; break;
            	case'0': dir = 0; break;
            	default: PRINTF("\r\nPlease input a valid number: 0-1 \r\n"); break;
            	}
            	Motor_selected = 0;
              }
            } */
    	    pitIsrFlag = false;
    	}  // if true == pitItrflag ends
    	/***** External Interrupt Zone *****/
        /*
    	if (true == ExtInterrupt) {
    		printf("external interrupt\n");

    		ExtInterrupt = false;

    		// Claw(0);
    	}
    	*/

    }

}
