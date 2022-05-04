/**
 * 17   27  22  selection
 * 0    0   1   M1
 * 0    1   0   M2
 * 0    1   1   M3
 * 1    0   0   M4
 * 1    0   1   M5
 * 1    1   0   M6
 *
 * 5    6   operation
 * x    0   NOP
 * val  1   set position to val
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <wiringPi.h>
#include <softPwm.h>

#define P1 17
#define P2 27
#define P3 22
#define P4 5
#define P5 6

void init();
int selectMotor(int motor);
int operate(int pos);
int handleInput(char* input);
