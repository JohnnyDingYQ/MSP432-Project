/*
 * button.h
 *
 *  Created on: Feb 6, 2023
 *      Author: dingy8
 */

#ifndef BUTTON_H_
#define BUTTON_H_

#include "msp.h"

#define button 0b00100000
#define button1 0b01000000

char buttonValue;
char button1Value;

extern void initButton(void);

extern void initButton1(void);

extern void button_detect(void);

extern void button1_detect(void);

extern int button1_Pressed(void);

// blocking
extern void button_WaitPress(void);

// blocking
extern void button_WaitRelease(void);

extern void button_LazyDebounce(void);

#endif /* BUTTON_H_ */
