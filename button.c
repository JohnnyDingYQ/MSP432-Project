/*
 * button.c
 *
 *  Created on: Feb 6, 2023
 *      Author: dingy8
 */

#include <msp.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "button.h"

void initButton(void)
{
    P1->SEL0 &= ~button;
    P1->SEL1 &= ~button;
    P1->DIR &= ~button;
    P1->OUT |= button;
    P1->REN |= button;
}

void initButton1(void)
{
    P1->SEL0 &= ~button1;
    P1->SEL1 &= ~button1;
    P1->DIR &= ~button1;
    P1->OUT |= button1;
    P1->REN |= button1;
}

void button_detect(void)
{
    buttonValue = P1->IN & button;
}

void button1_detect(void)
{
    button1Value = P1->IN & button1;
}

void button_WaitPress(void)
{
    button_detect();
    while (buttonValue)
    {
        button_detect();
    }
    button_LazyDebounce();
}

void button_WaitRelease(void)
{
    button_detect();
    while (!buttonValue)
    {
        button_detect();
    }
    button_LazyDebounce();
}

void button_LazyDebounce(void)
{
    unsigned int DELAY = 20;
    while (DELAY != 0)
    {
        DELAY -= 1;
    }
}

int button1_Pressed(void) {
    if (!button1Value) {
        return 1;
    }
    return 0;
}

