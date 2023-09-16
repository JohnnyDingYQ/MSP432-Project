/*
 * speaker.c
 *
 *  Created on: Feb 6, 2023
 *      Author: dingy8
 */

#include <msp.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "speaker.h"

void initSpeaker()
{
    /* Configure GPIO for speaker */
    P2->DIR |= BIT4;            // set P2.4 as output
    P2->SEL0 |= BIT4;           // P2.4 set to TA0.1
    P2->SEL1 &= ~BIT4;

    /* Configure Timer_A0 */
    // Configure CCR1 for Compare mode with Set/Reset output
    //          - sets when Timer_A0 == CCR1
    //          - resets when Timer_A0 == CCR0
    // configure CCR1
    TIMER_A0->CCTL[1] = 0x0060;
    // Configure Timer_A0 in Stop Mode with source SMCLK prescale 4:1
    //      Tick rate will be (48MHz/4) with rollover at value in CCR0
    // configure Timer_A0
    TIMER_A0->CTL = 0x0284;
}

void speaker_Enable()
{
    TIMER_A0->CTL = 0x0294;
}

void speaker_Disable()
{
    TIMER_A0->CTL = 0x0284;
}

void speaker_SetPlayFreq(int freq)
{
    TIMER_A0->CCR[0] = freq - 1;
    TIMER_A0->CCR[1] = (freq / 2) - 1;
}
