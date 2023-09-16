/*! \file */
/*!
 * servoDriver.c
 *
 * Description: Servo motor driver for MSP432P4111 Launchpad.
 *              Assumes SMCLK configured with 48MHz HFXT as source.
 *              Uses Timer_A2 and P5.6 (TA2.1)
 *
 *  Created on:
 *      Author:
 */

#include "servoDriver.h"
#include "msp.h"

/* Global Variables  */
uint16_t pulseWidthTicks = SERVO_MIN_ANGLE;


void initServoMotor(void) {
    //  configure servo pin (P5.6) for primary module function (TA2.1),
    //  output, initially LOW
    ServoPort->SEL0 |= ServoMask;
    ServoPort->SEL1 &= ~ServoMask;
    ServoPort->OUT &= ~ServoMask;
    ServoPort->DIR |= ServoMask;

    /* Configure Timer_A2 and CCR1 */
    // Set period of Timer_A2 in CCR0 register for Up Mode
    TIMER_A2->CCR[0] = SERVO_TMR_PERIOD;
    // Set initial positive pulse-width of PWM in CCR1 register
    TIMER_A2->CCR[1] = SERVO_MIN_ANGLE;

    // configure TA2CCR1 for Compare mode, Reset/Set output mode, with interrupt disabled
    TIMER_A2->CCTL[1] = 0b0000000011100000;

    // Configure Timer_A2 in Up Mode, with source SMCLK, prescale 16:1, and
    //  interrupt disabled  -  tick rate will be 3MHz (for SMCLK = 48MHz)
    // configure Timer_A2 (requires setting control AND expansion register)
    TIMER_A2->CTL = 0b0000001010010100;
    TIMER_A2->EX0 = 0b0000000000000011;

}

void incrementTenDegree(void) {
    // update pulse-width for <current angle> + <10 degrees>
    pulseWidthTicks += TEN_DEGREE_TICKS;
    if (pulseWidthTicks > SERVO_MAX_ANGLE) {
        pulseWidthTicks = SERVO_MIN_ANGLE;
    }
    // update CCR1 register to set new positive pulse-width
    TIMER_A2->CCR[1] = pulseWidthTicks;
}

void servo_SetAngle(int16_t angle) {
    // NOT NEEDED FOR EXERCISE - but would be useful function for driver
    pulseWidthTicks = (SERVO_MAX_ANGLE - SERVO_MIN_ANGLE) * ( (float) angle
            / 180) + SERVO_MIN_ANGLE;
    TIMER_A2->CCR[1] = pulseWidthTicks;
}
