/*
 * TwoAnalogChannels.c
 *
 *  Created on: February 2, 2023
 *      Author: song
 */

#include <msp.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "msp.h"
#include "JoystickAnalogChannels.h"

//Timer A1 CCR0 interrupt at 20Hz to read POT value
//Configure POT as an analog input
//Advanced: configure Photocell as an analog input. Configure ADC14 to get voltages from two channels
void initJoystick(void)
{
    JoystickSwitchPort->DIR &= ~JoystickSwitchPin; //INPUT
    JoystickSwitchPort->SEL0 &= ~JoystickSwitchPin;
    JoystickSwitchPort->SEL1 &= ~JoystickSwitchPin;
    JoystickSwitchPort->OUT |= JoystickSwitchPin; //PULLUP
    JoystickSwitchPort->REN |= JoystickSwitchPin; //pullup enabled

    CurDir = MIDDLE;
    PrevDir = MIDDLE;
    RotationDir = CLOCKWISE;
    rotCount = 0;

    X_axis_PORT->DIR &= ~X_axis_PIN;
    X_axis_PORT->SEL0 |= X_axis_PIN;
    X_axis_PORT->SEL1 |= X_axis_PIN;

    Y_axis_PORT->DIR &= ~Y_axis_PIN;
    Y_axis_PORT->SEL0 |= Y_axis_PIN;
    Y_axis_PORT->SEL1 |= Y_axis_PIN;

    /* Configure ADC (CTL0 and CTL1) registers for:
     *      clock source - default MODCLK, clock prescale 1:1,
     *      sample input signal (SHI) source - software controlled (ADC14SC),
     *      Pulse Sample mode with sampling period of 16 ADC14CLK cycles,
     *      Single-channel, single-conversion mode, 12-bit resolution,
     *      ADC14 conversion start address ADC14MEM1, and Low-power mode
     */
    ADC14->CTL0 = ADC14_CTL0_SHP                // Pulse Sample Mode
    | ADC14_CTL0_SHT0__16       // 16 cycle sample-and-hold time (for ADC14MEM1)
            | ADC14_CTL0_PDIV__1        // Predivide by 1
            | ADC14_CTL0_DIV__1         // /1 clock divider
            | ADC14_CTL0_SHS_0      // ADC14SC bit sample-and-hold source select
            | ADC14_CTL0_SSEL__MODCLK   // clock source select MODCLK
            //                  | ADC14_CTL0_CONSEQ_0       // Single-channel, single-conversion mode
            | ADC14_CTL0_ON;            // ADC14 on

    ADC14->CTL0 |= ADC14_CTL0_MSC; /*!< ADC14 multiple sample and conversion */

    ADC14->CTL0 |= ADC14_CTL0_CONSEQ_1;         // Sequence of channels

    ADC14->CTL1 = ADC14_CTL1_RES__12BIT         // 12-bit conversion results
    | (0x1 << ADC14_CTL1_CSTARTADD_OFS) // ADC14MEM1 - conversion start address
            | ADC14_CTL1_PWRMD_2;               // Low-power mode

    // TODO Configure ADC14MCTL1 as storage register for result
    //          Single-ended mode with Vref+ = Vcc and Vref- = Vss,
    //          Input channel - A15, and comparator window disabled
    ADC14->MCTL[1] = 0x000F;  //channel bits 4-0 = 0b01111 for A15, P6.0, X-axis
    ADC14->MCTL[2] = 0x000E;  //channel bits 4-0 = 0b10000 for A14, P6.1, Y-axis

    ADC14->MCTL[2] |= 0b10000000;   //turn on End of Sequence bit
    //    ADC14->MCTL[2] |= ADC14_MCTLN_EOS;

    /* Configure Timer_A1 and CCRs */
    // Set initial period in CCR0 register. This assumes timer starts at 0
    TIMER_A1->CCR[0] = Frequency20Hz;
    // Configure CCR0 for Compare mode with interrupt enabled (no output mode - 0)
    // TODO configure CCR0
    TIMER_A1->CCTL[0] = 0X0010;   //0b0000 0000 0001 CCIE=1 0000
    // Configure Timer_A1 in UP Mode with source ACLK prescale 1:1 and no interrupt
    // TODO configure Timer_A1: ACLK, UP mode, TACLR=1 bit 2
    TIMER_A1->CTL = 0x0112;  //0b0000 0001 ACLK 0001 UP 0100

    /* Configure global interrupts and NVIC */
    // Enable TA1 TA1CCR0 compare interrupt
    // TODO enable interrupt by setting IRQ bit in NVIC ISER0 register
    NVIC->ISER[0] |= (1) << TA1_0_IRQn;
}

void checkForNewDirection(signed int x, signed int y)
{
    if (x < -1800 && 400 > y > -400)
    {
        CurDir = SOUTH;
    }
    else if (x > 1800 && 400 > y > -400)
    {
        CurDir = NORTH;
    }
    else if (y > 1800 && 400 > x > -400)
    {
        CurDir = WEST;
    }
    else if (y < -1800 && 400 > x > -400)
    {
        CurDir = EAST;
    }
    else
    {
        CurDir = MIDDLE;
    }
}
unsigned int checkClockwiseMovement(int cur, int prev)
{
    if (cur == 1 && prev == 0)
    {
        return 1;
    }
    if (cur == 2 && prev == 1)
    {
        return 1;
    }
    if (cur == 3 && prev == 2)
    {
        return 1;
    }
    if (cur == 0 && prev == 3)
    {
        return 1;
    }
    return 0;
}

unsigned int checkCounterClockwiseMovement(int cur, int prev)
{
    if (cur == 0 && prev == 1)
    {
        return 1;
    }
    if (cur == 3 && prev == 0)
    {
        return 1;
    }
    if (cur == 2 && prev == 3)
    {
        return 1;
    }
    if (cur == 1 && prev == 2)
    {
        return 1;
    }
    return 0;
}

void joystick_UpdateDirection(void)
{
    float absoluteX = (float) abs(x_axis_value) / 2048;
    float absoluteY = (float) abs(y_axis_value) / 2048;

    checkForNewDirection(x_axis_value, y_axis_value);
    if (CurDir == MIDDLE)
    {
        rotCount = 0;
    }
    else if (PrevDir != MIDDLE && CurDir != MIDDLE && PrevDir != CurDir)
    {
        int movedClockwise = checkClockwiseMovement(CurDir, PrevDir);
        //printf("%d", movedClockwise);
        int movedCounterClockwise = checkCounterClockwiseMovement(CurDir,
                                                                  PrevDir);
        if (rotCount == 0)
        {
            if (movedClockwise)
            {
                rotCount++;
                RotationDir = CLOCKWISE;
            }
            else
            {
                rotCount--;
                RotationDir = COUNTERCLOCKWISE;
            }
        }
        else
        {
            if (RotationDir == CLOCKWISE && movedClockwise)
            {
                rotCount++;

            }
            else if (RotationDir == COUNTERCLOCKWISE
                    && movedCounterClockwise)
            {
                rotCount--;
            }
            else if (RotationDir == CLOCKWISE && movedCounterClockwise)
            {
                rotCount = 0;
            }
            else if (RotationDir == COUNTERCLOCKWISE && movedClockwise)
            {
                rotCount = 0;
            }
        }
    }
    PrevDir = CurDir;
}

// Timer A1 CCR0 interrupt service routine
//This interrupt occurs at 20Hz to update POT value
void TA1_0_IRQHandler(void)
{
    /* Not necessary to check which flag is set because only one IRQ
     *  mapped to this interrupt vector     */
    if (TIMER_A1->CCTL[0] & TIMER_A_CCTLN_CCIFG)
    {
        // TODO clear timer compare flag in TA3CCTL0
        TIMER_A1->CCTL[0] &= ~TIMER_A_CCTLN_CCIFG;  //clear interrupt flag
        ADC14->CTL0 |= 0b11; //start conversion
//ADC14_IFGR0_IFG2 is set when ADC14MEM2 is loaded with conversion result
//The flag is cleared when ADC14MEM2 register is read
        while ((ADC14->IFGR0 & ADC14_IFGR0_IFG2) == 0)
        {
        }; //wait for sequence conversion to be over
        x_axis_value = ADC14->MEM[1] - 2048;   //IFG1 flag is cleared
        y_axis_value = ADC14->MEM[2] - 2048;
    }

}
