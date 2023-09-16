/*!
 * lcd.c
 *
 *      Description: Helper file for LCD library. For Hitachi HD44780 parallel LCD
 *               in 8-bit mode. Assumes the following connections:
 *               P2.7 <-----> RS
 *               P2.6 <-----> E
 *                            R/W --->GND
 *                P4  <-----> DB
 *
 *          This module uses SysTick timer for delays.
 *
 *      Author: ece230
 */

#include <msp.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "lcd.h"
#include "sysTickDelays.h"

#define NONHOME_MASK        0xFC

#define LONG_INSTR_DELAY    2000
#define SHORT_INSTR_DELAY   50
#define DIVIDER_CEILING 16
#define DIVIDER_FLOOR 2

int divider = 4;
int count = 0;
int blinking = 0;

void configLCD(uint32_t clkFreq)
{
    // configure pins as GPIO
    LCD_DB_PORT->SEL0 = 0;
    LCD_DB_PORT->SEL1 = 0;
    LCD_RS_PORT->SEL0 &= ~LCD_RS_MASK;
    LCD_RS_PORT->SEL1 &= ~LCD_RS_MASK;
    LCD_EN_PORT->SEL0 &= ~LCD_EN_MASK;
    LCD_EN_PORT->SEL1 &= ~LCD_EN_MASK;
    // initialize En output to Low
    LCD_EN_PORT->OUT &= ~LCD_EN_MASK;
    // set pins as outputs
    LCD_DB_PORT->DIR = 0xFF;
    LCD_RS_PORT->DIR |= LCD_RS_MASK;
    LCD_EN_PORT->DIR |= LCD_EN_MASK;

    initDelayTimer(clkFreq);
}

/*!
 * Delay method based on instruction execution time.
 *   Execution times from Table 6 of HD44780 data sheet, with buffer.
 *
 * \param mode RS mode selection
 * \param instruction Instruction/data to write to LCD
 *
 * \return None
 */
void instructionDelay(uint8_t mode, uint8_t instruction)
{
    // if instruction is Return Home or Clear Display, use long delay for
    //  instruction execution; otherwise, use short delay
    if ((mode == DATA_MODE) || (instruction & NONHOME_MASK))
    {
        delayMicroSec(SHORT_INSTR_DELAY);
    }
    else
    {
        delayMicroSec(LONG_INSTR_DELAY);
    }
}

/*!
 * Function to write instruction/data to LCD.
 *
 * \param mode          Write mode: 0 - control, 1 - data
 * \param instruction   Instruction/data to write to LCD
 *
 * \return None
 */
void writeInstruction(uint8_t mode, uint8_t instruction)
{
    // TODO set 8-bit data on LCD DB port
    LCD_DB_PORT->OUT = instruction;

    // TODO set RS for data or control instruction mode
    //      use bit-masking to avoid affecting other pins of port
    if (mode == DATA_MODE)
    {
        LCD_RS_PORT->OUT |= LCD_RS_MASK;
    }
    else
    {

        LCD_RS_PORT->OUT &= ~LCD_RS_MASK;
    }
    // pulse E to execute instruction on LCD
    // TODO set Enable signal high
    //      use bit-masking to avoid affecting other pins of port
    LCD_EN_PORT->OUT |= LCD_EN_MASK;

    delayMicroSec(1);
    // TODO set Enable signal low
    //      use bit-masking to avoid affecting other pins of port

    LCD_EN_PORT->OUT &= ~LCD_EN_MASK;
    // delay to allow instruction execution to complete
    instructionDelay(mode, instruction);
}

/*!
 * Function to write command instruction to LCD.
 *
 * \param command Command instruction to write to LCD
 *
 * \return None
 */
void commandInstruction(uint8_t command)
{
    writeInstruction(CTRL_MODE, command);
}

/*!
 * Function to write data instruction to LCD. Writes ASCII value to current
 *  cursor location.
 *
 * \param data ASCII value/data to write to LCD
 *
 * \return None
 */
void dataInstruction(uint8_t data)
{
    writeInstruction(DATA_MODE, data);
}

void initLCD(void)
{
    // follows initialization sequence described for 8-bit data mode in
    // Figure 23 of HD447780 data sheet
    delayMilliSec(40);
    commandInstruction(FUNCTION_SET_MASK | DL_FLAG_MASK);
    delayMilliSec(5);
    commandInstruction(FUNCTION_SET_MASK | DL_FLAG_MASK);
    delayMicroSec(150);
    commandInstruction(FUNCTION_SET_MASK | DL_FLAG_MASK);
    delayMicroSec(SHORT_INSTR_DELAY);
    commandInstruction(FUNCTION_SET_MASK | DL_FLAG_MASK | N_FLAG_MASK);
    delayMicroSec(SHORT_INSTR_DELAY);
    commandInstruction(DISPLAY_CTRL_MASK);
    delayMicroSec(SHORT_INSTR_DELAY);
    commandInstruction(CLEAR_DISPLAY_MASK);
    delayMicroSec(SHORT_INSTR_DELAY);
    commandInstruction(ENTRY_MODE_MASK | ID_FLAG_MASK);
    delayMicroSec(LONG_INSTR_DELAY);

    // after initialization and configuration, turn display ON
    commandInstruction(DISPLAY_CTRL_MASK | D_FLAG_MASK);

    TIMER_A2->CCR[0] = 65535;
    //TIMER_A2->CCR[0] = 1000;
    TIMER_A2->CCTL[0] = 0X0010;
    TIMER_A2->CTL = 0x02C2;
    TIMER_A2->EX0 = 0b111;

    NVIC->ISER[0] |= (1) << TA2_0_IRQn;
}

void printChar(char character)
{
    // print ASCII \b character to current cursor position
    dataInstruction(character);
}

void lcd_clear()
{
    // clear the LCD display and return cursor to home position
    commandInstruction(CLEAR_DISPLAY_MASK);
}

void lcd_SetLineNumber(unsigned char position)
{
    writeInstruction(CTRL_MODE, 0x80 | position); // The "cursor move" command is indicated by MSB=1 (0x80)
//    // followed by the panel position address (0x00- 0x7F)
}

/* write a string of chars to the LCD */
void lcd_PutString(char arr[])
{
    char DataBuffer[16];
    char *string = DataBuffer;
    sprintf(DataBuffer, arr);
    while (*string != 0) // Last character in a C-language string is always "0" (ASCII NULL character)
        writeInstruction(DATA_MODE, *string++);
}

void lcd_ClearLine(unsigned char pos)
{
    lcd_SetLineNumber(pos);
    lcd_PutString("                ");
}

void lcd_SetBlinkFrameA(char s[])
{
    blinkFrameA = s;
}

void lcd_SetBlinkFrameB(char s[])
{
    blinkFrameB = s;
}

void lcd_SetBlinkDivider(int rate)
{
    float a = (float) rate * (float) (DIVIDER_CEILING - DIVIDER_FLOOR) / 100;
    divider = DIVIDER_FLOOR + (int) a;
}

void lcd_StartBlinking()
{
    TIMER_A2->CTL = 0x02D2;
    blinking = 1;
}

void lcd_StopBlinking()
{
    TIMER_A2->CTL = 0x02C2;
    blinking = 0;
}

void TA2_0_IRQHandler(void)
{
    if (blinking == 1)
    {
        count++;
        if (count > divider)
        {
            count = 0;
            P1->OUT ^= BIT0;
            lcd_SetLineNumber(SecondLine);
            //lcd_ClearLine(SecondLine);
            if (currentFrame == 0)
            {
                lcd_PutString(blinkFrameA);
                currentFrame = 1;
            }
            else
            {
                lcd_PutString(blinkFrameB);
                currentFrame = 0;
            }
        }
    }
    TIMER_A2->CCTL[0] &= ~TIMER_A_CCTLN_CCIFG;

}
