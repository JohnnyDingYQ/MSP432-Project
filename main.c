#include "msp.h"
/* Standard Includes */
#include <msp.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <time.h>
/**
 * TimerA0   ---->    Speaker control
 * TimerA1   ---->    JoyStick read control
 * TimerA2   ---->    LCD blink timer
 * TimerA3   ---->    Servo Control
 */

#include "csHFXT.h"

#include "lcd.h"
#include "speaker.h"
#include "servoDriver.h"
#include "button.h"
#include "JoystickAnalogChannels.h"

#define CLK_FREQUENCY 48000000

#define NOTECNT 3
#define NOTEA4  27273
#define NOTEB4  24297
#define NOTEC5  22933

#define HOLD_TIME 250000
#define LEVEL_SIZE 4
int16_t SERVO_OPEN = 180;
int16_t SERVO_CLOSE = 150;

char levels[LEVEL_SIZE * 2 + 1];
char A[LEVEL_SIZE * 2 + 1];
char B[LEVEL_SIZE * 2 + 1];
int level = 0;
int blindMode;

int getRot(int clockwise);

void waitTurn(int turns, int clockwise);

void generateLevel(void);

int checkUnlock(void);

/**
 * main.c
 */
void main(void)
{
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;		// stop watchdog timer

    srand(time(NULL));

    // ** Start Configuration

// Master clock
    configHFXT();

// LCD
    configLCD(CLK_FREQUENCY);
    initLCD();

// Speaker
    initSpeaker();

// Servo
    initServoMotor();

// Button
    initButton();
    initButton1();

// JoyStick
    initJoystick();

// Test LED
    P1->SEL0 &= ~BIT0;
    P1->SEL1 &= ~BIT0;
// set pin direction to output
    P1->DIR |= BIT0;
// initialize LED to "off"
    P1->OUT |= BIT0;

// ** End Configuration
    while (1)
    {
        level = 0;
        blindMode = 0;

        servo_SetAngle(SERVO_CLOSE);

        lcd_StopBlinking();
        lcd_ClearLine(FirstLine);
        lcd_ClearLine(SecondLine);
        lcd_SetLineNumber(FirstLine);
        lcd_PutString("Press Button");
        lcd_SetBlinkDivider(100);
        lcd_SetBlinkFrameA("1 - Normal");
        lcd_SetBlinkFrameB("2 -  Blind");
        lcd_StartBlinking();

        button1_detect();
        button_detect();
        while (button1Value && buttonValue)
        {
            button1_detect();
            button_detect();
        }

        if (button1_Pressed())
        {
            blindMode = 1;
        }

        generateLevel();

        lcd_ClearLine(FirstLine);
        lcd_ClearLine(SecondLine);
        lcd_SetLineNumber(FirstLine);
        if (!blindMode)
        {
            lcd_PutString(levels);
        }
        else
        {
            lcd_PutString("Blind Mode!");
        }

        lcd_SetBlinkFrameA(A);
        lcd_SetBlinkFrameB(B);
        lcd_StartBlinking();

        while (level < LEVEL_SIZE)
        {
            int clockwise = 1;
            if (level % 2 == 0)
            {
                clockwise = 0;
            }
            waitTurn((levels[2 * level]) - '0', clockwise);
        }
        lcd_StopBlinking();
        lcd_ClearLine(FirstLine);
        lcd_ClearLine(SecondLine);
        lcd_SetLineNumber(FirstLine);
        if (checkUnlock() == 0)
        {
            lcd_PutString("Failed to");
            lcd_SetLineNumber(SecondLine);
            lcd_PutString("Open Lock");
        }
        else
        {
            lcd_StopBlinking();
            initServoMotor();
            servo_SetAngle(SERVO_OPEN);
            lcd_PutString("Successfully");
            lcd_SetLineNumber(SecondLine);
            lcd_PutString("Opened Lock");
        }
        button_WaitPress();
        button_WaitRelease();
    }
}
int getRot(int clockwise)
{
    int rot;
    joystick_UpdateDirection();
    if (clockwise == 0)
    {
        rot = rotCount;
    }
    else
    {
        rot = -rotCount;
    }
    return rot;
}

void waitTurn(int turns, int clockwise)
{
    int rot;
    rot = getRot(clockwise);
    while (rot < turns * 4)
    {
        int delay = 0;
        for (delay = 0; delay < 10000; delay++)
            ;
        rot = getRot(clockwise);

        int rate;
        if (rotCount < 0 && !clockwise)
        {
            rate = 100;
        }
        else if (rotCount > 0 && clockwise)
        {
            rate = 100;
        }
        else
        {
            rate = (int) ((float) (turns * 4 - abs(rot)) / (float) (turns * 4)
                    * 100);
        }
        lcd_SetBlinkDivider(rate);
        printf("\r\n%d", rotCount);
    }
    A[2 * level] = 126;
    B[2 * level] = 127;
    lcd_SetBlinkFrameA(A);
    lcd_SetBlinkFrameB(B);
    long int tElapsed = 0;

    speaker_Enable();
    speaker_SetPlayFreq(NOTEB4);
    while (rot == turns * 4 && tElapsed < HOLD_TIME)
    {
        tElapsed++;
        rot = getRot(clockwise);
    }
    speaker_Disable();
    if (tElapsed < HOLD_TIME)
    {
        A[2 * level] = 'X';
        B[2 * level] = 'X';
    }
    else
    {
        A[2 * level] = '0';
        B[2 * level] = '0';
    }
    lcd_SetBlinkFrameA(A);
    lcd_SetBlinkFrameB(B);
    lcd_SetBlinkDivider(0);
    rotCount = 0;
    level++;
    B[2 * level] = ' ';
}

void generateLevel(void)
{
    int r;

    int levelSize = LEVEL_SIZE * 2;
    int levelCount = 0;
    for (levelCount = 0; levelCount < levelSize; levelCount += 1)
    {
        r = rand() % 3 + 1;
        levels[levelCount] = r + '0';
        A[levelCount] = '!';
        if (levelCount == 0)
        {
            B[levelCount] = ' ';
        }
        else
        {
            B[levelCount] = '!';
        }
        levelCount++;
        levels[levelCount] = ' ';
        A[levelCount] = ' ';
        B[levelCount] = ' ';
    }
    levels[LEVEL_SIZE * 2] = '\0';
    A[LEVEL_SIZE * 2] = '\0';
    B[LEVEL_SIZE * 2] = '\0';
}

int checkUnlock(void)
{
    int levelSize = LEVEL_SIZE * 2;
    int levelCount = 0;
    for (levelCount = 0; levelCount < levelSize; levelCount += 1)
    {
        if (A[levelCount] == 'X')
        {
            return 0;
        }
    }
    return 1;
}
