/*
 * JoystickAnalogChannel.h
 *
 *  Created on: Feb 3, 2023
 *      Author: ruschagd
 */

#ifndef JOYSTICKANALOGCHANNEL_H_
#define JOYSTICKANALOGCHANNEL_H_

#define JoystickSwitchPort P5
#define JoystickSwitchPin BIT4

#define  Frequency20Hz 800  // 25ms*32/ms = 800
#define ADC_START_ADDRESS 1 //Conversion memory 1

#define X_axis_PORT P6
#define X_axis_PIN   BIT0    //P6.0

#define Y_axis_PORT P6
#define Y_axis_PIN   BIT1    //P6.1

long unsigned int dividerCount;
signed int rotCount;
int16_t x_axis_value;  //digital value from POT
int16_t y_axis_value;  //digital value from POT
enum
{
    CHANGED, NoCHANGE
} POTstatus;
enum
{
    NORTH, EAST, SOUTH, WEST, MIDDLE
} CurDir, PrevDir;
enum
{
    CLOCKWISE, COUNTERCLOCKWISE
} RotationDir;

void checkForNewDirection(signed int x, signed int y);

unsigned int checkClockwiseMovement(int cur, int prev);

unsigned int checkCounterClockwiseMovement(int cur, int prev);

void ConfigureAnalogInputTimerA1(void);

void initJoystick(void);

void joystick_UpdateDirection(void);


#endif /* JOYSTICKANALOGCHANNEL_H_ */
