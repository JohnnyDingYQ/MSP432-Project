/*
 * speaker.h
 *
 *  Created on: Feb 6, 2023
 *      Author: dingy8
 */

#ifndef SPEAKER_H_
#define SPEAKER_H_

#include <msp.h>

extern void initSpeaker(void);

extern void speaker_Enable(void);

extern void speaker_Disable(void);

extern void speaker_SetPlayFreq(int);




#endif /* SPEAKER_H_ */
