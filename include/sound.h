#ifndef __SOUND_H__
#define __SOUND_H__

#include <stddef.h>

typedef enum { IDLE = 1, ACC = 2, DEC = 3, FULL_THROTTLE = 4} sound_t;

void sound_init();

void sound_play();

#endif // #ifndef __SOUND_H__