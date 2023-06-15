#include "sound.h"
#include <SDL2/SDL.h>
#include <SDL2/SDL2_gfxPrimitives.h>
#include <SDL2/SDL_mixer.h>

Mix_Chunk *idle;
Mix_Chunk *acc;
Mix_Chunk *dec;
Mix_Chunk *full_throttle;

const char *idle_path = "assets/MX Idle.wav";
const char *acc_path = "assets/MX Acceleration.wav";
const char *dec_path = "assets/MX Deceleration.wav";
const char *full_throttle_path = "assets/MX Full Throttle.wav";

void sound_init() {
  const int frequency = 48000;
  const int channels = 2;
  const int chunk_size = 2048;
  Mix_OpenAudio(frequency, MIX_DEFAULT_FORMAT, channels, chunk_size);
  idle = Mix_LoadWAV(idle_path);
  acc = Mix_LoadWAV(acc_path);
  dec = Mix_LoadWAV(dec_path);
  full_throttle = Mix_LoadWAV(full_throttle_path);
}

void sound_play(sound_t sound) {
  Mix_Chunk *to_play;
  switch (sound) {
  case IDLE:
    to_play = idle;
    break;
  case ACC:
    to_play = acc;
    break;
  case DEC:
    to_play = dec;
    break;
  case FULL_THROTTLE:
    to_play = full_throttle;
    break;
  }
  Mix_HaltChannel(-1);
  Mix_PlayChannel(-1, to_play, 0);
}
