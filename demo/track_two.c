#include "body.h"
#include "list.h"
#include "polygon.h"
#include "scene.h"
#include "sdl_wrapper.h"
#include "state.h"
#include "vector.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

const double TRACK_MASS = INFINITY;
const rgb_color_t TRACK_COLOR = (rgb_color_t){0.05, 0.4, 0};

const vector_t WINDOW = (vector_t){.x = 1000, .y = 300};

const size_t NUM_BODIES = 60;
#define NUM_COORDS 240
const vector_t COORDS[NUM_COORDS] = {
    (vector_t){0, 0},        // y = 5  //0
    (vector_t){0, 5},        // 1
    (vector_t){84.46, 5},    // 2
    (vector_t){84.46, 0},    // 3
    (vector_t){84.46, 0},    // y = 0.06(x-90)+5.334 / 4
    (vector_t){84.46, 5},    // 5
    (vector_t){90, 5.332},   // 6
    (vector_t){90, 0},       // 7
    (vector_t){90, 0},       // y = 10e^{(-0.001)(x - 30 - 125)^2 + 5.186} // 8
    (vector_t){90, 5.332},   // 9
    (vector_t){94, 5.428},   // 10
    (vector_t){94, 0},       // 11
    (vector_t){94, 0},       // 12
    (vector_t){94, 5.428},   // 13
    (vector_t){96, 5.494},   // 14
    (vector_t){96, 0},       // 15
    (vector_t){96, 0},       // 16
    (vector_t){96, 5.494},   // 17
    (vector_t){100, 5.672},  // 18
    (vector_t){100, 0},      // 19
    (vector_t){100, 0},      // 20
    (vector_t){100, 5.672},  // 21
    (vector_t){103, 5.855},  // 22
    (vector_t){103, 0},      // 23
    (vector_t){103, 0},      // 24
    (vector_t){103, 5.855},  // 25
    (vector_t){105, 6.007},  // 26
    (vector_t){105, 0},      // 27
    (vector_t){105, 0},      // 28
    (vector_t){105, 6.007},  // 29
    (vector_t){110, 6.506},  // 30
    (vector_t){110, 0},      // 31
    (vector_t){110, 0},      // 32
    (vector_t){110, 6.506},  // 33
    (vector_t){114, 7.048},  // 34
    (vector_t){114, 0},      // 35
    (vector_t){114, 0},      // 36
    (vector_t){114, 7.048},  // 37
    (vector_t){118, 7.73},   // 38
    (vector_t){118, 0},      // 39
    (vector_t){118, 0},      // 40
    (vector_t){118, 7.73},   // 41
    (vector_t){124, 9.011},  // 42
    (vector_t){124, 0},      // 43
    (vector_t){124, 0},      // 44
    (vector_t){124, 9.011},  // 45
    (vector_t){128, 10.01},  // 46
    (vector_t){128, 0},      // 47
    (vector_t){128, 0},      // 48
    (vector_t){128, 10.01},  // 49
    (vector_t){134, 11.62},  // 50
    (vector_t){134, 0},      // 51
    (vector_t){134, 0},      // 52
    (vector_t){134, 11.62},  // 53
    (vector_t){138, 12.670}, // 54
    (vector_t){138, 0},      // 55
    (vector_t){138, 0},      // 56
    (vector_t){138, 12.670}, // 57
    (vector_t){172, 12.676}, // 58
    (vector_t){172, 0},      // 59
    (vector_t){172, 0},      // 60
    (vector_t){172, 12.676}, // 61
    (vector_t){176, 11.62},  // 62
    (vector_t){176, 0},      // 63
    (vector_t){176, 0},      // 64
    (vector_t){176, 11.62},  // 65
    (vector_t){182, 10.01},  // 66
    (vector_t){182, 0},      // 67
    (vector_t){182, 0},      // 68
    (vector_t){182, 10.01},  // 69
    (vector_t){186, 9.011},  // 70
    (vector_t){186, 0},      // 71
    (vector_t){186, 0},      // 72
    (vector_t){186, 9.011},  // 73
    (vector_t){192, 7.73},   // 74
    (vector_t){192, 0},      // 75
    (vector_t){192, 0},      // 76
    (vector_t){192, 7.73},   // 77
    (vector_t){196, 7.048},  // 78
    (vector_t){196, 0},      // 79
    (vector_t){196, 0},      // 80
    (vector_t){196, 7.048},  // 81
    (vector_t){200, 6.506},  // 82
    (vector_t){200, 0},      // 83
    (vector_t){200, 0},      // 84
    (vector_t){200, 6.506},  // 85
    (vector_t){205, 6.007},  // 86
    (vector_t){205, 0},      // 87
    (vector_t){205, 0},      // 88
    (vector_t){205, 6.007},  // 89
    (vector_t){207, 5.855},  // 90
    (vector_t){207, 0},      // 91
    (vector_t){207, 0},      // 92
    (vector_t){207, 5.855},  // 93
    (vector_t){210, 5.672},  // 94
    (vector_t){210, 0},      // 95
    (vector_t){210, 0},      // 96
    (vector_t){210, 5.672},  // 97
    (vector_t){214, 5.494},  // 98
    (vector_t){214, 0},      // 99
    (vector_t){214, 0},      // 100
    (vector_t){214, 5.494},  // 101
    (vector_t){216, 5.428},  // 102
    (vector_t){216, 0},      // 103
    (vector_t){216, 0},      // 104
    (vector_t){216, 5.428},  // 105
    (vector_t){225, 5.26},   // 106
    (vector_t){225, 0},      // 107
    (vector_t){225, 0},      // 108
    (vector_t){225, 5.26},   // 109
    (vector_t){240, 5.193},  // 110
    (vector_t){240, 0},      // 111
    (vector_t){240, 0},      // y = 10 e^{-0.01(x - 310)^2 + 5.119}  //112
    (vector_t){240, 5.193},  // 113
    (vector_t){248, 5.332},  // 114
    (vector_t){248, 0},      // 115
    (vector_t){248, 0},      // 116
    (vector_t){248, 5.332},  // 117
    (vector_t){251.034, 5.428},  // 118
    (vector_t){251.034, 0},      // 119
    (vector_t){251.034, 0},      // 120
    (vector_t){251.034, 5.428},  // 121
    (vector_t){252.699, 5.494},  // 122
    (vector_t){252.699, 0},      // 123
    (vector_t){252.699, 0},      // 124
    (vector_t){252.699, 5.494},  // 125
    (vector_t){256.195, 5.672},  // 126
    (vector_t){256.195, 0},      // 127
    (vector_t){256.195, 0},      // 128
    (vector_t){256.195, 5.672},  // 129
    (vector_t){258.921, 5.855},  // 130
    (vector_t){258.921, 0},      // 131
    (vector_t){258.921, 0},      // 132
    (vector_t){258.921, 5.855},  // 133
    (vector_t){260.793, 6.007},  // 134
    (vector_t){260.793, 0},      // 135
    (vector_t){260.793, 0},      // 136
    (vector_t){260.793, 6.007},  // 137
    (vector_t){265.554, 6.506},  // 138
    (vector_t){265.554, 0},      // 139
    (vector_t){265.554, 0},      // 140
    (vector_t){265.554, 6.506},  // 141
    (vector_t){269.434, 7.048},  // 142
    (vector_t){269.434, 0},      // 143
    (vector_t){269.434, 0},      // 144
    (vector_t){269.434, 7.048},  // 145
    (vector_t){273.355, 7.73},   // 146
    (vector_t){273.355, 0},      // 147
    (vector_t){273.355, 0},      // 148
    (vector_t){273.355, 7.73},   // 149
    (vector_t){279.281, 9.011},  // 150
    (vector_t){279.281, 0},      // 151
    (vector_t){279.281, 0},      // 152
    (vector_t){279.281, 9.011},  // 153
    (vector_t){283.257, 10.01},  // 154
    (vector_t){283.257, 0},      // 155
    (vector_t){283.257, 0},      // 156
    (vector_t){283.257, 10.01},  // 157
    (vector_t){289.248, 11.62},  // 158
    (vector_t){289.248, 0},      // 159
    (vector_t){289.248, 0},      // 160
    (vector_t){289.248, 11.62},  // 161
    (vector_t){293.263, 12.676}, // 162
    (vector_t){293.263, 0},      // 163
    (vector_t){293.263, 0},      // 164
    (vector_t){293.263, 12.676}, // 165
    (vector_t){326.737, 12.676}, // 166
    (vector_t){326.737, 0},      // 167
    (vector_t){326.737, 0},      // 168
    (vector_t){326.737, 12.676}, // 169
    (vector_t){330.752, 11.62},  // 170
    (vector_t){330.752, 0},      // 171
    (vector_t){330.752, 0},      // 172
    (vector_t){330.752, 11.62},  // 173
    (vector_t){336.743, 10.01},  // 174
    (vector_t){336.743, 0},      // 175
    (vector_t){336.743, 0},      // 176
    (vector_t){336.743, 10.01},  // 177
    (vector_t){340.719, 9.011},  // 178
    (vector_t){340.719, 0},      // 179
    (vector_t){340.719, 0},      // 180
    (vector_t){340.719, 9.011},  // 181
    (vector_t){346.645, 7.73},   // 182
    (vector_t){346.645, 0},      // 183
    (vector_t){346.645, 0},      // 184
    (vector_t){346.645, 7.73},   // 185
    // (vector_t) {346.645, 7.73}, //186
    (vector_t){350.566, 7.048}, // 187
    (vector_t){350.566, 0},     // 188
    (vector_t){350.566, 0},     // 189
    (vector_t){350.566, 7.048}, // 190
    (vector_t){354.446, 6.506}, // 191
    (vector_t){354.446, 0},     // 192
    (vector_t){354.446, 0},     // 193
    (vector_t){354.446, 6.506}, // 194
    (vector_t){359.207, 6.007}, // 195
    (vector_t){359.207, 0},     // 196
    (vector_t){359.207, 0},     // 197
    (vector_t){359.207, 6.007}, // 198
    (vector_t){361.079, 5.855}, // 199
    (vector_t){361.079, 0},     // 200
    (vector_t){361.079, 0},     // 201
    (vector_t){361.079, 5.855}, // 202
    (vector_t){363.805, 5.672}, // 203
    (vector_t){363.805, 0},     // 204
    (vector_t){363.805, 0},     // 205
    (vector_t){363.805, 5.672}, // 206
    (vector_t){367.301, 5.494}, // 207
    (vector_t){367.301, 0},     // 208
    (vector_t){367.301, 0},     // 209
    (vector_t){367.301, 5.494}, // 210
    (vector_t){368.966, 5.428}, // 211
    (vector_t){368.966, 0},     // 212
    (vector_t){368.966, 0},     // 213
    (vector_t){368.966, 5.428}, // 214
    (vector_t){372.041, 5.332}, // 215
    (vector_t){372.041, 0},     // 216
    (vector_t){372.041, 0},     // 217
    (vector_t){372.041, 5.332}, // 218
    (vector_t){400, 5.122},     // 219
    (vector_t){400, 0},         // 220
    (vector_t){400, 0},         // y = 5.12  /221
    (vector_t){400, 5.122},     // 222
    (vector_t){500, 5.12},      // 223
    (vector_t){500, 0},         // 224
};

typedef struct state {
  scene_t *scene;
} state_t;

list_t *make_track_two() {
  list_t *bodies = list_init(NUM_BODIES, free);
  size_t i = 0;
  for (size_t j = 0; j < NUM_BODIES; j++) {
    list_t *shape = list_init(4, free);
    for (size_t k = 0; k < 4; k++) {
      vector_t *coord = malloc(sizeof(vector_t));
      *coord = COORDS[i + k];
      list_add(shape, coord);
    }
    i += 4;
    body_t *body = body_init(shape, TRACK_MASS, TRACK_COLOR);
    list_add(bodies, body);
  }
  return bodies;
}

void initialize_body_list(scene_t *scene) {
  list_t *bodies = make_track_two();
  for (size_t i = 0; i < list_size(bodies); i++) {
    body_t *body = list_get(bodies, i);
    scene_add_body(scene, body);
  }
}

state_t *emscripten_init() {
  vector_t min = VEC_ZERO;
  vector_t max = WINDOW;
  sdl_init(min, max);
  state_t *state = malloc(sizeof(state_t));
  state->scene = scene_init();
  initialize_body_list(state->scene);
  return state;
}

void emscripten_main(state_t *state) {
  double dt = time_since_last_tick();
  scene_t *scene = state->scene;
  scene_tick(scene, dt);
  sdl_render_scene(scene);
}

void emscripten_free(state_t *state) {
  scene_free(state->scene);
  free(state);
}