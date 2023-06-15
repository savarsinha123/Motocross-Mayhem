#include "body.h"
#include "collision.h"
#include "color.h"
#include "forces.h"
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

// constants
const vector_t WINDOW = ((vector_t){.x = 2000, .y = 1000});
#define CENTER vec_multiply(0.5, WINDOW)
#define STARTING_POSITION                                                      \
  (vector_t) { WINDOW.x / 2.0, 0.6 * WINDOW.y }

// button constants
const double BUTTON_MASS = 1.0;
#define BUTTON_DIM                                                             \
  (vector_t) { 0.2 * WINDOW.x, 0.15 * WINDOW.y }
#define BACK_BUTTON_DIM                                                        \
  (vector_t) { 0.1 * WINDOW.y, 0.1 * WINDOW.y }
#define PLAY_POSITION                                                          \
  (vector_t) {                                                                 \
    (WINDOW.x / 3.0 - BUTTON_DIM.x) / 2.0, WINDOW.y / 2.0 + 0.5 * BUTTON_DIM.y \
  }
#define CUSTOMIZE_POSITION                                                     \
  (vector_t) {                                                                 \
    (WINDOW.x / 3.0 - BUTTON_DIM.x) / 2.0 + WINDOW.x / 3.0,                    \
        WINDOW.y / 2.0 + 0.5 * BUTTON_DIM.y                                    \
  }
#define SETTINGS_POSITION                                                      \
  (vector_t) {                                                                 \
    (WINDOW.x / 3.0 - BUTTON_DIM.x) / 2.0 + 2.0 * WINDOW.x / 3.0,              \
        WINDOW.y / 2.0 + 0.5 * BUTTON_DIM.y                                    \
  }
const rgb_color_t BUTTON_COLOR = (rgb_color_t){0.5, 0.5, 0.5};
#define TITLE_DIMENSIONS                                                       \
  (vector_t) { WINDOW.x / 2.0, BUTTON_DIM.y }
#define TITLE_POSITION                                                         \
  (vector_t) { WINDOW.x / 4.0, WINDOW.y - BUTTON_DIM.y / 2.0 }
#define RED_POSITION                                                           \
  (vector_t) {                                                                 \
    (WINDOW.x / 3.0 - BUTTON_DIM.x) / 2.0,                                     \
        2.0 * WINDOW.y / 3.0 + 0.5 * BUTTON_DIM.y                              \
  }
#define ORANGE_POSITION                                                        \
  (vector_t) {                                                                 \
    (WINDOW.x / 3.0 - BUTTON_DIM.x) / 2.0 + WINDOW.x / 3.0,                    \
        2.0 * WINDOW.y / 3.0 + 0.5 * BUTTON_DIM.y                              \
  }
#define GREEN_POSITION                                                         \
  (vector_t) {                                                                 \
    (WINDOW.x / 3.0 - BUTTON_DIM.x) / 2.0 + 2.0 * WINDOW.x / 3.0,              \
        2.0 * WINDOW.y / 3.0 + 0.5 * BUTTON_DIM.y                              \
  }
#define BLUE_POSITION                                                          \
  (vector_t) {                                                                 \
    (WINDOW.x / 3.0 - BUTTON_DIM.x) / 2.0, WINDOW.y / 3.0 + 0.5 * BUTTON_DIM.y \
  }
#define PURPLE_POSITION                                                        \
  (vector_t) {                                                                 \
    (WINDOW.x / 3.0 - BUTTON_DIM.x) / 2.0 + WINDOW.x / 3.0,                    \
        WINDOW.y / 3.0 + 0.5 * BUTTON_DIM.y                                    \
  }
#define BLACK_POSITION                                                         \
  (vector_t) {                                                                 \
    (WINDOW.x / 3.0 - BUTTON_DIM.x) / 2.0 + 2.0 * WINDOW.x / 3.0,              \
        WINDOW.y / 3.0 + 0.5 * BUTTON_DIM.y                                    \
  }
#define BACK_POSITION                                                          \
  (vector_t) { BUTTON_DIM.y / 2, WINDOW.y - BUTTON_DIM.y / 2 }
const rgb_color_t RED = {0.5, 0.0, 0.0};
const rgb_color_t ORANGE = {0.5, 0.3, 0.0};
const rgb_color_t GREEN = {0.0, 0.3, 0.0};
const rgb_color_t BLUE = {0.0, 0.0, 0.7};
const rgb_color_t PURPLE = {0.5, 0.0, 0.5};
const rgb_color_t WHITE = {1.0, 1.0, 1.0};
const rgb_color_t YELLOW = {0.5, 0.5, 0.0};

// bike constants
const size_t BIKE_NUM_POINTS = 500;
const double BIKE_MASS = 1.0;
const double BIKE_MOMENT = 0.75;
const rgb_color_t BIKE_COLOR = (rgb_color_t){0.5, 0, 0};
const vector_t START = (vector_t){-1.45, 1.045}; // 1
// each section's end point becomes the next section's start point.
#define NUM_SECTIONS 54
#define NUM_CURVES 19
const vector_t COORDS[NUM_SECTIONS] = {
    (vector_t){-0.6, 1.0249},    // / 0
    (vector_t){-1.05, 1.191},    // / 1
    (vector_t){-1.1, 1.412},     // 2 curve
    (vector_t){-0.8752, 2.7991}, // / 3
    (vector_t){-0.35, 2.336},    //  / 4
    (vector_t){0.417, 1.9778},   //  5 curve
    (vector_t){0.6441, 1.9888},  // / 6
    (vector_t){0.8, 1.8933},     // / 7
    (vector_t){0.94, 1.5667},    // 8 curve
    (vector_t){-1.788, 0.8834},  // 9 curve
    (vector_t){-8.6, 1.0551},    // 10 curve
    (vector_t){-4.306, -0.4199}, // / 11
    (vector_t){-6.58, -1.2912},  // 12 curve
    (vector_t){-6.42, -1.7088},  // / 13
    (vector_t){-5.611, -1.3999}, // 14 curve
    (vector_t){-5.772, -0.9804}, // / 15
    (vector_t){-5.345, -0.8174}, // 16 curve
    (vector_t){-5.185, -1.236},  // / 17
    (vector_t){-3.99, -0.7786},  // 18 curve
    (vector_t){-3.752, -1.1},    // / 19
    (vector_t){-1.6274, -1.1},   // 20 curve
    (vector_t){-0.6476, -1.1},   // / 21
    (vector_t){0.2912, -1.1},    // 22 curve
    (vector_t){1.53, 0.1899},    // / 23
    (vector_t){2.2932, -1.5851}, // 24 curve
    (vector_t){2.7068, -1.4149}, // / 25
    (vector_t){2.364, -0.616},   // 26 curve
    (vector_t){1.9514, 0.7936},  // / 27
    (vector_t){1.771, -0.3737},  // 28 curve
    (vector_t){2.184, -0.1961},  // / 29
    (vector_t){1.9708, 0.3014},  // 30 curve
    (vector_t){4.5, -0.09585},   // / 31
    (vector_t){3.5, 0.7268},     // 32 curve
    (vector_t){1.69, 0.9567},    // / 33
    (vector_t){1.22, 2.0501},    // / 34
    (vector_t){0.741, 2.3056},   // 35 curve
    (vector_t){0.5566, 2.4422},  // / 36
    (vector_t){-0.2, 2.78},      // / 37
    (vector_t){-0.83, 3.595},    // / 38
    (vector_t){-0.86, 3.7},      // / 39
    (vector_t){-0.9148, 3.748},  // / 40
    (vector_t){-0.3925, 3.675},  // / 41
    (vector_t){-0.25, 3.96},     // / 42
    (vector_t){-0.37, 3.96},     // / 43
    (vector_t){-0.7, 4.05},      // / 44
    (vector_t){-0.8, 4.19},      // / 45
    (vector_t){-0.71, 4.703},    // / 46
    (vector_t){-0.5529, 4.904},  // / 47
    (vector_t){0.0861, 4.91},    // / 48
    (vector_t){-0.4122, 5.228},  // 49 curve
    (vector_t){-1.701, 3.858},   // / 50
    (vector_t){-1.587, 3.842},   // 51 curve
    (vector_t){-2.125, 0.9167},  // / 52
    (vector_t){-0.5, 0.988},     // / 53
};

const double PROPORTIONS[NUM_CURVES] = {
    3.0 / 100.0, 1.0 / 100.0, 4.5 / 100.0, 6.0 / 100.0, 4.5 / 100.0,
    1.5 / 100.0, 4.5 / 100.0, 6.0 / 100.0, 1.5 / 100.0, 3.0 / 100.0,
    6.5 / 100.0, 2.5 / 100.0, 2.0 / 100.0, 1.0 / 100.0, 4.0 / 100.0,
    6.0 / 100.0, 2.0 / 100.0, 1.0 / 100.0, 4.5 / 100.0,
};

const size_t NUM_BODIES = 60;
#define NUM_COORDS 240
const vector_t TRACK_ONE_COORDS[NUM_COORDS] = {
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

const size_t NUM_BODIES2 = 60;
#define NUM_COORDS2 240
const vector_t TRACK_TWO_COORDS[NUM_COORDS2] = {
    (vector_t){0, 0},                              // 0 new line
    (vector_t){0, 5},                              // 1
    (vector_t){16, 5},                             // 2
    (vector_t){16, 0},                             // 3
    (vector_t){16, 0},                             // new curve
    (vector_t){16, 5},       (vector_t){18, 5.04}, // 4
    (vector_t){18, 0},                             // 5
    (vector_t){18, 0},       (vector_t){18, 5.04},
    (vector_t){20, 5.16}, // 6
    (vector_t){20, 0},    // 7
    (vector_t){20, 0},       (vector_t){20, 5.16},
    (vector_t){22, 5.36}, // 8
    (vector_t){22, 0},    // 9
    (vector_t){22, 0},       (vector_t){22, 5.36},
    (vector_t){23, 5.49}, // 10
    (vector_t){23, 0},    // 11
    (vector_t){23, 0},       (vector_t){23, 5.49},
    (vector_t){24, 5.64}, // 12
    (vector_t){24, 0},    // 13
    (vector_t){24, 0},       (vector_t){24, 5.64},
    (vector_t){25, 5.81}, // 14
    (vector_t){25, 0},    // 15
    (vector_t){25, 0},       (vector_t){25, 5.81},
    (vector_t){26, 6}, // 16
    (vector_t){26, 0}, // 17
    (vector_t){26, 0},       (vector_t){26, 6},
    (vector_t){29, 6.69}, // 18
    (vector_t){29, 0},    // 19
    (vector_t){29, 0},       (vector_t){29, 6.69},
    (vector_t){32, 7.56}, // 20
    (vector_t){32, 0},    // 21
    (vector_t){32, 0},       (vector_t){32, 7.56},
    (vector_t){36, 9}, // 22
    (vector_t){36, 0}, // 23
    (vector_t){36, 0},       (vector_t){36, 9},
    (vector_t){40, 10.76}, // 24
    (vector_t){40, 0},     // 25
    (vector_t){40, 0},       (vector_t){40, 10.76},
    (vector_t){44, 12.84}, // 26
    (vector_t){44, 0},     // 27
    (vector_t){44, 0},       (vector_t){44, 12.84},
    (vector_t){47, 14.61}, // 28
    (vector_t){47, 0},     // 29
    (vector_t){47, 0},       (vector_t){47, 14.61},
    (vector_t){50, 16.56},                           // 30
    (vector_t){50, 0},                               // 31
    (vector_t){50, 0},                               // new curve
    (vector_t){50, 16.56},   (vector_t){60, 23.36},  // 32
    (vector_t){60, 0},                               // 33
    (vector_t){60, 0},                               // new curve
    (vector_t){60, 23.36},   (vector_t){65, 25.61},  // 34
    (vector_t){65, 0},                               // 35
    (vector_t){65, 0},                               // new curve
    (vector_t){65, 25.61},   (vector_t){70, 26.86},  // 36
    (vector_t){70, 0},                               // 37
    (vector_t){70, 0},                               // new curve
    (vector_t){70, 26.86},   (vector_t){75, 27.61},  // 38
    (vector_t){75, 0},                               // 39
    (vector_t){75, 0},                               // new line
    (vector_t){75, 27.61},   (vector_t){100, 27.61}, // 40
    (vector_t){100, 0},                              // 41
    (vector_t){100, 0},                              // new curve
    (vector_t){100, 27.61},  (vector_t){105, 26.86}, // 42
    (vector_t){105, 0},                              // 43
    (vector_t){105, 0},                              // new curve
    (vector_t){105, 26.86},  (vector_t){110, 25.61}, // 44
    (vector_t){110, 0},                              // 45
    (vector_t){110, 0},                              // new curve
    (vector_t){110, 25.61},  (vector_t){115, 23.36}, // 46
    (vector_t){115, 0},                              // 47
    (vector_t){115, 0},                              // new curve
    (vector_t){115, 23.36},  (vector_t){125, 16.56}, // 48
    (vector_t){125, 0},                              // 49
    (vector_t){125, 0},                              // new curve
    (vector_t){125, 16.56},  (vector_t){128, 14.61}, // 50
    (vector_t){128, 0},                              // 51
    (vector_t){128, 0},      (vector_t){128, 14.61},
    (vector_t){131, 12.84}, // 52
    (vector_t){131, 0},     // 53
    (vector_t){131, 0},      (vector_t){131, 12.84},
    (vector_t){135, 10.76}, // 54
    (vector_t){135, 0},     // 55
    (vector_t){135, 0},      (vector_t){135, 10.76},
    (vector_t){139, 9}, // 56
    (vector_t){139, 0}, // 57
    (vector_t){139, 0},      (vector_t){139, 9},
    (vector_t){143, 7.56}, // 58
    (vector_t){143, 0},    // 59
    (vector_t){143, 0},      (vector_t){143, 7.56},
    (vector_t){146, 6.69}, // 60
    (vector_t){146, 0},    // 61
    (vector_t){146, 0},      (vector_t){146, 6.69},
    (vector_t){149, 6}, // 62
    (vector_t){149, 0}, // 63
    (vector_t){149, 0},      (vector_t){149, 6},
    (vector_t){150, 5.81}, // 64
    (vector_t){150, 0},    // 65
    (vector_t){150, 0},      (vector_t){150, 5.81},
    (vector_t){151, 5.64}, // 66
    (vector_t){151, 0},    // 67
    (vector_t){151, 0},      (vector_t){151, 5.64},
    (vector_t){153, 5.36}, // 68
    (vector_t){153, 0},    // 69
    (vector_t){153, 0},      (vector_t){153, 5.36},
    (vector_t){155, 5.16}, // 70
    (vector_t){155, 0},    // 71
    (vector_t){155, 0},      (vector_t){155, 5.16},
    (vector_t){157, 5.04}, // 72
    (vector_t){157, 0},    // 73
    (vector_t){157, 0},      (vector_t){157, 5.04},
    (vector_t){159, 5},                             // 74
    (vector_t){159, 0},                             // 75
    (vector_t){159, 0},                             // line
    (vector_t){159, 5},      (vector_t){200, 5},    // 76
    (vector_t){200, 0},                             // 77
    (vector_t){200, 0},                             // curve
    (vector_t){200, 5},      (vector_t){202, 5.02}, // 78
    (vector_t){202, 0},                             // 79
    (vector_t){202, 0},      (vector_t){202, 5.02},
    (vector_t){204, 5.08}, // 80
    (vector_t){204, 0},    // 81
    (vector_t){204, 0},      (vector_t){204, 5.08},
    (vector_t){206, 5.18}, // 82
    (vector_t){206, 0},    // 83
    (vector_t){206, 0},      (vector_t){206, 5.18},
    (vector_t){209, 5.405}, // 84
    (vector_t){209, 0},     // 85
    (vector_t){209, 0},      (vector_t){209, 5.405},
    (vector_t){212, 5.72}, // 86
    (vector_t){212, 0},    // 87
    (vector_t){212, 0},      (vector_t){212, 5.72},
    (vector_t){216, 6.28}, // 88
    (vector_t){216, 0},    // 89
    (vector_t){216, 0},      (vector_t){216, 6.28},
    (vector_t){220, 7}, // 90
    (vector_t){220, 0}, // 91
    (vector_t){220, 0},      (vector_t){220, 7},
    (vector_t){226, 8.38}, // 92
    (vector_t){226, 0},    // 93
    (vector_t){226, 0},      (vector_t){226, 8.38},
    (vector_t){230, 9.5}, // 94
    (vector_t){230, 0},   // 95
    (vector_t){230, 0},      (vector_t){230, 9.5},
    (vector_t){235, 11.125}, // 96
    (vector_t){235, 0},      // 97
    (vector_t){235, 0},      (vector_t){235, 11.125},
    (vector_t){240, 13}, // 98
    (vector_t){240, 0},  // 99
    (vector_t){240, 0},      (vector_t){240, 13},
    (vector_t){244, 14.68}, // 100
    (vector_t){244, 0},     // 101
    (vector_t){244, 0},      (vector_t){244, 14.68},
    (vector_t){250, 17.5}, // 102
    (vector_t){250, 0},    // 103
    (vector_t){250, 0},      (vector_t){250, 17.5},
    (vector_t){255, 20.125}, // 104
    (vector_t){255, 0},      // 105
    (vector_t){255, 0},      (vector_t){255, 20.125},
    (vector_t){262, 24.22}, // 106
    (vector_t){262, 0},     // 107
    (vector_t){262, 0},      (vector_t){262, 24.22},
    (vector_t){265, 26.125},                          // 108
    (vector_t){265, 0},                               // 109
    (vector_t){265, 0},                               // new curve
    (vector_t){265, 26.125}, (vector_t){320, 61.875}, // 110
    (vector_t){320, 0},                               // 111

    (vector_t){400, 0},                             // new curve // 112
    (vector_t){400, 25},                            // 113
    (vector_t){420, 18},                            // 114
    (vector_t){420, 0},                             // 115
    (vector_t){420, 0},                             // new curve
    (vector_t){420, 18},     (vector_t){422, 17.4}, // 116
    (vector_t){422, 0},                             // 117
    (vector_t){422, 0},                             // new curve
    (vector_t){422, 17.4},   (vector_t){440, 12.9}, // 118
    (vector_t){440, 0},                             // 119
    (vector_t){440, 0},                             // new curve
    (vector_t){440, 12.9},   (vector_t){492.7, 5},  // 120
    (vector_t){492.7, 0},                           // 121
    (vector_t){492.7, 0},                           // new line
    (vector_t){492.7, 5},    (vector_t){600, 5},    // 122
    (vector_t){600, 0},                             // 123
};

const size_t WHEEL_NUM_POINTS = 250;
const double WHEEL_OUTER_RAD = 1.8 * 10;
const double WHEEL_INNER_RAD = 1.35 * 10;
const double WHEEL_MASS = 100;
const rgb_color_t WHEEL_COLOR = (rgb_color_t){0, 0, 0};
body_t *front_wheel;
body_t *back_wheel;
const double x_back = 12 * 10;
const double x_front = 20.5 * 10;
const double y = 22 / 10.0;

// timer constants
#define TIME_LENGTH 10
const double MIN_TO_SEC = 60;
const double START_TIME = 120.0;
const size_t FONT_SIZE = 96;
#define TIMER_DIMENSIONS                                                       \
  (vector_t) { 0.1 * WINDOW.x, 0.15 * WINDOW.y }
#define TIMER_POSITION                                                         \
  (vector_t) { WINDOW.x - TIMER_DIMENSIONS.x, WINDOW.y }
const rgb_color_t TEXT_COLOR = (rgb_color_t){0.0, 0.0, 0.0};

// track constants
const double TRACK_HEIGHT = 20.0;
const double TRACK_MASS = INFINITY;
const rgb_color_t TRACK_COLOR = {0.545098039216, 0.270588235294,
                                 0.0745098039216};

const double GRAVITATIONAL_ACCELERATION = 100.0;
const double DRAG = 0.2;
const double SUSPENSION_CONSTANT = 10000.0;
const double EQ_DIST = 10.0;

const double AIR_ANGULAR_VELOCITY = 1.0;
const double BIKE_ACCELERATION = 200.0;

typedef enum { MENU = 1, TIMER = 2, SCORE = 3 } game_state_t;

typedef struct button {
  text_input_t text_input;
  body_t *body;
} button_t;

typedef struct state {
  scene_t *scene;
  double clock;
  double dt;
  text_input_t timer_text;
  bool pushed_down;
  bool in_air;
  bool game_over;
  game_state_t game_state;
  list_t *button_list;
  text_input_t title;
  rgb_color_t bike_color;
  int16_t past_second;
  size_t level;
  sound_t sound;
  bool sound_changed;
  double sound_timer;
  double goal;
  bool win;
  size_t score;
} state_t;

typedef list_t *(*track_t)();

typedef enum { BIKE = 1, TRACK = 2 } body_type_t;

// helper functions
void button_free(button_t *button) {
  sdl_remove_text(button->text_input);
  body_remove(button->body);
  free(button);
}

// bike functions
void section_two(list_t *shape, vector_t start, vector_t end,
                 size_t num_points) {
  double dx = (end.x - start.x) / num_points;
  vector_t *coord_zero = malloc(sizeof(vector_t));
  *coord_zero = start;
  list_add(shape, coord_zero);
  for (size_t t = 1; t < num_points; t++) {
    double x = start.x + dx * t;
    double y = 2 + 0.5 * sinh(10 * (x + 1));
    vector_t *coord = malloc(sizeof(vector_t));
    *coord = (vector_t){x, y};
    list_add(shape, coord);
  }
}

void section_five(list_t *shape, vector_t start, vector_t end,
                  size_t num_points) {
  double dx = (end.x - start.x) / num_points;
  vector_t *coord_zero = malloc(sizeof(vector_t));
  *coord_zero = start;
  list_add(shape, coord_zero);
  for (size_t t = 1; t < num_points; t++) {
    double x = start.x + dx * t;
    double y = 2.2 - sqrt(0.06 - (x - 0.52) * (x - 0.52));
    vector_t *coord = malloc(sizeof(vector_t));
    *coord = (vector_t){x, y};
    list_add(shape, coord);
  }
}

void section_eight(list_t *shape, vector_t start, vector_t end,
                   size_t num_points) {
  double dx = (end.x - start.x) / num_points;
  vector_t *coord_zero = malloc(sizeof(vector_t));
  *coord_zero = start;
  list_add(shape, coord_zero);
  for (size_t t = 1; t < num_points; t++) {
    double x = start.x + dx * t;
    double y = 0.95 - 0.34 * cos(0.7 * x - 1) / (x - 1.46);
    vector_t *coord = malloc(sizeof(vector_t));
    *coord = (vector_t){x, y};
    list_add(shape, coord);
  }
}

void section_nine(list_t *shape, vector_t start, vector_t end,
                  size_t num_points) {
  double dx = (end.x - start.x) / num_points;
  vector_t *coord_zero = malloc(sizeof(vector_t));
  *coord_zero = start;
  list_add(shape, coord_zero);
  for (size_t t = 1; t < num_points; t++) {
    double x = start.x + dx * t;
    double y = 1.005 + 0.15 * cos(0.55 * x + 3.5);
    vector_t *coord = malloc(sizeof(vector_t));
    *coord = (vector_t){x, y};
    list_add(shape, coord);
  }
}

void section_ten(list_t *shape, vector_t start, vector_t end,
                 size_t num_points) {
  double dx = (end.x - start.x) / num_points;
  vector_t *coord_zero = malloc(sizeof(vector_t));
  *coord_zero = start;
  list_add(shape, coord_zero);
  for (size_t t = 1; t < num_points; t++) {
    double x = start.x + dx * t;
    double y = 1.15 - exp(0.65 * (x + 5.0));
    vector_t *coord = malloc(sizeof(vector_t));
    *coord = (vector_t){x, y};
    list_add(shape, coord);
  }
}

void section_twelve(list_t *shape, vector_t start, size_t num_points) {
  vector_t *coord_zero = malloc(sizeof(vector_t));
  *coord_zero = start;
  list_add(shape, coord_zero);
  double dt = M_PI / num_points;
  for (size_t i = 1; i < num_points; i++) {
    double t = 0.61646 * M_PI + dt * i;
    double x = 0.22361 * cos(t) - 6.5;
    double y = 0.22361 * sin(t) - 1.5;
    vector_t *coord = malloc(sizeof(vector_t));
    *coord = (vector_t){x, y};
    list_add(shape, coord);
  }
}

void section_fourteen(list_t *shape, vector_t start, size_t num_points) {
  double interval_length = 5.7735;
  double interval_start = 1.4606;
  vector_t *coord_zero = malloc(sizeof(vector_t));
  *coord_zero = start;
  list_add(shape, coord_zero);
  double dt = interval_length / num_points;
  for (size_t i = 1; i < num_points; i++) {
    double t = interval_start + dt * i;
    double x = 0.89443 * sin(t) - 6.5;
    double y = 0.89443 * cos(t) - 1.5;
    vector_t *coord = malloc(sizeof(vector_t));
    *coord = (vector_t){x, y};
    list_add(shape, coord);
  }
}

void section_sixteen(list_t *shape, vector_t start, size_t num_points) {
  double interval_length = 5.946;
  double interval_start = 0.534;
  vector_t *coord_zero = malloc(sizeof(vector_t));
  *coord_zero = start;
  list_add(shape, coord_zero);
  double dt = interval_length / num_points;
  for (size_t i = 1; i < num_points; i++) {
    double t = interval_start + dt * i;
    double x = 1.3416 * cos(t) - 6.5;
    double y = 1.3416 * sin(t) - 1.5;
    vector_t *coord = malloc(sizeof(vector_t));
    *coord = (vector_t){x, y};
    list_add(shape, coord);
  }
}

void section_twenty(list_t *shape, vector_t start, vector_t end,
                    size_t num_points) {
  double dx = (end.x - start.x) / num_points;
  vector_t *coord_zero = malloc(sizeof(vector_t));
  *coord_zero = start;
  list_add(shape, coord_zero);
  for (size_t t = 1; t < num_points; t++) {
    double x = start.x + dx * t;
    double y = -1.4 - 1.0 / (24 * (x + 1.2) * (x + 1.2) - 3 * (x + 1.2) - 9);
    vector_t *coord = malloc(sizeof(vector_t));
    *coord = (vector_t){x, y};
    list_add(shape, coord);
  }
}

void section_twenty_two(list_t *shape, vector_t start, vector_t end,
                        size_t num_points) {
  double dx = (end.x - start.x) / num_points;
  vector_t *coord_zero = malloc(sizeof(vector_t));
  *coord_zero = start;
  list_add(shape, coord_zero);
  for (size_t t = 1; t < num_points; t++) {
    double x = start.x + dx * t;
    double y = 2.0 - 4.0 * cosh(0.4 * (x - 0.2)) / (x + 1);
    vector_t *coord = malloc(sizeof(vector_t));
    *coord = (vector_t){x, y};
    list_add(shape, coord);
  }
}

void section_twenty_four(list_t *shape, vector_t start, size_t num_points) {
  double interval_length = 3.141;
  double interval_start = 3.532;
  vector_t *coord_zero = malloc(sizeof(vector_t));
  *coord_zero = start;
  list_add(shape, coord_zero);
  double dt = interval_length / num_points;
  for (size_t i = 1; i < num_points; i++) {
    double t = interval_start + dt * i;
    double x = 0.22361 * cos(t) + 2.5;
    double y = 0.22361 * sin(t) - 1.5;
    vector_t *coord = malloc(sizeof(vector_t));
    *coord = (vector_t){x, y};
    list_add(shape, coord);
  }
}

void section_twenty_six(list_t *shape, vector_t start, size_t num_points) {
  double interval_length = 5.47;
  double interval_start = -0.1525;
  vector_t *coord_zero = malloc(sizeof(vector_t));
  *coord_zero = start;
  list_add(shape, coord_zero);
  double dt = interval_length / num_points;
  for (size_t i = 1; i < num_points; i++) {
    double t = interval_start + dt * i;
    double x = 0.89443 * sin(t) + 2.5;
    double y = 0.89443 * cos(t) - 1.5;
    vector_t *coord = malloc(sizeof(vector_t));
    *coord = (vector_t){x, y};
    list_add(shape, coord);
  }
}

void section_twenty_eight(list_t *shape, vector_t start, size_t num_points) {
  double interval_length = 5.94;
  double interval_start = 2.15;
  vector_t *coord_zero = malloc(sizeof(vector_t));
  *coord_zero = start;
  list_add(shape, coord_zero);
  double dt = interval_length / num_points;
  for (size_t i = 1; i < num_points; i++) {
    double t = interval_start + dt * i;
    double x = 1.3416 * cos(t) + 2.5;
    double y = 1.3416 * sin(t) - 1.5;
    vector_t *coord = malloc(sizeof(vector_t));
    *coord = (vector_t){x, y};
    list_add(shape, coord);
  }
}

void section_thirty(list_t *shape, vector_t start, vector_t end,
                    size_t num_points) {
  double dx = (end.x - start.x) / num_points;
  vector_t *coord_zero = malloc(sizeof(vector_t));
  *coord_zero = start;
  list_add(shape, coord_zero);
  for (size_t t = 1; t < num_points; t++) {
    double x = start.x + dx * t;
    double y = 2.0 - 4.0 * cosh(0.4 * (x - 0.2)) / (x + 1);
    vector_t *coord = malloc(sizeof(vector_t));
    *coord = (vector_t){x, y};
    list_add(shape, coord);
  }
}

void section_thirty_two(list_t *shape, vector_t start, vector_t end,
                        size_t num_points) {
  double dx = (end.x - start.x) / num_points;
  vector_t *coord_zero = malloc(sizeof(vector_t));
  *coord_zero = start;
  list_add(shape, coord_zero);
  for (size_t t = 1; t < num_points; t++) {
    double x = start.x + dx * t;
    double y = 0.7 + 0.6 * sqrt(0.1 * (3.52 - x));
    vector_t *coord = malloc(sizeof(vector_t));
    *coord = (vector_t){x, y};
    list_add(shape, coord);
  }
}

void section_thirty_five(list_t *shape, vector_t start, vector_t end,
                         size_t num_points) {
  double dx = (end.x - start.x) / num_points;
  vector_t *coord_zero = malloc(sizeof(vector_t));
  *coord_zero = start;
  list_add(shape, coord_zero);
  for (size_t t = 1; t < num_points; t++) {
    double x = start.x + dx * t;
    double y = 2.2 + sqrt(0.06 - (x - 0.52) * (x - 0.52));
    vector_t *coord = malloc(sizeof(vector_t));
    *coord = (vector_t){x, y};
    list_add(shape, coord);
  }
}

void section_forty_nine(list_t *shape, vector_t start, size_t num_points) {
  vector_t *coord_zero = malloc(sizeof(vector_t));
  double interval_length = 0.9526 * M_PI;
  double interval_start = 0.3 * M_PI;
  *coord_zero = start;
  list_add(shape, coord_zero);
  double dt = interval_length / num_points;
  for (size_t i = 1; i < num_points; i++) {
    double t = interval_start + dt * i;
    double x = cos(t) - 1;
    double y = 0.9 * sin(t) + 4.5;
    vector_t *coord = malloc(sizeof(vector_t));
    *coord = (vector_t){x, y};
    list_add(shape, coord);
  }
}

void section_fifty_one(list_t *shape, vector_t start, vector_t end,
                       size_t num_points) {
  double dx = (end.x - start.x) / num_points;
  vector_t *coord_zero = malloc(sizeof(vector_t));
  *coord_zero = start;

  list_add(shape, coord_zero);
  for (size_t t = 1; t < num_points; t++) {
    double x = start.x + dx * t;
    double y = 4.0 + (x + 1.2) / (4.0 * (x + 2.2));
    vector_t *coord = malloc(sizeof(vector_t));
    *coord = (vector_t){x, y};
    list_add(shape, coord);
  }
}

list_t *make_bike_shape() {
  list_t *shape = list_init(BIKE_NUM_POINTS, free);
  size_t j = 0;
  for (size_t i = 0; i < NUM_SECTIONS; i++) {
    if (i == 2) {
      section_two(shape, COORDS[i], COORDS[i + 1],
                  PROPORTIONS[j] * BIKE_NUM_POINTS);
      j++;
    }
    if (i == 5) {
      section_five(shape, COORDS[i], COORDS[i + 1],
                   PROPORTIONS[j] * BIKE_NUM_POINTS);
      j++;
    } else if (i == 8) {
      section_eight(shape, COORDS[i], COORDS[i + 1],
                    PROPORTIONS[j] * BIKE_NUM_POINTS);
      j++;
    } else if (i == 9) {
      section_nine(shape, COORDS[i], COORDS[i + 1],
                   PROPORTIONS[j] * BIKE_NUM_POINTS);
      j++;
    } else if (i == 10) {
      section_ten(shape, COORDS[i], COORDS[i + 1],
                  PROPORTIONS[j] * BIKE_NUM_POINTS);
      j++;
    } else if (i == 12) {
      section_twelve(shape, COORDS[i], PROPORTIONS[j] * BIKE_NUM_POINTS);
      j++;
    } else if (i == 14) {
      section_fourteen(shape, COORDS[i], PROPORTIONS[j] * BIKE_NUM_POINTS);
      j++;
    } else if (i == 16) {
      section_sixteen(shape, COORDS[i], PROPORTIONS[j] * BIKE_NUM_POINTS);
      j++;
    } else if (i == 18) {
      section_ten(shape, COORDS[i], COORDS[i + 1],
                  PROPORTIONS[j] * BIKE_NUM_POINTS);
      j++;
    } else if (i == 20) {
      section_twenty(shape, COORDS[i], COORDS[i + 1],
                     PROPORTIONS[j] * BIKE_NUM_POINTS);
      j++;
    } else if (i == 22) {
      section_twenty_two(shape, COORDS[i], COORDS[i + 1],
                         PROPORTIONS[j] * BIKE_NUM_POINTS);
    } else if (i == 24) {
      section_twenty_four(shape, COORDS[i], PROPORTIONS[j] * BIKE_NUM_POINTS);
    } else if (i == 26) {
      section_twenty_six(shape, COORDS[i], PROPORTIONS[j] * BIKE_NUM_POINTS);
    } else if (i == 28) {
      section_twenty_eight(shape, COORDS[i], PROPORTIONS[j] * BIKE_NUM_POINTS);
    } else if (i == 30) {
      section_thirty(shape, COORDS[i], COORDS[i + 1],
                     PROPORTIONS[j] * BIKE_NUM_POINTS);
    } else if (i == 32) {
      section_thirty_two(shape, COORDS[i], COORDS[i + 1],
                         PROPORTIONS[j] * BIKE_NUM_POINTS);
    } else if (i == 35) {
      section_thirty_five(shape, COORDS[i], COORDS[i + 1],
                          PROPORTIONS[j] * BIKE_NUM_POINTS);
    } else if (i == 49) {
      section_forty_nine(shape, COORDS[i], PROPORTIONS[j] * BIKE_NUM_POINTS);
    } else if (i == 51) {
      section_fifty_one(shape, COORDS[i], COORDS[i + 1],
                        PROPORTIONS[j] * BIKE_NUM_POINTS);
    } else {
      vector_t *coord = malloc(sizeof(vector_t));
      *coord = COORDS[i];
      list_add(shape, coord);
    }
  }
  return shape;
}

list_t *scale_polygon(double scalar, list_t *list) {
  list_t *scaled_polygon = list_init(list_size(list), free);
  for (size_t i = 0; i < list_size(list); i++) {
    vector_t *new_vector = malloc(sizeof(vector_t));
    *new_vector = vec_multiply(scalar, *(vector_t *)list_get(list, i));
    list_add(scaled_polygon, new_vector);
  }
  return scaled_polygon;
}

list_t *make_track_one() {
  list_t *bodies = list_init(NUM_BODIES - 4, NULL);
  size_t i = 0;
  for (size_t j = 0; j < NUM_BODIES - 4; j++) {
    list_t *shape = list_init(4, free);
    for (size_t k = 0; k < 4; k++) {
      vector_t *coord = malloc(sizeof(vector_t));
      *coord = TRACK_ONE_COORDS[i + k];
      list_add(shape, coord);
    }
    list_t *new_shape1 = scale_polygon(90.0, shape);
    list_t *new_shape2 = scale_polygon(90.0, shape);
    polygon_translate(new_shape2, (vector_t){0, 30.0});
    list_free(shape);
    i += 4;
    body_type_t *type = malloc(sizeof(*type));
    *type = TRACK;
    body_t *body1 =
        body_init_with_info(new_shape1, TRACK_MASS, TRACK_COLOR, type, free);
    body_t *body2 =
        body_init_with_info(new_shape2, TRACK_MASS, GREEN, type, free);
    list_add(bodies, body2);
    list_add(bodies, body1);
  }
  return bodies;
}

list_t *make_track_two() {
  list_t *bodies = list_init(NUM_BODIES2, NULL);
  size_t i = 0;
  for (size_t j = 0; j < NUM_BODIES2; j++) {
    list_t *shape = list_init(4, free);
    for (size_t k = 0; k < 4; k++) {
      vector_t *coord = malloc(sizeof(vector_t));
      *coord = TRACK_TWO_COORDS[i + k];
      list_add(shape, coord);
    }
    list_t *new_shape1 = scale_polygon(90.0, shape);
    list_t *new_shape2 = scale_polygon(90.0, shape);
    polygon_translate(new_shape2, (vector_t){0, 30.0});
    list_free(shape);
    i += 4;
    body_type_t *type = malloc(sizeof(*type));
    *type = TRACK;
    body_t *body1 =
        body_init_with_info(new_shape1, TRACK_MASS, TRACK_COLOR, type, free);
    body_t *body2 =
        body_init_with_info(new_shape2, TRACK_MASS, GREEN, type, free);
    list_add(bodies, body2);
    list_add(bodies, body1);
  }
  return bodies;
}

list_t *make_track_three() {
  list_t *bodies1 = make_track_one();
  list_t *bodies2 = make_track_one();
  list_t *last_shape =
      body_get_shape(list_get(bodies1, list_size(bodies1) - 1));
  double last_x =
      ((vector_t *)list_get(last_shape, list_size(last_shape) - 1))->x;
  for (size_t i = 0; i < list_size(bodies2); i++) {
    body_t *body = list_get(bodies2, i);
    vector_t centroid = body_get_centroid(body);
    body_set_centroid(body, (vector_t){centroid.x + 10 * last_x, centroid.y});
  }
  list_free(last_shape);
  // list_append(bodies1, bodies2);
  list_free(bodies1);
  return bodies2;
}

body_t *make_bike(rgb_color_t color) {
  list_t *shape = make_bike_shape();
  list_t *scaled_shape = scale_polygon(10, shape);
  list_free(shape);
  body_type_t *type = malloc(sizeof(*type));
  *type = BIKE;
  body_t *bike =
      body_init_with_info(scaled_shape, BIKE_MASS, color, type, free);
  body_set_centroid(bike, STARTING_POSITION); //
  body_set_normal_moment_of_inertia(bike, BIKE_MOMENT);
  return bike;
}

// basic track building
// list_t *make_track() {
//   list_t *track = list_init(4, free);
//   vector_t *v1 = malloc(sizeof(vector_t));
//   *v1 = VEC_ZERO;
//   vector_t *v2 = malloc(sizeof(vector_t));
//   *v2 = (vector_t){0, TRACK_HEIGHT};
//   vector_t *v3 = malloc(sizeof(vector_t));
//   *v3 = (vector_t){WINDOW.x, (WINDOW.y - TRACK_HEIGHT) / 2.0};
//   vector_t *v4 = malloc(sizeof(vector_t));
//   *v4 = (vector_t){WINDOW.x, 0};
//   list_add(track, v1);
//   list_add(track, v2);
//   list_add(track, v3);
//   list_add(track, v4);
//   return track;
// }

// list_t *make_actual_track() {
//   list_t *track = list_init(4, free);
//   vector_t *v1 = malloc(sizeof(vector_t));
//   *v1 = (vector_t){0, 10};
//   vector_t *v2 = malloc(sizeof(vector_t));
//   *v2 = (vector_t){0, TRACK_HEIGHT + 10};
//   vector_t *v3 = malloc(sizeof(vector_t));
//   *v3 = (vector_t){WINDOW.x, (WINDOW.y - TRACK_HEIGHT) / 2.0 + 10};
//   vector_t *v4 = malloc(sizeof(vector_t));
//   *v4 = (vector_t){WINDOW.x, 10};
//   list_add(track, v1);
//   list_add(track, v2);
//   list_add(track, v3);
//   list_add(track, v4);
//   return track;
// }

void initialize_body_list(state_t *state, track_t make_track) {
  scene_add_body(state->scene, make_bike(state->bike_color));
  list_t *bodies = make_track();
  for (size_t i = 0; i < list_size(bodies); i++) {
    body_t *body = list_get(bodies, i);
    scene_add_body(state->scene, body);
  }
  list_free(bodies);
}

bool double_is_close(double a, double b, double threshold) {
  return fabs(a - b) < threshold;
}

list_t *create_triangle(double side) {
  list_t *triangle = list_init(3, free);
  vector_t *v1 = malloc(sizeof(vector_t));
  *v1 = (vector_t){side / 2.0, -sqrt(3) / 2 * side};
  vector_t *v2 = malloc(sizeof(vector_t));
  *v2 = (vector_t){-side / 2.0, -sqrt(3) / 2 * side};
  vector_t *v3 = malloc(sizeof(vector_t));
  *v3 = (vector_t){0, side};
  list_add(triangle, v1);
  list_add(triangle, v2);
  list_add(triangle, v3);
  return triangle;
}

list_t *create_collision_triangle() { return create_triangle(0.0001); }

vector_t find_colliding_point(body_t *body1, body_t *body2) {
  list_t *body1_shape = body_get_shape(body1);
  list_t *body2_shape = body_get_shape(body2);
  for (size_t i = 0; i < list_size(body1_shape); i++) {
    vector_t centroid = *(vector_t *)list_get(body1_shape, i);
    list_t *triangle = create_collision_triangle();
    polygon_translate(triangle, centroid);
    collision_info_t collision = find_collision(triangle, body2_shape);
    if (collision.collided) {
      list_free(body1_shape);
      list_free(body2_shape);
      return centroid;
    }
  }
  list_free(body1_shape);
  list_free(body2_shape);
  return vec_negate(WINDOW);
}

void ground_collision(body_t *body, body_t *ground, vector_t axis, void *aux) {
  list_t *wheel_shape = body_get_shape(body);
  list_t *track_shape = body_get_shape(ground);
  double angle_diff = body_get_rotation(body) - vec_angle(axis);
  // char str[10];
  // sprintf(str, "%.9f", angle_diff);
  // puts(str);
  if (!double_is_close(fabs(angle_diff), M_PI / 2, 0.02) &&
      !double_is_close(fabs(angle_diff), 3 * M_PI / 2, 0.02)) {
    vector_t intersect = find_colliding_point(body, ground);
    if (!double_is_close(intersect.x, -WINDOW.x, 1e-5)) {
      body_set_pivot(body, intersect);
      if (angle_diff > -M_PI / 2) {
        body_set_angular_velocity(body, -0.2);
      } else if (angle_diff < M_PI / 2) {
        body_set_angular_velocity(body, 0.2);
      } else {
        body_set_angular_velocity(body, -0.2);
      }
    } else {
      body_set_angular_velocity(body, 0.0);
    }
  } else {
    body_set_angular_velocity(body, 0.0);
  }
  list_free(wheel_shape);
  list_free(track_shape);
}

void create_ground_collision(state_t *state, body_t *body, body_t *ground) {
  create_collision(state->scene, body, ground, ground_collision, state, free);
}

void initialize_force_list(state_t *state) {
  // char str[10];
  // sprintf(str, "%d", scene_bodies(state->scene));
  // puts(str);
  body_t *bike = scene_get_body(state->scene, 0);
  assert(*(body_type_t *)body_get_info(bike) == BIKE);
  create_downwards_gravity(state->scene, GRAVITATIONAL_ACCELERATION, bike);
  create_drag(state->scene, DRAG, bike);
  for (size_t i = 1; i < scene_bodies(state->scene); i++) {
    create_ground_collision(state, bike, scene_get_body(state->scene, i));
    create_physics_collision(state->scene, 0.0, bike,
                             scene_get_body(state->scene, i));
    create_normal(state->scene, bike, scene_get_body(state->scene, i));
  }
}

void clear_buttons(state_t *state) {
  for (int16_t i = list_size(state->button_list) - 1; i >= 0; i--) {
    button_t *button = list_remove(state->button_list, i);
    button_free(button);
  }
  sdl_remove_text(state->title);
  scene_tick(state->scene, 0.0);
}

void clear_scene(state_t *state) {
  for (size_t i = 0; i < scene_bodies(state->scene); i++) {
    body_t *body = scene_get_body(state->scene, i);
    body_remove(body);
  }
  scene_tick(state->scene, 0.0);
  free(state->timer_text.string);
}

void initialize_game(state_t *state) {
  clear_buttons(state);
  // clear_scene(state);
  state->pushed_down = false;
  state->dt = 0.0;
  state->in_air = false;
  state->game_over = false;
  track_t track_function;
  switch (state->level) {
  case 1:
    track_function = make_track_one;
    state->goal = 372.041 * 90.0;
    break;
  case 2:
    track_function = make_track_two;
    break;
  case 3:
    track_function = make_track_three;
    break;
  }
  sdl_clear_text();
  initialize_body_list(state, track_function);
  initialize_force_list(state);
  if (state->game_state == TIMER) {
    // scene_free(state->scene);
    // state->scene = scene_init();
    state->clock = START_TIME;
    state->past_second = START_TIME;
    // sdl_write_text(state->timer_text, "LeagueGothic", "Regular");
  } else if (state->game_state == SCORE) {
    state->score = 0;
  }
}

void on_key(state_t *state, char key, key_event_type_t type, double held_time) {
  body_t *bike = scene_get_body(state->scene, 0);
  double angle = body_get_rotation(bike);
  if (type == KEY_PRESSED) {
    switch (key) {
    case LEFT_ARROW:
      if (!state->pushed_down) {
        state->pushed_down = true;
        create_applied(state->scene,
                       (vector_t){-BIKE_MASS * BIKE_ACCELERATION * cos(angle),
                                  -sin(angle)},
                       bike);
        state->sound = DEC;
        state->sound_changed = true;
      } else {
        state->sound_changed = false;
      }
      break;
    case RIGHT_ARROW:
      if (!state->pushed_down) {
        state->pushed_down = true;
        create_applied(
            state->scene,
            (vector_t){BIKE_MASS * BIKE_ACCELERATION * cos(angle), sin(angle)},
            bike);
        state->sound = ACC;
        state->sound_changed = true;
      } else {
        state->sound_changed = false;
      }
      break;
    case UP_ARROW:
      if (state->in_air) {
        body_increment_angular_velocity(bike, 2.0 * state->dt);
      }
      break;
    case DOWN_ARROW:
      if (state->in_air) {
        body_increment_angular_velocity(bike, -2.0 * state->dt);
      }
      break;
    case SPACE:
      state->game_over = true;
      state->win = true;
      break;
    }
  } else if (type == KEY_RELEASED) {
    switch (key) {
    case LEFT_ARROW:
      state->pushed_down = false;
      create_applied(
          state->scene,
          (vector_t){BIKE_MASS * BIKE_ACCELERATION * cos(angle), sin(angle)},
          bike);
      state->sound = IDLE;
      state->sound_changed = true;
      break;
    case RIGHT_ARROW:
      state->pushed_down = false;
      create_applied(
          state->scene,
          (vector_t){-BIKE_MASS * BIKE_ACCELERATION * cos(angle), -sin(angle)},
          bike);
      state->sound = IDLE;
      state->sound_changed = true;
      break;
    }
  }
}

body_t *make_rectangle(double width, double height, rgb_color_t color) {
  vector_t *bottom_left = malloc(sizeof(vector_t));
  *bottom_left = (vector_t){0, 0};
  vector_t *top_left = malloc(sizeof(vector_t));
  *top_left = (vector_t){0, height};
  vector_t *bottom_right = malloc(sizeof(vector_t));
  *bottom_right = (vector_t){width, 0};
  vector_t *top_right = malloc(sizeof(vector_t));
  *top_right = (vector_t){width, height};
  list_t *rectangle_shape = list_init(4, free);
  list_add(rectangle_shape, bottom_left);
  list_add(rectangle_shape, top_left);
  list_add(rectangle_shape, top_right);
  list_add(rectangle_shape, bottom_right);
  body_t *rectangle = body_init(rectangle_shape, BUTTON_MASS, color);
  return rectangle;
}

void make_button(state_t *state, char *string, size_t font_size,
                 vector_t position, vector_t dim, rgb_color_t text_color,
                 rgb_color_t button_color) {
  text_input_t text_input = {.string = string,
                             .font_size = font_size,
                             .position = position,
                             .dim = dim,
                             .color = text_color};
  sdl_write_text(text_input, "Montserrat", "SemiBold");
  body_t *button = make_rectangle(1.5 * dim.x, 1.5 * dim.y, button_color);
  vector_t centroid =
      (vector_t){position.x + dim.x / 2.0, position.y - dim.y / 2.0};
  body_set_centroid(button, centroid);
  scene_add_body(state->scene, button);
  button_t *button_struct = malloc(sizeof(button_t));
  button_struct->text_input = text_input;
  button_struct->body = button;
  list_add(state->button_list, button_struct);
}

// mouse handlers
void on_mouse_start_menu(state_t *state, char key, key_event_type_t type,
                         double x, double y);
void on_mouse_color_menu(state_t *state, char key, key_event_type_t type,
                         double x, double y);
void on_mouse_game_menu(state_t *state, char key, key_event_type_t type,
                        double x, double y);
void on_mouse_level_menu(state_t *state, char key, key_event_type_t type,
                         double x, double y);
void on_mouse_game_over_menu(state_t *state, char key, key_event_type_t type,
                             double x, double y);

void reset_scene(state_t *state) {
  for (size_t i = 0; i < list_size(state->button_list); i++) {
    button_t *button = list_get(state->button_list, i);
    sdl_remove_text(button->text_input);
    free(button);
  }
  scene_free(state->scene);
  state->scene = scene_init();
  sdl_clear_text();
  sdl_render_scene(state->scene);
}

void create_start_menu(state_t *state) {
  reset_scene(state);
  text_input_t title = {.string = "MOTOCROSS MAYHEM",
                        .font_size = FONT_SIZE,
                        .position = TITLE_POSITION,
                        .dim = TITLE_DIMENSIONS,
                        .color = TEXT_COLOR};
  state->title = title;
  sdl_write_text(title, "ChunkFive", "Regular");
  make_button(state, "PLAY", FONT_SIZE, PLAY_POSITION, BUTTON_DIM, TEXT_COLOR,
              BUTTON_COLOR);
  make_button(state, "CUSTOMIZE", FONT_SIZE, CUSTOMIZE_POSITION, BUTTON_DIM,
              TEXT_COLOR, BUTTON_COLOR);
  make_button(state, "SETTINGS", FONT_SIZE, SETTINGS_POSITION, BUTTON_DIM,
              TEXT_COLOR, BUTTON_COLOR);
}

bool color_equals(rgb_color_t color1, rgb_color_t color2) {
  return color1.r == color2.r && color1.g == color2.g && color1.b == color2.b;
}

void create_color_menu(state_t *state) {
  clear_buttons(state);
  text_input_t title = {.string = "BIKE COLOR MENU",
                        .font_size = FONT_SIZE,
                        .position = TITLE_POSITION,
                        .dim = TITLE_DIMENSIONS,
                        .color = TEXT_COLOR};
  state->title = title;
  sdl_write_text(title, "ChunkFive", "Regular");
  rgb_color_t red_button_color, orange_button_color, green_button_color,
      blue_button_color, purple_button_color;
  red_button_color = orange_button_color = green_button_color =
      blue_button_color = purple_button_color = TEXT_COLOR;
  rgb_color_t black_button_color = WHITE;
  if (color_equals(state->bike_color, RED)) {
    red_button_color = YELLOW;
  } else if (color_equals(state->bike_color, ORANGE)) {
    orange_button_color = YELLOW;
  } else if (color_equals(state->bike_color, GREEN)) {
    green_button_color = YELLOW;
  } else if (color_equals(state->bike_color, BLUE)) {
    blue_button_color = YELLOW;
  } else if (color_equals(state->bike_color, PURPLE)) {
    purple_button_color = YELLOW;
  } else {
    black_button_color = YELLOW;
  }
  make_button(state, "RED", FONT_SIZE, RED_POSITION, BUTTON_DIM,
              red_button_color, RED);
  make_button(state, "ORANGE", FONT_SIZE, ORANGE_POSITION, BUTTON_DIM,
              orange_button_color, ORANGE);
  make_button(state, "GREEN", FONT_SIZE, GREEN_POSITION, BUTTON_DIM,
              green_button_color, GREEN);
  make_button(state, "BLUE", FONT_SIZE, BLUE_POSITION, BUTTON_DIM,
              blue_button_color, BLUE);
  make_button(state, "PURPLE", FONT_SIZE, PURPLE_POSITION, BUTTON_DIM,
              purple_button_color, PURPLE);
  make_button(state, "BLACK", FONT_SIZE, BLACK_POSITION, BUTTON_DIM,
              black_button_color, TEXT_COLOR);
  make_button(state, "<--", FONT_SIZE, BACK_POSITION, BACK_BUTTON_DIM,
              TEXT_COLOR, BUTTON_COLOR);
}

void create_game_menu(state_t *state) {
  clear_buttons(state);
  text_input_t title = {.string = "GAME MODE MENU",
                        .font_size = FONT_SIZE,
                        .position = TITLE_POSITION,
                        .dim = TITLE_DIMENSIONS,
                        .color = TEXT_COLOR};
  state->title = title;
  sdl_write_text(title, "ChunkFive", "Regular");
  make_button(state, "TIMED", FONT_SIZE, ORANGE_POSITION, BUTTON_DIM,
              TEXT_COLOR, BUTTON_COLOR);
  make_button(state, "TRICKS", FONT_SIZE, PURPLE_POSITION, BUTTON_DIM,
              TEXT_COLOR, BUTTON_COLOR);
  make_button(state, "<--", FONT_SIZE, BACK_POSITION, BACK_BUTTON_DIM,
              TEXT_COLOR, BUTTON_COLOR);
}

void create_level_menu(state_t *state) {
  clear_buttons(state);
  text_input_t title = {.string = "LEVEL SELECT",
                        .font_size = FONT_SIZE,
                        .position = TITLE_POSITION,
                        .dim = TITLE_DIMENSIONS,
                        .color = TEXT_COLOR};
  state->title = title;
  sdl_write_text(title, "ChunkFive", "Regular");
  make_button(state, "1", FONT_SIZE, PLAY_POSITION, BACK_BUTTON_DIM, TEXT_COLOR,
              BUTTON_COLOR);
  make_button(state, "2", FONT_SIZE, CUSTOMIZE_POSITION, BACK_BUTTON_DIM,
              TEXT_COLOR, BUTTON_COLOR);
  make_button(state, "3", FONT_SIZE, SETTINGS_POSITION, BACK_BUTTON_DIM,
              TEXT_COLOR, BUTTON_COLOR);
  make_button(state, "<--", FONT_SIZE, BACK_POSITION, BACK_BUTTON_DIM,
              TEXT_COLOR, BUTTON_COLOR);
}

void create_game_over_menu(state_t *state) {
  clear_buttons(state);
  text_input_t title = {.string = "GAME OVER :(",
                        .font_size = FONT_SIZE,
                        .position = TITLE_POSITION,
                        .dim = TITLE_DIMENSIONS,
                        .color = TEXT_COLOR};
  state->title = title;
  sdl_write_text(title, "ChunkFive", "Regular");
  make_button(state, "RESTART", FONT_SIZE, ORANGE_POSITION, BUTTON_DIM,
              TEXT_COLOR, BUTTON_COLOR);
  make_button(state, "EXIT", FONT_SIZE, PURPLE_POSITION, BUTTON_DIM, TEXT_COLOR,
              BUTTON_COLOR);
}

void create_win_menu(state_t *state) {
  clear_buttons(state);
  text_input_t title = {.string = "YOU WIN! :)",
                        .font_size = FONT_SIZE,
                        .position = TITLE_POSITION,
                        .dim = TITLE_DIMENSIONS,
                        .color = TEXT_COLOR};
  state->title = title;
  sdl_write_text(title, "ChunkFive", "Regular");
  make_button(state, "RESTART", FONT_SIZE, ORANGE_POSITION, BUTTON_DIM,
              TEXT_COLOR, BUTTON_COLOR);
  make_button(state, "EXIT", FONT_SIZE, PURPLE_POSITION, BUTTON_DIM, TEXT_COLOR,
              BUTTON_COLOR);
}

// start menu mouse handler
void on_mouse_start_menu(state_t *state, char key, key_event_type_t type,
                         double x, double y) {
  list_t *collision_tester = create_collision_triangle();
  polygon_translate(collision_tester, (vector_t){x, y});
  button_t *button;
  body_t *button_box;
  list_t *button_shape;
  collision_info_t collision;
  if (type == MOUSE_BUTTON_RELEASED) {
    switch (key) {
    case LEFT_CLICK:
      for (size_t i = 0; i < list_size(state->button_list); i++) {
        button = list_get(state->button_list, i);
        button_box = button->body;
        button_shape = body_get_shape(button_box);
        collision = find_collision(collision_tester, button_shape);
        if (collision.collided) {
          switch (i) {
          case 0:
            create_game_menu(state);
            sdl_on_mouse((mouse_handler_t)on_mouse_game_menu);
            break;
          case 1:
            create_color_menu(state);
            sdl_on_mouse((mouse_handler_t)on_mouse_color_menu);
            break;
          case 2:
            puts("😎");
            break;
          }
          break;
        }
      }
      list_free(button_shape);
    }
  }
  list_free(collision_tester);
}

// color menu mouse handler
void on_mouse_color_menu(state_t *state, char key, key_event_type_t type,
                         double x, double y) {
  list_t *collision_tester = create_collision_triangle();
  polygon_translate(collision_tester, (vector_t){x, y});
  button_t *button;
  body_t *button_box;
  list_t *button_shape;
  collision_info_t collision;
  if (type == MOUSE_BUTTON_RELEASED) {
    switch (key) {
    case LEFT_CLICK:
      for (size_t i = 0; i < list_size(state->button_list); i++) {
        button = list_get(state->button_list, i);
        button_box = button->body;
        button_shape = body_get_shape(button_box);
        collision = find_collision(collision_tester, button_shape);
        if (collision.collided) {
          switch (i) {
          case 0:
            state->bike_color = RED;
            create_color_menu(state);
            break;
          case 1:
            state->bike_color = ORANGE;
            create_color_menu(state);
            break;
          case 2:
            state->bike_color = GREEN;
            create_color_menu(state);
            break;
          case 3:
            state->bike_color = BLUE;
            create_color_menu(state);
            break;
          case 4:
            state->bike_color = PURPLE;
            create_color_menu(state);
            break;
          case 5:
            state->bike_color = TEXT_COLOR;
            create_color_menu(state);
            break;
          case 6:
            create_start_menu(state);
            sdl_on_mouse((mouse_handler_t)on_mouse_start_menu);
            break;
          }
          break;
        }
      }
      list_free(button_shape);
    }
  }
  list_free(collision_tester);
}

// game menu mouse handler
void on_mouse_game_menu(state_t *state, char key, key_event_type_t type,
                        double x, double y) {
  list_t *collision_tester = create_collision_triangle();
  polygon_translate(collision_tester, (vector_t){x, y});
  button_t *button;
  body_t *button_box;
  list_t *button_shape;
  collision_info_t collision;
  if (type == MOUSE_BUTTON_RELEASED) {
    switch (key) {
    case LEFT_CLICK:
      for (size_t i = 0; i < list_size(state->button_list); i++) {
        button = list_get(state->button_list, i);
        button_box = button->body;
        button_shape = body_get_shape(button_box);
        collision = find_collision(collision_tester, button_shape);
        if (collision.collided) {
          switch (i) {
          case 0:
            state->game_state = TIMER;
            sdl_on_mouse((mouse_handler_t)on_mouse_level_menu);
            create_level_menu(state);
            break;
          case 1:
            state->game_state = SCORE;
            sdl_on_mouse((mouse_handler_t)on_mouse_level_menu);
            create_level_menu(state);
            puts("score");
            break;
          case 2:
            create_start_menu(state);
            sdl_on_mouse((mouse_handler_t)on_mouse_start_menu);
            break;
          }
          break;
        }
      }
      list_free(button_shape);
    }
  }
  list_free(collision_tester);
}

// game menu mouse handler
void on_mouse_level_menu(state_t *state, char key, key_event_type_t type,
                         double x, double y) {
  list_t *collision_tester = create_collision_triangle();
  polygon_translate(collision_tester, (vector_t){x, y});
  button_t *button;
  body_t *button_box;
  list_t *button_shape;
  collision_info_t collision;
  if (type == MOUSE_BUTTON_RELEASED) {
    switch (key) {
    case LEFT_CLICK:
      for (size_t i = 0; i < list_size(state->button_list); i++) {
        button = list_get(state->button_list, i);
        button_box = button->body;
        button_shape = body_get_shape(button_box);
        collision = find_collision(collision_tester, button_shape);
        if (collision.collided) {
          switch (i) {
          case 0:
            state->level = 1;
            sdl_on_key(on_key);
            sdl_on_mouse(NULL);
            initialize_game(state);
            break;
          case 1:
            state->level = 2;
            sdl_on_key(on_key);
            sdl_on_mouse(NULL);
            initialize_game(state);
            break;
          case 2:
            state->level = 3;
            sdl_on_key(on_key);
            sdl_on_mouse(NULL);
            initialize_game(state);
            break;
          case 3:
            create_game_menu(state);
            sdl_on_mouse((mouse_handler_t)on_mouse_game_menu);
            break;
          }
          break;
        }
      }
      list_free(button_shape);
    }
  }
  list_free(collision_tester);
}

void on_mouse_game_over_menu(state_t *state, char key, key_event_type_t type,
                             double x, double y) {
  list_t *collision_tester = create_collision_triangle();
  polygon_translate(collision_tester, (vector_t){x, y});
  button_t *button;
  body_t *button_box;
  list_t *button_shape;
  collision_info_t collision;
  body_t *bike = scene_get_body(state->scene, 0);
  if (type == MOUSE_BUTTON_RELEASED) {
    switch (key) {
    case LEFT_CLICK:
      for (size_t i = 0; i < list_size(state->button_list); i++) {
        button = list_get(state->button_list, i);
        button_box = button->body;
        button_shape = body_get_shape(button_box);
        collision = find_collision(collision_tester, button_shape);
        if (collision.collided) {
          switch (i) {
          case 0:
            clear_buttons(state);
            scene_tick(state->scene, 0.0);
            sdl_remove_text(state->title);
            sdl_render_scene(state->scene);
            state->game_over = false;
            state->win = false;
            sdl_on_key(on_key);
            sdl_on_mouse(NULL);
            state->clock = START_TIME;
            break;
          case 1:
            // state->level = 0;
            // emscripten_free(state);
            // state = emscripten_init();
            // sdl_render_scene(state->scene);
            // clear_buttons(state);
            // for (size_t i = 0; i < scene_bodies(state->scene); i++) {
            //   body_t *body = scene_get_body(state->scene, i);
            //   body_remove(body);
            // }
            // scene_tick(state->scene, 0.0);
            // sdl_move_window(CENTER);
            // sdl_on_mouse(on_mouse_start_menu);
            // create_start_menu(state);
            break;
          }
        }
      }
      list_free(button_shape);
    }
  }
  list_free(collision_tester);
}

state_t *emscripten_init() {
  srand(time(NULL));
  vector_t min = VEC_ZERO;
  vector_t max = WINDOW;
  sdl_init(min, max);
  state_t *state = malloc(sizeof(state_t));
  state->scene = scene_init();
  state->bike_color = RED;
  state->game_state = MENU;
  state->button_list = list_init(3, free);
  state->level = 0;
  state->timer_text = (text_input_t){.string = "02:00",
                                     .font_size = FONT_SIZE,
                                     .position = TIMER_POSITION,
                                     .dim = TIMER_DIMENSIONS,
                                     .color = TEXT_COLOR};
  text_input_t title = {.string = "MOTOCROSS MAYHEM",
                        .font_size = FONT_SIZE,
                        .position = TITLE_POSITION,
                        .dim = TITLE_DIMENSIONS,
                        .color = TEXT_COLOR};
  state->title = title;
  sdl_on_mouse((mouse_handler_t)on_mouse_start_menu);
  sdl_add_image("assets/windows-xp-wallpaper-bliss-1024x576.jpg",
                (vector_t){0, WINDOW.y});
  sound_init();
  state->sound = IDLE;
  state->sound_changed = true;
  state->sound_timer = 0.0;
  create_start_menu(state);
  return state;
}

void update_timer(state_t *state) {
  if ((int)state->clock != state->past_second) {
    if (state->clock <= 0.0) {
      state->game_over = true;
      return;
    }
    size_t minutes = state->clock / MIN_TO_SEC;
    size_t seconds = fmod(state->clock, MIN_TO_SEC);
    char timer[TIME_LENGTH] = "";
    char sec[TIME_LENGTH] = "";
    sprintf(timer, "0%lu", minutes);
    strcat(timer, ":");
    sprintf(sec, "%lu", seconds);
    if (seconds / 10 < 1) {
      sprintf(sec, "0%lu", seconds);
    }
    strcat(timer, sec);
    state->timer_text.string = timer;
    sdl_clear_text();
    body_t *bike = scene_get_body(state->scene, 0);
    state->timer_text.position = vec_add(body_get_centroid(bike), CENTER);
    state->timer_text.position.x -= state->timer_text.dim.x;
    sdl_write_text(state->timer_text, "LeagueGothic", "Regular");
  }
}

void update_score(state_t *state) {
  if (!state->game_over) {
    char score_string[TIME_LENGTH] = "";
    sprintf(score_string, "%lu", state->score);
    state->timer_text.string = score_string;
    sdl_clear_text();
    body_t *bike = scene_get_body(state->scene, 0);
    state->timer_text.position = vec_add(body_get_centroid(bike), CENTER);
    state->timer_text.position.x -= state->timer_text.dim.x;
    sdl_write_text(state->timer_text, "LeagueGothic", "Regular");
  }
}

bool check_track_collision(state_t *state) {
  body_t *bike = scene_get_body(state->scene, 0);
  list_t *bike_triangle = create_triangle(100.0);
  polygon_translate(bike_triangle, body_get_pivot(bike));
  assert(scene_bodies(state->scene) > 1);
  for (size_t i = 0; i < scene_bodies(state->scene); i++) {
    body_t *track = scene_get_body(state->scene, i);
    body_type_t *type = body_get_info(track);
    if (*type == TRACK) {
      list_t *track_shape = body_get_shape(track);
      collision_info_t collision = find_collision(bike_triangle, track_shape);
      list_free(track_shape);
      if (collision.collided) {
        list_free(bike_triangle);
        return true;
      }
    }
  }
  list_free(bike_triangle);
  return false;
}

void check_loss(state_t *state) {
  body_t *bike = scene_get_body(state->scene, 0);
  double angle = body_get_rotation(bike);
  if (fabs(fmod(angle, 2 * M_PI)) > M_PI / 2 &&
      fabs(fmod(angle, 2 * M_PI)) < 3 * M_PI / 2) {
    state->game_over = true;
  }
  if (body_get_centroid(bike).y < -50.0) {
    state->game_over = true;
  }
}

bool check_win(state_t *state) {
  body_t *bike = scene_get_body(state->scene, 0);
  vector_t centroid = body_get_centroid(bike);
  if (centroid.x > state->goal) {
    state->game_over = true;
    return true;
  }
  return false;
}

void emscripten_main(state_t *state) {
  state->dt = time_since_last_tick();
  state->sound_timer -= state->dt;
  if (state->sound_timer <= 0.0 || state->sound_changed) {
    sound_play(state->sound);
    state->sound_changed = false;
    state->sound_timer = 10.0;
  }
  if (state->game_over) {
    state->clock = 0.0;
    state->score = 0.0;
    sdl_on_key(NULL);
    sdl_on_mouse(on_mouse_game_over_menu);
    sdl_move_window(STARTING_POSITION);
    body_t *bike = scene_get_body(state->scene, 0);
    body_set_centroid(bike, STARTING_POSITION);
    body_set_rotation(bike, 0.0);
    body_set_velocity(bike, VEC_ZERO);
    body_set_angular_velocity(bike, 0.0);
    if (state->win) {
      create_win_menu(state);
    } else {
      create_game_over_menu(state);
    }
    // emscripten_free(state);
    // sdl_on_key(NULL);
    // state = emscripten_init();
    // body_t *bike = scene_get_body(state->scene, 0);
    // state->game_over = false;
    // sdl_on_key(NULL);
    // body_set_centroid(bike, STARTING_POSITION);
    // sdl_move_window(STARTING_POSITION);
    // body_set_rotation(bike, 0.0);
    // body_set_angular_velocity(bike, 0.0);
    // body_set_velocity(bike, VEC_ZERO);
    // sdl_on_key(on_key);
    // // initialize_game(state);
  }
  if (state->game_state == TIMER && state->level != 0) {
    if (!state->win) {
      state->win = check_win(state);
    }
    state->clock -= state->dt;
    body_t *bike = scene_get_body(state->scene, 0);
    sdl_move_window(body_get_centroid(bike));
    update_timer(state);
    bool collision_checker = check_track_collision(state);
    if (!state->in_air && !collision_checker) {
      if (body_get_velocity(bike).x > 0) {
        body_increment_angular_velocity(bike, AIR_ANGULAR_VELOCITY);
      } else {
        body_increment_angular_velocity(bike, -AIR_ANGULAR_VELOCITY);
      }
      state->in_air = true;
    } else if (collision_checker) {
      check_loss(state);
      state->in_air = false;
    }
    scene_tick(state->scene, state->dt);
    // sdl_render_scene(state->scene);
  } else if (state->game_state == SCORE && state->level != 0) {
    if (!state->win) {
      state->win = check_win(state);
    }
    body_t *bike = scene_get_body(state->scene, 0);
    sdl_move_window(body_get_centroid(bike));
    update_score(state);
    bool collision_checker = check_track_collision(state);
    if (!state->in_air && !collision_checker) {
      if (body_get_velocity(bike).x > 0) {
        body_increment_angular_velocity(bike, AIR_ANGULAR_VELOCITY);
      } else {
        body_increment_angular_velocity(bike, -AIR_ANGULAR_VELOCITY);
      }
      state->in_air = true;
    } else if (collision_checker) {
      check_loss(state);
      state->in_air = false;
    } else if (state->in_air) {
      state->score += 10;
    }
    scene_tick(state->scene, state->dt);
  }
  sdl_render_scene(state->scene);
}

void emscripten_free(state_t *state) {
  list_free(state->button_list);
  sdl_clear_text();
  scene_free(state->scene);
  free(state);
}