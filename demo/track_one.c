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
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

const double TRACK_MASS = INFINITY;
const rgb_color_t TRACK_COLOR = (rgb_color_t){0.05, 0.4, 0};

const vector_t WINDOW = (vector_t){.x = 1000, .y = 300};

const size_t NUM_BODIES = 60;
#define NUM_COORDS 240
const vector_t COORDS[NUM_COORDS] = {
    (vector_t) {0,0}, // 0 new line
    (vector_t) {0,5}, // 1
    (vector_t) {16, 5}, // 2
    (vector_t) {16, 0}, // 3
    (vector_t) {16, 0}, // new curve
    (vector_t) {16,5},
    (vector_t) {18,5.04}, // 4
    (vector_t) {18,0}, // 5
    (vector_t) {18,0},
    (vector_t) {18, 5.04},
    (vector_t) {20, 5.16}, // 6
    (vector_t) {20, 0}, // 7
    (vector_t) {20,0},
    (vector_t) {20, 5.16},
    (vector_t) {22, 5.36}, // 8
    (vector_t) {22, 0}, // 9
    (vector_t) {22, 0},
    (vector_t) {22, 5.36},
    (vector_t) {23, 5.49}, //10
    (vector_t) {23,0}, // 11
    (vector_t) {23, 0},
    (vector_t) {23, 5.49},
    (vector_t) {24, 5.64}, // 12
    (vector_t) {24, 0}, // 13
    (vector_t) {24, 0},
    (vector_t) {24, 5.64},
    (vector_t) {25, 5.81}, // 14
    (vector_t) {25, 0}, // 15
    (vector_t) {25, 0},
    (vector_t) {25, 5.81},
    (vector_t) {26, 6}, // 16
    (vector_t) {26, 0}, // 17
    (vector_t) {26, 0},
    (vector_t) {26, 6},
    (vector_t) {29, 6.69}, // 18
    (vector_t) {29, 0}, // 19
    (vector_t) {29, 0},
    (vector_t) {29, 6.69},
    (vector_t) {32, 7.56}, // 20
    (vector_t) {32, 0}, // 21
    (vector_t) {32, 0},
    (vector_t) {32, 7.56},
    (vector_t) {36, 9}, // 22
    (vector_t) {36, 0}, // 23
    (vector_t) {36, 0},
    (vector_t) {36, 9},
    (vector_t) {40, 10.76}, // 24
    (vector_t) {40, 0}, // 25
    (vector_t) {40, 0},
    (vector_t) {40, 10.76},
    (vector_t) {44, 12.84}, // 26
    (vector_t) {44, 0}, // 27
    (vector_t) {44, 0},
    (vector_t) {44, 12.84},
    (vector_t) {47, 14.61}, // 28
    (vector_t) {47, 0}, // 29
    (vector_t) {47, 0},
    (vector_t) {47, 14.61},
    (vector_t) {50, 16.56}, // 30
    (vector_t) {50, 0}, // 31
    (vector_t) {50, 0}, // new curve
    (vector_t) {50, 16.56},
    (vector_t) {60, 23.36}, // 32
    (vector_t) {60, 0}, // 33
    (vector_t) {60, 0}, // new curve
    (vector_t) {60, 23.36},
    (vector_t) {65, 25.61}, // 34
    (vector_t) {65, 0}, // 35
    (vector_t) {65, 0}, // new curve
    (vector_t) {65, 25.61},
    (vector_t) {70, 26.86}, // 36
    (vector_t) {70, 0}, // 37
    (vector_t) {70, 0}, // new curve
    (vector_t) {70, 26.86},
    (vector_t) {75, 27.61}, // 38
    (vector_t) {75, 0}, // 39
    (vector_t) {75, 0}, // new line
    (vector_t) {75, 27.61},
    (vector_t) {100, 27.61}, // 40
    (vector_t) {100,0}, // 41
    (vector_t) {100,0}, // new curve
    (vector_t) {100, 27.61},
    (vector_t) {105, 26.86}, // 42
    (vector_t) {105, 0}, // 43 
    (vector_t) {105, 0}, // new curve
    (vector_t) {105, 26.86},
    (vector_t) {110, 25.61}, // 44
    (vector_t) {110, 0}, // 45
    (vector_t) {110, 0}, // new curve
    (vector_t) {110, 25.61},
    (vector_t) {115, 23.36}, // 46
    (vector_t) {115, 0}, // 47
    (vector_t) {115, 0}, // new curve
    (vector_t) {115, 23.36},
    (vector_t) {125, 16.56}, // 48
    (vector_t) {125, 0}, // 49
    (vector_t) {125, 0}, // new curve
    (vector_t) {125, 16.56},
    (vector_t) {128, 14.61}, // 50
    (vector_t) {128, 0}, // 51
    (vector_t) {128, 0},
    (vector_t) {128, 14.61},
    (vector_t) {131, 12.84}, // 52
    (vector_t) {131, 0}, // 53
    (vector_t) {131, 0},
    (vector_t) {131, 12.84},
    (vector_t) {135, 10.76}, // 54
    (vector_t) {135, 0}, // 55
    (vector_t) {135, 0},
    (vector_t) {135, 10.76},
    (vector_t) {139, 9}, // 56
    (vector_t) {139, 0}, // 57
    (vector_t) {139, 0},
    (vector_t) {139, 9},
    (vector_t) {143,7.56}, // 58
    (vector_t) {143,0}, // 59
    (vector_t) {143,0},
    (vector_t) {143,7.56},
    (vector_t) {146, 6.69}, // 60
    (vector_t) {146, 0}, // 61
    (vector_t) {146, 0},
    (vector_t) {146, 6.69},
    (vector_t) {149,6}, // 62
    (vector_t) {149, 0}, // 63
    (vector_t) {149, 0},
    (vector_t) {149, 6},
    (vector_t) {150, 5.81}, // 64
    (vector_t) {150, 0}, // 65
    (vector_t) {150, 0},
    (vector_t) {150,5.81},
    (vector_t) {151, 5.64}, // 66
    (vector_t) {151, 0}, // 67
    (vector_t) {151, 0},
    (vector_t) {151, 5.64},
    (vector_t) {153, 5.36}, // 68
    (vector_t) {153, 0}, // 69
    (vector_t) {153, 0},
    (vector_t) {153, 5.36},
    (vector_t) {155, 5.16}, // 70
    (vector_t) {155, 0}, // 71
    (vector_t) {155, 0},
    (vector_t) {155, 5.16},
    (vector_t) {157, 5.04}, // 72
    (vector_t) {157, 0}, // 73
    (vector_t) {157, 0},
    (vector_t) {157, 5.04},
    (vector_t) {159, 5}, // 74
    (vector_t) {159, 0}, // 75
    (vector_t) {159, 0},// line
    (vector_t) {159, 5},
    (vector_t) {200, 5}, // 76
    (vector_t) {200, 0}, // 77
    (vector_t) {200, 0}, // curve
    (vector_t) {200, 5},
    (vector_t) {202, 5.02}, // 78
    (vector_t) {202, 0}, // 79
    (vector_t) {202, 0},
    (vector_t) {202, 5.02},
    (vector_t) {204, 5.08}, // 80
    (vector_t) {204, 0}, // 81
    (vector_t) {204, 0},
    (vector_t) {204, 5.08},
    (vector_t) {206, 5.18}, // 82
    (vector_t) {206, 0}, // 83
    (vector_t) {206, 0},
    (vector_t) {206, 5.18},
    (vector_t) {209, 5.405}, // 84
    (vector_t) {209, 0}, // 85
    (vector_t) {209, 0},
    (vector_t) {209, 5.405},
    (vector_t) {212, 5.72}, // 86
    (vector_t) {212, 0}, // 87
    (vector_t) {212, 0},
    (vector_t) {212, 5.72},
    (vector_t) {216, 6.28}, // 88
    (vector_t) {216, 0}, // 89
    (vector_t) {216, 0},
    (vector_t) {216, 6.28},
    (vector_t) {220, 7}, // 90
    (vector_t) {220, 0}, // 91
    (vector_t) {220, 0},
    (vector_t) {220, 7},
    (vector_t) {226, 8.38}, // 92
    (vector_t) {226, 0}, // 93
    (vector_t) {226, 0},
    (vector_t) {226, 8.38},
    (vector_t) {230, 9.5}, // 94
    (vector_t) {230, 0}, // 95
    (vector_t) {230, 0},
    (vector_t) {230, 9.5},
    (vector_t) {235, 11.125}, // 96
    (vector_t) {235, 0}, // 97
    (vector_t) {235, 0},
    (vector_t) {235, 11.125},
    (vector_t) {240, 13}, // 98
    (vector_t) {240, 0}, // 99
    (vector_t) {240, 0},
    (vector_t) {240, 13},
    (vector_t) {244, 14.68}, // 100
    (vector_t) {244, 0}, // 101
    (vector_t) {244, 0},
    (vector_t) {244, 14.68},
    (vector_t) {250, 17.5}, // 102
    (vector_t) {250, 0}, // 103
    (vector_t) {250, 0},
    (vector_t) {250, 17.5},
    (vector_t) {255, 20.125}, // 104
    (vector_t) {255, 0}, // 105
    (vector_t) {255, 0},
    (vector_t) {255, 20.125},
    (vector_t) {262, 24.22}, // 106
    (vector_t) {262, 0}, // 107
    (vector_t) {262, 0},
    (vector_t) {262, 24.22},
    (vector_t) {265, 26.125}, // 108
    (vector_t) {265, 0}, // 109
    (vector_t) {265, 0}, // new curve
    (vector_t) {265, 26.125},
    (vector_t) {320, 61.875}, // 110
    (vector_t) {320, 0}, // 111

    (vector_t) {400, 0}, // new curve // 112
    (vector_t) {400, 25}, // 113
    (vector_t) {420, 18}, // 114
    (vector_t) {420, 0}, // 115
    (vector_t) {420, 0}, // new curve
    (vector_t) {420, 18},
    (vector_t) {422, 17.4}, // 116
    (vector_t) {422, 0}, // 117
    (vector_t) {422, 0}, // new curve
    (vector_t) {422, 17.4},
    (vector_t) {440, 12.9}, // 118
    (vector_t) {440, 0}, // 119
    (vector_t) {440, 0}, // new curve
    (vector_t) {440, 12.9},
    (vector_t) {492.7, 5}, // 120
    (vector_t) {492.7, 0}, // 121
    (vector_t) {492.7, 0}, // new line
    (vector_t) {492.7, 5},
    (vector_t) {600, 5}, // 122
    (vector_t) {600, 0}, // 123
};

typedef struct state {
  scene_t *scene;
} state_t;

list_t *make_track_one(){
    list_t *bodies = list_init(NUM_BODIES, free);
    size_t i = 0;
    bool t = 0;
    for (size_t j = 0; j < NUM_BODIES; j++){
        list_t *shape = list_init(4, free);
        for (size_t k = 0; k < 4; k++){            
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
  list_t *bodies = make_track_one();
  for (size_t i = 0; i < list_size(bodies); i++){
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
