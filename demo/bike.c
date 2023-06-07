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

const vector_t WINDOW = (vector_t){.x = 80, .y = 50};
vector_t *front_anchor;
vector_t *back_anchor;
const size_t BIKE_NUM_POINTS = 1000;
const double BIKE_MASS = 100.0;
const rgb_color_t BIKE_COLOR = (rgb_color_t){1, 0, 0};
const vector_t START = (vector_t){-1.45, 1.045}; // 1
// each section's end point becomes the next section's start point.
#define NUM_SECTIONS 39
#define NUM_CURVES 11
const vector_t COORDS[NUM_SECTIONS] = {
    (vector_t){-1.45, 1.045},  // 0
    (vector_t){-1.85, 1.18},   // 1 1
    (vector_t){-1.9, 1.412},   // 1 2
    (vector_t){-1.67, 2.795},  // 5 3
    (vector_t){-1.16, 2.343},  // 1 4
    (vector_t){-0.43, 2.032},  // 1 5
    (vector_t){-0.1, 1.997},   // 2.5 6
    (vector_t){0, 1.94},       // 1 7
    (vector_t){0.27, 1.033},   // 1 8
    (vector_t){-2.798, 1.033}, // 1 9
    (vector_t){-6.12, 1.132},  // 5 10
    (vector_t){-9, 0.981},     // 5 11
    (vector_t){-3.85, -1.0},   // 11 12
    (vector_t){-2.1, -1.0},    // 1 13
    (vector_t){-1.15, -1.0},   // 5 14
    (vector_t){0.35, -0.968},  // 1 15
    (vector_t){4.5, -0.096},   // 11 16
    (vector_t){3.5, 0.727},    // 1 17
    (vector_t){0.8, 1.013},    // 5 18
    (vector_t){0.4, 2.117},    // 1 19
    (vector_t){-0.05, 2.3485}, // 1 20
    (vector_t){-0.25, 2.445},  // 2.5 21
    (vector_t){-1, 2.765},     // 1 22
    (vector_t){-1.63, 3.6},    // 1 23
    (vector_t){-1.66, 3.7},    // 1 24
    (vector_t){-1.7, 3.735},   // 1 25
    (vector_t){-1.2, 3.66},    // 1 26
    (vector_t){-1.057, 3.96},  // 1 27
    (vector_t){-1.2, 3.96},    // 1 28
    (vector_t){-1.5, 4.04},    // 1 29
    (vector_t){-1.6, 4.2},     // 1 30
    (vector_t){-1.44, 4.703},  // 1 31
    (vector_t){-1.262, 4.895}, // 1 32
    (vector_t){-0.7, 4.901},   // 1 33
    (vector_t){-1.212, 5.228}, // 34
    (vector_t){-2.496, 3.854}, // 10 35
    (vector_t){-2.4, 3.833},   // 1 36
    (vector_t){-2.922, 1.05},  // 10 37
    (vector_t){-1.415, 1.04},  // 1 38
};

const double PROPORTIONS[NUM_CURVES] = {
    5.0 / 100.0,  2.5 / 100.0,  5.0 / 100.0,  5.0 / 100.0,
    11.0 / 100.0, 5.0 / 100.0,  11.0 / 100.0, 5.0 / 100.0,
    2.5 / 100.0,  10.0 / 100.0, 10.0 / 100.0,
};

const size_t WHEEL_NUM_POINTS = 1000;
const double WHEEL_OUTER_RAD = 2;
const double WHEEL_INNER_RAD = 1;
const double WHEEL_MASS = 100;
const rgb_color_t WHEEL_COLOR = (rgb_color_t){0, 1, 0};

typedef struct state {
  scene_t *scene;
} state_t;

body_t *make_wheel(double x, double y) {
  list_t *shape = list_init(1, free);
  for (int t = 0; t < WHEEL_NUM_POINTS; t++) {
    vector_t *coord = malloc(sizeof(vector_t));
    if (t < WHEEL_NUM_POINTS / 2) {
      *coord =
          (vector_t){WHEEL_INNER_RAD * cos((8.0 * t - WHEEL_NUM_POINTS) /
                                           (2.0 * WHEEL_NUM_POINTS) * M_PI),
                     WHEEL_INNER_RAD * sin((8.0 * t - WHEEL_NUM_POINTS) /
                                           (2.0 * WHEEL_NUM_POINTS) * M_PI)};
    } else {
      *coord = (vector_t){
          WHEEL_OUTER_RAD * cos((200.0 * t - 127 * WHEEL_NUM_POINTS) /
                                (50.0 * WHEEL_NUM_POINTS) * M_PI),
          WHEEL_OUTER_RAD * sin((200.0 * t - 127 * WHEEL_NUM_POINTS) /
                                (50.0 * WHEEL_NUM_POINTS) * M_PI)};
    }
    list_add(shape, coord);
  }
  body_t *wheel = body_init(shape, WHEEL_MASS, WHEEL_COLOR);
  body_set_centroid(wheel, (vector_t){x, y});
  return wheel;
}

void section_three(list_t *shape, vector_t start, vector_t end,
                   size_t num_points) {
  double dx = (end.x - start.x) / num_points;
  vector_t *coord_zero = malloc(sizeof(vector_t));
  *coord_zero = start;
  list_add(shape, coord_zero);
  for (size_t t = 1; t < num_points; t++) {
    double x = start.x + dx * t;
    double y = 2 + 0.5 * sinh(10 * (x + 1.8));
    vector_t *coord = malloc(sizeof(vector_t));
    *coord = (vector_t){x, y};
    list_add(shape, coord);
  }
}

void section_six(list_t *shape, vector_t start, vector_t end,
                 size_t num_points) {
  double dx = (end.x - start.x) / num_points;
  vector_t *coord_zero = malloc(sizeof(vector_t));
  *coord_zero = start;
  list_add(shape, coord_zero);
  for (size_t t = 1; t < num_points; t++) {
    double x = start.x + dx * t;
    double y = 2.2 - sqrt(0.06 - (x + 0.25) * (x + 0.25));
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
    double y = 1.1 - 0.2 * cos(x) / x;
    vector_t *coord = malloc(sizeof(vector_t));
    *coord = (vector_t){x, y};
    list_add(shape, coord);
  }
}

void section_eleven(list_t *shape, vector_t start, vector_t end,
                    size_t num_points) {
  double dx = (end.x - start.x) / num_points;
  vector_t *coord_zero = malloc(sizeof(vector_t));
  *coord_zero = start;
  list_add(shape, coord_zero);
  for (size_t t = 1; t < num_points; t++) {
    double x = start.x + dx * t;
    double y = 1.1791 - exp(-0.5 * x - 6.121);
    vector_t *coord = malloc(sizeof(vector_t));
    *coord = (vector_t){x, y};
    list_add(shape, coord);
  }
}

void section_twelve(list_t *shape, vector_t start, vector_t end,
                    size_t num_points) {
  double dx = (end.x - start.x) / num_points;
  vector_t *coord_zero = malloc(sizeof(vector_t));
  *coord_zero = start;
  list_add(shape, coord_zero);
  for (size_t t = 1; t < num_points; t++) {
    double x = start.x + dx * t;
    double y = 1.0 - exp(0.6 * (x + 5.0));
    vector_t *coord = malloc(sizeof(vector_t));
    *coord = (vector_t){x, y};
    list_add(shape, coord);
    if (t == (size_t)(0.583 * num_points)) {
      back_anchor = coord;
    }
  }
}

void section_fourteen(list_t *shape, vector_t start, vector_t end,
                      size_t num_points) {
  double dx = (end.x - start.x) / num_points;
  vector_t *coord_zero = malloc(sizeof(vector_t));
  *coord_zero = start;
  list_add(shape, coord_zero);
  for (size_t t = 1; t < num_points; t++) {
    double x = start.x + dx * t;
    double y = -1.3 - 1.0 / (24 * (x + 1.7) * (x + 1.7) - 3 * (x + 1.7) - 9);
    vector_t *coord = malloc(sizeof(vector_t));
    *coord = (vector_t){x, y};
    list_add(shape, coord);
  }
}

void section_sixteen(list_t *shape, vector_t start, vector_t end,
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
    if (t == (size_t)(0.518 * num_points)) {
      front_anchor = coord;
    }
  }
}

void section_eighteen(list_t *shape, vector_t start, vector_t end,
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

void section_twenty_one(list_t *shape, vector_t start, vector_t end,
                        size_t num_points) {
  double dx = (end.x - start.x) / num_points;
  vector_t *coord_zero = malloc(sizeof(vector_t));
  *coord_zero = start;
  list_add(shape, coord_zero);
  for (size_t t = 1; t < num_points; t++) {
    double x = start.x + dx * t;
    double y = 2.2 + sqrt(0.06 - (x + 0.25) * (x + 0.25));
    vector_t *coord = malloc(sizeof(vector_t));
    *coord = (vector_t){x, y};
    list_add(shape, coord);
  }
}

void section_thirty_five(list_t *shape, vector_t start, size_t num_points) {
  vector_t *coord_zero = malloc(sizeof(vector_t));
  *coord_zero = start;
  list_add(shape, coord_zero);
  double dt = ((1.254 - 0.3) * M_PI) / num_points;
  for (size_t i = 1; i < num_points; i++) {
    double t = 0.3 * M_PI + dt * i;
    double x = cos(t) - 1.8;
    double y = 0.9 * sin(t) + 4.5;
    vector_t *coord = malloc(sizeof(vector_t));
    *coord = (vector_t){x, y};
    list_add(shape, coord);
  }
}

void section_thirty_seven(list_t *shape, vector_t start, vector_t end,
                          size_t num_points) {
  double dx = (end.x - start.x) / num_points;
  vector_t *coord_zero = malloc(sizeof(vector_t));
  *coord_zero = start;

  list_add(shape, coord_zero);
  for (size_t t = 1; t < num_points; t++) {
    double x = start.x + dx * t;
    double y = 4.0 + (x + 2.0) / (4.0 * (x + 3));
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
      section_three(shape, COORDS[i], COORDS[i + 1],
                    PROPORTIONS[j] * BIKE_NUM_POINTS);
      j++;
    }
    if (i == 5) {
      section_six(shape, COORDS[i], COORDS[i + 1],
                  PROPORTIONS[j] * BIKE_NUM_POINTS);
      j++;
    } else if (i == 9) {
      section_ten(shape, COORDS[i], COORDS[i + 1],
                  PROPORTIONS[j] * BIKE_NUM_POINTS);
      j++;
    } else if (i == 10) {
      section_eleven(shape, COORDS[i], COORDS[i + 1],
                     PROPORTIONS[j] * BIKE_NUM_POINTS);
      j++;
    } else if (i == 11) {
      section_twelve(shape, COORDS[i], COORDS[i + 1],
                     PROPORTIONS[j] * BIKE_NUM_POINTS);
      j++;
    } else if (i == 13) {
      section_fourteen(shape, COORDS[i], COORDS[i + 1],
                       PROPORTIONS[j] * BIKE_NUM_POINTS);
      j++;
    } else if (i == 15) {
      section_sixteen(shape, COORDS[i], COORDS[i + 1],
                      PROPORTIONS[j] * BIKE_NUM_POINTS);
      j++;
    } else if (i == 17) {
      section_eighteen(shape, COORDS[i], COORDS[i + 1],
                       PROPORTIONS[j] * BIKE_NUM_POINTS);
      j++;
    } else if (i == 20) {
      section_twenty_one(shape, COORDS[i], COORDS[i + 1],
                         PROPORTIONS[j] * BIKE_NUM_POINTS);
      j++;
    } else if (i == 34) {
      section_thirty_five(shape, COORDS[i], PROPORTIONS[j] * BIKE_NUM_POINTS);
      j++;
    } else if (i == 36) {
      section_thirty_seven(shape, COORDS[i], COORDS[i + 1],
                           PROPORTIONS[j] * BIKE_NUM_POINTS);
    } else {
      vector_t *coord = malloc(sizeof(vector_t));
      *coord = COORDS[i];
      list_add(shape, coord);
    }
  }
  return shape;
}

body_t *make_bike() {
  list_t *shape = make_bike_shape();
  body_t *bike = body_init(shape, BIKE_MASS, BIKE_COLOR);
  body_set_centroid(bike, (vector_t){10, 25});
  return bike;
}

void initialize_body_list(scene_t *scene) {
  scene_add_body(scene, make_bike());
  scene_add_body(scene, make_wheel((*back_anchor).x, 22.0));
  scene_add_body(scene, make_wheel((*front_anchor).x, 22.0));
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