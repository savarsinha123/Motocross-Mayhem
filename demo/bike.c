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

const vector_t WINDOW = (vector_t){.x = 60, .y = 30};
const size_t BIKE_NUM_POINTS = 10;
const double BIKE_MASS = 100.0;
const rgb_color_t BIKE_COLOR = (rgb_color_t){1, 0, 0};
const vector_t START = (vector_t){-1.45, 1.045}; // 1
// each section's end point becomes the next section's start point.
#define NUM_SECTIONS 54
#define NUM_CURVES 19
const vector_t COORDS[NUM_SECTIONS] = {
    (vector_t){-0.6, 1.0249},    // / 0
    (vector_t){-1.05, 1.191},    // / 1
    (vector_t){-1.1, 1.412},    // 2 curve
    (vector_t){-0.8752, 2.7991},   // / 3
    (vector_t){-0.35, 2.336},   //  / 4 
    (vector_t){0.417, 1.9778},   //  5 curve
    (vector_t){0.6441, 1.9888},    // / 6
    (vector_t){0.8, 1.8933},        // / 7
    (vector_t){0.94, 1.5667},    // 8 curve
    (vector_t){-1.788, 0.8834},  // 9 curve
    (vector_t){-8.6, 1.0551},    // 10 curve
    (vector_t){-4.306, -0.4199},     // / 11
    (vector_t){-6.58, -1.2912},     // 12 curve
    (vector_t){-6.42, -1.7088},    // / 13 
    (vector_t){-5.611, -1.3999},      // 14 curve
    (vector_t){-5.772, -0.9804},    // / 15 
    (vector_t){-5.345, -0.8174},     // 16 curve
    (vector_t){-5.185, -1.236},     // / 17 
    (vector_t){-3.99, -0.7786},     // 18 curve
    (vector_t){-3.752, -1.1},  // / 19
    (vector_t){-1.6274, -1.1},   // 20 curve
    (vector_t){-0.6476, -1.1},      // / 21
    (vector_t){0.2912, -1.1},     // 22 curve
    (vector_t){1.53, 0.1899},     // / 23
    (vector_t){2.2932, -1.5851},    // 24 curve 
    (vector_t){2.7068, -1.4149},     // / 25
    (vector_t){2.364, -0.616},   // 26 curve
    (vector_t){1.9514, 0.7936},     // / 27
    (vector_t){1.771, -0.3737},     // 28 curve
    (vector_t){2.184, -0.1961},      // / 29
    (vector_t){1.9708, 0.3014},   // 30 curve
    (vector_t){4.5, -0.09585},  // / 31 
    (vector_t){3.5, 0.7268},    // 32 curve
    (vector_t){1.69, 0.9567},  // / 33
    (vector_t){1.22, 2.0501},  // / 34
    (vector_t){0.741, 2.3056},    // 35 curve
    (vector_t){0.5566, 2.4422}, // / 36
    (vector_t){-0.2, 2.78},   // / 37
    (vector_t){-0.83, 3.595},     // / 38
    (vector_t){-0.86, 3.7},     // / 39
    (vector_t){-0.9148, 3.748},    // / 40
    (vector_t){-0.3925, 3.675},     // / 41
    (vector_t){-0.25, 3.96},   // / 42
    (vector_t){-0.37, 3.96},     // / 43
    (vector_t){-0.7, 4.05},     // / 44
    (vector_t){-0.8, 4.19},      // / 45
    (vector_t){-0.71, 4.703},   // / 46
    (vector_t){-0.5529, 4.904},  // / 47
    (vector_t){0.0861, 4.91},    // / 48
    (vector_t){-0.4122, 5.228},  // 49 curve
    (vector_t){-1.701, 3.858},  // / 50
    (vector_t){-1.587, 3.842},    // 51 curve
    (vector_t){-2.125, 0.9167}, // / 52
    (vector_t){-0.5, 0.988},   // / 53
};

const double PROPORTIONS[NUM_CURVES] = {
    3.0 / 100.0,  1.0 / 100.0,  4.5 / 100.0,  6.0 / 100.0,
    4.5 / 100.0, 1.5 / 100.0,  4.5 / 100.0, 6.0 / 100.0,
    1.5 / 100.0, 3.0 / 100.0,  6.5 / 100.0, 2.5 / 100.0,
    2.0 / 100.0, 1.0 / 100.0,  4.0 / 100.0, 6.0 / 100.0,
    2.0 / 100.0,  1.0 / 100.0, 4.5 / 100.0, 
};

typedef struct state {
  scene_t *scene;
} state_t;

typedef enum {
  BIKE = 1,
  WHEEL_POWERUP = 2,
  SPEED_POWERUP = 3
} body_type_t;

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

void section_twenty(list_t *shape, vector_t start, vector_t end, size_t num_points) {
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

void section_twenty_two(list_t *shape, vector_t start, vector_t end, size_t num_points) {
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
      section_sixteen(shape, COORDS[i],
                      PROPORTIONS[j] * BIKE_NUM_POINTS);
      j++;
    } else if (i == 18) {
      section_ten(shape, COORDS[i], COORDS[i + 1],
                       PROPORTIONS[j] * BIKE_NUM_POINTS);
      j++;
    } else if (i == 20) {
      section_twenty(shape, COORDS[i], COORDS[i + 1], PROPORTIONS[j] * BIKE_NUM_POINTS);
      j++;
    } else if (i == 22) {
      section_twenty_two(shape, COORDS[i], COORDS[i + 1],
                          PROPORTIONS[j] * BIKE_NUM_POINTS);
    } else if (i == 24) {
      section_twenty_four(shape, COORDS[i],
                          PROPORTIONS[j] * BIKE_NUM_POINTS);
    } else if (i == 26) {
      section_twenty_six(shape, COORDS[i], PROPORTIONS[j] * BIKE_NUM_POINTS);
    } else if (i == 28) {
      section_twenty_eight(shape, COORDS[i],
                          PROPORTIONS[j] * BIKE_NUM_POINTS);
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

body_t *make_bike() {
  list_t *shape = make_bike_shape();
  body_type_t *type = malloc(sizeof(*type));
  *type = BIKE;
  body_t *bike = body_init_with_info(shape, BIKE_MASS, BIKE_COLOR, type, free);
  body_set_centroid(bike, (vector_t){40, 25}); //
  return bike;
}

void initialize_body_list(scene_t *scene) {
  scene_add_body(scene, make_bike());
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
