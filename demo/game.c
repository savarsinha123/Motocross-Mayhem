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

// button constants
const double BUTTON_MASS = 1.0;
#define BUTTON_DIM                                                             \
  (vector_t) { 0.2 * WINDOW.x, 0.15 * WINDOW.y }
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
const double SUSPENSION_CONSTANT = 10000.0;
const double EQ_DIST = 10.0;

const double ANGULAR_VELOCITY = 1.0;
const double BIKE_ACCELERATION = 300.0;

typedef struct state {
  scene_t *scene;
  double clock;
  text_input_t timer_text;
  bool pushed_down;
  game_state_t game_state;
} state_t;

typedef enum {
  BIKE = 1,
  WHEEL_POWERUP = 2,
  SPEED_POWERUP = 3,
  TRACK = 4
} body_type_t;

typedef enum { MENU = 1, TIMER = 2, SCORE = 3 } game_state_t;

// helper functions

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

body_t *make_bike() {
  list_t *shape = make_bike_shape();
  list_t *scaled_shape = scale_polygon(10, shape);
  list_free(shape);
  body_type_t *type = malloc(sizeof(*type));
  *type = BIKE;
  body_t *bike =
      body_init_with_info(scaled_shape, BIKE_MASS, BIKE_COLOR, type, free);
  body_set_centroid(bike, CENTER); //
  body_set_normal_moment_of_inertia(bike, BIKE_MOMENT);
  return bike;
}

// basic track building
list_t *make_track() {
  list_t *track = list_init(4, free);
  vector_t *v1 = malloc(sizeof(vector_t));
  *v1 = VEC_ZERO;
  vector_t *v2 = malloc(sizeof(vector_t));
  *v2 = (vector_t){0, TRACK_HEIGHT};
  vector_t *v3 = malloc(sizeof(vector_t));
  *v3 = (vector_t){WINDOW.x, (WINDOW.y - TRACK_HEIGHT) / 2.0};
  vector_t *v4 = malloc(sizeof(vector_t));
  *v4 = (vector_t){WINDOW.x, 0};
  list_add(track, v1);
  list_add(track, v2);
  list_add(track, v3);
  list_add(track, v4);
  return track;
}

list_t *make_actual_track() {
  list_t *track = list_init(4, free);
  vector_t *v1 = malloc(sizeof(vector_t));
  *v1 = (vector_t){0, 10};
  vector_t *v2 = malloc(sizeof(vector_t));
  *v2 = (vector_t){0, TRACK_HEIGHT + 10};
  vector_t *v3 = malloc(sizeof(vector_t));
  *v3 = (vector_t){WINDOW.x, (WINDOW.y - TRACK_HEIGHT) / 2.0 + 10};
  vector_t *v4 = malloc(sizeof(vector_t));
  *v4 = (vector_t){WINDOW.x, 10};
  list_add(track, v1);
  list_add(track, v2);
  list_add(track, v3);
  list_add(track, v4);
  return track;
}

void initialize_body_list(scene_t *scene) {
  scene_add_body(scene, make_bike());
  body_t *track1 =
      body_init(make_actual_track(), TRACK_MASS, (rgb_color_t){0, 0, 0});
  body_t *track2 = body_init(make_track(), TRACK_MASS, TRACK_COLOR);
  scene_add_body(scene, track1);
  scene_add_body(scene, track2);
}

bool double_is_close(double a, double b, double threshold) {
  return fabs(a - b) < threshold;
}

list_t *create_collision_triangle() {
  list_t *triangle = list_init(3, free);
  vector_t *v1 = malloc(sizeof(vector_t));
  *v1 = (vector_t){0.00005, -sqrt(3) / 2 * 0.0001};
  vector_t *v2 = malloc(sizeof(vector_t));
  *v2 = (vector_t){-0.00005, -sqrt(3) / 2 * 0.0001};
  vector_t *v3 = malloc(sizeof(vector_t));
  *v3 = (vector_t){0, 0.0001};
  list_add(triangle, v1);
  list_add(triangle, v2);
  list_add(triangle, v3);
  return triangle;
}

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
  // state_t *state = aux;
  list_t *wheel_shape = body_get_shape(body);
  list_t *track_shape = body_get_shape(ground);
  // collision_info_t collision = find_collision(wheel_shape, track_shape);
  double angle_diff = body_get_rotation(body) - vec_angle(axis);
  // char str[10];
  // sprintf(str, "%.9f", angle_diff);
  // puts(str);
  // if (!double_is_close(fabs(angle_diff), M_PI / 2, 1e-5) &&
  // !double_is_close(fabs(angle_diff), 3 * M_PI / 2, 1e-5)) {
  vector_t intersect = find_colliding_point(body, ground);
  if (!double_is_close(intersect.x, -WINDOW.x, 1e-5)) {
    body_set_pivot(body, intersect);
    if (angle_diff < M_PI / 2) {
      body_set_angular_velocity(body, ANGULAR_VELOCITY);
    } else {
      body_set_angular_velocity(body, -ANGULAR_VELOCITY);
    }
    // }
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
  create_downwards_gravity(state->scene, GRAVITATIONAL_ACCELERATION,
                           scene_get_body(state->scene, 0));
  create_ground_collision(state, scene_get_body(state->scene, 0),
                          scene_get_body(state->scene, 1));
  create_physics_collision(state->scene, 0.0, scene_get_body(state->scene, 0),
                           scene_get_body(state->scene, 1));
  create_normal(state->scene, scene_get_body(state->scene, 0),
                scene_get_body(state->scene, 1));

  create_ground_collision(state, scene_get_body(state->scene, 0),
                          scene_get_body(state->scene, 2));
  create_physics_collision(state->scene, 0.0, scene_get_body(state->scene, 0),
                           scene_get_body(state->scene, 2));
  create_normal(state->scene, scene_get_body(state->scene, 0),
                scene_get_body(state->scene, 2));
}

// key handler function
// void on_key(state_t *state, char key, key_event_type_t type, double
// held_time) {
//   // TODO: key handlers
// }

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
      }
      break;
    case RIGHT_ARROW:
      if (!state->pushed_down) {
        state->pushed_down = true;
        create_applied(
            state->scene,
            (vector_t){BIKE_MASS * BIKE_ACCELERATION * cos(angle), sin(angle)},
            bike);
      }
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
      break;
    case RIGHT_ARROW:
      state->pushed_down = false;
      create_applied(
          state->scene,
          (vector_t){-BIKE_MASS * BIKE_ACCELERATION * cos(angle), -sin(angle)},
          bike);
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

void make_button(scene_t *scene, char *string, size_t font_size,
                 vector_t position, vector_t dim, rgb_color_t text_color,
                 rgb_color_t button_color) {
  text_input_t text_input = {.string = string,
                             .font_size = font_size,
                             .position = position,
                             .dim = dim,
                             .color = text_color};
  sdl_write_text(text_input);
  body_t *button = make_rectangle(1.5 * dim.x, 1.5 * dim.y, button_color);
  vector_t centroid =
      (vector_t){position.x + dim.x / 2.0, position.y - dim.y / 2.0};
  body_set_centroid(button, centroid);
  scene_add_body(scene, button);
}

void create_start_menu(state_t *state) {
  make_button(state->scene, "PLAY", FONT_SIZE, PLAY_POSITION, BUTTON_DIM,
              TEXT_COLOR, BUTTON_COLOR);
  make_button(state->scene, "CUSTOMIZE", FONT_SIZE, CUSTOMIZE_POSITION,
              BUTTON_DIM, TEXT_COLOR, BUTTON_COLOR);
  make_button(state->scene, "SETTINGS", FONT_SIZE, SETTINGS_POSITION,
              BUTTON_DIM, TEXT_COLOR, BUTTON_COLOR);
}

state_t *emscripten_init() {
  srand(time(NULL));
  vector_t min = VEC_ZERO;
  vector_t max = WINDOW;
  sdl_init(min, max);
  // sdl_on_key((key_handler_t)on_key);
  state_t *state = malloc(sizeof(state_t));
  state->scene = scene_init();
  state->clock = START_TIME;
  // initialize_body_list(state->scene);
  // initialize_force_list(state);
  // state->pushed_down = false;
  // state->timer_text = (text_input_t) {
  //   .string = "120",
  //   .font_size = FONT_SIZE,
  //   .position = TIMER_POSITION,
  //   .dim = TIMER_DIMENSIONS,
  //   .color = TEXT_COLOR
  // };
  // sdl_write_text(state->timer_text);
  sdl_add_image("assets/windows-xp-wallpaper-bliss-1024x576.jpg",
                (vector_t){0, WINDOW.y});
  create_start_menu(state);
  return state;
}

void update_timer(state_t *state) {
  const size_t TIME_LENGTH = 4;
  // const double MIN_TO_SEC = 60;
  // size_t minutes = state->clock / MIN_TO_SEC;
  // size_t seconds = fmod(state->clock, MIN_TO_SEC);
  char time[TIME_LENGTH];
  // char sec[TIME_LENGTH];
  // sprintf(time, "0%lu", minutes);
  // strcat(time, ":");
  // sprintf(sec, "%lu", seconds);
  sprintf(time, "%d", (int)state->clock);
  // if (seconds / 10 < 1) {
  //   sprintf(sec, "0%lu", seconds);
  // }
  // strcat(time, sec);

  sdl_remove_text(state->timer_text);
  state->timer_text.string = time;
  body_t *bike = scene_get_body(state->scene, 0);
  state->timer_text.position = vec_add(body_get_centroid(bike), CENTER);
  state->timer_text.position.x -= state->timer_text.dim.x;
  sdl_write_text(state->timer_text);
}

void emscripten_main(state_t *state) {
  double dt = time_since_last_tick();
  state->clock -= dt;
  // body_t *bike = scene_get_body(state->scene, 0);
  // sdl_move_window(body_get_centroid(bike));
  // update_timer(state);
  scene_tick(state->scene, dt);
  sdl_render_scene(state->scene);
}

void emscripten_free(state_t *state) {
  scene_free(state->scene);
  free(state);
}