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
const vector_t WINDOW = ((vector_t){.x = 1500, .y = 1000});

// bike constants
const size_t BIKE_NUM_POINTS = 1000;
const double BIKE_MASS = 10000.0;
const rgb_color_t BIKE_COLOR = (rgb_color_t){1, 0, 0};
const vector_t START = (vector_t){-1.45, 1.045}; // 1
// each section's end point becomes the next section's start point.
#define NUM_SECTIONS 38
#define NUM_CURVES 11
const vector_t COORDS[NUM_SECTIONS] = {
    (vector_t){-1.5, 1.052},    // 0
    (vector_t){-1.85, 1.18},    // 1 1
    (vector_t){-1.9, 1.412},    // 1 2
    (vector_t){-1.67, 2.795},   // 5 3
    (vector_t){-1.16, 2.343},   // 1 4
    (vector_t){-0.43, 2.032},   // 1 5
    (vector_t){-0.1, 1.997},    // 2.5 6
    (vector_t){0, 1.94},        // 1 7
    (vector_t){0.14, 1.473},    // 1 8
    (vector_t){-1.788, 0.878},  // 10 9
    (vector_t){-8.5, 1.058},    // 5 11
    (vector_t){-3.8, -1.1},     // 11 12
    (vector_t){-2.1, -1.1},     // 1 13
    (vector_t){-1.15, -1.1},    // 5 14
    (vector_t){0.3, -1.1},      // 1 15
    (vector_t){4.5, -0.096},    // 11 16
    (vector_t){3.5, 0.727},     // 1 17
    (vector_t){0.8, 1.013},     // 5 18
    (vector_t){0.4, 2.117},     // 1 19
    (vector_t){-0.05, 2.3485},  // 1 20
    (vector_t){-0.25, 2.445},   // 2.5 21
    (vector_t){-1, 2.765},      // 1 22
    (vector_t){-1.63, 3.6},     // 1 23
    (vector_t){-1.66, 3.7},     // 1 24
    (vector_t){-1.7, 3.735},    // 1 25
    (vector_t){-1.2, 3.66},     // 1 26
    (vector_t){-1.057, 3.96},   // 1 27 jhj
    (vector_t){-1.2, 3.96},     // 1 28
    (vector_t){-1.5, 4.04},     // 1 29
    (vector_t){-1.6, 4.4},      // 1 30
    (vector_t){-1.44, 4.703},   // 1 31
    (vector_t){-1.262, 4.895},  // 1 32
    (vector_t){-0.7, 4.901},    // 1 33
    (vector_t){-1.212, 5.228},  // 34
    (vector_t){-2.496, 3.854},  // 10 35
    (vector_t){-2.4, 3.833},    // 1 36
    (vector_t){-2.924, 0.9605}, // 10 37
    (vector_t){-1.25, 0.967},   // 1 38
};

const double PROPORTIONS[NUM_CURVES] = {
    5.0 / 100.0,  2.5 / 100.0,  5.0 / 100.0,  7.0 / 100.0,
    10.0 / 100.0, 5.0 / 100.0,  11.0 / 100.0, 5.0 / 100.0,
    2.5 / 100.0,  10.0 / 100.0, 10.0 / 100.0,
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

// track constants
const double TRACK_HEIGHT = 20.0;
const double TRACK_MASS = INFINITY;
const rgb_color_t TRACK_COLOR = {0.545098039216, 0.270588235294,
                                 0.0745098039216};

const double GRAVITATIONAL_ACCELERATION = 100.0;
const double SUSPENSION_CONSTANT = 10000.0;
const double EQ_DIST = 10.0;

typedef struct state {
  scene_t *scene;
  body_t *bike_body;
  body_t *back_wheel;
  body_t *front_wheel;
  vector_t *back_anchor;
  vector_t *front_anchor;
} state_t;

typedef enum {
  BIKE = 1,
  BACK_WHEEL = 2,
  FRONT_WHEEL = 3,
  WHEEL_POWERUP = 4,
  SPEED_POWERUP = 5,
  TRACK = 6
} body_type_t;

// helper functions

// bike functions
body_t *make_wheel(double x, double y, body_type_t type) {
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
  body_type_t *info = malloc(sizeof(*info));
  *info = type;
  body_t *wheel =
      body_init_with_info(shape, WHEEL_MASS, WHEEL_COLOR, info, free);
  body_set_centroid(wheel, (vector_t){x, y}); //
  return wheel;
}

void section_two(list_t *shape, vector_t start, vector_t end,
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

void section_five(list_t *shape, vector_t start, vector_t end,
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

void section_eight(list_t *shape, vector_t start, vector_t end,
                   size_t num_points) {
  double dx = (end.x - start.x) / num_points;
  vector_t *coord_zero = malloc(sizeof(vector_t));
  *coord_zero = start;
  list_add(shape, coord_zero);
  for (size_t t = 1; t < num_points; t++) {
    double x = start.x + dx * t;
    double y = 1 - 0.5 * cos(0.7 * x - 1) / (x - 0.8);
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
    double y = 1 + 0.15 * cos(0.55 * x + 3.5);
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

void section_twelve(list_t *shape, vector_t start, vector_t end,
                    size_t num_points) {
  double dx = (end.x - start.x) / num_points;
  vector_t *coord_zero = malloc(sizeof(vector_t));
  *coord_zero = start;
  list_add(shape, coord_zero);
  for (size_t t = 1; t < num_points; t++) {
    double x = start.x + dx * t;
    double y = -1.4 - 1.0 / (24 * (x + 1.7) * (x + 1.7) - 3 * (x + 1.7) - 9);
    vector_t *coord = malloc(sizeof(vector_t));
    *coord = (vector_t){x, y};
    list_add(shape, coord);
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
    double y = 2.0 - 4.0 * cosh(0.4 * (x - 0.2)) / (x + 1);
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
    double y = 0.7 + 0.6 * sqrt(0.1 * (3.52 - x));
    vector_t *coord = malloc(sizeof(vector_t));
    *coord = (vector_t){x, y};
    list_add(shape, coord);
  }
}

void section_nineteen(list_t *shape, vector_t start, vector_t end,
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

void section_thirty_three(list_t *shape, vector_t start, size_t num_points) {
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

void section_thirty_five(list_t *shape, vector_t start, vector_t end,
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
      section_twelve(shape, COORDS[i], COORDS[i + 1],
                     PROPORTIONS[j] * BIKE_NUM_POINTS);
      j++;
    } else if (i == 14) {
      section_fourteen(shape, COORDS[i], COORDS[i + 1],
                       PROPORTIONS[j] * BIKE_NUM_POINTS);
      j++;
    } else if (i == 16) {
      section_sixteen(shape, COORDS[i], COORDS[i + 1],
                      PROPORTIONS[j] * BIKE_NUM_POINTS);
      j++;
    } else if (i == 19) {
      section_nineteen(shape, COORDS[i], COORDS[i + 1],
                       PROPORTIONS[j] * BIKE_NUM_POINTS);
      j++;
    } else if (i == 33) {
      section_thirty_three(shape, COORDS[i], PROPORTIONS[j] * BIKE_NUM_POINTS);
      j++;
    } else if (i == 35) {
      section_thirty_five(shape, COORDS[i], COORDS[i + 1],
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

body_t *make_bike(state_t *state) {
  list_t *shape = make_bike_shape();
  list_t *scaled_shape = scale_polygon(10, shape);
  list_free(shape);
  size_t left_index = list_size(scaled_shape) * 7 / 20;
  size_t right_index = list_size(scaled_shape) / 2;
  state->back_anchor = list_get(scaled_shape, left_index);
  state->front_anchor = list_get(scaled_shape, right_index);
  body_type_t *type = malloc(sizeof(*type));
  *type = BIKE;
  body_t *bike =
      body_init_with_info(scaled_shape, BIKE_MASS, BIKE_COLOR, type, free);
  double x = (body_get_centroid(state->back_wheel).x +
              body_get_centroid(state->front_wheel).x) /
             2.0;
  body_set_centroid(bike, (vector_t){x, 25});
  return bike;
}

// void set_anchors(state_t *state) {
//   state->back_anchor = body_get_centroid(state->back_wheel);
//   state->front_anchor = body_get_centroid(state->front_wheel);
// }

void initialize_bike(state_t *state) {
  state->back_wheel = make_wheel(x_back, y, BACK_WHEEL);
  state->front_wheel = make_wheel(x_front, y, FRONT_WHEEL);
  // set_anchors(state);
  state->bike_body = make_bike(state);
  scene_add_body(state->scene, state->bike_body);
  scene_add_body(state->scene, state->back_wheel);
  scene_add_body(state->scene, state->front_wheel);
}

void move_bike(state_t *state, vector_t position) {
  vector_t translation =
      vec_subtract(position, body_get_centroid(state->bike_body));
  body_set_centroid(state->bike_body, position);
  body_set_centroid(state->back_wheel,
                    vec_add(body_get_centroid(state->back_wheel), translation));
  body_set_centroid(
      state->front_wheel,
      vec_add(body_get_centroid(state->front_wheel), translation));
  // set_anchors(state);
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
  // body_t *body =
  //     body_init(make_circle(CIRCLE_RADIUS), CIRCLE_MASS, CIRCLE_COLOR);
  // body_set_centroid(body, (vector_t){WINDOW.x / 2.0, WINDOW.y});
  // scene_add_body(scene, body);
  // initialize_wheels();
  body_t *track1 =
      body_init(make_actual_track(), TRACK_MASS, (rgb_color_t){0, 0, 0});
  body_t *track2 = body_init(make_track(), TRACK_MASS, TRACK_COLOR);
  scene_add_body(scene, track1);
  scene_add_body(scene, track2);
}

bool double_is_close(double a, double b, double threshold) {
  return fabs(a - b) < threshold;
}

bool is_zero_vector(vector_t v) {
  return double_is_close(v.x, 0.0, 1e-5) && double_is_close(v.y, 0.0, 1e-5);
}

typedef struct bike_rotate_args {
  state_t *state;
  double angular_velocity;
} bike_rotate_args_t;

void wheel_ground_collision(body_t *wheel, body_t *track, vector_t axis,
                            void *aux) {
  bike_rotate_args_t *args = aux;
  list_t *wheel_shape = body_get_shape(wheel);
  list_t *track_shape = body_get_shape(track);
  collision_info_t collision = find_collision(wheel_shape, track_shape);
  double angle_diff =
      body_get_rotation(args->state->bike_body) - vec_angle(axis);
  if (!double_is_close(fabs(angle_diff), M_PI / 2, 1e-5)) {
    // body_set_rotation(args->state->bike_body, M_PI / 2);
    body_type_t *type = body_get_info(wheel);
    if (*type == BACK_WHEEL) {
      body_set_pivot(args->state->bike_body,
                     body_get_centroid(args->state->back_wheel));
      body_set_pivot(args->state->front_wheel,
                     body_get_centroid(args->state->back_wheel));
      body_increment_angular_velocity(args->state->bike_body,
                                      args->angular_velocity);
      body_increment_angular_velocity(args->state->front_wheel,
                                      args->angular_velocity);
      body_set_angular_velocity(args->state->back_wheel, 0.0);
    } else {
      body_set_pivot(args->state->bike_body,
                     body_get_centroid(args->state->front_wheel));
      body_set_pivot(args->state->back_wheel,
                     body_get_centroid(args->state->front_wheel));
      body_increment_angular_velocity(args->state->bike_body,
                                      args->angular_velocity);
      body_increment_angular_velocity(args->state->back_wheel,
                                      args->angular_velocity);
      body_set_angular_velocity(args->state->front_wheel, 0.0);
    }
  } else {
    body_set_angular_velocity(args->state->bike_body, 0.0);
    body_set_angular_velocity(args->state->back_wheel, 0.0);
    body_set_angular_velocity(args->state->front_wheel, 0.0);
  }
  list_free(wheel_shape);
  list_free(track_shape);
  // else {
  //   body_set_angular_velocity(args->state->bike_body, 0.0);
  // }
  // list_t *back_shape = body_get_shape(back_wheel);
  // list_t *front_shape = body_get_shape(front_wheel);
  // for(size_t i = 0; i < scene_bodies(state->scene); i++) {
  //   body_t *track_piece = scene_get_body(state->scene, i);
  //   body_type_t *type = body_get_info(track_piece);
  //   if (*type == TRACK) {
  //     collision_info_t back_collision = find_collision(back_wheel,
  //     track_piece); collision_info_t front_collision =
  //     find_collision(front_wheel, track_piece); if ()
  //   }
  // }
}

void create_wheel_collision(scene_t *scene, body_t *wheel, body_t *track,
                            state_t *state, double angular_velocity) {
  bike_rotate_args_t *aux = malloc(sizeof(bike_rotate_args_t));
  aux->angular_velocity = angular_velocity;
  aux->state = state;
  create_collision(scene, wheel, track, wheel_ground_collision, aux, free);
}

void initialize_force_list(state_t *state) {
  create_downwards_gravity(state->scene, GRAVITATIONAL_ACCELERATION,
                           state->bike_body);
  create_downwards_gravity(state->scene, GRAVITATIONAL_ACCELERATION,
                           state->front_wheel);
  create_downwards_gravity(state->scene, GRAVITATIONAL_ACCELERATION,
                           state->back_wheel);
  create_suspension(
      state->scene, SUSPENSION_CONSTANT,
      vec_magn(vec_subtract(*state->back_anchor,
                            body_get_centroid(state->back_wheel))),
      state->back_wheel, state->bike_body, state->back_anchor);
  create_suspension(
      state->scene, SUSPENSION_CONSTANT,
      vec_magn(vec_subtract(*state->front_anchor,
                            body_get_centroid(state->front_wheel))),
      state->front_wheel, state->bike_body, state->front_anchor);
  create_physics_collision(state->scene, 0.0, state->bike_body,
                           state->front_wheel);
  create_physics_collision(state->scene, 0.0, state->bike_body,
                           state->back_wheel);
  create_physics_collision(state->scene, 0.0, state->bike_body,
                           scene_get_body(state->scene, 3));
  create_physics_collision(state->scene, 0.0, state->front_wheel,
                           scene_get_body(state->scene, 3));
  create_physics_collision(state->scene, 0.0, state->back_wheel,
                           scene_get_body(state->scene, 3));
  create_wheel_collision(state->scene, state->back_wheel,
                         scene_get_body(state->scene, 3), state, -1);
  create_wheel_collision(state->scene, state->front_wheel,
                         scene_get_body(state->scene, 3), state, 1);
  create_normal(state->scene, state->bike_body,
                scene_get_body(state->scene, 3));
  create_normal(state->scene, state->front_wheel,
                scene_get_body(state->scene, 3));
  create_normal(state->scene, state->back_wheel,
                scene_get_body(state->scene, 3));

  create_physics_collision(state->scene, 0.0, state->bike_body,
                           scene_get_body(state->scene, 4));
  create_physics_collision(state->scene, 0.0, state->front_wheel,
                           scene_get_body(state->scene, 4));
  create_physics_collision(state->scene, 0.0, state->back_wheel,
                           scene_get_body(state->scene, 4));
  create_normal(state->scene, state->bike_body,
                scene_get_body(state->scene, 4));
  create_normal(state->scene, state->front_wheel,
                scene_get_body(state->scene, 4));
  create_normal(state->scene, state->back_wheel,
                scene_get_body(state->scene, 4));
}

// key handler function
void on_key(state_t *state, char key, key_event_type_t type, double held_time) {
  // TODO: key handlers
}

state_t *emscripten_init() {
  srand(time(NULL));
  vector_t min = VEC_ZERO;
  vector_t max = WINDOW;
  sdl_init(min, max);
  sdl_on_key((key_handler_t)on_key);
  state_t *state = malloc(sizeof(state_t));
  state->scene = scene_init();
  initialize_bike(state);
  initialize_body_list(state->scene);
  initialize_force_list(state);
  move_bike(state, vec_multiply(0.5, WINDOW));
  return state;
}

void emscripten_main(state_t *state) {
  double dt = time_since_last_tick();
  sdl_move_window(body_get_centroid(state->bike_body));
  scene_tick(state->scene, dt);
  sdl_render_scene(state->scene);
}

void emscripten_free(state_t *state) {
  scene_free(state->scene);
  free(state);
}