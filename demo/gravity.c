#include "list.h"
#include "polygon.h"
#include "sdl_wrapper.h"
#include "state.h"
#include "vector.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

// window constants
const vector_t WINDOW = (vector_t){.x = 1000, .y = 500};
const vector_t STARTING_POSITION = (vector_t){.x = 0, .y = 500};

// circle constants
const size_t STAR_RAD_1 = 50;
const size_t STAR_RAD_2 = 25;
const double TWO_PI = 2.0 * M_PI;

// kinematic constants
const vector_t INITIAL_VELOCITY = (vector_t){100, 0};
const double ANGULAR_VELOCITY = 0.5 * M_PI;
const vector_t GRAVITY = (vector_t){0, -500};

// number of stars
const size_t NUMBER_OF_STARS = 8;

typedef struct shape {
  list_t *polygon;
  vector_t velocity;
  rgb_color_t color;
} shape_t;

typedef struct state {
  list_t *shapes;
  vector_t acceleration;
  double add_time;
  double time_since_added;
} state_t;

size_t rand_size(size_t lower_bound, size_t upper_bound) {
  size_t num_generated = rand();
  size_t return_num = num_generated % (upper_bound - lower_bound) + lower_bound;
  return return_num;
}

double rand_double(double lower_bound, double upper_bound) {
  double num_generated = rand();
  double return_num =
      num_generated / RAND_MAX * (upper_bound - lower_bound) + lower_bound;
  return return_num;
}

list_t *make_star(size_t inner_radius, size_t outer_radius, size_t num_points) {
  list_t *shape = list_init(2 * num_points, free);
  for (size_t i = 0; i < num_points; i++) {
    // creates outer vertices of star
    vector_t *outer_vector = malloc(sizeof(vector_t));
    *outer_vector = (vector_t){0, outer_radius};
    double outer_angle = i * (TWO_PI / num_points);
    *outer_vector = vec_rotate(*outer_vector, outer_angle);
    *outer_vector = vec_add(*outer_vector, STARTING_POSITION);
    list_add(shape, outer_vector);

    // creates inner vertices of star
    vector_t *inner_vector = malloc(sizeof(vector_t));
    *inner_vector = (vector_t){0, inner_radius};
    double inner_angle = (2 * i + 1) * (M_PI / num_points);
    *inner_vector = vec_rotate(*inner_vector, inner_angle);
    *inner_vector = vec_add(*inner_vector, STARTING_POSITION);
    list_add(shape, inner_vector);
  }
  return shape;
}

rgb_color_t get_random_rgb() {
  rgb_color_t color = {.r = rand_double(0.0, 1.0),
                       .g = rand_double(0.0, 1.0),
                       .b = rand_double(0.0, 1.0)};
  return color;
}

void add_new_shape(list_t *shape_list) {
  shape_t *new_shape = malloc(sizeof(shape_t));
  size_t points = rand_size(4, 12);
  new_shape->polygon = make_star(STAR_RAD_1, STAR_RAD_2, points);
  new_shape->velocity = INITIAL_VELOCITY;
  new_shape->color = get_random_rgb();
  list_add(shape_list, new_shape);
}

state_t *emscripten_init() {
  srand(time(NULL));
  vector_t min = VEC_ZERO;
  vector_t max = WINDOW;
  sdl_init(min, max);

  state_t *state = malloc(sizeof(state_t));
  state->add_time = WINDOW.x / (INITIAL_VELOCITY.x * NUMBER_OF_STARS);
  state->time_since_added = 0.0;
  state->shapes = list_init(NUMBER_OF_STARS + 1, (free_func_t)list_free);
  add_new_shape(state->shapes);
  state->acceleration = GRAVITY;
  return state;
}

void bounce(shape_t *current_shape) {
  for (size_t i = 0; i < list_size(current_shape->polygon); i++) {
    vector_t *vec_ptr = list_get(current_shape->polygon, i);
    // bouncing
    if (vec_ptr->y < 0 && current_shape->velocity.y < 0) {
      double elasticity = rand_double(0.85, 1.0);
      current_shape->velocity.y *= -1 * elasticity;
      break;
    }
  }
}

void emscripten_main(state_t *state) {
  sdl_clear();
  double dt = time_since_last_tick();
  state->time_since_added += dt;

  // spawn new stars
  if (state->time_since_added >= state->add_time) {
    state->time_since_added -= state->add_time;
    add_new_shape(state->shapes);
  }

  // remove off-screen stars
  shape_t *first_shape = list_get(state->shapes, 0);
  double left_position = polygon_centroid(first_shape->polygon).x - STAR_RAD_1;
  if (left_position >= WINDOW.x) {
    list_remove(state->shapes, 0);
    free(first_shape);
  }

  // update shapes
  for (size_t i = 0; i < list_size(state->shapes); i++) {
    shape_t *current_shape = list_get(state->shapes, i);
    bounce(current_shape);
    double angle = ANGULAR_VELOCITY * dt;
    current_shape->velocity =
        vec_add(vec_multiply(dt, state->acceleration), current_shape->velocity);
    polygon_rotate(current_shape->polygon, angle,
                   polygon_centroid(current_shape->polygon));
    polygon_translate(current_shape->polygon,
                      vec_multiply(dt, current_shape->velocity));
    sdl_draw_polygon(current_shape->polygon, current_shape->color);
    sdl_show();
  }
}

void emscripten_free(state_t *state) {
  list_free(state->shapes);
  free(state);
}
