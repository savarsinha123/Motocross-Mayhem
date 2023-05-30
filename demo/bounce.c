#include "list.h"
#include "polygon.h"
#include "sdl_wrapper.h"
#include "state.h"
#include "vector.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

// window constants
const vector_t WINDOW = (vector_t){.x = 1000, .y = 500};
const vector_t CENTER = (vector_t){.x = 500, .y = 250};

// circle constants
const size_t POINTS = 5;
const size_t STAR_RAD_1 = 50;
const size_t STAR_RAD_2 = 25;

// color constantmak
const rgb_color_t RGB = {.r = 0.5, .g = 0.0, .b = 0.5};

const double TWO_PI = 2.0 * M_PI;
const vector_t INITIAL_VELOCITY = (vector_t){200, 200};
const double ANGULAR_VELOCITY = 0.5 * M_PI;

typedef struct state {
  list_t *shape;
  vector_t velocity;
} state_t;

list_t *make_star(size_t inner_radius, size_t outer_radius, size_t num_points) {
  list_t *shape = list_init(2 * num_points, free);
  for (size_t i = 0; i < num_points; i++) {
    // creates outer vertices of star
    vector_t *outer_vector = malloc(sizeof(vector_t));
    *outer_vector = (vector_t){0, outer_radius};
    double outer_angle = i * (TWO_PI / num_points);
    *outer_vector = vec_rotate(*outer_vector, outer_angle);
    *outer_vector = vec_add(*outer_vector, CENTER);
    list_add(shape, outer_vector);

    // creates inner vertices of star
    vector_t *inner_vector = malloc(sizeof(vector_t));
    *inner_vector = (vector_t){0, inner_radius};
    double inner_angle = (2 * i + 1) * (M_PI / num_points);
    *inner_vector = vec_rotate(*inner_vector, inner_angle);
    *inner_vector = vec_add(*inner_vector, CENTER);
    list_add(shape, inner_vector);
  }
  return shape;
}

state_t *emscripten_init() {
  vector_t min = VEC_ZERO;
  vector_t max = WINDOW;
  sdl_init(min, max);

  state_t *state = malloc(sizeof(state_t));
  state->shape = make_star(STAR_RAD_1, STAR_RAD_2, POINTS);
  state->velocity = INITIAL_VELOCITY;
  return state;
}

void emscripten_main(state_t *state) {
  sdl_clear();
  double dt = time_since_last_tick();

  // check if out of bounds
  for (size_t i = 0; i < list_size(state->shape); i++) {
    vector_t *vec_ptr = list_get(state->shape, i);
    if (vec_ptr->x > WINDOW.x && state->velocity.x > 0) {
      state->velocity.x *= -1;
      break;
    }
    if (vec_ptr->y > WINDOW.y && state->velocity.y > 0) {
      state->velocity.y *= -1;
      break;
    }
    if (vec_ptr->x < 0 && state->velocity.x < 0) {
      state->velocity.x *= -1;
      break;
    }
    if (vec_ptr->y < 0 && state->velocity.y < 0) {
      state->velocity.y *= -1;
      break;
    }
  }

  // compute new position and orientation
  list_t *shape = state->shape;
  double angle = ANGULAR_VELOCITY * dt;
  polygon_rotate(shape, angle, polygon_centroid(shape));
  polygon_translate(shape, vec_multiply(dt, state->velocity));
  sdl_draw_polygon(shape, RGB);
  sdl_show();
}

void emscripten_free(state_t *state) {
  list_t *shape = state->shape;
  list_free(shape);
  free(state);
}
