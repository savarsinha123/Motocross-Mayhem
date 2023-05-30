#include "body.h"
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

// window constants
const vector_t WINDOW = (vector_t){.x = 1000, .y = 500};
const vector_t STARTING_POSITION = (vector_t){.x = 500, .y = 250};

// body constants
const size_t NUM_STARS = 200;
const size_t NUM_POINTS = 4;
const double DENSITY = 1.0;

// force constants
const double G = 10.0;

// numerical constants
const double TWO_PI = 2.0 * M_PI;

typedef struct state {
  scene_t *scene;
} state_t;

double rand_double(double lower_bound, double upper_bound) {
  double num_generated = rand();
  double return_num =
      num_generated / RAND_MAX * (upper_bound - lower_bound) + lower_bound;
  return return_num;
}

vector_t random_vector(double x_min, double x_max, double y_min, double y_max) {
  double x = rand_double(x_min, x_max);
  double y = rand_double(y_min, y_max);
  return (vector_t){x, y};
}

rgb_color_t random_color() {
  rgb_color_t color = {
      .r = rand_double(0, 1), .g = rand_double(0, 1), .b = rand_double(0, 1)};
  return color;
}

list_t *make_star() {
  list_t *shape = list_init(2 * NUM_POINTS, free);
  double size = rand_double(10, 40);
  vector_t starting_position = random_vector(0, WINDOW.x, 0, WINDOW.y);
  for (size_t i = 0; i < NUM_POINTS; i++) {
    // creates outer vertices of star
    vector_t *outer_vector = malloc(sizeof(vector_t));
    *outer_vector = (vector_t){0, size};
    double outer_angle = i * (TWO_PI / NUM_POINTS);
    *outer_vector = vec_rotate(*outer_vector, outer_angle);
    *outer_vector = vec_add(*outer_vector, starting_position);
    list_add(shape, outer_vector);

    // creates inner vertices of star
    vector_t *inner_vector = malloc(sizeof(vector_t));
    *inner_vector = (vector_t){0, size / 2};
    double inner_angle = (2 * i + 1) * (M_PI / NUM_POINTS);
    *inner_vector = vec_rotate(*inner_vector, inner_angle);
    *inner_vector = vec_add(*inner_vector, starting_position);
    list_add(shape, inner_vector);
  }
  return shape;
}

body_t *make_star_body() {
  list_t *star_polygon = make_star();
  double mass = DENSITY * polygon_area(star_polygon);
  body_t *body = body_init(star_polygon, mass, random_color());
  return body;
}

void initialize_body_list(scene_t *scene) {
  for (size_t i = 0; i < NUM_STARS; i++) {
    scene_add_body(scene, make_star_body());
  }
}

void initialize_force_list(scene_t *scene) {
  for (size_t i = 0; i < NUM_STARS; i++) {
    for (size_t j = i + 1; j < NUM_STARS; j++) {
      body_t *body1 = scene_get_body(scene, i);
      body_t *body2 = scene_get_body(scene, j);
      create_newtonian_gravity(scene, G, body1, body2);
    }
  }
}

state_t *emscripten_init() {
  srand(time(NULL));
  vector_t min = VEC_ZERO;
  vector_t max = WINDOW;
  sdl_init(min, max);

  state_t *state = malloc(sizeof(state_t));
  state->scene = scene_init();
  initialize_body_list(state->scene);
  initialize_force_list(state->scene);
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
