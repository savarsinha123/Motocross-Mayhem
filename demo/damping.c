#include "body.h"
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
const vector_t CENTER = (vector_t){.x = 500, .y = 250};

// bead constants
const size_t NUM_BEADS = 50;
const double DENSITY = 5;
const double ANCHOR_RAD = 0.01;
const double BEAD_NUM_POINTS = 100;
const vector_t AMPLITUDE = {.x = 0, .y = 200};
const rgb_color_t white = (rgb_color_t){1.0, 1.0, 1.0};

// force constants
const double k = -1000.0;
const double DRAG = -15.0;

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

list_t *make_circle(double radius, size_t num_points) {
  list_t *polygon = list_init(num_points, free);
  double step_size = 2 * M_PI / num_points;
  for (double i = 0; i < num_points; i++) {
    vector_t *to_add = malloc(sizeof(*to_add));
    *to_add = vec_rotate((vector_t){radius, 0}, 0);
    list_add(polygon, to_add);
    polygon_rotate(polygon, step_size, VEC_ZERO);
  }
  return polygon;
}

body_t *make_bead(double density, double radius) {
  list_t *shape = make_circle(radius, BEAD_NUM_POINTS);
  double mass = density * polygon_area(shape);
  body_t *bead = body_init(shape, mass, random_color());
  return bead;
}

body_t *make_anchor() {
  list_t *shape = make_circle(ANCHOR_RAD, BEAD_NUM_POINTS);
  double mass = INFINITY;
  body_t *anchor = body_init(shape, mass, white);
  return anchor;
}

vector_t calculate_position(double idx, double radius) {
  double x_pos = (idx / NUM_BEADS) * WINDOW.x + radius;
  return (vector_t){.x = x_pos, .y = CENTER.y};
}

void initialize_body_list(scene_t *scene) {
  double radius = WINDOW.x / (2 * NUM_BEADS);
  for (size_t i = 0; i < NUM_BEADS; i++) {
    body_t *bead = make_bead(DENSITY, radius);
    body_t *anchor = make_anchor();
    vector_t position = calculate_position(i, radius);
    body_set_centroid(bead, vec_add(position, AMPLITUDE));
    body_set_centroid(anchor, position);
    scene_add_body(scene, bead);
    scene_add_body(scene, anchor);
  }
}

void initialize_force_list(scene_t *scene) {
  for (size_t i = 0; i < 2 * NUM_BEADS; i += 2) {
    body_t *bead = scene_get_body(scene, i);
    body_t *anchor = scene_get_body(scene, i + 1);
    create_spring(scene, k, bead, anchor);
    create_drag(scene, DRAG * i, bead);
  }
}

state_t *emscripten_init() {
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
