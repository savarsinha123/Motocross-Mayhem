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

// window constants
const vector_t WINDOW = (vector_t){.x = 1000, .y = 500};
const vector_t STARTING_POSITION = (vector_t){.x = 500, .y = 250};

// body constants
const double PELLET_MASS = 1.0;
const rgb_color_t COLOR = {.r = 0.9, .g = 0.9, .b = 0.0};
const size_t NUM_PELLETS = 30;
const size_t PELLET_NUM_POINTS = 30;
const size_t PACMAN_NUM_POINTS = 200;
const size_t PELLET_RAD = 5;
const size_t PACMAN_RAD = 30;
const vector_t MOUTH_POINT = {.x = 0.01, .y = 0};
const double MOUTH_ANGLE = M_PI / 4;
const double UNIFORM_VELOCITY = 100;
const double ACCELERATION = 200;

typedef struct state {
  scene_t *scene;
} state_t;

vector_t random_vector() {
  double x = (double)rand() / RAND_MAX * WINDOW.x;
  double y = (double)rand() / RAND_MAX * WINDOW.y;
  return (vector_t){x, y};
}

list_t *make_circle(double radius, double start, size_t num_points) {
  list_t *polygon = list_init(num_points, free);
  double end = 2 * M_PI - start;
  double step_size = (end - start) / num_points;
  // creates pacman and pellets
  for (double i = 0; i < num_points; i++) {
    vector_t *to_add = malloc(sizeof(*to_add));
    *to_add = vec_rotate((vector_t){radius, 0}, start);
    list_add(polygon, to_add);
    polygon_rotate(polygon, step_size, VEC_ZERO);
  }
  vector_t *centroid = malloc(sizeof(*centroid));
  *centroid = MOUTH_POINT;
  list_add(polygon, centroid);
  return polygon;
}

body_t *make_pellet() {
  list_t *shape = list_init(1, free);
  shape = make_circle(PELLET_RAD, 0, PELLET_NUM_POINTS);
  polygon_translate(shape, random_vector());
  body_t *pellet = body_init(shape, PELLET_MASS, COLOR);
  return pellet;
}

body_t *make_pacman() {
  list_t *shape = list_init(1, free);
  shape = make_circle(PACMAN_RAD, MOUTH_ANGLE, PACMAN_NUM_POINTS);
  polygon_translate(shape, STARTING_POSITION);
  body_t *pacman = body_init(shape, PELLET_MASS, COLOR);
  return pacman;
}

void initialize_body_list(scene_t *scene) {
  scene_add_body(scene, make_pacman());
  for (size_t i = 0; i < NUM_PELLETS; i++) {
    scene_add_body(scene, make_pellet());
  }
}

void on_key(state_t *state, char key, key_event_type_t type, double held_time) {
  body_t *pacman = scene_get_body(state->scene, 0);
  // handles key events
  if (type == KEY_PRESSED) {
    vector_t new_velocity;
    switch (key) {
    case LEFT_ARROW:
      new_velocity =
          vec_add((vector_t){-UNIFORM_VELOCITY, 0},
                  vec_multiply(held_time, (vector_t){-ACCELERATION, 0}));
      body_set_rotation(pacman, M_PI);
      break;
    case UP_ARROW:
      new_velocity =
          vec_add((vector_t){0, UNIFORM_VELOCITY},
                  vec_multiply(held_time, (vector_t){0, ACCELERATION}));
      body_set_rotation(pacman, M_PI / 2);
      break;
    case RIGHT_ARROW:
      new_velocity =
          vec_add((vector_t){UNIFORM_VELOCITY, 0},
                  vec_multiply(held_time, (vector_t){ACCELERATION, 0}));
      body_set_rotation(pacman, 0);
      break;
    case DOWN_ARROW:
      new_velocity =
          vec_add((vector_t){0, -UNIFORM_VELOCITY},
                  vec_multiply(held_time, (vector_t){0, -ACCELERATION}));
      body_set_rotation(pacman, 3 * M_PI / 2);
      break;
    }
    body_set_velocity(pacman, new_velocity);
  } else if (type == KEY_RELEASED) {
    switch (key) {
    case LEFT_ARROW:
      body_set_velocity(pacman, (vector_t){-UNIFORM_VELOCITY, 0});
      break;
    case UP_ARROW:
      body_set_velocity(pacman, (vector_t){0, UNIFORM_VELOCITY});
      break;
    case RIGHT_ARROW:
      body_set_velocity(pacman, (vector_t){UNIFORM_VELOCITY, 0});
      break;
    case DOWN_ARROW:
      body_set_velocity(pacman, (vector_t){0, -UNIFORM_VELOCITY});
      break;
    }
  }
}

state_t *emscripten_init() {
  srand(time(NULL));
  vector_t min = VEC_ZERO;
  vector_t max = WINDOW;
  sdl_init(min, max);
  sdl_on_key((key_handler_t)on_key);

  state_t *state = malloc(sizeof(state_t));
  state->scene = scene_init();
  initialize_body_list(state->scene);
  return state;
}

void wrap_around(scene_t *scene, double radius) {
  body_t *pacman = scene_get_body(scene, 0);
  vector_t centroid = body_get_centroid(pacman);
  vector_t velocity = body_get_velocity(pacman);
  vector_t new_position = centroid;

  // check out of bounds
  if (centroid.x > WINDOW.x + radius && velocity.x > 0) {
    new_position = (vector_t){-radius, centroid.y};
  } else if (centroid.x < -radius && velocity.x < 0) {
    new_position = (vector_t){WINDOW.x + radius, centroid.y};
  } else if (centroid.y > WINDOW.y + radius && velocity.y > 0) {
    new_position = (vector_t){centroid.x, -radius};
  } else if (centroid.y < -radius && velocity.y < 0) {
    new_position = (vector_t){centroid.x, WINDOW.x + radius};
  }
  body_set_centroid(pacman, new_position);
}

bool in_pacman(body_t *pacman, vector_t vector) {
  vector_t pacman_centroid = body_get_centroid(pacman);
  vector_t diff = vec_subtract(vector, pacman_centroid);
  if (sqrt(vec_dot(diff, diff)) < PACMAN_RAD) {
    return 1;
  }
  return 0;
}

void eat_pellets(scene_t *scene) {
  body_t *pacman = scene_get_body(scene, 0);
  // check if inside pacman
  for (size_t i = scene_bodies(scene) - 1; i > 0; i--) {
    body_t *pellet_body = scene_get_body(scene, i);
    if (in_pacman(pacman, body_get_centroid(pellet_body))) {
      scene_remove_body(scene, i);
      scene_add_body(scene, make_pellet());
    }
  }
}

void emscripten_main(state_t *state) {
  double dt = time_since_last_tick();
  scene_t *scene = state->scene;
  wrap_around(scene, PACMAN_RAD);
  eat_pellets(scene);
  scene_tick(scene, dt);
  sdl_render_scene(scene);
}

void emscripten_free(state_t *state) {
  scene_free(state->scene);
  free(state);
}
