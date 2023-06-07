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

// window constants
const vector_t WINDOW = ((vector_t){.x = 1500, .y = 1000});
#define CENTER vec_multiply(0.5, WINDOW)

// circle constants
const size_t CIRCLE_POINTS = 1000;
const double CIRCLE_RADIUS = 30.0;
const double CIRCLE_MASS = 1.0;
const rgb_color_t CIRCLE_COLOR = {1, 0, 0};
const double CIRCLE_ACCELERATION = 100.0;

// track constants
const double TRACK_HEIGHT = 20.0;
const double TRACK_MASS = INFINITY;
const rgb_color_t TRACK_COLOR = {0, 0.5, 0};

// physics constants
const double GRAVITATIONAL_ACCELERATION = 100.0;

typedef struct state {
  scene_t *scene;
  bool pushed_down;
} state_t;

void on_key(state_t *state, char key, key_event_type_t type, double held_time) {
  body_t *ball = scene_get_body(state->scene, 0);
  if (type == KEY_PRESSED) {
    switch (key) {
    case LEFT_ARROW:
      if (!state->pushed_down) {
        state->pushed_down = true;
        create_applied(state->scene,
                       (vector_t){-CIRCLE_MASS * CIRCLE_ACCELERATION, 0}, ball);
      }
      break;
    case RIGHT_ARROW:
      if (!state->pushed_down) {
        state->pushed_down = true;
        create_applied(state->scene,
                       (vector_t){CIRCLE_MASS * CIRCLE_ACCELERATION, 0}, ball);
      }
      break;
    }
  } else if (type == KEY_RELEASED) {
    switch (key) {
    case LEFT_ARROW:
      state->pushed_down = false;
      create_applied(state->scene,
                     (vector_t){CIRCLE_MASS * CIRCLE_ACCELERATION, 0}, ball);
      break;
    case RIGHT_ARROW:
      state->pushed_down = false;
      create_applied(state->scene,
                     (vector_t){-CIRCLE_MASS * CIRCLE_ACCELERATION, 0}, ball);
      break;
    }
  }
}

list_t *make_circle(double radius) {
  list_t *circle = list_init(CIRCLE_POINTS, free);
  for (size_t i = 0; i < CIRCLE_POINTS; i++) {
    double angle = 2.0 * M_PI * i / CIRCLE_POINTS;
    vector_t *v = malloc(sizeof(vector_t));
    *v = (vector_t){.x = radius * cos(angle), .y = radius * sin(angle)};
    list_add(circle, v);
  }
  return circle;
}

list_t *make_track() {
  list_t *track = list_init(4, free);
  vector_t *v1 = malloc(sizeof(vector_t));
  *v1 = VEC_ZERO;
  vector_t *v2 = malloc(sizeof(vector_t));
  *v2 = (vector_t){0, TRACK_HEIGHT};
  vector_t *v3 = malloc(sizeof(vector_t));
  *v3 = (vector_t){WINDOW.x, WINDOW.y - TRACK_HEIGHT};
  vector_t *v4 = malloc(sizeof(vector_t));
  *v4 = (vector_t){WINDOW.x, 0};
  list_add(track, v1);
  list_add(track, v2);
  list_add(track, v3);
  list_add(track, v4);
  return track;
}

void initialize_body_list(scene_t *scene) {
  body_t *body =
      body_init(make_circle(CIRCLE_RADIUS), CIRCLE_MASS, CIRCLE_COLOR);
  body_set_centroid(body, (vector_t){WINDOW.x / 2.0, WINDOW.y});
  scene_add_body(scene, body);
  body_t *track = body_init(make_track(), TRACK_MASS, TRACK_COLOR);
  scene_add_body(scene, track);
}

void initialize_force_list(scene_t *scene) {
  create_downwards_gravity(scene, GRAVITATIONAL_ACCELERATION,
                           scene_get_body(scene, 0));
  create_physics_collision(scene, 0.0, scene_get_body(scene, 0),
                           scene_get_body(scene, 1));
  create_normal(scene, scene_get_body(scene, 0), scene_get_body(scene, 1));
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
  initialize_force_list(state->scene);
  state->pushed_down = false;
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
