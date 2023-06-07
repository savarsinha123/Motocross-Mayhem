#include "body.h"
#include "collision.h"
#include "color.h"
#include "forces.h"
#include "list.h"
#include "polygon.h"
#include "scene.h"
#include "sdl_wrapper.h"
#include "state.h"
#include "torques.h"
#include "vector.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

// constants
const vector_t WINDOW = ((vector_t){.x = 1500, .y = 1000});
const double BRICK_HEIGHT = 400;
const double BRICK_WIDTH = 800;
const double BRICK_MASS = 5;
const double MOMENT = 0.75;
const rgb_color_t BRICK_COLOR = (rgb_color_t){0, 0, 1};
const double INCREMENT_VALUE = 1;

typedef struct state {
  scene_t *scene;

  // global storage of current pivot point; 1 = centorid, 0 = bottom left corner
  bool curr_pivot;
} state_t;

body_t *make_brick(double height, double width) {
  vector_t *bottom_left = malloc(sizeof(vector_t));
  *bottom_left = (vector_t){0, 0};
  vector_t *top_left = malloc(sizeof(vector_t));
  *top_left = (vector_t){0, height};
  vector_t *bottom_right = malloc(sizeof(vector_t));
  *bottom_right = (vector_t){width, 0};
  vector_t *top_right = malloc(sizeof(vector_t));
  *top_right = (vector_t){width, height};
  list_t *brick_shape = list_init(4, free);
  list_add(brick_shape, bottom_left);
  list_add(brick_shape, top_left);
  list_add(brick_shape, top_right);
  list_add(brick_shape, bottom_right);
  body_t *brick = body_init(brick_shape, BRICK_MASS, BRICK_COLOR);
  body_set_normal_moment_of_inertia(brick, MOMENT);
  return brick;
}

void initialize_body_list(scene_t *scene) {
  body_t *brick = make_brick(BRICK_HEIGHT, BRICK_WIDTH);
  body_set_centroid(brick, (vector_t){500, 500});
  scene_add_body(scene, brick);
}

void set_pivot_to_corner(body_t *body) {
  body_set_angular_velocity(body, 0);
  body_set_angular_acceleration(body, 0);
  list_t *corner_list = body_get_shape(body);
  vector_t *new_pivot = list_get(corner_list, 0);
  body_set_pivot(body, *new_pivot);
  list_free(corner_list);
}

void set_pivot_to_centroid(body_t *body) {
  body_set_angular_velocity(body, 0);
  body_set_angular_acceleration(body, 0);
  vector_t centroid = body_get_centroid(body);
  body_set_pivot(body, centroid);
}

void on_key(state_t *state, char key, key_event_type_t type, double dt) {
  body_t *brick = scene_get_body(state->scene, 0);
  if (type == KEY_PRESSED) {
    switch (key) {
    case UP_ARROW:
      if (state->curr_pivot != 1) {
        state->curr_pivot = 0;
        set_pivot_to_centroid(brick);
      }
      body_increment_angular_velocity(brick, INCREMENT_VALUE);
    case DOWN_ARROW:
      if (state->curr_pivot != 1) {
        state->curr_pivot = 0;
        set_pivot_to_centroid(brick);
      }
      body_increment_angular_velocity(brick, -1 * INCREMENT_VALUE);
    case LEFT_ARROW:
      if (state->curr_pivot != 0) {
        state->curr_pivot = 1;
        set_pivot_to_corner(brick);
      }
      body_increment_angular_velocity(brick, INCREMENT_VALUE);
    case RIGHT_ARROW:
      if (state->curr_pivot != 0) {
        state->curr_pivot = 1;
        set_pivot_to_corner(brick);
      }
      body_increment_angular_velocity(brick, -1 * INCREMENT_VALUE);
    }
  } else if (type == KEY_RELEASED) {
    switch (key) {
    case RIGHT_ARROW:
      if (state->curr_pivot != 0) {
        state->curr_pivot = 1;
        set_pivot_to_centroid(brick);
      }
      body_increment_angular_velocity(brick, 1 * INCREMENT_VALUE);
    }
  }
}

state_t *emscripten_init() {
  vector_t min = VEC_ZERO;
  vector_t max = WINDOW;
  sdl_init(min, max);
  sdl_on_key((key_handler_t)on_key);

  state_t *state = malloc(sizeof(state_t));
  state->scene = scene_init();
  state->curr_pivot = 1;
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
