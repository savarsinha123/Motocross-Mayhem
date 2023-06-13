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
const vector_t WINDOW = ((vector_t){.x = 2500, .y = 1000});
const double BRICK_MASS = 5;
const double MOMENT = 0.75;
const rgb_color_t GRAY = (rgb_color_t){0.5, 0.5, 0.5};
const rgb_color_t TEXT_COLOR = (rgb_color_t){0, 0, 0};
const size_t FONT_SIZE = 96;
#define TEXT_POSITION1                                                         \
  (vector_t) { 0.0, WINDOW.y }
#define TEXT_POSITION2 vec_multiply(0.5, WINDOW)
const vector_t TEXT_DIMENSIONS = (vector_t){300.0, 100.0};
#define BRICK_POSITION                                                         \
  (vector_t) {                                                                 \
    TEXT_POSITION2.x + 0.5 * TEXT_DIMENSIONS.x,                                \
        TEXT_POSITION2.y - 0.5 * TEXT_DIMENSIONS.y                             \
  }

typedef struct state {
  scene_t *scene;
} state_t;

body_t *make_brick(double height, double width, rgb_color_t color) {
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
  body_t *brick = body_init(brick_shape, BRICK_MASS, color);
  body_set_normal_moment_of_inertia(brick, MOMENT);
  return brick;
}

void initialize_body_list(scene_t *scene) {
  body_t *brick = make_brick(TEXT_DIMENSIONS.y, TEXT_DIMENSIONS.x, GRAY);
  body_set_centroid(brick, BRICK_POSITION);
  scene_add_body(scene, brick);
}

state_t *emscripten_init() {
  vector_t min = VEC_ZERO;
  vector_t max = WINDOW;
  sdl_init(min, max);
  state_t *state = malloc(sizeof(state_t));
  state->scene = scene_init();
  sdl_add_image("assets/windows-xp-wallpaper-bliss-1024x576.jpg",
                (vector_t){0, WINDOW.y});
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