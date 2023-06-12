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
#define TEXT_POSITION1 (vector_t) { 0.0, WINDOW.y }
#define TEXT_POSITION2 vec_multiply(0.5, WINDOW)
const vector_t TEXT_DIMENSIONS = (vector_t) { 300.0, 100.0 };
#define BRICK_POSITION (vector_t) { TEXT_POSITION2.x + 0.5 * TEXT_DIMENSIONS.x, TEXT_POSITION2.y - 0.5 * TEXT_DIMENSIONS.y }

typedef struct state {
  scene_t *scene;
  list_t *texts;
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

void on_mouse(state_t *state, char mouse_button, mouse_event_type_t type,
              double x, double y) {
  text_input_t *text_input;
  if (type == MOUSE_BUTTON_PRESSED) {
    switch (mouse_button) {
    case LEFT_CLICK:
      if (list_size(state->texts) > 0) {
        text_input = list_remove(state->texts, 0);
        sdl_remove_text(*text_input);
      }
      break;
    }
  }
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
  sdl_on_mouse(on_mouse);
  state_t *state = malloc(sizeof(state_t));
  state->scene = scene_init();
  state->texts = list_init(2, free);
  text_input_t *text_input1 = malloc(sizeof(text_input_t));
  *text_input1 = (text_input_t) {
    .string = "Hello World!",
    .position = TEXT_POSITION1,
    .dim = TEXT_DIMENSIONS,
    .color = TEXT_COLOR
  };
  text_input_t *text_input2 = malloc(sizeof(text_input_t));
  *text_input2 = (text_input_t) {
    .string = "foo bar",
    .position = TEXT_POSITION2,
    .dim = TEXT_DIMENSIONS,
    .color = TEXT_COLOR
  };
  sdl_write_text(*text_input1);
  sdl_write_text(*text_input2);
  list_add(state->texts, text_input1);
  list_add(state->texts, text_input2);\
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