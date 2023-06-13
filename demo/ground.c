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

// constants
const double BRICK_HEIGHT = 100;
const double BRICK_WIDTH = 200;
const double BRICK_MASS = 1.0;
const double MOMENT = 0.75;
const rgb_color_t BRICK_COLOR = (rgb_color_t){0, 0, 1};

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
const rgb_color_t ACTUAL_TRACK_COLOR = {0, 0.0, 0};

// physics constants
const double GRAVITATIONAL_ACCELERATION = 100.0;

typedef struct state {
  scene_t *scene;
  double angular_velocity;
  bool pushed_down;
} state_t;

void on_key(state_t *state, char key, key_event_type_t type, double held_time) {
  body_t *ball = scene_get_body(state->scene, 0);
  double angle = body_get_rotation(ball);
  if (type == KEY_PRESSED) {
    switch (key) {
    case LEFT_ARROW:
      if (!state->pushed_down) {
        state->pushed_down = true;
        create_applied(
            state->scene,
            (vector_t){-CIRCLE_MASS * CIRCLE_ACCELERATION * cos(angle),
                       -sin(angle)},
            ball);
      }
      break;
    case RIGHT_ARROW:
      if (!state->pushed_down) {
        state->pushed_down = true;
        create_applied(
            state->scene,
            (vector_t){CIRCLE_MASS * CIRCLE_ACCELERATION * cos(angle),
                       sin(angle)},
            ball);
      }
      break;
    }
  } else if (type == KEY_RELEASED) {
    switch (key) {
    case LEFT_ARROW:
      state->pushed_down = false;
      create_applied(state->scene,
                     (vector_t){CIRCLE_MASS * CIRCLE_ACCELERATION * cos(angle),
                                sin(angle)},
                     ball);
      break;
    case RIGHT_ARROW:
      state->pushed_down = false;
      create_applied(state->scene,
                     (vector_t){-CIRCLE_MASS * CIRCLE_ACCELERATION * cos(angle),
                                -sin(angle)},
                     ball);
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

list_t *make_star(size_t inner_radius, size_t outer_radius, size_t num_points) {
  list_t *shape = list_init(2 * num_points, free);
  for (size_t i = 0; i < num_points; i++) {
    // creates outer vertices of star
    vector_t *outer_vector = malloc(sizeof(vector_t));
    *outer_vector = (vector_t){0, outer_radius};
    double outer_angle = i * (2 * M_PI / num_points);
    *outer_vector = vec_rotate(*outer_vector, outer_angle);
    *outer_vector = vec_add(*outer_vector, CENTER);
    list_add(shape, outer_vector);

    // creates inner vertices of star
    // vector_t *inner_vector = malloc(sizeof(vector_t));
    // *inner_vector = (vector_t){0, inner_radius};
    // double inner_angle = (2 * i + 1) * (M_PI / num_points);
    // *inner_vector = vec_rotate(*inner_vector, inner_angle);
    // *inner_vector = vec_add(*inner_vector, CENTER);
    // list_add(shape, inner_vector);
  }
  return shape;
}

body_t *make_star_body() {
  list_t *star = make_star(100, 50, 20);
  return body_init(star, BRICK_MASS, BRICK_COLOR);
}

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

list_t *make_actual_track() {
  list_t *track = list_init(4, free);
  vector_t *v1 = malloc(sizeof(vector_t));
  *v1 = (vector_t){0, 20.0};
  vector_t *v2 = malloc(sizeof(vector_t));
  *v2 = (vector_t){0, TRACK_HEIGHT + 20.0};
  vector_t *v3 = malloc(sizeof(vector_t));
  *v3 = (vector_t){WINDOW.x, (WINDOW.y - TRACK_HEIGHT + 20.0)};
  vector_t *v4 = malloc(sizeof(vector_t));
  *v4 = (vector_t){WINDOW.x, 20.0};
  list_add(track, v1);
  list_add(track, v2);
  list_add(track, v3);
  list_add(track, v4);
  return track;
}

void initialize_body_list(scene_t *scene) {
  body_t *body = make_brick(BRICK_HEIGHT, BRICK_WIDTH); // make_star_body();
  body_set_centroid(body, (vector_t){WINDOW.x / 2.0, WINDOW.y});
  scene_add_body(scene, body);
  body_t *actual_track =
      body_init(make_actual_track(), TRACK_MASS, ACTUAL_TRACK_COLOR);
  scene_add_body(scene, actual_track);
  body_t *track = body_init(make_track(), TRACK_MASS, TRACK_COLOR);
  scene_add_body(scene, track);
}

bool double_is_close(double a, double b, double threshold) {
  return fabs(a - b) < threshold;
}

list_t *create_collision_triangle() {
  list_t *triangle = list_init(3, free);
  vector_t *v1 = malloc(sizeof(vector_t));
  *v1 = (vector_t){0.00005, -sqrt(3) / 2 * 0.0001};
  vector_t *v2 = malloc(sizeof(vector_t));
  *v2 = (vector_t){-0.00005, -sqrt(3) / 2 * 0.0001};
  vector_t *v3 = malloc(sizeof(vector_t));
  *v3 = (vector_t){0, 0.0001};
  list_add(triangle, v1);
  list_add(triangle, v2);
  list_add(triangle, v3);
  return triangle;
}

vector_t find_colliding_point(body_t *body1, body_t *body2) {
  list_t *body1_shape = body_get_shape(body1);
  list_t *body2_shape = body_get_shape(body2);
  for (size_t i = 0; i < list_size(body1_shape); i++) {
    vector_t centroid = *(vector_t *)list_get(body1_shape, i);
    list_t *triangle = create_collision_triangle();
    polygon_translate(triangle, centroid);
    collision_info_t collision = find_collision(triangle, body2_shape);
    if (collision.collided) {
      list_free(body1_shape);
      list_free(body2_shape);
      return centroid;
    }
  }
  list_free(body1_shape);
  list_free(body2_shape);
  return (vector_t){INFINITY, INFINITY};
}

void ground_collision(body_t *body, body_t *ground, vector_t axis, void *aux) {
  state_t *state = aux;
  list_t *wheel_shape = body_get_shape(body);
  list_t *track_shape = body_get_shape(ground);
  // collision_info_t collision = find_collision(wheel_shape, track_shape);
  double angle_diff = body_get_rotation(body) - vec_angle(axis);
  // char str[10];
  // sprintf(str, "%.9f", angle_diff);
  // puts(str);
  if (!double_is_close(fabs(angle_diff), M_PI / 2, 1e-5) &&
      !double_is_close(fabs(angle_diff), 3 * M_PI / 2, 1e-5)) {
    vector_t intersect = find_colliding_point(body, ground);
    if (intersect.x != INFINITY) {
      body_set_pivot(body, intersect);
      if (angle_diff < M_PI / 2) {
        body_set_angular_velocity(body, state->angular_velocity);
      } else {
        body_set_angular_velocity(body, -state->angular_velocity);
      }
    }
    // else {
    //   assert(1 == 0);
    // }
  } else {
    body_set_angular_velocity(body, 0.0);
  }
  list_free(wheel_shape);
  list_free(track_shape);
}

void create_ground_collision(state_t *state, body_t *body, body_t *ground) {
  create_collision(state->scene, body, ground, ground_collision, state, free);
}

void initialize_force_list(state_t *state) {
  create_downwards_gravity(state->scene, GRAVITATIONAL_ACCELERATION,
                           scene_get_body(state->scene, 0));
  create_ground_collision(state, scene_get_body(state->scene, 0),
                          scene_get_body(state->scene, 1));
  create_physics_collision(state->scene, 0.0, scene_get_body(state->scene, 0),
                           scene_get_body(state->scene, 1));
  create_normal(state->scene, scene_get_body(state->scene, 0),
                scene_get_body(state->scene, 1));

  create_ground_collision(state, scene_get_body(state->scene, 0),
                          scene_get_body(state->scene, 2));
  create_physics_collision(state->scene, 0.0, scene_get_body(state->scene, 0),
                           scene_get_body(state->scene, 2));
  create_normal(state->scene, scene_get_body(state->scene, 0),
                scene_get_body(state->scene, 2));
}

state_t *emscripten_init() {
  srand(time(NULL));
  vector_t min = VEC_ZERO;
  vector_t max = WINDOW;
  sdl_init(min, max);
  sdl_on_key((key_handler_t)on_key);

  state_t *state = malloc(sizeof(state_t));
  state->scene = scene_init();
  state->angular_velocity = 1.0;
  initialize_body_list(state->scene);
  initialize_force_list(state);
  state->pushed_down = false;
  sdl_add_image("assets/windows-xp-wallpaper-bliss-1024x576.jpg");
  return state;
}

void emscripten_main(state_t *state) {
  double dt = time_since_last_tick();
  scene_t *scene = state->scene;
  body_set_angular_velocity(scene_get_body(scene, 0), 1.0);
  sdl_move_window(body_get_centroid(scene_get_body(state->scene, 0)));
  scene_tick(scene, dt);
  sdl_render_scene(scene);
}

void emscripten_free(state_t *state) {
  scene_free(state->scene);
  free(state);
}
