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
const vector_t WINDOW = ((vector_t){.x = 1000, .y = 1000});

// brick constants
#define COLORS 10
const size_t BRICKS_PER_ROW = 10;
const size_t BRICK_ROWS = 3;
const double HORIZONTAL_SPACING_BRICK = 2.0;
const double VERTICAL_SPACING = 10.0;
const rgb_color_t RAINBOW[COLORS] = {(rgb_color_t){1, 0, 0},
                                     (rgb_color_t){1, 0.647, 0},
                                     (rgb_color_t){0.196, 0.804, 0.196},
                                     (rgb_color_t){0, 1, 0},
                                     (rgb_color_t){0, 1, 0},
                                     (rgb_color_t){0.2, 0.875, 1},
                                     (rgb_color_t){0, 0.392, 1},
                                     (rgb_color_t){0, 0, 1},
                                     (rgb_color_t){1, 0, 1},
                                     (rgb_color_t){1, 0.196, 0.588}};

// block constants (both brick and paddle)
#define BLOCK_WIDTH ((WINDOW).x / BRICKS_PER_ROW) * 0.9
const size_t BLOCK_NUM_POINTS = 4;
const size_t BLOCK_HEIGHT = 40;
const double BLOCK_MASS = INFINITY;
const vector_t PADDLE_STARTING_POSITION = (vector_t){500, 50};

// paddle constants
#define PADDLE_STARTING_POINT                                                  \
  (vector_t) { WINDOW.x / 2, BLOCK_HEIGHT / 2 }
const rgb_color_t PADDLE_COLOR = (rgb_color_t){1, 0, 0};
const double PADDLE_WIDTH = 150.0;
const double PADDLE_HEIGHT = 30.0;
const double PADDLE_SPEED = 600.0;

// ball constants
#define BALL_STARTING_POINT                                                    \
  (vector_t) { WINDOW.x / 2 - 20, 100 }
const double DEFAULT_ELASTICITY = 1.0;
const double BALL_RADIUS = 10.0;
const rgb_color_t BALL_COLOR = (rgb_color_t){1, 0, 0};
const size_t BALL_NUM_POINTS = 40;
const double BALL_MASS = 1.0;
const vector_t BALL_INITIAL_VELOCITY = (vector_t){400, 800};

// wall constants
const double WALL_THICKNESS = 10.0;
const rgb_color_t WHITE = (rgb_color_t){1, 1, 1};

// powerup constants
const double POWERUP_COOLDOWN = 5.0;
const double POWERUP_SIZE = 30.0;
const rgb_color_t BALL_POWERUP_COLOR = (rgb_color_t){0.5, 0.5, 0.1};
const rgb_color_t SPEED_POWERUP_COLOR = (rgb_color_t){0.7, 0.7, 0.9};

typedef struct state {
  scene_t *scene;
  double powerup_timer;
  double paddle_speed;
} state_t;

typedef enum {
  BRICK = 1,
  PADDLE = 2,
  BALL = 3,
  WALL = 4,
  BALL_POWERUP = 5,
  SPEED_POWERUP = 6
} body_type_t;

double rand_double(double lower_bound, double upper_bound) {
  double num_generated = rand();
  double return_num =
      num_generated / RAND_MAX * (upper_bound - lower_bound) + lower_bound;
  return return_num;
}

list_t *make_circle(double radius, size_t num_points) {
  list_t *polygon = list_init(num_points, free);
  double step_size = 2 * M_PI / num_points;
  for (size_t i = 0; i < num_points; i++) {
    vector_t *to_add = malloc(sizeof(*to_add));
    *to_add = vec_rotate((vector_t){radius, 0}, i * step_size);
    list_add(polygon, to_add);
  }
  return polygon;
}

vector_t calculate_brick_position(size_t row, size_t col) {
  double spacing = WINDOW.x / BRICKS_PER_ROW;
  double x_pos = spacing / 2;
  x_pos += col * spacing;
  double y_pos = WINDOW.y - BLOCK_HEIGHT - VERTICAL_SPACING;
  y_pos -= row * (BLOCK_HEIGHT + VERTICAL_SPACING);
  return (vector_t){x_pos, y_pos};
}

list_t *make_rectangle(double width, double height) {
  list_t *rectangle_shape = list_init(BLOCK_NUM_POINTS, free);
  vector_t *v1 = malloc(sizeof(*v1));
  *v1 = VEC_ZERO;
  vector_t *v2 = malloc(sizeof(*v2));
  *v2 = (vector_t){width, 0};
  vector_t *v3 = malloc(sizeof(*v3));
  *v3 = (vector_t){width, height};
  vector_t *v4 = malloc(sizeof(*v4));
  *v4 = (vector_t){0, height};
  list_add(rectangle_shape, v1);
  list_add(rectangle_shape, v2);
  list_add(rectangle_shape, v3);
  list_add(rectangle_shape, v4);
  return rectangle_shape;
}

body_t *make_ball() {
  list_t *shape = make_circle(BALL_RADIUS, BALL_NUM_POINTS);
  body_type_t *type = malloc(sizeof(*type));
  *type = BALL;
  body_t *ball = body_init_with_info(shape, BALL_MASS, BALL_COLOR, type, free);
  return ball;
}

body_t *make_brick(rgb_color_t color) {
  list_t *shape = make_rectangle(BLOCK_WIDTH, BLOCK_HEIGHT);
  body_type_t *type = malloc(sizeof(*type));
  *type = BRICK;
  body_t *brick = body_init_with_info(shape, BLOCK_MASS, color, type, free);
  return brick;
}

body_t *make_paddle(rgb_color_t color, vector_t position) {
  list_t *shape = make_rectangle(PADDLE_WIDTH, PADDLE_HEIGHT);
  body_type_t *type = malloc(sizeof(*type));
  *type = PADDLE;
  body_t *paddle = body_init_with_info(shape, BLOCK_MASS, color, type, free);
  body_set_centroid(paddle, PADDLE_STARTING_POSITION);
  return paddle;
}

body_t **make_walls() {
  body_t **walls = malloc(3 * sizeof(body_t *));
  body_type_t *type = malloc(sizeof(body_type_t));
  *type = WALL;
  list_t *wall1 = make_rectangle(WALL_THICKNESS, WINDOW.y);
  list_t *wall2 = make_rectangle(WINDOW.x, WALL_THICKNESS);
  list_t *wall3 = make_rectangle(WALL_THICKNESS, WINDOW.y);
  walls[0] = body_init_with_info(wall1, BLOCK_MASS, WHITE, type, free);
  walls[1] = body_init_with_info(wall2, BLOCK_MASS, WHITE, type, free);
  walls[2] = body_init_with_info(wall3, BLOCK_MASS, WHITE, type, free);
  body_set_centroid(walls[0], (vector_t){-WALL_THICKNESS / 2, WINDOW.y / 2});
  body_set_centroid(walls[1],
                    (vector_t){WINDOW.x / 2, WINDOW.y + WALL_THICKNESS / 2});
  body_set_centroid(walls[2],
                    (vector_t){WINDOW.x + WALL_THICKNESS / 2, WINDOW.y / 2});
  return walls;
}

void initialize_body_list(scene_t *scene) {
  const size_t NUM_WALLS = 3;
  body_t *paddle = make_paddle(PADDLE_COLOR, PADDLE_STARTING_POINT);
  scene_add_body(scene, paddle);
  body_t *ball = make_ball();
  body_set_centroid(ball, BALL_STARTING_POINT);
  body_set_velocity(ball, BALL_INITIAL_VELOCITY);
  scene_add_body(scene, ball);
  body_t **walls = make_walls();
  for (size_t i = 0; i < NUM_WALLS; i++) {
    scene_add_body(scene, walls[i]);
  }
  for (size_t row = 0; row < BRICK_ROWS; row++) {
    for (size_t column = 0; column < BRICKS_PER_ROW; column++) {
      body_t *curr_brick = make_brick(RAINBOW[column % COLORS]);
      body_set_centroid(curr_brick, calculate_brick_position(row, column));
      scene_add_body(scene, curr_brick);
    }
  }
  free(walls);
}

double calculate_impulse_ball(body_t *ball, double elasticity, vector_t axis) {
  double comp = vec_scalar_project(body_get_velocity(ball), axis);
  return -body_get_mass(ball) * (1 + elasticity) * comp;
}

void ball_collision_handler(body_t *ball, body_t *brick, vector_t axis,
                            void *aux) {
  double elasticity = *(double *)aux;
  if (!body_is_removed(brick)) {
    double impulse = calculate_impulse_ball(ball, elasticity, axis);
    vector_t impulse_vector = vec_multiply(1 / vec_magn(axis) * impulse, axis);
    body_add_impulse(ball, impulse_vector);
  }
  body_remove(brick);
}

void initialize_force_list(scene_t *scene) {
  body_t *pallet = scene_get_body(scene, 0);
  body_t *ball = scene_get_body(scene, 1);
  create_physics_collision(scene, DEFAULT_ELASTICITY, pallet, ball);
  for (size_t i = 2; i < scene_bodies(scene); i++) {
    body_t *body = scene_get_body(scene, i);
    body_type_t *type = body_get_info(body);
    if (*type == WALL) {
      create_physics_collision(scene, DEFAULT_ELASTICITY, body, ball);
    } else if (*type == BRICK) {
      double *elasticity = malloc(sizeof(double));
      *elasticity = DEFAULT_ELASTICITY;
      create_collision(scene, ball, body, ball_collision_handler, elasticity,
                       free);
    }
  }
}

void add_new_ball_collisions(scene_t *scene, body_t *ball) {
  // Adds collisions to added balls
  for (size_t i = 0; i < scene_bodies(scene); i++) {
    body_t *body = scene_get_body(scene, i);
    body_type_t *type = body_get_info(body);
    if (*type == WALL || *type == PADDLE) {
      create_physics_collision(scene, DEFAULT_ELASTICITY, body, ball);
    } else if (*type == BRICK) {
      double *elasticity = malloc(sizeof(double));
      *elasticity = DEFAULT_ELASTICITY;
      create_collision(scene, ball, body, ball_collision_handler, elasticity,
                       free);
    }
  }
}

void collect_powerup(body_t *paddle, body_t *powerup, vector_t axis,
                     void *aux) {
  // Collision handler for powerup
  body_type_t *type = body_get_info(powerup);
  if (*type == BALL_POWERUP) {
    scene_t *scene = aux;
    vector_t starting_position = body_get_centroid(paddle);
    starting_position.y = BALL_STARTING_POINT.y;
    body_t *ball1 = make_ball();
    body_set_centroid(ball1, starting_position);
    body_set_velocity(ball1, BALL_INITIAL_VELOCITY);
    add_new_ball_collisions(scene, ball1);
    scene_add_body(scene, ball1);
    body_t *ball2 = make_ball();
    body_set_centroid(ball2, starting_position);
    body_set_velocity(
        ball2, (vector_t){-BALL_INITIAL_VELOCITY.x, BALL_INITIAL_VELOCITY.y});
    add_new_ball_collisions(scene, ball2);
    scene_add_body(scene, ball2);
    body_remove(powerup);
  } else {
    state_t *state = aux;
    state->paddle_speed *= 2;
    body_remove(powerup);
  }
}

void add_powerup(state_t *state, body_type_t *type) {
  scene_t *scene = state->scene;
  list_t *square = make_rectangle(POWERUP_SIZE, POWERUP_SIZE);
  body_t *powerup;
  if (*type == BALL_POWERUP) {
    powerup =
        body_init_with_info(square, BLOCK_MASS, BALL_POWERUP_COLOR, type, free);
    create_collision(scene, scene_get_body(scene, 0), powerup, collect_powerup,
                     scene, NULL);
  } else {
    powerup = body_init_with_info(square, BLOCK_MASS, SPEED_POWERUP_COLOR, type,
                                  free);
    create_collision(scene, scene_get_body(scene, 0), powerup, collect_powerup,
                     state, NULL);
  }
  vector_t starting_position = (vector_t){
      rand_double(WINDOW.x / 4, 3 * WINDOW.x / 4), PADDLE_STARTING_POSITION.y};
  body_set_centroid(powerup, starting_position);
  scene_add_body(scene, powerup);
}

void on_key(state_t *state, char key, key_event_type_t type, double held_time) {
  body_t *paddle = scene_get_body(state->scene, 0);
  if (type == KEY_PRESSED) {
    switch (key) {
    case LEFT_ARROW:
      body_set_velocity(paddle, (vector_t){-state->paddle_speed, 0});
      break;
    case RIGHT_ARROW:
      body_set_velocity(paddle, (vector_t){state->paddle_speed, 0});
      break;
    }
  } else if (type == KEY_RELEASED) {
    switch (key) {
    case LEFT_ARROW:
      body_set_velocity(paddle, VEC_ZERO);
      break;
    case RIGHT_ARROW:
      body_set_velocity(paddle, VEC_ZERO);
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
  state->powerup_timer = POWERUP_COOLDOWN;
  state->paddle_speed = PADDLE_SPEED;
  initialize_body_list(state->scene);
  initialize_force_list(state->scene);
  return state;
}

bool check_victory(scene_t *scene) {
  for (size_t i = 1; i < scene_bodies(scene); i++) {
    body_t *body = scene_get_body(scene, i);
    body_type_t *type = body_get_info(body);
    if (*type == BRICK) {
      break;
    }
    if (i == scene_bodies(scene) - 1) {
      return true;
    }
  }
  return false;
}

bool check_game_over(scene_t *scene) {
  for (size_t i = 1; i < scene_bodies(scene); i++) {
    body_t *body = scene_get_body(scene, i);
    body_type_t *type = body_get_info(body);
    if (*type == BALL) {
      break;
    }
    if (i == scene_bodies(scene) - 1) {
      return true;
    }
  }
  return false;
}

void check_out_of_bounds(scene_t *scene) {
  body_t *paddle = scene_get_body(scene, 0);
  vector_t centroid = body_get_centroid(paddle);
  if (centroid.x - PADDLE_WIDTH / 2 < 0) {
    body_set_centroid(paddle, (vector_t){PADDLE_WIDTH / 2, centroid.y});
  } else if (centroid.x + PADDLE_WIDTH / 2 > WINDOW.x) {
    body_set_centroid(paddle,
                      (vector_t){WINDOW.x - PADDLE_WIDTH / 2, centroid.y});
  }
  for (size_t i = 1; i < scene_bodies(scene); i++) {
    body_t *body = scene_get_body(scene, i);
    body_type_t *type = body_get_info(body);
    if (*type == BALL) {
      if (body_get_centroid(body).y < -BALL_RADIUS) {
        body_remove(body);
      }
    }
  }
}

void emscripten_main(state_t *state) {
  scene_t *scene = state->scene;
  if (check_victory(scene)) {
    exit(0);
  }
  if (check_game_over(scene)) {
    scene_free(state->scene);
    state->scene = scene_init();
    state->powerup_timer = 10.0;
    state->paddle_speed = PADDLE_SPEED;
    initialize_body_list(state->scene);
    initialize_force_list(state->scene);
  }
  if (state->powerup_timer < 0.0) {
    state->paddle_speed = PADDLE_SPEED;
    body_type_t *type = malloc(sizeof(*type));
    if (rand_double(0, 1) < 0.5) {
      *type = BALL_POWERUP;
    } else {
      *type = SPEED_POWERUP;
    }
    add_powerup(state, type);
    state->powerup_timer = POWERUP_COOLDOWN;
  }
  check_out_of_bounds(state->scene);
  double dt = time_since_last_tick();
  state->powerup_timer -= dt;
  scene_tick(scene, dt);
  sdl_render_scene(scene);
}

void emscripten_free(state_t *state) {
  scene_free(state->scene);
  free(state);
}