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
const vector_t WINDOW = (vector_t){.x = 1000, .y = 1000};

// defender constants
const size_t DEFENDER_NUM_POINTS = 150.0;
const double DEFENDER_SEMI_MAJOR = 50.0;
const double DEFENDER_SEMI_MINOR = 10.0;
const double DEFENDER_COOLDOWN = 1.0;
const double DEFENDER_MASS = 1.0;
const rgb_color_t DEFENDER_COLOR = (rgb_color_t){0, 1, 0};
const vector_t STARTING_VECTOR_POSITION = (vector_t){500, 50};

// invader constants
const size_t INVADERS_PER_ROW = 8;
const size_t INVADER_ROWS = 3;
const size_t INVADER_NUM_POINTS = 100;
const double INVADER_RAD = 50.0;
const double INVADER_ANGLE = (1.0 / 12.0) * M_PI;
const double INVADER_MASS = 1.0;
const double HORIZONTAL_SPACING_DIVIDER = 2.0;
const double VERTICAL_SPACING_DIVIDER = 1.2;
const double INVADER_COOLDOWN = 1.0;
const rgb_color_t INVADER_COLOR = (rgb_color_t){0.5, 0.5, 0.5};

// bullet constants
const size_t BULLET_NUM_POINTS = 4;
const size_t BULLET_HEIGHT = 40.0;
const size_t BULLET_WIDTH = 10.0;
const double BULLET_MASS = 1.0;

// velocity constants
const vector_t DEFENDER_VELOCITY = {.x = 500.0, .y = 0.0};
const vector_t INVADER_VELOCITY = {.x = 50.0, .y = 0.0};
const vector_t BULLET_VELOCITY = {.x = 0.0, .y = 900.0};

typedef struct state {
  scene_t *scene;
  double invader_time;
  double defender_time;
} state_t;

typedef enum {
  DEFENDER = 1,
  ENEMY = 2,
  DEFENDER_BULLET = 3,
  ENEMY_BULLET = 4
} body_type_t;

list_t *make_circle(double radius, double start, size_t num_points) {
  list_t *polygon = list_init(num_points, free);
  double end = M_PI - start;
  double step_size = (end - start) / num_points;
  for (double i = 0; i < num_points; i++) {
    vector_t *to_add = malloc(sizeof(*to_add));
    *to_add = vec_rotate((vector_t){radius, 0}, start);
    list_add(polygon, to_add);
    polygon_rotate(polygon, step_size, VEC_ZERO);
  }
  vector_t *centroid = malloc(sizeof(*centroid));
  *centroid = VEC_ZERO;
  list_add(polygon, centroid);
  return polygon;
}

vector_t calculate_invader_position(size_t row, size_t col) {
  double spacing = WINDOW.x / INVADERS_PER_ROW;
  double x_pos = spacing / HORIZONTAL_SPACING_DIVIDER;
  x_pos += col * spacing;
  double y_pos = WINDOW.y - INVADER_RAD;
  y_pos -= row * INVADER_RAD * VERTICAL_SPACING_DIVIDER;
  return (vector_t){x_pos, y_pos};
}

body_t *make_invader() {
  list_t *shape = list_init(INVADER_NUM_POINTS, free);
  shape = make_circle(INVADER_RAD, INVADER_ANGLE, INVADER_NUM_POINTS);
  body_type_t *type = malloc(sizeof(*type));
  *type = ENEMY;
  body_t *invader =
      body_init_with_info(shape, INVADER_MASS, INVADER_COLOR, type, free);
  body_set_velocity(invader, INVADER_VELOCITY);
  return invader;
}

list_t *make_ellipse(double a, double b, double h, double k) {
  // creates ellipse using (x - h)^2/a^2 + (y-k)^2/b^2 = 1
  // a is semi-major, b is semi-minor, h is center x coord, k is center y coord
  list_t *shape = list_init(1, free);
  double dx = 4 * a / DEFENDER_NUM_POINTS;
  for (size_t j = 0; j < DEFENDER_NUM_POINTS; j++) {
    double x, y;
    if (j < DEFENDER_NUM_POINTS / 2) {
      x = h + a - j * dx;
      y = b * sqrt((1 - (x - h) / a) * (1 + (x - h) / a)) + k;
    } else {
      x = h - a + j % (DEFENDER_NUM_POINTS / 2) * dx;
      y = -b * sqrt((1 - (x - h) / a) * (1 + (x - h) / a)) + k;
    }
    vector_t *v = malloc(sizeof(*v));
    *v = (vector_t){x, y};
    list_add(shape, v);
  }
  return shape;
}

body_t *make_defender() {
  list_t *shape =
      make_ellipse(DEFENDER_SEMI_MAJOR, DEFENDER_SEMI_MINOR,
                   STARTING_VECTOR_POSITION.x, STARTING_VECTOR_POSITION.y);
  body_type_t *type = malloc(sizeof(*type));
  *type = DEFENDER;
  body_t *defender_body =
      body_init_with_info(shape, DEFENDER_MASS, DEFENDER_COLOR, type, free);
  return defender_body;
}

body_t *make_bullet(body_type_t type) {
  list_t *rectangle_shape = list_init(BULLET_NUM_POINTS, free);
  vector_t *v1 = malloc(sizeof(*v1));
  *v1 = VEC_ZERO;
  vector_t *v2 = malloc(sizeof(*v2));
  *v2 = (vector_t){BULLET_WIDTH, 0};
  vector_t *v3 = malloc(sizeof(*v3));
  *v3 = (vector_t){BULLET_WIDTH, BULLET_HEIGHT};
  vector_t *v4 = malloc(sizeof(*v4));
  *v4 = (vector_t){0, BULLET_HEIGHT};
  list_add(rectangle_shape, v1);
  list_add(rectangle_shape, v2);
  list_add(rectangle_shape, v3);
  list_add(rectangle_shape, v4);
  rgb_color_t color;
  if (type == DEFENDER_BULLET) {
    color = DEFENDER_COLOR;
  } else {
    color = INVADER_COLOR;
  }
  body_type_t *bullet_type = malloc(sizeof(*bullet_type));
  *bullet_type = type;
  body_t *bullet = body_init_with_info(rectangle_shape, BULLET_MASS, color,
                                       bullet_type, free);
  return bullet;
}

void initialize_body_list(scene_t *scene) {
  scene_add_body(scene, make_defender());
  for (size_t i = 0; i < INVADER_ROWS; i++) {
    for (size_t j = 0; j < INVADERS_PER_ROW; j++) {
      body_t *invader = make_invader();
      body_set_centroid(invader, calculate_invader_position(i, j));
      scene_add_body(scene, invader);
    }
  }
}

void initialize_force_list(scene_t *scene) {
  body_t *defender_ship = scene_get_body(scene, 0);
  for (size_t i = 1; i <= INVADER_ROWS * INVADERS_PER_ROW; i++) {
    body_t *enemy_ship = scene_get_body(scene, i);
    create_destructive_collision(scene, defender_ship, enemy_ship);
  }
}

void defender_shoot_bullet(scene_t *scene) {
  body_t *bullet = make_bullet(DEFENDER_BULLET);
  body_set_centroid(bullet, body_get_centroid(scene_get_body(scene, 0)));
  body_set_velocity(bullet, BULLET_VELOCITY);
  scene_add_body(scene, bullet);
  for (size_t i = 0; i < scene_bodies(scene); i++) {
    body_t *body = scene_get_body(scene, i);
    body_type_t *bullet_type = body_get_info(body);
    if (*bullet_type == ENEMY) {
      create_destructive_collision(scene, bullet, body);
    }
  }
}

void on_key(state_t *state, char key, key_event_type_t type, double held_time) {
  body_t *defender_ship = scene_get_body(state->scene, 0);
  // handles key events
  if (type == KEY_PRESSED) {
    switch (key) {
    case LEFT_ARROW:
      body_set_velocity(defender_ship, vec_negate(DEFENDER_VELOCITY));
      break;
    case RIGHT_ARROW:
      body_set_velocity(defender_ship, DEFENDER_VELOCITY);
      break;
    }
  } else if (type == KEY_RELEASED) {
    switch (key) {
    case LEFT_ARROW:
      body_set_velocity(defender_ship, VEC_ZERO);
      break;
    case RIGHT_ARROW:
      body_set_velocity(defender_ship, VEC_ZERO);
      break;
    case SPACE:
      if (state->defender_time > DEFENDER_COOLDOWN) {
        state->defender_time = 0.0;
        defender_shoot_bullet(state->scene);
      }
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
  state->defender_time = 0.0;
  state->invader_time = 0.0;
  initialize_body_list(state->scene);
  initialize_force_list(state->scene);
  return state;
}

void check_out_of_bounds(scene_t *scene) {
  for (size_t i = 0; i < scene_bodies(scene); i++) {
    body_t *body = scene_get_body(scene, i);
    body_type_t type = *(body_type_t *)body_get_info(body);
    vector_t centroid = body_get_centroid(body);
    switch (type) {
    case DEFENDER:
      if (centroid.x - DEFENDER_SEMI_MAJOR < 0) {
        body_set_centroid(body, (vector_t){DEFENDER_SEMI_MAJOR, centroid.y});
      } else if (centroid.x + DEFENDER_SEMI_MAJOR > WINDOW.x) {
        body_set_centroid(
            body, (vector_t){WINDOW.x - DEFENDER_SEMI_MAJOR, centroid.y});
      }
      break;
    case ENEMY:
      if (centroid.x - INVADER_RAD < 0 || centroid.x + INVADER_RAD > WINDOW.x) {
        body_set_centroid(
            body, (vector_t){centroid.x, centroid.y - VERTICAL_SPACING_DIVIDER *
                                                          INVADER_RAD *
                                                          INVADER_ROWS});
        body_set_velocity(body, vec_negate(body_get_velocity(body)));
      }
      break;
    case DEFENDER_BULLET:
      if (centroid.y > WINDOW.y) {
        body_remove(body);
      }
      break;
    case ENEMY_BULLET:
      if (centroid.y < 0.0) {
        body_remove(body);
      }
      break;
    }
  }
}

bool check_game_over(scene_t *scene) {
  // check if defender is dead
  body_type_t *first_type = body_get_info(scene_get_body(scene, 0));
  if (*first_type != DEFENDER) {
    return 1;
  }

  // check if enemy ships are alive
  for (size_t i = 0; i < scene_bodies(scene); i++) {
    body_t *body = scene_get_body(scene, i);
    body_type_t *type = body_get_info(body);
    if (*type == ENEMY) {
      break;
    }
    if (i == scene_bodies(scene) - 1) {
      return 1;
    }
  }

  // check if below ground
  for (size_t i = 0; i < scene_bodies(scene); i++) {
    body_t *body = scene_get_body(scene, i);
    body_type_t *type = body_get_info(body);
    if (*type == ENEMY) {
      if (body_get_centroid(body).y < 0.0) {
        return 1;
      }
    }
  }

  return 0;
}

body_t *find_closest_invader(scene_t *scene) {
  if (scene_bodies(scene) == 0) {
    body_t *placeholder = make_invader();
    body_set_centroid(placeholder, WINDOW);
    return placeholder;
  }
  vector_t defender_centroid = body_get_centroid(scene_get_body(scene, 0));
  body_t *closest_invader = scene_get_body(scene, 0);
  double closest_distance = INFINITY;
  for (size_t i = 1; i < scene_bodies(scene); i++) {
    body_t *body = scene_get_body(scene, i);
    if (*(body_type_t *)body_get_info(body) == ENEMY) {
      vector_t invader_centroid = body_get_centroid(body);
      double distance = fabs(defender_centroid.x - invader_centroid.x);
      if (distance <= closest_distance) {
        closest_invader = body;
        closest_distance = distance;
      }
    }
  }
  return closest_invader;
}

void invader_shoot_bullet(scene_t *scene) {
  body_t *bullet = make_bullet(ENEMY_BULLET);
  body_t *closest_invader = find_closest_invader(scene);
  body_set_centroid(bullet, body_get_centroid(closest_invader));
  body_set_velocity(bullet, vec_negate(BULLET_VELOCITY));
  scene_add_body(scene, bullet);
  create_destructive_collision(scene, bullet, scene_get_body(scene, 0));
}

void emscripten_main(state_t *state) {
  scene_t *scene = state->scene;
  if (check_game_over(scene)) {
    exit(0);
  }
  double dt = time_since_last_tick();
  state->invader_time += dt;
  state->defender_time += dt;
  check_out_of_bounds(state->scene);
  if (state->invader_time > INVADER_COOLDOWN) {
    invader_shoot_bullet(state->scene);
    state->invader_time -= INVADER_COOLDOWN;
  }
  scene_tick(scene, dt);
  sdl_render_scene(scene);
}

void emscripten_free(state_t *state) {
  scene_free(state->scene);
  free(state);
}