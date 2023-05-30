#include "body.h"
#include "color.h"
#include "list.h"
#include "polygon.h"
#include "vector.h"
#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>

typedef struct body {
  list_t *polygon;
  double mass;
  double angle;
  vector_t centroid;
  vector_t velocity;
  vector_t acceleration;
  vector_t force;
  vector_t impulse;
  rgb_color_t color;
  bool removed;
  void *info;
  free_func_t info_freer;
} body_t;

body_t *body_init(list_t *shape, double mass, rgb_color_t color) {
  body_t *result = malloc(sizeof(body_t));
  assert(result != NULL);

  result->polygon = shape;
  result->mass = mass;
  result->angle = 0.0;
  result->centroid = polygon_centroid(shape);
  result->color = color;
  result->velocity = VEC_ZERO;
  result->acceleration = VEC_ZERO;
  result->force = VEC_ZERO;
  result->impulse = VEC_ZERO;
  result->removed = 0;
  result->info = NULL;
  result->info_freer = NULL;
  return result;
}

body_t *body_init_with_info(list_t *shape, double mass, rgb_color_t color,
                            void *info, free_func_t info_freer) {
  body_t *body = body_init(shape, mass, color);
  body->info = info;
  body->info_freer = info_freer;
  return body;
}

void body_free(body_t *body) {
  list_free(body->polygon);
  if (body->info_freer != NULL) {
    body->info_freer(body->info);
  }
  free(body);
}

list_t *body_get_shape(body_t *body) {
  list_t *shape = list_init(list_size(body->polygon), free);
  for (size_t i = 0; i < list_size(body->polygon); i++) {
    vector_t vector = *((vector_t *)list_get(body->polygon, i));
    vector_t *added_vector = malloc(sizeof(*added_vector));
    *added_vector = vector;
    list_add(shape, added_vector);
  }
  return shape;
}

vector_t body_get_centroid(body_t *body) { return (body->centroid); }

vector_t body_get_velocity(body_t *body) { return (body->velocity); }

double body_get_mass(body_t *body) { return body->mass; }

rgb_color_t body_get_color(body_t *body) { return (body->color); }

void *body_get_info(body_t *body) { return body->info; }

void body_set_centroid(body_t *body, vector_t x) {
  polygon_translate(body->polygon, vec_subtract(x, body->centroid));
  body->centroid = x;
}

void body_set_velocity(body_t *body, vector_t v) { body->velocity = v; }

void body_set_rotation(body_t *body, double angle) {
  double angle_diff = angle - body->angle;
  polygon_rotate(body->polygon, angle_diff, body->centroid);
  body->angle = angle;
}

void body_add_force(body_t *body, vector_t force) {
  body->force = vec_add(body->force, force);
}

void body_add_impulse(body_t *body, vector_t impulse) {
  body->impulse = vec_add(body->impulse, impulse);
}

vector_t get_final_velocity(body_t *body, double dt) {
  vector_t result =
      vec_add(body->velocity, vec_multiply(dt, body->acceleration));
  result = vec_add(result, vec_multiply(1 / body->mass, body->impulse));
  return result;
}

void body_tick(body_t *body, double dt) {
  body->acceleration = vec_multiply(1 / body->mass, body->force);
  vector_t final_velocity = get_final_velocity(body, dt);
  vector_t new_centroid =
      vec_add(body->centroid,
              vec_multiply(dt, vec_average(body->velocity, final_velocity)));
  body_set_centroid(body, new_centroid);
  body->velocity = final_velocity;
  body->force = VEC_ZERO;
  body->impulse = VEC_ZERO;
}

void body_remove(body_t *body) { body->removed = 1; }

bool body_is_removed(body_t *body) { return body->removed; }
