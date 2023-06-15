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
  double moment_of_inertia;
  double curr_moment_of_inertia;
  vector_t centroid;
  vector_t velocity;
  vector_t acceleration;
  vector_t force;
  vector_t impulse;
  vector_t reference_vector;
  vector_t *reference_pointer;
  rgb_color_t color;
  double angular_velocity;
  double angular_acceleration;
  double torque;
  double angular_impulse;
  bool removed;
  vector_t curr_pivot_point;
  void *info;
  free_func_t info_freer;
} body_t;

body_t *body_init(list_t *shape, double mass, rgb_color_t color) {
  body_t *result = malloc(sizeof(body_t));
  assert(result != NULL);

  result->polygon = shape;
  result->mass = mass;
  result->angle = 0.0;
  result->moment_of_inertia = INFINITY;
  result->curr_moment_of_inertia = INFINITY;
  result->centroid = polygon_centroid(shape);
  result->color = color;
  result->angular_velocity = 0.0;
  result->angular_acceleration = 0.0;
  result->torque = 0.0;
  result->angular_impulse = 0.0;
  result->velocity = VEC_ZERO;
  result->acceleration = VEC_ZERO;
  result->force = VEC_ZERO;
  result->impulse = VEC_ZERO;
  result->reference_pointer = list_get(shape, 0);
  result->reference_vector =
      vec_subtract(*result->reference_pointer, result->centroid);
  result->removed = 0;
  result->curr_pivot_point = polygon_centroid(shape);
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
    free(body->reference_pointer);
  }
  free(body);
}

list_t *body_get_shape(body_t *body) {
  list_t *shape = list_init(list_size(body->polygon), free);
  for (size_t i = 0; i < list_size(body->polygon); i++) {
    vector_t vector = *((vector_t *)list_get(body->polygon, i));
    vector_t *added_vector = malloc(sizeof(vector_t));
    assert(added_vector != NULL);
    *added_vector = vector;
    list_add(shape, added_vector);
  }
  return shape;
}

vector_t body_get_centroid(body_t *body) { return (body->centroid); }

vector_t body_get_velocity(body_t *body) { return (body->velocity); }

double body_get_mass(body_t *body) { return body->mass; }

double body_get_moment_of_inertia(body_t *body) {
  return body->curr_moment_of_inertia;
}

rgb_color_t body_get_color(body_t *body) { return (body->color); }

void *body_get_info(body_t *body) { return body->info; }

void body_set_centroid(body_t *body, vector_t x) {
  polygon_translate(body->polygon, vec_subtract(x, body->centroid));
  body->curr_pivot_point =
      vec_add(body->curr_pivot_point, vec_subtract(x, body->centroid));
  body->centroid = x;
}

void body_set_velocity(body_t *body, vector_t v) { body->velocity = v; }

void body_set_rotation(body_t *body, double angle) {
  double angle_diff = angle - body->angle;
  polygon_rotate(body->polygon, angle_diff, body->centroid);
  // polygon_rotate(body->polygon, angle_diff, body->curr_pivot_point);
  body->angle = angle;
}

void body_rotate(body_t *body, double angle) {
  polygon_rotate(body->polygon, angle, body->curr_pivot_point);
  // vector_t new_reference = vec_subtract(*body->reference_pointer,
  // body->centroid); body->angle = vec_angle(new_reference) -
  // vec_angle(body->reference_vector);
  body->angle += angle;
}

double body_get_rotation(body_t *body) { return body->angle; }

void body_add_force(body_t *body, vector_t force) {
  body->force = vec_add(body->force, force);
}

vector_t body_get_force(body_t *body) { return body->force; }

void body_add_impulse(body_t *body, vector_t impulse) {
  body->impulse = vec_add(body->impulse, impulse);
}

void body_set_normal_moment_of_inertia(body_t *body, double moment) {
  assert(moment > 0.0);
  body->moment_of_inertia = moment;
}

void body_set_angular_velocity(body_t *body, double angular_velocity) {
  body->angular_velocity = angular_velocity;
}

void body_set_angular_acceleration(body_t *body, double angular_acceleration) {
  body->angular_acceleration = angular_acceleration;
}

void body_increment_angular_velocity(body_t *body, double increment) {
  double curr_a_v = body->angular_velocity;
  body_set_angular_velocity(body, curr_a_v + increment);
}

void body_add_torque(body_t *body, double torque) {
  body->torque = body->torque + torque;
}

void body_add_angular_impulse(body_t *body, double angular_impulse) {
  body->angular_impulse = body->angular_impulse + angular_impulse;
}

vector_t get_final_velocity(body_t *body, double dt) {
  vector_t result =
      vec_add(body->velocity, vec_multiply(dt, body->acceleration));
  result = vec_add(result, vec_multiply(1 / body->mass, body->impulse));
  return result;
}

double body_get_final_angular_velocity(body_t *body, double dt) {
  double new_velocity =
      body->angular_velocity + (body->angular_acceleration * dt);
  assert(body->moment_of_inertia != 0.0);
  new_velocity =
      new_velocity + (body->angular_impulse / body->moment_of_inertia);
  return new_velocity;
}

double body_find_delta_angle(double curr_angular_vel, double new_angular_vel,
                             double dt) {
  double delta_angle = 0.5 * (curr_angular_vel + new_angular_vel) * dt;
  return delta_angle;
}

void body_set_pivot(body_t *body, vector_t pivot) {
  body->curr_pivot_point = pivot;
  vector_t displacement = vec_subtract(body->centroid, pivot);
  double d = vec_magn(displacement);
  body->curr_moment_of_inertia = body->moment_of_inertia + (body->mass * d * d);
}

vector_t body_get_pivot(body_t *body) {
  return body->curr_pivot_point;
}

void body_reset_pivot(body_t *body) {
  body_set_pivot(body, body_get_centroid(body));
  body->curr_moment_of_inertia = body->moment_of_inertia;
}

void body_tick(body_t *body, double dt) {
  body->acceleration = vec_multiply(1 / body->mass, body->force);
  vector_t final_velocity = get_final_velocity(body, dt);
  vector_t new_centroid =
      vec_add(body->centroid,
              vec_multiply(dt, vec_average(body->velocity, final_velocity)));
  vector_t new_pivot = vec_add(body->curr_pivot_point,
                               vec_subtract(new_centroid, body->centroid));
  body_set_centroid(body, new_centroid);
  body_set_pivot(body, new_pivot);
  body->angular_acceleration = body->torque / body->moment_of_inertia;
  double final_angular_velocity = body_get_final_angular_velocity(body, dt);
  // double new_angle = body->angle +
  // body_find_delta_angle(body->angular_velocity, final_angular_velocity, dt);
  body_rotate(body, body_find_delta_angle(body->angular_velocity,
                                          final_angular_velocity, dt));
  body->velocity = final_velocity;
  body->force = VEC_ZERO;
  body->impulse = VEC_ZERO;
  body->angular_velocity = final_angular_velocity;
  body->angular_acceleration = 0.0;
  body->angular_impulse = 0.0;
  body->angular_velocity = final_angular_velocity;
  body->angular_acceleration = 0.0;
  body->angular_impulse = 0.0;
}

void body_remove(body_t *body) { body->removed = 1; }

bool body_is_removed(body_t *body) { return body->removed; }