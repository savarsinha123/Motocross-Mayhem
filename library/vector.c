#include "vector.h"
#include <assert.h>
#include <math.h>

const vector_t VEC_ZERO = {.x = 0, .y = 0};

vector_t vec_add(vector_t v1, vector_t v2) {
  vector_t result = {v1.x + v2.x, v1.y + v2.y};
  return result;
}

vector_t vec_multiply(double scalar, vector_t v) {
  vector_t result = {v.x * scalar, v.y * scalar};
  return result;
}

vector_t vec_negate(vector_t v) {
  vector_t result = vec_multiply(-1, v);
  return result;
}

vector_t vec_subtract(vector_t v1, vector_t v2) {
  vector_t result = vec_add(v1, vec_negate(v2));
  return result;
}

double vec_dot(vector_t v1, vector_t v2) {
  return (v1.x * v2.x) + (v1.y * v2.y);
}

double vec_angle(vector_t v) { return atan2(v.y, v.x); }

double vec_magn(vector_t v) { return sqrt(vec_dot(v, v)); }

double vec_cross(vector_t v1, vector_t v2) {
  double result = v1.x * v2.y - v1.y * v2.x;
  return result;
}

vector_t vec_rotate(vector_t v, double angle) {
  double magn_v = vec_magn(v);
  double new_angle = vec_angle(v) + angle;
  vector_t result = {cos(new_angle), sin(new_angle)};
  return vec_multiply(magn_v, result);
}

vector_t vec_average(vector_t v1, vector_t v2) {
  vector_t sum = vec_add(v1, v2);
  vector_t result = vec_multiply(0.5, sum);
  return result;
}

vector_t vec_normal(vector_t v) { return (vector_t){-v.y, v.x}; }

double vec_scalar_project(vector_t v1, vector_t v2) {
  return vec_dot(v1, v2) / vec_magn(v2);
}
