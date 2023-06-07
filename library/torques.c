#include "torques.h"
#include "body.h"
#include "collision.h"
#include "forces.h"
#include "list.h"
#include "scene.h"
#include "vector.h"
#include <assert.h>
#include <math.h>
#include <stdlib.h>

typedef struct torques_arg {
  vector_t force;
  list_t *body;
  vector_t pivot_point;
  vector_t point_of_application;
} torques_arg_t;

void general_torque_creator(void *aux) {
  vector_t force = ((torques_arg_t *)aux)->force;
  body_t *body = ((torques_arg_t *)aux)->body;
  vector_t pivot_point = ((torques_arg_t *)aux)->pivot_point;
  vector_t point_of_application = ((torques_arg_t *)aux)->point_of_application;
  vector_t r = vec_subtract(point_of_application, pivot_point);
  double torque = vec_cross(r, force);
  body_add_torque(body, torque);
}
