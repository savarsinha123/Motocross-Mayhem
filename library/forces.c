#include "forces.h"
#include "body.h"
#include "collision.h"
#include "list.h"
#include "polygon.h"
#include "scene.h"
#include "vector.h"
#include <assert.h>
#include <math.h>
#include <stdlib.h>

typedef struct force_arg {
  double constant;
  list_t *bodies;
} force_arg_t;

typedef struct collision_arg {
  body_t *body1;
  body_t *body2;
  collision_handler_t handler;
  void *aux;
  free_func_t freer;
  bool has_collided;
} collision_arg_t;

void collision_arg_free(collision_arg_t *collision_arg) {
  if (collision_arg->freer != NULL) {
    collision_arg->freer(collision_arg->aux);
  }
  free(collision_arg);
}

void newtonian_gravity_creator(void *aux) {
  double G = ((force_arg_t *)aux)->constant;
  list_t *bodies = ((force_arg_t *)aux)->bodies;
  body_t *body1 = list_get(bodies, 0);
  body_t *body2 = list_get(bodies, 1);
  vector_t centroid1 = body_get_centroid(body1);
  vector_t centroid2 = body_get_centroid(body2);
  vector_t separation = vec_subtract(centroid2, centroid1);
  vector_t direction = vec_multiply(1 / vec_magn(separation), separation);
  double distance = vec_magn(separation);

  // prevent gravity from blowing up
  if (distance >= 5.0) {
    double gravity_magn =
        G * body_get_mass(body1) * body_get_mass(body2) / (distance * distance);
    body_add_force(body1, vec_multiply(gravity_magn, direction));
    body_add_force(body2, vec_multiply(-gravity_magn, direction));
  }
}

void spring_creator(void *aux) {
  double k = ((force_arg_t *)aux)->constant;
  list_t *bodies = ((force_arg_t *)aux)->bodies;
  body_t *body1 = list_get(bodies, 0);
  body_t *body2 = list_get(bodies, 1);
  vector_t centroid1 = body_get_centroid(body1);
  vector_t centroid2 = body_get_centroid(body2);
  vector_t x = vec_subtract(centroid2, centroid1);
  body_add_force(body1, vec_multiply(k, x));
  body_add_force(body2, vec_multiply(-k, x));
}

void drag_creator(void *aux) {
  double gamma = ((force_arg_t *)aux)->constant;
  list_t *bodies = ((force_arg_t *)aux)->bodies;
  body_t *body = list_get(bodies, 0);
  vector_t velocity = body_get_velocity(body);
  vector_t force = vec_multiply(-gamma, velocity);
  body_add_force(body, force);
}

double calculate_impulse(body_t *body1, body_t *body2, double elasticity,
                         vector_t axis) {
  double component1 = vec_scalar_project(body_get_velocity(body1), axis);
  double component2 = vec_scalar_project(body_get_velocity(body2), axis);
  double mass1 = body_get_mass(body1);
  double mass2 = body_get_mass(body2);
  double mass_eff;
  if (mass1 == INFINITY) {
    mass_eff = mass2;
  } else if (mass2 == INFINITY) {
    mass_eff = mass1;
  } else {
    mass_eff = (mass1 * mass2) / (mass1 + mass2);
  }
  return mass_eff * (1 + elasticity) * (component2 - component1);
}

void destructive_collision_handler(body_t *body1, body_t *body2, vector_t axis,
                                   void *aux) {
  body_remove(body1);
  body_remove(body2);
}

void physics_collision_handler(body_t *body1, body_t *body2, vector_t axis,
                               void *aux) {
  double elasticity = *(double *)aux;
  double impulse = calculate_impulse(body1, body2, elasticity, axis);
  vector_t impulse_vector = vec_multiply(1 / vec_magn(axis) * impulse, axis);
  body_add_impulse(body1, impulse_vector);
  body_add_impulse(body2, vec_negate(impulse_vector));
}

void collision_creator(void *aux) {
  collision_arg_t *collision_arg = aux;
  list_t *shape1 = body_get_shape(collision_arg->body1);
  list_t *shape2 = body_get_shape(collision_arg->body2);
  collision_info_t collision = find_collision(shape1, shape2);
  if (collision.collided && !collision_arg->has_collided) {
    collision_arg->handler(collision_arg->body1, collision_arg->body2,
                           collision.axis, collision_arg->aux);
    collision_arg->has_collided = true;
  } else if (!collision.collided) {
    collision_arg->has_collided = false;
  }
  list_free(shape1);
  list_free(shape2);
}

void create_newtonian_gravity(scene_t *scene, double G, body_t *body1,
                              body_t *body2) {
  list_t *bodies = list_init(2, NULL);
  list_add(bodies, body1);
  list_add(bodies, body2);
  force_arg_t *gravity_args = malloc(sizeof(force_arg_t));
  *gravity_args = (force_arg_t){.constant = G, .bodies = bodies};
  scene_add_bodies_force_creator(scene, newtonian_gravity_creator, gravity_args,
                                 bodies, (free_func_t)free);
}

void create_spring(scene_t *scene, double k, body_t *body1, body_t *body2) {
  list_t *bodies = list_init(2, NULL);
  list_add(bodies, body1);
  list_add(bodies, body2);
  force_arg_t *spring_args = malloc(sizeof(force_arg_t));
  *spring_args = (force_arg_t){.constant = k, .bodies = bodies};
  scene_add_bodies_force_creator(scene, spring_creator, spring_args, bodies,
                                 (free_func_t)free);
}

void create_drag(scene_t *scene, double gamma, body_t *body) {
  list_t *bodies = list_init(1, NULL);
  list_add(bodies, body);
  force_arg_t *drag_args = malloc(sizeof(force_arg_t));
  *drag_args = (force_arg_t){.constant = gamma, .bodies = bodies};
  scene_add_bodies_force_creator(scene, drag_creator, drag_args, bodies,
                                 (free_func_t)free);
}

void create_collision(scene_t *scene, body_t *body1, body_t *body2,
                      collision_handler_t handler, void *aux,
                      free_func_t freer) {
  list_t *bodies = list_init(2, NULL);
  list_add(bodies, body1);
  list_add(bodies, body2);
  collision_arg_t *collision_arg = malloc(sizeof(collision_arg_t));
  collision_arg->body1 = body1;
  collision_arg->body2 = body2;
  collision_arg->handler = handler;
  collision_arg->aux = aux;
  collision_arg->freer = freer;
  scene_add_bodies_force_creator(scene, (force_creator_t)collision_creator,
                                 collision_arg, bodies,
                                 (free_func_t)collision_arg_free);
}

void create_destructive_collision(scene_t *scene, body_t *body1,
                                  body_t *body2) {
  create_collision(scene, body1, body2, destructive_collision_handler, NULL,
                   NULL);
}

void create_physics_collision(scene_t *scene, double elasticity, body_t *body1,
                              body_t *body2) {
  double *aux = malloc(sizeof(double));
  *aux = elasticity;
  create_collision(scene, body1, body2,
                   (collision_handler_t)physics_collision_handler, aux, free);
};
