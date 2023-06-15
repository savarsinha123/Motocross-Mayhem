#include "forces.h"
#include "body.h"
#include "collision.h"
#include "list.h"
#include "polygon.h"
#include "scene.h"
#include "vector.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

typedef struct force_arg {
  double constant;
  list_t *bodies;
} force_arg_t;

typedef struct applied_force_arg {
  vector_t force;
  list_t *bodies;
} applied_force_arg_t;

typedef struct collision_arg {
  body_t *body1;
  body_t *body2;
  collision_handler_t handler;
  void *aux;
  free_func_t freer;
  bool has_collided;
} collision_arg_t;

bool is_close(double a, double b, double threshold) {
  return fabs(a - b) < threshold;
}

void collision_arg_free(collision_arg_t *collision_arg) {
  if (collision_arg->freer != NULL) {
    collision_arg->freer(collision_arg->aux);
  }
  free(collision_arg);
}

void applied_force_creator(void *aux) {
  vector_t force = ((applied_force_arg_t *)aux)->force;
  body_t *body = list_get(((applied_force_arg_t *)aux)->bodies, 0);
  body_add_force(body, force);
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

void downwards_gravity_creator(void *aux) {
  double g = ((force_arg_t *)aux)->constant;
  body_t *body = list_get(((force_arg_t *)aux)->bodies, 0);
  vector_t gravity_vector = {.x = 0, .y = -body_get_mass(body) * g};
  body_add_force(body, gravity_vector);
}

void normal_creator(void *aux) {
  list_t *bodies = ((force_arg_t *)aux)->bodies;
  body_t *body = list_get(bodies, 0);
  body_t *surface = list_get(bodies, 1);
  list_t *body_shape = body_get_shape(body);
  list_t *surface_shape = body_get_shape(surface);
  collision_info_t collision = find_collision(body_shape, surface_shape);
  if (collision.collided) {
    double angle_diff =
        2 * M_PI - (body_get_rotation(body) - vec_angle(collision.axis));
    if (is_close(fmod(2 * fabs(angle_diff) / M_PI, 2), 0.0, 0.1)) {
      body_set_angular_velocity(body, 0.0);
    }
    list_t *edges = polygon_edges(body_shape);
    for (size_t i = 0; i < list_size(edges); i++) {
      vector_t *edge = list_get(edges, i);
      if (is_close(vec_dot(*edge, collision.axis), 0, 1e-5)) {
        vector_t surface_vector = vec_rotate(collision.axis, M_PI / 2);
        if (surface_vector.x < 0) {
          surface_vector = vec_negate(surface_vector);
        }
        break;
      }
    }
    vector_t displacement = collision.axis;
    if (displacement.y < 0) {
      displacement = vec_negate(displacement);
    }
    body_set_centroid(
        body,
        vec_add(body_get_centroid(body),
                vec_multiply(fabs(0.05 *
                                  vec_scalar_project(body_get_velocity(body),
                                                     displacement) /
                                  vec_magn(displacement)),
                             displacement)));
    list_free(edges);
  }
  list_free(body_shape);
  list_free(surface_shape);
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
    // if (collision.collided) {
    collision_arg->handler(collision_arg->body1, collision_arg->body2,
                           collision.axis, collision_arg->aux);
    collision_arg->has_collided = true;
  } else if (!collision.collided) {
    collision_arg->has_collided = false;
  }
  list_free(shape1);
  list_free(shape2);
}

void create_applied(scene_t *scene, vector_t force, body_t *body) {
  list_t *bodies = list_init(1, NULL);
  list_add(bodies, body);
  applied_force_arg_t *force_args = malloc(sizeof(applied_force_arg_t));
  *force_args = (applied_force_arg_t){.force = force, .bodies = bodies};
  scene_add_bodies_force_creator(scene, applied_force_creator, force_args,
                                 bodies, free);
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

void create_downwards_gravity(scene_t *scene, double g, body_t *body) {
  list_t *bodies = list_init(1, NULL);
  list_add(bodies, body);
  force_arg_t *gravity_args = malloc(sizeof(force_arg_t));
  *gravity_args = (force_arg_t){.constant = g, .bodies = bodies};
  scene_add_bodies_force_creator(scene, downwards_gravity_creator, gravity_args,
                                 bodies, free);
}

void create_normal(scene_t *scene, body_t *body, body_t *surface) {
  list_t *bodies = list_init(2, NULL);
  list_add(bodies, body);
  list_add(bodies, surface);
  force_arg_t *normal_args = malloc(sizeof(force_arg_t));
  *normal_args = (force_arg_t){.constant = 0.0, .bodies = bodies};
  scene_add_bodies_force_creator(scene, normal_creator, normal_args, bodies,
                                 free);
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

typedef struct suspension_args {
  double suspension_constant;
  double eq_dist;
  body_t *body1;
  body_t *body2;
  vector_t *anchor;
} suspension_args_t;

void suspension_creator(void *aux) {
  suspension_args_t *suspension_args = aux;
  vector_t displacement = vec_subtract(
      *suspension_args->anchor, body_get_centroid(suspension_args->body1));
  // checks if springs are displaced at all
  if (!is_close(vec_magn(displacement), suspension_args->eq_dist, 1e-5)) {
    double length_diff = vec_magn(displacement) - suspension_args->eq_dist;
    double total_force = suspension_args->suspension_constant * length_diff;
    vector_t force =
        vec_multiply(total_force / vec_magn(displacement), displacement);
    body_add_force(suspension_args->body1, force);
    body_add_force(suspension_args->body2, vec_negate(force));
    // body_set_pivot(suspension_args->body1,
    // body_get_centroid(suspension_args->body2));
    // body_set_angular_velocity(suspension_args->body1, 1.0);
    // body_set_angular_velocity(suspension_args->body2, 1.0);
  }
}

void create_suspension(scene_t *scene, double suspension_constant,
                       double eq_dist, body_t *body1, body_t *body2,
                       vector_t *anchor) {
  suspension_args_t *suspension_args = malloc(sizeof(suspension_args_t));
  list_t *bodies = list_init(1, NULL);
  list_add(bodies, body1);
  list_add(bodies, body2);
  *suspension_args =
      (suspension_args_t){.suspension_constant = suspension_constant,
                          .eq_dist = eq_dist,
                          .body1 = body1,
                          .body2 = body2,
                          .anchor = anchor};
  scene_add_bodies_force_creator(scene, suspension_creator, suspension_args,
                                 bodies, NULL);
}