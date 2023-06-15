#include "scene.h"
#include "body.h"
#include "forces.h"
#include "list.h"
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

const size_t BASE_NUM_BODIES = 10;

typedef struct scene {
  list_t *bodies;
  list_t *forces;
} scene_t;

typedef struct force {
  force_creator_t forcer;
  void *aux;
  list_t *bodies;
  free_func_t freer;
} force_t;

force_t *force_init(force_creator_t forcer, void *aux, list_t *bodies,
                    free_func_t freer) {
  force_t *force = malloc(sizeof(*force));
  *force =
      (force_t){.forcer = forcer, .aux = aux, .bodies = bodies, .freer = freer};
  return force;
}

void force_free(force_t *force) {
  if (force->freer != NULL) {
    force->freer(force->aux);
  }
  list_free(force->bodies);
  free(force);
}

scene_t *scene_init() {
  scene_t *result = malloc(sizeof(scene_t));
  assert(result != NULL);
  result->bodies = list_init(BASE_NUM_BODIES, (free_func_t)body_free);
  result->forces = list_init(BASE_NUM_BODIES, (free_func_t)force_free);
  return result;
}

void scene_free(scene_t *scene) {
  list_free(scene->bodies);
  list_free(scene->forces);
  free(scene);
}

size_t scene_bodies(scene_t *scene) { return (list_size(scene->bodies)); }

body_t *scene_get_body(scene_t *scene, size_t index) {
  assert(index < list_size(scene->bodies));
  return (list_get(scene->bodies, index));
}

void scene_add_body(scene_t *scene, body_t *body) {
  list_add(scene->bodies, body);
}

void scene_remove_body(scene_t *scene, size_t index) {
  assert(index < list_size(scene->bodies));
  body_remove(list_get(scene->bodies, index));
}

void scene_add_force_creator(scene_t *scene, force_creator_t forcer, void *aux,
                             free_func_t freer) {
  scene_add_bodies_force_creator(scene, forcer, aux, list_init(0, NULL), freer);
}

void scene_add_bodies_force_creator(scene_t *scene, force_creator_t forcer,
                                    void *aux, list_t *bodies,
                                    free_func_t freer) {
  list_add(scene->forces, force_init(forcer, aux, bodies, freer));
}

void scene_tick(scene_t *scene, double dt) {
  // ticking force creators
  for (size_t i = 0; i < list_size(scene->forces); i++) {
    force_t *force = list_get(scene->forces, i);
    force->forcer(force->aux);
  }

  // removing force creators
  for (int32_t i = list_size(scene->forces) - 1; i >= 0; i--) {
    force_t *force = list_get(scene->forces, i);
    list_t *bodies = force->bodies;
    for (int32_t j = list_size(bodies) - 1; j >= 0; j--) {
      body_t *body = list_get(bodies, j);
      if (body_is_removed(body)) {
        force = list_remove(scene->forces, i);
        force_free(force);
        break;
      }
    }
  }

  // ticking bodies
  for (int32_t i = scene_bodies(scene) - 1; i >= 0; i--) {
    body_t *body = scene_get_body(scene, i);
    if (body_is_removed(body)) {
      body = list_remove(scene->bodies, i);
      body_free(body);
    } else {
      body_tick(body, dt);
    }
  }
}

void scene_clear_bodies(scene_t *scene) {
  size_t force_len = list_size(scene->forces);
  for (size_t i = 0; i < force_len; i++) {
    force_t *force = list_remove(scene->forces, 0);
    // force_free(force);
  }
  size_t body_len = list_size(scene->bodies);
  for (size_t i = 0; i < body_len; i++) {
    body_t *body = list_remove(scene->bodies, 0);
    // body_free(body);
  }
  // list_free(scene->bodies);
  // scene->bodies = list_init(BASE_NUM_BODIES, (free_func_t)body_free);
  // list_free(scene->forces);
  // scene->forces = list_init(BASE_NUM_BODIES, (free_func_t)force_free);
}

void scene_unload_bodies(scene_t *scene, list_t *bodies, list_t *forces) {
  size_t force_len = list_size(scene->forces);
  for (size_t i = 0; i < force_len; i++) {
    force_t *force = list_remove(scene->forces, 0);
    list_add(forces, force);
  }
  size_t body_len = list_size(scene->bodies);
  for (size_t i = 0; i < body_len; i++) {
    body_t *body = list_remove(scene->bodies, 0);
    list_add(bodies, body);
  }
}

void scene_load_bodies(scene_t *scene, list_t *bodies, list_t *forces) {
  size_t force_len = list_size(forces);
  for (size_t i = 0; i < force_len; i++) {
    force_t *force = list_remove(forces, 0);
    list_add(scene->forces, force);
  }
  size_t body_len = list_size(bodies);
  for (size_t i = 0; i < body_len; i++) {
    body_t *body = list_remove(bodies, 0);
    list_add(scene->bodies, body);
  }
}