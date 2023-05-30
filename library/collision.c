#include "collision.h"
#include "body.h"
#include "forces.h"
#include "polygon.h"
#include "scene.h"
#include "vector.h"
#include <assert.h>
#include <math.h>
#include <stdlib.h>

list_t *get_normals(list_t *edges1, list_t *edges2) {
  size_t len1 = list_size(edges1);
  size_t len2 = list_size(edges2);
  size_t len_total = len1 + len2;
  list_t *result = list_init(len_total, free);
  for (size_t i = 0; i < len_total; i++) {
    vector_t *normal = malloc(sizeof(vector_t));
    assert(normal != NULL);
    if (i < len1) {
      *normal = vec_normal(*(vector_t *)list_get(edges1, i));
    } else {
      *normal = vec_normal(*(vector_t *)list_get(edges2, i - len1));
    }
    list_add(result, normal);
  }
  return result;
}

double min(double a, double b) { return a < b ? a : b; }

collision_info_t find_collision(list_t *shape1, list_t *shape2) {
  list_t *edges1 = polygon_edges(shape1);
  list_t *edges2 = polygon_edges(shape2);
  list_t *normals = get_normals(edges1, edges2);
  collision_info_t result;

  list_free(edges1);
  list_free(edges2);
  double max1, min1;
  double max2, min2;
  double min_difference = INFINITY;
  for (size_t i = 0; i < list_size(normals); i++) {
    vector_t *normal = list_get(normals, i);
    max1 = min1 = vec_scalar_project(*(vector_t *)list_get(shape1, 0), *normal);
    for (size_t j = 1; j < list_size(shape1); j++) {
      double proj =
          vec_scalar_project(*(vector_t *)list_get(shape1, j), *normal);
      if (proj > max1) {
        max1 = proj;
      }
      if (proj < min1) {
        min1 = proj;
      }
    }
    max2 = min2 = vec_scalar_project(*(vector_t *)list_get(shape2, 0), *normal);
    for (size_t j = 1; j < list_size(shape2); j++) {
      double proj =
          vec_scalar_project(*(vector_t *)list_get(shape2, j), *normal);
      if (proj > max2) {
        max2 = proj;
      }
      if (proj < min2) {
        min2 = proj;
      }
    }
    if ((min1 > max2) || (max1 < min2)) {
      list_free(normals);
      result.collided = false;
      result.axis = VEC_ZERO;
      return result;
    }
    double curr_difference = min(fabs(min1 - max2), fabs(min2 - max1));
    if (min_difference > curr_difference) {
      min_difference = curr_difference;
      result.axis = *normal;
    }
  }
  list_free(normals);
  result.collided = true;
  return result;
}
