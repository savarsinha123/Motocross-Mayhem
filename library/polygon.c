#include "list.h"
#include "vec_list.h"
#include "vector.h"
#include <assert.h>
#include <stdlib.h>

double polygon_area(list_t *polygon) {
  const int AREA_SCALING_FACTOR = 2;
  double area = 0;
  size_t size = list_size(polygon);
  // shoelace theorem
  for (size_t i = 0; i < size; i++) {
    vector_t *v_curr = list_get(polygon, i);
    vector_t *v_next = NULL;
    if (i == size - 1) {
      v_next = list_get(polygon, 0);
    } else {
      v_next = list_get(polygon, i + 1);
    }
    area += vec_cross(*v_curr, *v_next);
  }
  area /= AREA_SCALING_FACTOR;
  return area;
}

vector_t polygon_centroid(list_t *polygon) {
  const int CENTROID_SCALING_FACTOR = 6;
  double x = 0;
  double y = 0;
  size_t size = list_size(polygon);
  for (size_t i = 0; i < size; i++) {
    vector_t *v_curr = list_get(polygon, i);
    vector_t *v_next = NULL;
    if (i == size - 1) {
      v_next = list_get(polygon, 0);
    } else {
      v_next = list_get(polygon, i + 1);
    }
    x += (v_curr->x + v_next->x) * vec_cross(*v_curr, *v_next);
    y += (v_curr->y + v_next->y) * vec_cross(*v_curr, *v_next);
  }

  vector_t centroid = {
      .x = x / (CENTROID_SCALING_FACTOR * polygon_area(polygon)),
      .y = y / (CENTROID_SCALING_FACTOR * polygon_area(polygon))};

  return centroid;
}

void polygon_translate(list_t *polygon, vector_t translation) {
  size_t size = list_size(polygon);
  for (size_t i = 0; i < size; i++) {
    vector_t *v_ptr = list_get(polygon, i);
    *v_ptr = vec_add(*v_ptr, translation);
  }
}

void polygon_rotate(list_t *polygon, double angle, vector_t point) {
  size_t size = list_size(polygon);
  // shift rotation to coincide with origin
  polygon_translate(polygon, vec_negate(point));
  for (size_t i = 0; i < size; i++) {
    vector_t *v_ptr = list_get(polygon, i);
    *v_ptr = vec_rotate(*v_ptr, angle);
  }
  // shift reference point back
  polygon_translate(polygon, point);
}

list_t *polygon_edges(list_t *polygon) {
  list_t *result = list_init(list_size(polygon), free);
  for (size_t i = 0; i < list_size(polygon); i++) {
    vector_t *vertex1 = list_get(polygon, i);
    vector_t *vertex2 = list_get(polygon, (i + 1) % list_size(polygon));
    vector_t *edge_vector = malloc(sizeof(*edge_vector));
    *edge_vector = vec_subtract(*vertex2, *vertex1);
    list_add(result, edge_vector);
  }
  return result;
}