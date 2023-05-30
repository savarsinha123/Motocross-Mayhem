#include "vec_list.h"
#include "list.h"
#include "vector.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>

typedef struct vec_list {
  list_t *internal_list;
} vec_list_t;

vec_list_t *vec_list_init(size_t initial_size) {
  vec_list_t *result = malloc(sizeof(*result));
  *result = (vec_list_t){list_init(initial_size, free)};
  return result;
}

void vec_list_free(vec_list_t *list) {
  list_free(list->internal_list);
  free(list);
}

size_t vec_list_size(vec_list_t *list) {
  return list_size(list->internal_list);
}

vector_t *vec_list_get(vec_list_t *list, size_t index) {
  assert(index < vec_list_size(list));
  return (vector_t *)list_get(list->internal_list, index);
}

void vec_list_add(vec_list_t *list, vector_t *value) {
  assert(sizeof(*value) == sizeof(vector_t));
  size_t capacity = list_capacity(list->internal_list);
  assert(list_size(list->internal_list) < capacity);
  list_add(list->internal_list, value);
}

vector_t *vec_list_remove(vec_list_t *list) {
  return list_remove(list->internal_list, list_size(list->internal_list) - 1);
}
