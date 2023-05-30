#include "list.h"
#include "vector.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>

const size_t SCALING_FACTOR = 2;

typedef struct list {
  size_t size;
  void **data;
  size_t capacity;
  free_func_t freer;
} list_t;

list_t *list_init(size_t initial_size, free_func_t freer) {
  list_t *result = malloc(sizeof(list_t));
  assert(sizeof(*result) == sizeof(list_t));
  if (initial_size <= 0) {
    result->capacity = 1;
  } else {
    result->capacity = initial_size;
  }
  result->data = malloc(result->capacity * sizeof(void *));
  result->size = 0;
  result->freer = freer;
  return result;
}

void list_free(list_t *list) {
  assert(list != NULL);
  if (list->freer != NULL) {
    for (size_t i = 0; i < list->size; i++) {
      list->freer(list->data[i]);
    }
  }
  free(list->data);
  free(list);
}

size_t list_size(list_t *list) { return list->size; }

size_t list_capacity(list_t *list) { return list->capacity; }

void *list_get(list_t *list, size_t index) {
  assert(index < list->size);
  return list->data[index];
}

void list_add(list_t *list, void *value) {
  assert(value != NULL);
  if (list->capacity == list->size) {
    list->capacity *= SCALING_FACTOR;
    list->data = realloc(list->data, list->capacity * sizeof(void *));
  }
  list->data[list->size] = value;
  list->size++;
}

void *list_remove(list_t *list, size_t idx) {
  assert(list->size > 0);
  assert(idx < list->size);
  void *return_element = list_get(list, idx);
  for (size_t i = idx; i < list->size - 1; i++) {
    list->data[i] = list->data[i + 1];
  }
  list->size--;
  return return_element;
}
