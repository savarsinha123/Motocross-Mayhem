#include "list.h"
#include "test_util.h"
#include <assert.h>
#include <stdlib.h>

void test_list_size0() {
  list_t *l = list_init(0, free);
  assert(list_size(l) == 0);
  list_free(l);
}

void test_list_size() {
  list_t *l = list_init(1, free);
  assert(list_size(l) == 0);
  // Add
  size_t *n = malloc(sizeof(*n));
  *n = 1;
  list_add(l, n);
  assert(list_size(l) == 1);
  // Remove
  assert(list_remove(l, 0) == n);
  free(n);
  assert(list_size(l) == 0);
  // Add again
  n = malloc(sizeof(*n));
  *n = 2;
  list_add(l, n);
  assert(list_size(l) == 1);
  assert(*((char *)list_get(l, 0)) == 2);
  // Modify
  *n = 3;
  assert(list_size(l) == 1);
  assert(*((char *)list_get(l, 0)) == 3);
  list_free(l);
}

void test_list_char() {
  list_t *l = list_init(1, free);
  assert(list_size(l) == 0);
  // Add
  char *c = malloc(sizeof(*c));
  *c = 'a';
  list_add(l, c);
  assert(list_size(l) == 1);
  // Remove
  assert(list_remove(l, 0) == c);
  free(c);
  assert(list_size(l) == 0);
  // Add again
  c = malloc(sizeof(*c));
  *c = 'b';
  list_add(l, c);
  assert(list_size(l) == 1);
  assert(*((char *)list_get(l, 0)) == 'b');
  // Modify
  *c = 'c';
  assert(list_size(l) == 1);
  assert(*((char *)list_get(l, 0)) == 'c');
  list_free(l);
}

void test_list_vector() {
  list_t *l = list_init(1, free);
  assert(list_size(l) == 0);
  // Add
  vector_t *v = malloc(sizeof(*v));
  *v = VEC_ZERO;
  list_add(l, v);
  assert(list_size(l) == 1);
  // Remove
  assert(list_remove(l, 0) == v);
  free(v);
  assert(list_size(l) == 0);
  // Add again
  v = malloc(sizeof(*v));
  v->x = v->y = 1;
  list_add(l, v);
  assert(list_size(l) == 1);
  assert(vec_equal(*((vector_t *)list_get(l, 0)), (vector_t){1, 1}));
  // Modify
  *v = (vector_t){1, 2};
  assert(list_size(l) == 1);
  assert(vec_equal(*((vector_t *)list_get(l, 0)), (vector_t){1, 2}));
  list_free(l);
}

void test_list_add_resize() {
  list_t *l = list_init(10, free);
  size_t *num = NULL;
  for (size_t i = 0; i < 30; i++) {
    num = malloc(sizeof(size_t));
    *num = i;
    list_add(l, num);
  }
  assert(list_size(l) == 30);
  size_t *last_num = (size_t *)list_get(l, 29);
  assert(*last_num == 29);
  list_free(l);
}

typedef struct {
  list_t *list;
  size_t index;
} list_access_t;
void get_out_of_bounds(void *aux) {
  list_access_t *access = (list_access_t *)aux;
  list_get(access->list, access->index);
}
void test_out_of_bounds_access() {
  const size_t max_size = 5;
  list_access_t *access = malloc(sizeof(*access));
  access->list = list_init(max_size, free);
  // This test takes several seconds to run
  fputs("test_out_of_bounds_access running...\n", stderr);

  // Try list with 0 elements, 1 element, ..., up to max_size elements
  for (size_t size = 0; size <= max_size; size++) {
    // Make sure negative indices report as out of bounds
    for (access->index = -3; (int)access->index < 0; access->index++) {
      assert(test_assert_fail(get_out_of_bounds, access));
    }

    // Make sure indices 0 through size - 1 are valid
    for (access->index = 0; access->index < size; access->index++) {
      // Store and retrieve an arbitrary vector
      vector_t new_vector = {size + access->index, size * access->index};
      *((vector_t *)list_get(access->list, access->index)) = new_vector;
      assert(vec_equal(*((vector_t *)list_get(access->list, access->index)),
                       new_vector));
    }

    // Assert indices greater than or equal to size are invalid
    for (access->index = size; access->index < size + 3; access->index++) {
      assert(test_assert_fail(get_out_of_bounds, access));
    }

    // Increase the size of the list by 1
    if (size < max_size) {
      list_add(access->list, malloc(sizeof(vector_t)));
    }
  }
  list_free(access->list);
  free(access);
}

int main(int argc, char *argv[]) {
  // Run all tests if there are no command-line arguments
  bool all_tests = argc == 1;
  // Read test name from file
  char testname[100];
  if (!all_tests) {
    read_testname(argv[1], testname, sizeof(testname));
  }

  DO_TEST(test_list_size0)
  DO_TEST(test_list_size)
  DO_TEST(test_list_char)
  DO_TEST(test_list_vector)
  DO_TEST(test_list_add_resize)
  DO_TEST(test_out_of_bounds_access)

  puts("list_test PASS");
}
