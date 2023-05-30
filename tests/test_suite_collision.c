#include "body.h"
#include "collision.h"
#include "forces.h"
#include "scene.h"
#include "test_util.h"
#include "vector.h"
#include <assert.h>
#include <math.h>
#include <stdlib.h>

list_t *make_triangle() {
  list_t *triangle_shape = list_init(3, free);
  vector_t *a_1 = malloc(sizeof(vector_t));
  *a_1 = (vector_t){1, 1};
  list_add(triangle_shape, a_1);
  vector_t *a_2 = malloc(sizeof(vector_t));
  *a_2 = (vector_t){2, 1};
  list_add(triangle_shape, a_2);
  vector_t *a_3 = malloc(sizeof(vector_t));
  *a_3 = (vector_t){1, 2};
  list_add(triangle_shape, a_3);
  return triangle_shape;
}

list_t *make_trapezoid() {
  list_t *trapezoid_shape = list_init(4, free);
  vector_t *b_1 = malloc(sizeof(vector_t));
  *b_1 = (vector_t){1.75, 1.5};
  list_add(trapezoid_shape, b_1);
  vector_t *b_2 = malloc(sizeof(vector_t));
  *b_2 = (vector_t){3, 1.5};
  list_add(trapezoid_shape, b_2);
  vector_t *b_3 = malloc(sizeof(vector_t));
  *b_3 = (vector_t){3, 2};
  list_add(trapezoid_shape, b_3);
  vector_t *b_4 = malloc(sizeof(vector_t));
  *b_4 = (vector_t){2.5, 2};
  list_add(trapezoid_shape, b_4);
  return trapezoid_shape;
}

void test_colliding() {
  double BODY_MASS = 100;
  vector_t BODY_VELOCITY = (vector_t){-0.75, 0};
  double DT = 1;
  list_t *triangle_shape = make_triangle();
  list_t *trapezoid_shape = make_trapezoid();
  body_t *triangle_body =
      body_init(triangle_shape, BODY_MASS, (rgb_color_t){0, 0, 0});
  body_t *trapezoid_body =
      body_init(trapezoid_shape, BODY_MASS, (rgb_color_t){0, 0, 0});
  body_set_velocity(trapezoid_body, BODY_VELOCITY);
  scene_t *scene = scene_init();
  scene_add_body(scene, triangle_body);
  scene_add_body(scene, trapezoid_body);
  for (size_t i = 0; i < 3; i++) {
    list_t *triangle_shape = body_get_shape(triangle_body);
    list_t *trapezoid_shape = body_get_shape(trapezoid_body);
    collision_info_t collision =
        find_collision(triangle_shape, trapezoid_shape);
    if (i == 0) {
      assert(!collision.collided);
    } else {
      assert(collision.collided);
    }
    scene_tick(scene, DT);
    list_free(triangle_shape);
    list_free(trapezoid_shape);
  }
  scene_free(scene);
}

int main(int argc, char *argv[]) {
  // Run all tests if there are no command-line arguments
  bool all_tests = argc == 1;
  // Read test name from file
  char testname[100];
  if (!all_tests) {
    read_testname(argv[1], testname, sizeof(testname));
  }
  DO_TEST(test_colliding)

  puts("Student Tests Passed Oh YEAHH 😎");
}
