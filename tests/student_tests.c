// TODO: IMPLEMENT YOUR TESTS IN THIS FILE
#include "body.h"
#include "collision.h"
#include "forces.h"
#include "scene.h"
#include "test_util.h"
#include "vector.h"
#include <assert.h>
#include <math.h>
#include <stdlib.h>

double vec_norm(vector_t v) { return sqrt(vec_dot(v, v)); }

list_t *make_shape() {
  list_t *shape = list_init(4, free);
  vector_t *v = malloc(sizeof(*v));
  *v = (vector_t){-1, -1};
  list_add(shape, v);
  v = malloc(sizeof(*v));
  *v = (vector_t){+1, -1};
  list_add(shape, v);
  v = malloc(sizeof(*v));
  *v = (vector_t){+1, +1};
  list_add(shape, v);
  v = malloc(sizeof(*v));
  *v = (vector_t){-1, +1};
  list_add(shape, v);
  return shape;
}

void test_drag() {
  const double gamma = 1;
  const double mass = 5;
  const size_t STEPS = 100000;
  const double DT = 1e-4;
  scene_t *scene = scene_init();
  body_t *particle = body_init(make_shape(), mass, (rgb_color_t){0, 0, 0});
  scene_add_body(scene, particle);
  vector_t initial_velocity = (vector_t){10, 0};
  double initial_speed = vec_norm(initial_velocity);
  body_set_velocity(particle, initial_velocity);
  create_drag(scene, gamma, particle);
  double prev_speed = initial_speed;
  double t = 0;
  for (size_t i = 0; i < STEPS; i++) {
    scene_tick(scene, DT);
    t += DT;
    double curr_speed = vec_norm(body_get_velocity(particle));
    assert(curr_speed < prev_speed);
    double expected_speed = (initial_speed)*exp(-gamma * t / mass);
    assert(fabs(expected_speed - curr_speed) <= 1e-4);
    prev_speed = curr_speed;
  }
  scene_free(scene);
}

void test_orbit() {
  const double ISS_MASS = 420000;      // mass of International Space Station
  const double G = 6.67e-11;           // mass of Earth
  const double RADIUS_ORBIT = 6778000; // orbit radius of ISS
  const size_t STEPS = 100000;
  const double DT = 1e-4;
  const double EARTH_MASS = 6e24;
  scene_t *scene = scene_init();
  body_t *iss = body_init(make_shape(), ISS_MASS, (rgb_color_t){0, 0, 0});
  body_t *earth = body_init(make_shape(), EARTH_MASS, (rgb_color_t){0, 0, 0});
  scene_add_body(scene, iss);
  scene_add_body(scene, earth);
  double orbit_speed = sqrt((G * EARTH_MASS) / RADIUS_ORBIT);
  vector_t orbit_velocity = (vector_t){orbit_speed, 0};
  body_set_centroid(earth, (vector_t){0, 0});
  body_set_centroid(iss, (vector_t){0, RADIUS_ORBIT});
  body_set_velocity(iss, orbit_velocity);
  create_newtonian_gravity(scene, G, iss, earth);
  for (size_t i = 0; i < STEPS; i++) {
    scene_tick(scene, DT);
    double curr_speed = vec_norm(body_get_velocity(iss));
    assert(fabs(orbit_speed - curr_speed) <= 1e-5);
    vector_t curr_separation =
        vec_subtract(body_get_centroid(earth), body_get_centroid(iss));
    double curr_distance = vec_norm(curr_separation);
    assert(fabs(curr_distance - RADIUS_ORBIT) <= 1e-5);
  }
  scene_free(scene);
}

double calc_elastic(double k, body_t *body1, body_t *body2) {
  vector_t distance =
      vec_subtract(body_get_centroid(body2), body_get_centroid(body1));
  return 0.5 * k * vec_dot(distance, distance);
}

double calc_kinetic(body_t *body) {
  return 0.5 * body_get_mass(body) *
         vec_dot(body_get_velocity(body), body_get_velocity(body));
}

void test_harmonic_oscillator_energy() {
  const double K = 10;
  const size_t STEPS = 100000;
  const double DT = 1e-6;
  const double M = 8;
  const double A = 5;
  scene_t *scene = scene_init();
  body_t *mass = body_init(make_shape(), M, (rgb_color_t){0, 0, 0});
  body_t *anchor = body_init(make_shape(), INFINITY, (rgb_color_t){0, 0, 0});
  scene_add_body(scene, mass);
  scene_add_body(scene, anchor);
  body_set_centroid(mass, (vector_t){A, 0});
  create_spring(scene, K, mass, anchor);
  double total_energy = calc_elastic(K, mass, anchor);
  for (size_t i = 0; i < STEPS; i++) {
    double calc_energy = calc_elastic(K, mass, anchor) + calc_kinetic(mass);
    assert(fabs(calc_energy - total_energy) < 1e-5);
    scene_tick(scene, DT);
  }
  scene_free(scene);
}

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
    if (i == 0) {
      assert(!(find_collision(body_get_shape(triangle_body),
                              body_get_shape(trapezoid_body))));
    } else {
      assert(find_collision(body_get_shape(triangle_body),
                            body_get_shape(trapezoid_body)));
    }
    scene_tick(scene, DT);
  }
  assert(!(find_collision(body_get_shape(triangle_body),
                          body_get_shape(trapezoid_body))));
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

  DO_TEST(test_drag)
  DO_TEST(test_orbit)
  DO_TEST(test_harmonic_oscillator_energy)
  DO_TEST(test_colliding)

  puts("Student Tests Passed Oh YEAHH 😎");
}
