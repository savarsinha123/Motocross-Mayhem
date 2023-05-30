#include "body.h"
#include "collision.h"
#include "color.h"
#include "forces.h"
#include "list.h"
#include "polygon.h"
#include "scene.h"
#include "sdl_wrapper.h"
#include "state.h"
#include "vector.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

// constants
const vector_t WINDOW = ((vector_t){.x = 1500, .y = 1000});

typedef struct state {
    scene_t *scene;
} state_t;

// helper functions

// key handler function
void on_key(state_t *state, char key, key_event_type_t type, double held_time) {
    // TODO: key handlers
}

state_t *emscripten_init() {
    srand(time(NULL));
    vector_t min = VEC_ZERO;
    vector_t max = WINDOW;
    sdl_init(min, max);
    sdl_on_key((key_handler_t)on_key);
}

void emscripten_main(state_t *state) {
    double dt = time_since_last_tick();
}

void emscripten_free(state_t *state) {
    scene_free(state->scene);
    free(state);
}