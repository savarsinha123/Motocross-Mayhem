#include "body.h"
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
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define NUM_POINTS = 60;
const vector_t POINTS[NUM_POINTS] = {
    (vector_t){0, 0},       (vector_t){0, 5},       (vector_t){16, 5},
    (vector_t){16, 0},      (vector_t){16, 0},      (vector_t){16, 5},
    (vector_t){18, 5.04},   (vector_t){18, 0},      (vector_t){18, 0},
    (vector_t){18, 5.04},   (vector_t){20, 5.16},   (vector_t){20, 0},
    (vector_t){20, 0},      (vector_t){20, 5.16},   (vector_t){22, 5.36},
    (vector_t){22, 0},      (vector_t){22, 0},      (vector_t){22, 5.36},
    (vector_t){23, 5.49},   (vector_t){23, 0},      (vector_t){23, 0},
    (vector_t){23, 5.49},   (vector_t){24, 5.64},   (vector_t){24, 0},
    (vector_t){24, 0},      (vector_t){24, 5.64},   (vector_t){25, 5.81},
    (vector_t){25, 0},      (vector_t){25, 0},      (vector_t){25, 5.81},
    (vector_t){26, 6},      (vector_t){26, 0},      (vector_t){26, 0},
    (vector_t){26, 6},      (vector_t){29, 6.69},   (vector_t){29, 0},
    (vector_t){29, 0},      (vector_t){29, 6.69},   (vector_t){32, 7.56},
    (vector_t){32, 0},      (vector_t){32, 0},      (vector_t){32, 7.56},
    (vector_t){36, 9},      (vector_t){36, 0},      (vector_t){36, 0},
    (vector_t){36, 9},      (vector_t){40, 10.76},  (vector_t){40, 0},
    (vector_t){40, 0},      (vector_t){40, 10.76},  (vector_t){44, 12.84},
    (vector_t){44, 0},      (vector_t){44, 0},      (vector_t){44, 12.84},
    (vector_t){47, 14.61},  (vector_t){47, 0},      (vector_t){47, 0},
    (vector_t){47, 14.61},  (vector_t){50, 16.56},  (vector_t){50, 0},
    (vector_t){50, 0},      (vector_t){50, 16.56},  (vector_t){60, 23.36},
    (vector_t){60, 0},      (vector_t){60, 0},      (vector_t){60, 23.36},
    (vector_t){65, 25.61},  (vector_t){65, 0},      (vector_t){65, 0},
    (vector_t){65, 25.61},  (vector_t){70, 26.86},  (vector_t){70, 0},
    (vector_t){70, 0},      (vector_t){70, 26.86},  (vector_t){75, 27.61},
    (vector_t){75, 0},      (vector_t){75, 0},      (vector_t){75, 27.61},
    (vector_t){100, 27.61}, (vector_t){100, 0},     (vector_t){100, 0},
    (vector_t){100, 27.61}, (vector_t){105, 26.86}, (vector_t){105, 0},
    (vector_t){105, 0},     (vector_t){105, 26.86}, (vector_t){110, 25.61},
    (vector_t){110, 0},     (vector_t){110, 0},     (vector_t){110, 25.61},
    (vector_t){115, 23.36}, (vector_t){115, 0},     (vector_t){115, 0},
    (vector_t){115, 23.36}, (vector_t){125, 16.56}, (vector_t){125, 0},
    (vector_t){125, 0},     (vector_t){125, 16.56}, (vector_t){128, 14.61},
    (vector_t){128, 0},     (vector_t){128, 0},     (vector_t){128, 14.61},
    (vector_t){131, 12.84}, (vector_t){131, 0},     (vector_t){131, 0},
    (vector_t){131, 12.84}, (vector_t){135, 10.76}, (vector_t){135, 0},
    (vector_t){135, 0},     (vector_t){135, 10.76}, (vector_t){139, 9},
    (vector_t){139, 0},     (vector_t){139, 0},     (vector_t){139, 9},
    (vector_t){143, 7.56},  (vector_t){143, 0},     (vector_t){143, 0},
    (vector_t){143, 7.56},  (vector_t){146, 6.69},  (vector_t){146, 0},
    (vector_t){146, 0},     (vector_t){146, 6.69},

}