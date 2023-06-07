#ifndef __TORQUES_H__
#define __TORQUES_H__

#include "scene.h"

typedef struct torques_arg torques_arg_t;

void create_general_torque(body_t *body, vector_t force,
                           vector_t point_of_evaluation, vector_t pivot);

#endif // #ifndef __TORQUES_H__
