#ifndef QUAT_H
#define QUAT_H

#include <stdbool.h>

#include "vec3.h"

typedef struct
{
    float w;
    float x;
    float y;
    float z;
} quat_t;

quat_t quat_normalize(quat_t q);
quat_t quat_mult(quat_t q1, quat_t q2);
float quat_norm(quat_t q);
quat_t quat_conjugate(quat_t q);
vec3_t quat_rotate_vec3(quat_t q, vec3_t v);
quat_t quat_identity(void);
bool quat_is_finite(quat_t q);

#endif