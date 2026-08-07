#ifndef HELIUS_MATH_QUAT_H
#define HELIUS_MATH_QUAT_H

#include "vec3.h"

typedef struct
{
    float w;
    float x;
    float y;
    float z;
} helius_quat_t;

helius_quat_t helius_quat_multiply(helius_quat_t q1, helius_quat_t q2);
helius_quat_t helius_quat_normalize(helius_quat_t q);
helius_quat_t helius_quat_from_axis_angle(helius_vec3_t w, float dt);
helius_vec3_t helius_quat_rotate_vec(helius_quat_t q, helius_vec3_t v);

#endif