#ifndef HELIUS_MATH_QUAT_H
#define HELIUS_MATH_QUAT_H

#include "vec3.h"

typedef struct
{
    float w;
    float x;
    float y;
    float z;
} quat_t;

void quat_multiply(const quat_t* q1, const quat_t* q2, quat_t* result);
void quat_conjugate(const quat_t* q, quat_t* result);
void quat_normalize(quat_t* q);
void quat_identity(quat_t* q);
void quat_rotate_vector(const quat_t* q, const vec3_t* v, vec3_t* result);
void quat_to_euler(const quat_t* q, vec3_t* euler);

#endif // HELIUS_MATH_QUAT_H