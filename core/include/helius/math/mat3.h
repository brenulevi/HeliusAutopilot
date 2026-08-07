#ifndef HELIUS_MATH_MAT3_H
#define HELIUS_MATH_MAT3_H

#include "quat.h"
#include "vec3.h"

typedef struct {
    float m[3][3];
} helius_mat3_t;

helius_mat3_t helius_mat3_identity(void);
helius_mat3_t helius_mat3_skew(helius_vec3_t v);
helius_mat3_t helius_mat3_from_quat(helius_quat_t q);
helius_mat3_t helius_mat3_mult(helius_mat3_t a, helius_mat3_t b);
helius_mat3_t helius_mat3_transpose(helius_mat3_t a);

#endif