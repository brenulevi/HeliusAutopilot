#ifndef MAT_H
#define MAT_H

#include <stdbool.h>

#include "vec3.h"

void mat6_mul(const float a[6][6], const float b[6][6], float out[6][6]);
void mat6_transpose(const float in[6][6], float out[6][6]);
void mat6_add(const float a[6][6], const float b[6][6], float out[6][6]);
void mat6_symmetrize(float p[6][6]);

void mat3_mul(const float a[3][3], const float b[3][3], float out[3][3]);
void mat3_transpose(const float in[3][3], float out[3][3]);
bool mat3_inverse(const float in[3][3], float out[3][3]);
void mat3_skew_from_vec3(vec3_t v, float out[3][3]);

#endif
