#ifndef HELIUS_MATH_MAT3_H_
#define HELIUS_MATH_MAT3_H_

#include <stdbool.h>

typedef struct
{
    float data[3][3];
} mat3_t;

void mat3_zero(mat3_t* m);
void mat3_identity(mat3_t* m);
void mat3_add(const mat3_t* a, const mat3_t* b, mat3_t* result);
void mat3_multiply(const mat3_t* a, const mat3_t* b, mat3_t* result);
void mat3_transpose(const mat3_t* m, mat3_t* result);
void mat3_copy(const mat3_t* src, mat3_t* dest);
bool mat3_inverse(const mat3_t* m, mat3_t* result);

#endif // HELIUS_MATH_MAT3_H_