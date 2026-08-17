#ifndef HELIUS_MATH_MAT6_H_
#define HELIUS_MATH_MAT6_H_

typedef struct
{
    float data[6][6];
} mat6_t;

void mat6_zero(mat6_t* m);
void mat6_identity(mat6_t* m);
void mat6_add(const mat6_t* a, const mat6_t* b, mat6_t* result);
void mat6_multiply(const mat6_t* a, const mat6_t* b, mat6_t* result);
void mat6_transpose(const mat6_t* m, mat6_t* result);

#endif // HELIUS_MATH_MAT6_H_