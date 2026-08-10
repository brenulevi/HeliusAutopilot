#include "mat.h"
#include "math/mat_backend.h"

void mat6_mul(const float a[6][6], const float b[6][6], float out[6][6])
{
    mat_backend_mat6_mul(a, b, out);
}

void mat6_transpose(const float in[6][6], float out[6][6])
{
    mat_backend_mat6_transpose(in, out);
}

void mat6_add(const float a[6][6], const float b[6][6], float out[6][6])
{
    mat_backend_mat6_add(a, b, out);
}

void mat6_symmetrize(float p[6][6])
{
    mat_backend_mat6_symmetrize(p);
}

void mat3_mul(const float a[3][3], const float b[3][3], float out[3][3])
{
    mat_backend_mat3_mul(a, b, out);
}

void mat3_transpose(const float in[3][3], float out[3][3])
{
    mat_backend_mat3_transpose(in, out);
}

bool mat3_inverse(const float in[3][3], float out[3][3])
{
    return mat_backend_mat3_inverse(in, out);
}

void mat3_skew_from_vec3(vec3_t v, float out[3][3])
{
    mat_backend_mat3_skew_from_vec3(v, out);
}
