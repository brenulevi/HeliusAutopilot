#include "mat3.h"

#include <math.h>

void mat3_zero(mat3_t *m)
{
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            m->data[i][j] = 0.0f;
        }
    }
}

void mat3_identity(mat3_t *m)
{
    for(int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            m->data[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }
}

void mat3_add(const mat3_t *a, const mat3_t *b, mat3_t *result)
{
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            result->data[i][j] = a->data[i][j] + b->data[i][j];
        }
    }
}

void mat3_multiply(const mat3_t *a, const mat3_t *b, mat3_t *result)
{
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            result->data[i][j] = 0.0f;
            for (int k = 0; k < 3; k++)
            {
                result->data[i][j] += a->data[i][k] * b->data[k][j];
            }
        }
    }
}

void mat3_transpose(const mat3_t *m, mat3_t *result)
{
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            result->data[i][j] = m->data[j][i];
        }
    }
}

void mat3_copy(const mat3_t *src, mat3_t *dest)
{
    // Compiler will optimize this to a single memcpy
    *dest = *src;
}

bool mat3_inverse(const mat3_t *m, mat3_t *result)
{
    float det = m->data[0][0] * (m->data[1][1] * m->data[2][2] - m->data[1][2] * m->data[2][1]) -
                m->data[0][1] * (m->data[1][0] * m->data[2][2] - m->data[1][2] * m->data[2][0]) +
                m->data[0][2] * (m->data[1][0] * m->data[2][1] - m->data[1][1] * m->data[2][0]);

    float amax = 0.0f;
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            float v = fabsf(m->data[i][j]);
            if (v > amax)
            {
                amax = v;
            }
        }
    }

    /* HPH^T is rank-2 for a unit-vector measurement, so det(S) is ~R and
     * much smaller than 1e-6 in absolute terms. Use a relative test. */
    if (amax < 1e-20f || fabsf(det) < (1e-12f * amax * amax * amax))
    {
        return false;
    }

    float inv_det = 1.0f / det;

    result->data[0][0] =  (m->data[1][1] * m->data[2][2] - m->data[1][2] * m->data[2][1]) * inv_det;
    result->data[0][1] = -(m->data[0][1] * m->data[2][2] - m->data[0][2] * m->data[2][1]) * inv_det;
    result->data[0][2] =  (m->data[0][1] * m->data[1][2] - m->data[0][2] * m->data[1][1]) * inv_det;

    result->data[1][0] = -(m->data[1][0] * m->data[2][2] - m->data[1][2] * m->data[2][0]) * inv_det;
    result->data[1][1] =  (m->data[0][0] * m->data[2][2] - m->data[0][2] * m->data[2][0]) * inv_det;
    result->data[1][2] = -(m->data[0][0] * m->data[1][2] - m->data[0][2] * m->data[1][0]) * inv_det;

    result->data[2][0] =  (m->data[1][0] * m->data[2][1] - m->data[1][1] * m->data[2][0]) * inv_det;
    result->data[2][1] = -(m->data[0][0] * m->data[2][1] - m->data[0][1] * m->data[2][0]) * inv_det;
    result->data[2][2] =  (m->data[0][0] * m->data[1][1] - m->data[0][1] * m->data[1][0]) * inv_det;

    return true;
}
