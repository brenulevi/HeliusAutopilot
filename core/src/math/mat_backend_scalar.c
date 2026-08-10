#include "math/mat_backend.h"

#include <math.h>

void mat_backend_mat6_mul(const float a[6][6], const float b[6][6], float out[6][6])
{
    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < 6; j++)
        {
            float s = 0.0f;
            for (int k = 0; k < 6; k++)
            {
                s += a[i][k] * b[k][j];
            }
            out[i][j] = s;
        }
    }
}

void mat_backend_mat6_transpose(const float in[6][6], float out[6][6])
{
    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < 6; j++)
        {
            out[i][j] = in[j][i];
        }
    }
}

void mat_backend_mat6_add(const float a[6][6], const float b[6][6], float out[6][6])
{
    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < 6; j++)
        {
            out[i][j] = a[i][j] + b[i][j];
        }
    }
}

void mat_backend_mat6_symmetrize(float p[6][6])
{
    for (int i = 0; i < 6; i++)
    {
        for (int j = i + 1; j < 6; j++)
        {
            float avg = 0.5f * (p[i][j] + p[j][i]);
            p[i][j] = avg;
            p[j][i] = avg;
        }
    }
}

void mat_backend_mat3_mul(const float a[3][3], const float b[3][3], float out[3][3])
{
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            float s = 0.0f;
            for (int k = 0; k < 3; k++)
            {
                s += a[i][k] * b[k][j];
            }
            out[i][j] = s;
        }
    }
}

void mat_backend_mat3_transpose(const float in[3][3], float out[3][3])
{
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            out[i][j] = in[j][i];
        }
    }
}

static float mat_backend_mat3_det(const float m[3][3])
{
    return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
        - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
        + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
}

bool mat_backend_mat3_inverse(const float in[3][3], float out[3][3])
{
    float det = mat_backend_mat3_det(in);
    if (fabsf(det) <= 1e-12f)
    {
        return false;
    }

    float inv_det = 1.0f / det;

    out[0][0] = (in[1][1] * in[2][2] - in[1][2] * in[2][1]) * inv_det;
    out[0][1] = (in[0][2] * in[2][1] - in[0][1] * in[2][2]) * inv_det;
    out[0][2] = (in[0][1] * in[1][2] - in[0][2] * in[1][1]) * inv_det;

    out[1][0] = (in[1][2] * in[2][0] - in[1][0] * in[2][2]) * inv_det;
    out[1][1] = (in[0][0] * in[2][2] - in[0][2] * in[2][0]) * inv_det;
    out[1][2] = (in[0][2] * in[1][0] - in[0][0] * in[1][2]) * inv_det;

    out[2][0] = (in[1][0] * in[2][1] - in[1][1] * in[2][0]) * inv_det;
    out[2][1] = (in[0][1] * in[2][0] - in[0][0] * in[2][1]) * inv_det;
    out[2][2] = (in[0][0] * in[1][1] - in[0][1] * in[1][0]) * inv_det;

    return true;
}

void mat_backend_mat3_skew_from_vec3(vec3_t v, float out[3][3])
{
    out[0][0] = 0.0f;
    out[0][1] = -v.z;
    out[0][2] = v.y;

    out[1][0] = v.z;
    out[1][1] = 0.0f;
    out[1][2] = -v.x;

    out[2][0] = -v.y;
    out[2][1] = v.x;
    out[2][2] = 0.0f;
}
