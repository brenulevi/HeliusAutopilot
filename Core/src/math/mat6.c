#include "mat6.h"

void mat6_zero(mat6_t* m)
{
    for (int i = 0; i < 6; ++i)
    {
        for (int j = 0; j < 6; ++j)
        {
            m->data[i][j] = 0.0f;
        }
    }
}

void mat6_identity(mat6_t* m)
{
    for(int i = 0; i < 6; ++i)
    {
        for (int j = 0; j < 6; ++j)
        {
            m->data[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }
}

void mat6_add(const mat6_t* a, const mat6_t* b, mat6_t* result)
{
    for (int i = 0; i < 6; ++i)
    {
        for (int j = 0; j < 6; ++j)
        {
            result->data[i][j] = a->data[i][j] + b->data[i][j];
        }
    }
}

void mat6_multiply(const mat6_t* a, const mat6_t* b, mat6_t* result)
{
    for (int i = 0; i < 6; ++i)
    {
        for (int j = 0; j < 6; ++j)
        {
            result->data[i][j] = 0.0f;
            for (int k = 0; k < 6; ++k)
            {
                result->data[i][j] += a->data[i][k] * b->data[k][j];
            }
        }
    }
}

void mat6_transpose(const mat6_t* m, mat6_t* result)
{
    for (int i = 0; i < 6; ++i)
    {
        for (int j = 0; j < 6; ++j)
        {
            result->data[i][j] = m->data[j][i];
        }
    }
}