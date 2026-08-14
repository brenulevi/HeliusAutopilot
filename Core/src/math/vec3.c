#include "vec3.h"

#include <math.h>

void vec3_add(const vec3_t* v1, const vec3_t* v2, vec3_t* result)
{
    result->x = v1->x + v2->x;
    result->y = v1->y + v2->y;
    result->z = v1->z + v2->z;
}

void vec3_subtract(const vec3_t* v1, const vec3_t* v2, vec3_t* result)
{
    result->x = v1->x - v2->x;
    result->y = v1->y - v2->y;
    result->z = v1->z - v2->z;
}

void vec3_scale(const vec3_t* v, float scalar, vec3_t* result)
{
    result->x = v->x * scalar;
    result->y = v->y * scalar;
    result->z = v->z * scalar;
}

float vec3_dot(const vec3_t* v1, const vec3_t* v2)
{
    return v1->x * v2->x + v1->y * v2->y + v1->z * v2->z;
}

void vec3_cross(const vec3_t* v1, const vec3_t* v2, vec3_t* result)
{
    result->x = v1->y * v2->z - v1->z * v2->y;
    result->y = v1->z * v2->x - v1->x * v2->z;
    result->z = v1->x * v2->y - v1->y * v2->x;
}

float vec3_length(const vec3_t* v)
{
    return sqrtf(v->x * v->x + v->y * v->y + v->z * v->z);
}

void vec3_normalize(vec3_t* v)
{
    float length = vec3_length(v);
    if(length < 1e-6f) // Avoid division by zero
    {
        v->x = 0.0f;
        v->y = 0.0f;
        v->z = 0.0f;
    }
    else
    {
        float inv_length = 1.0f / length;
        v->x *= inv_length;
        v->y *= inv_length;
        v->z *= inv_length;
    }
}

void vec3_zero(vec3_t* v)
{
    v->x = 0.0f;
    v->y = 0.0f;
    v->z = 0.0f;
}