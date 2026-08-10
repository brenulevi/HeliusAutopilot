#include "vec3.h"

#include <math.h>

vec3_t vec3_add(vec3_t a, vec3_t b)
{
    vec3_t r = {
        .x = a.x + b.x,
        .y = a.y + b.y,
        .z = a.z + b.z,
    };
    return r;
}

vec3_t vec3_sub(vec3_t a, vec3_t b)
{
    vec3_t r = {
        .x = a.x - b.x,
        .y = a.y - b.y,
        .z = a.z - b.z,
    };
    return r;
}

vec3_t vec3_scale(vec3_t v, float s)
{
    vec3_t r = {
        .x = v.x * s,
        .y = v.y * s,
        .z = v.z * s,
    };
    return r;
}

float vec3_dot(vec3_t a, vec3_t b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

vec3_t vec3_cross(vec3_t a, vec3_t b)
{
    vec3_t r = {
        .x = a.y * b.z - a.z * b.y,
        .y = a.z * b.x - a.x * b.z,
        .z = a.x * b.y - a.y * b.x,
    };
    return r;
}

float vec3_norm(vec3_t v)
{
    return sqrtf(vec3_dot(v, v));
}

vec3_t vec3_normalize(vec3_t v)
{
    float n = vec3_norm(v);
    if (n <= 1e-12f)
    {
        return vec3_zero();
    }

    return vec3_scale(v, 1.0f / n);
}

vec3_t vec3_zero(void)
{
    vec3_t z = {0.0f, 0.0f, 0.0f};
    return z;
}

bool vec3_is_finite(vec3_t v)
{
    return isfinite(v.x) && isfinite(v.y) && isfinite(v.z);
}
