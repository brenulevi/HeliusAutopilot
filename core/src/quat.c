#include "quat.h"

#include <math.h>

float quat_norm(quat_t q)
{
    return sqrtf(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
}

quat_t quat_normalize(quat_t q)
{
    float n = quat_norm(q);
    if (n <= 1e-12f)
    {
        return quat_identity();
    }

    quat_t r = {
        .w = q.w / n,
        .x = q.x / n,
        .y = q.y / n,
        .z = q.z / n,
    };
    return r;
}

quat_t quat_mult(quat_t q1, quat_t q2)
{
    quat_t r = {
        .w = q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z,
        .x = q1.w*q2.x + q1.x*q2.w + q1.y*q2.z - q1.z*q2.y,
        .y = q1.w*q2.y - q1.x*q2.z + q1.y*q2.w + q1.z*q2.x,
        .z = q1.w*q2.z + q1.x*q2.y - q1.y*q2.x + q1.z*q2.w
    };
    return r;
}

quat_t quat_conjugate(quat_t q)
{
    quat_t r = {
        .w = q.w,
        .x = -q.x,
        .y = -q.y,
        .z = -q.z,
    };
    return r;
}

vec3_t quat_rotate_vec3(quat_t q, vec3_t v)
{
    quat_t qn = quat_normalize(q);

    quat_t p = {
        .w = 0.0f,
        .x = v.x,
        .y = v.y,
        .z = v.z,
    };

    quat_t r = quat_mult(quat_mult(qn, p), quat_conjugate(qn));

    vec3_t vr = {
        .x = r.x,
        .y = r.y,
        .z = r.z,
    };
    return vr;
}

quat_t quat_identity(void)
{
    quat_t q = {
        .w = 1.0f,
        .x = 0.0f,
        .y = 0.0f,
        .z = 0.0f,
    };
    return q;
}

bool quat_is_finite(quat_t q)
{
    return isfinite(q.w) && isfinite(q.x) && isfinite(q.y) && isfinite(q.z);
}
