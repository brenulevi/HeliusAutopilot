#include "quat.h"

#include <math.h>

void quat_multiply(const quat_t* q1, const quat_t* q2, quat_t* result)
{
    result->w = q1->w * q2->w - q1->x * q2->x - q1->y * q2->y - q1->z * q2->z;
    result->x = q1->w * q2->x + q1->x * q2->w + q1->y * q2->z - q1->z * q2->y;
    result->y = q1->w * q2->y - q1->x * q2->z + q1->y * q2->w + q1->z * q2->x;
    result->z = q1->w * q2->z + q1->x * q2->y - q1->y * q2->x + q1->z * q2->w;
}

void quat_conjugate(const quat_t* q, quat_t* result)
{
    result->w = q->w;
    result->x = -q->x;
    result->y = -q->y;
    result->z = -q->z;
}

void quat_normalize(quat_t* q)
{
    float norm = sqrtf(q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z);
    if (norm < 1e-6f) // Avoid division by zero
    {
        q->w = 1.0f;
        q->x = 0.0f;
        q->y = 0.0f;
        q->z = 0.0f;
    }
    else
    {
        float inv_norm = 1.0f / norm;
        q->w *= inv_norm;
        q->x *= inv_norm;
        q->y *= inv_norm;
        q->z *= inv_norm;
    }
}

void quat_identity(quat_t *q)
{
    q->w = 1.0f;
    q->x = 0.0f;
    q->y = 0.0f;
    q->z = 0.0f;
}

void quat_rotate_vector(const quat_t *q, const vec3_t *v, vec3_t *result)
{
    // Convert vector to quaternion
    quat_t v_quat = {0.0f, v->x, v->y, v->z};

    // Calculate the conjugate of the quaternion
    quat_t q_conj;
    quat_conjugate(q, &q_conj);

    // Perform the rotation: result = q * v_quat * q_conj  (body → inertial)
    quat_t temp;
    quat_multiply(q, &v_quat, &temp);
    quat_t rotated_quat;
    quat_multiply(&temp, &q_conj, &rotated_quat);

    // Extract the rotated vector from the resulting quaternion
    result->x = rotated_quat.x;
    result->y = rotated_quat.y;
    result->z = rotated_quat.z;
}

void quat_rotate_vector_inverse(const quat_t *q, const vec3_t *v, vec3_t *result)
{
    quat_t v_quat = {0.0f, v->x, v->y, v->z};

    quat_t q_conj;
    quat_conjugate(q, &q_conj);

    // result = q* * v * q  (inertial → body)
    quat_t temp;
    quat_multiply(&q_conj, &v_quat, &temp);
    quat_t rotated_quat;
    quat_multiply(&temp, q, &rotated_quat);

    result->x = rotated_quat.x;
    result->y = rotated_quat.y;
    result->z = rotated_quat.z;
}

void quat_to_euler(const quat_t *q, vec3_t *euler)
{
    // Roll (x-axis rotation)
    float sinr_cosp = 2.0f * (q->w * q->x + q->y * q->z);
    float cosr_cosp = 1.0f - 2.0f * (q->x * q->x + q->y * q->y);
    euler->x = atan2f(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    float sinp = 2.0f * (q->w * q->y - q->z * q->x);
    if (fabsf(sinp) >= 1.0f)
        euler->y = copysignf(M_PI / 2.0f, sinp); // Use 90 degrees if out of range
    else
        euler->y = asinf(sinp);

    // Yaw (z-axis rotation)
    float siny_cosp = 2.0f * (q->w * q->z + q->x * q->y);
    float cosy_cosp = 1.0f - 2.0f * (q->y * q->y + q->z * q->z);
    euler->z = atan2f(siny_cosp, cosy_cosp);
}
