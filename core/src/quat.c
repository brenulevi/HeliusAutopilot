#include "quat.h"
#include <math.h>

helius_quat_t helius_quat_multiply(helius_quat_t q1, helius_quat_t q2)
{
    helius_quat_t q;
    q.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
    q.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y;
    q.y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x;
    q.z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w;
    return q;
}

helius_quat_t helius_quat_normalize(helius_quat_t q)
{
    float norm = sqrtf(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    
    // Evita divisao por zero
    if (norm < 1e-9f) {
        return (helius_quat_t){1.0f, 0.0f, 0.0f, 0.0f};
    }
    
    float inv_norm = 1.0f / norm;
    q.w *= inv_norm;
    q.x *= inv_norm;
    q.y *= inv_norm;
    q.z *= inv_norm;
    return q;
}

helius_quat_t helius_quat_from_axis_angle(helius_vec3_t w, float dt)
{
    // Incremental rotation vector
    float dtheta_x = w.x * dt;
    float dtheta_y = w.y * dt;
    float dtheta_z = w.z * dt;

    // Theta^2 angle
    float theta_sq = dtheta_x * dtheta_x + dtheta_y * dtheta_y + dtheta_z * dtheta_z;

    helius_quat_t dq;

    // If zero, use Taylor approximation instead
    if (theta_sq < 1e-8f) {
        // sin(theta/2)/theta ≈ 0.5 - theta^2 / 48
        float scale = 0.5f - theta_sq * (1.0f / 48.0f);
        dq.w = 1.0f - theta_sq * (1.0f / 8.0f); // cos(theta/2) ≈ 1 - theta^2 / 8
        dq.x = dtheta_x * scale;
        dq.y = dtheta_y * scale;
        dq.z = dtheta_z * scale;
    } else {
        float theta = sqrtf(theta_sq);
        float half_theta = 0.5f * theta;
        float sin_half_theta_over_theta = sinf(half_theta) / theta;

        dq.w = cosf(half_theta);
        dq.x = dtheta_x * sin_half_theta_over_theta;
        dq.y = dtheta_y * sin_half_theta_over_theta;
        dq.z = dtheta_z * sin_half_theta_over_theta;
    }

    return dq;
}

helius_vec3_t helius_quat_rotate_vec(helius_quat_t q, helius_vec3_t v)
{
    // v_rot = v + 2 * q_v x (q_v x v + q_w * v)
    helius_vec3_t q_vec = {q.x, q.y, q.z};

    // t = 2 * (q_vec x v)
    helius_vec3_t t = {
        2.0f * (q_vec.y * v.z - q_vec.z * v.y),
        2.0f * (q_vec.z * v.x - q_vec.x * v.z),
        2.0f * (q_vec.x * v.y - q_vec.y * v.x)
    };

    // v_rot = v + q.w * t + (q_vec x t)
    helius_vec3_t v_rot = {
        v.x + q.w * t.x + (q_vec.y * t.z - q_vec.z * t.y),
        v.y + q.w * t.y + (q_vec.z * t.x - q_vec.x * t.z),
        v.z + q.w * t.z + (q_vec.x * t.y - q_vec.y * t.x)
    };

    return v_rot;
}
