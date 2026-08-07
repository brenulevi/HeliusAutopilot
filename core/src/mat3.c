#include "mat3.h"

helius_mat3_t helius_mat3_identity(void)
{
    helius_mat3_t res = {{
        {1.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 1.0f}
    }};
    return res;
}

helius_mat3_t helius_mat3_skew(helius_vec3_t v)
{
    // Skew-symmetric (cross-product) matrix [v]_x
    helius_mat3_t res = {{
        { 0.0f, -v.z,   v.y},
        {  v.z,  0.0f, -v.x},
        {-v.y,   v.x,  0.0f}
    }};
    return res;
}

helius_mat3_t helius_mat3_from_quat(helius_quat_t q)
{
    // Converts unit quaternion q = [w, x, y, z] to Direction Cosine Matrix (DCM)
    float w = q.w;
    float x = q.x;
    float y = q.y;
    float z = q.z;

    float xx = x * x;
    float yy = y * y;
    float zz = z * z;

    float xy = x * y;
    float xz = x * z;
    float yz = y * z;

    float wx = w * x;
    float wy = w * y;
    float wz = w * z;

    helius_mat3_t res = {{
        {1.0f - 2.0f * (yy + zz), 2.0f * (xy - wz),        2.0f * (xz + wy)},
        {2.0f * (xy + wz),        1.0f - 2.0f * (xx + zz), 2.0f * (yz - wx)},
        {2.0f * (xz - wy),        2.0f * (yz + wx),        1.0f - 2.0f * (xx + yy)}
    }};

    return res;
}

helius_mat3_t helius_mat3_mult(helius_mat3_t a, helius_mat3_t b)
{
    helius_mat3_t res;

    // Unrolled 3x3 matrix multiplication
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            res.m[i][j] = a.m[i][0] * b.m[0][j] +
                          a.m[i][1] * b.m[1][j] +
                          a.m[i][2] * b.m[2][j];
        }
    }

    return res;
}

helius_mat3_t helius_mat3_transpose(helius_mat3_t a)
{
    helius_mat3_t res = {{
        {a.m[0][0], a.m[1][0], a.m[2][0]},
        {a.m[0][1], a.m[1][1], a.m[2][1]},
        {a.m[0][2], a.m[1][2], a.m[2][2]}
    }};

    return res;
}