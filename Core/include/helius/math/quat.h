#ifndef HELIUS_MATH_QUAT_H
#define HELIUS_MATH_QUAT_H

typedef struct
{
    float w;
    float x;
    float y;
    float z;
} quat_t;

void quat_multiply(const quat_t* q1, const quat_t* q2, quat_t* result);
void quat_conjugate(const quat_t* q, quat_t* result);
void quat_normalize(quat_t* q);
void quat_identity(quat_t* q);

#endif // HELIUS_MATH_QUAT_H