#ifndef HELIUS_MATH_VEC3_H
#define HELIUS_MATH_VEC3_H

typedef struct
{
    float x;
    float y;
    float z;
} vec3_t;

void vec3_add(const vec3_t* v1, const vec3_t* v2, vec3_t* result);
void vec3_subtract(const vec3_t* v1, const vec3_t* v2, vec3_t* result);
void vec3_scale(const vec3_t* v, float scalar, vec3_t* result);
float vec3_dot(const vec3_t* v1, const vec3_t* v2);
void vec3_cross(const vec3_t* v1, const vec3_t* v2, vec3_t* result);
float vec3_length(const vec3_t* v);
void vec3_normalize(vec3_t* v);
void vec3_zero(vec3_t* v);

#endif // HELIUS_MATH_VEC3_H