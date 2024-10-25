#ifndef _CORE
#define _CORE

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define PRECISION float
typedef PRECISION real;
#define real_sqrt sqrtf

typedef struct
{
    real x, y, z;
    real padding; // just for making the structure quadword for performance purposes; memory is optimized for quadwords
} Vector;

#define Vector(x, y, z) ((Vector){.x = x, .y = y, .z = z, .padding = 0.0})
#define Vector() ((Vector){.x = 0.0, .y = 0.0, .z = 0.0, .padding = 0.0})
#define magnitude(vector) (real_sqrt((vector).x * (vector).x + (vector).y * (vector).y + (vector).z * (vector).z))
#define squaredMagnitude(vector) ((vector).x * (vector).x + (vector).y * (vector).y + (vector).z * (vector).z)
#define dotProduct(vectorA, vectorB) (vectorA->x * vectorB->x + vectorA->y * vectorB->y + vectorA->z * vectorB->z) // returns dot product of vectorA and vectorB

static inline void invert(Vector *vector);                                        // invert the sign of all the components of vector
static inline void scale(Vector *vector, real scalar);                            // scale vector by real constant scalar
static inline void normalize(Vector *vector);                                     // normalize if vector is non-zero; do nothing otherwise
static inline void addScaled(Vector *vectorDest, Vector *vectorSrc, real scalar); // does vectorDest = vectorDest + scalar * vectorSrc
static inline void componentProduct(Vector *vectorDest, Vector *vectorSrc);
static inline void crossProduct(Vector *vectorDest, Vector *vectorSrc);

static void invert(Vector *vector)
{
    vector->x = -vector->x;
    vector->y = -vector->y;
    vector->z = -vector->z;
}

static inline void scale(Vector *vector, real scalar)
{
    vector->x *= vector->x * scalar;
    vector->y *= vector->y * scalar;
    vector->z *= vector->z * scalar;
}

static inline void normalize(Vector *vector)
{
    real mag = magnitude(*vector);
    if (mag > 0)
    {
        scale(vector, 1.0 / mag);
    }
}

static inline void addScaled(Vector *vectorDest, Vector *vectorSrc, real scalar)
{
    Vector scaledVector = *vectorSrc;
    scale(&scaledVector, scalar);

    vectorDest->x += scaledVector.x;
    vectorDest->y += scaledVector.y;
    vectorDest->z += scaledVector.z;
}

static inline void componentProduct(Vector *vectorDest, Vector *vectorSrc)
{
    vectorDest->x *= vectorSrc->x;
    vectorDest->y *= vectorSrc->y;
    vectorDest->z *= vectorSrc->z;
}

static inline void crossProduct(Vector *vectorDest, const Vector *vectorSrc1, const Vector *vectorSrc2)
{
    vectorDest->x = vectorSrc1->y * vectorSrc2->z - vectorSrc1->z * vectorSrc2->y;
    vectorDest->y = vectorSrc1->z * vectorSrc2->x - vectorSrc1->x * vectorSrc2->z;
    vectorDest->z = vectorSrc1->x * vectorSrc2->y - vectorSrc1->y * vectorSrc2->x;
}
#endif