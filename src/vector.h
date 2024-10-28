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

#define vectorDef(vx, vy, vz) \
    (Vector) { .x = vx, .y = vy, .z = vz, .padding = 0.0 }
#define nullVectorDef() \
    (Vector) { .x = 0.0, .y = 0.0, .z = 0.0, .padding = 0.0 }
#define magnitude(vector) (real_sqrt((vector).x * (vector).x + (vector).y * (vector).y + (vector).z * (vector).z))
#define squaredMagnitude(vector) ((vector).x * (vector).x + (vector).y * (vector).y + (vector).z * (vector).z)
#define dotProduct(vectorA, vectorB) (vectorA->x * vectorB->x + vectorA->y * vectorB->y + vectorA->z * vectorB->z) // returns dot product of vectorA and vectorB

static inline void invert(Vector *vector);                                                           // invert the sign of all the components of vector
static inline void scale(Vector *vector, real scalar);                                               // scale vector by real constant scalar
static inline void normalize(Vector *vector);                                                        // normalize if vector is non-zero; do nothing otherwise
static inline void addScaled(Vector *vectorDest, Vector *vectorSrc, real scalarOne, real scalarTwo); // does vectorDest = vectorDest + scalar * vectorSrc
static inline void componentProduct(Vector *vectorDest, const Vector *vectorSrc);
static inline void crossProduct(Vector *vectorDest, const Vector *vectorSrc);
static inline void vecAdd(Vector *vectorDest, const Vector *vectorSrc);

static void invert(Vector *vector)
{
    vector->x = -vector->x;
    vector->y = -vector->y;
    vector->z = -vector->z;
}

static inline void scale(Vector *vector, real scalar)
{
    vector->x *= scalar;
    vector->y *= scalar;
    vector->z *= scalar;
}

static inline void normalize(Vector *vector)
{
    real mag = magnitude(*vector);
    if (mag > 0)
    {
        scale(vector, 1.0 / mag);
    }
}

static inline void addScaled(Vector *vectorDest, Vector *vectorSrc, real scalarOne, real scalarTwo)
{
    Vector scaledVector = *vectorSrc;
    scale(&scaledVector, scalarTwo);
    scale(vectorDest, scalarOne);

    vectorDest->x += scaledVector.x;
    vectorDest->y += scaledVector.y;
    vectorDest->z += scaledVector.z;
}

static inline void componentProduct(Vector *vectorDest, const Vector *vectorSrc)
{
    vectorDest->x *= vectorSrc->x;
    vectorDest->y *= vectorSrc->y;
    vectorDest->z *= vectorSrc->z;
}

static inline void crossProduct(Vector *vectorDest, const Vector *vectorSrc)
{
    real destX = vectorDest->x;
    real destY = vectorDest->y;
    real destZ = vectorDest->z;

    vectorDest->x = destY * vectorSrc->z - destZ * vectorSrc->y;
    vectorDest->y = destZ * vectorSrc->x - destX * vectorSrc->z;
    vectorDest->z = destX * vectorSrc->y - destY * vectorSrc->x;
}

static inline void vecAdd(Vector *vectorDest, const Vector *vectorSrc)
{
    vectorDest->x += vectorSrc->x;
    vectorDest->y += vectorSrc->y;
    vectorDest->z += vectorSrc->z;
}
#endif