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

static inline void invert(Vector *vector); // invert the sign of all the components of vector
static inline void scale(Vector *vector, real scalar); // scale vector by real constant scalar
static inline void normalize(Vector *vector); // normalize if vector is non-zero; do nothing otherwise

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

int main(int argc, char **argv)
{
    printf("hello, world!\n");
    return EXIT_SUCCESS;
}