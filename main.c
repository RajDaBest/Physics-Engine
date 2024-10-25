#include <stdio.h>
#include <stdlib.h>

#define PRECISION float
typedef PRECISION real;

typedef struct 
{
    real x, y, z;
    real padding; // just for making the structure quadword for performance purposes; memory is optimized for quadwords
} Vector3;

#define Vector3(x, y, z) ((Vector3){.x = x, .y = y, .z = z, .padding = 0.0})
#define Vector3() ((Vector3){.x = 0.0, .y = 0.0, .z = 0.0, .padding = 0.0})

static void invert(Vector3 *vector);

static void invert(Vector3 *vector)
{
    vector->x = -vector->x;
    vector->y = -vector->y;
    vector->z = -vector->z;
}

int main(int argc, char **argv)
{
    printf("hello, world!\n");
    return EXIT_SUCCESS;
}