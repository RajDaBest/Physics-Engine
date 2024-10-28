/* An implication of Newton's second law is that we cannot do anything to
an object to directly change its position or velocity; we can only do that indirectly by
applying a force to change the acceleration and wait until the object reaches our target
position or velocity. Physics engines need to abuse this
law to look good, but for now I’ll keep it intact.
Because of Newton 2, we will treat the acceleration of the particle different from
velocity and position. Both velocity and position keep track of a quantity from frame
to frame during the game. They change, but not directly, only by the influence of accelerations. Acceleration, by contrast, can be different from one moment to another.
We can simply set the acceleration of an object as we see fit (although we’ll use the
force equations), and the behavior of the object will look fine. If we
directly set the velocity or position, the particle will appear to jolt or jump. Because
of this the position and velocity properties will only be altered by the integrator and
should not be manually altered (other than setting up the initial position and velocity
for an object, of course). The acceleration property can be set at any time, and it will
be left alone by the integrator. */

// When we come to perform the integration, we will remove a proportion of the
// object’s velocity at each update. The damping parameter controls how velocity is left
// after the update. If the damping is zero, then the velocity will be reduced to nothing:
// this would mean that the object couldn’t sustain any motion without a force and
// would look odd to the player. A value of 1 means that the object keeps all its velocity
// (equivalent to no damping).
// If you don’t want the object to look like it is experiencing
// drag, then values near but less than 1 are optimal—0.995, for example.

/*

inverseMass holds the inverse of the mass of the particle. It is more useful to hold the
inverse mass because integration is simpler and because in real-time simulation it is more
useful to have objects with infinite mass (immovable) than zero mass (completely unstable in numerical simulation).
The former can be represented by a zero inverseMass and the latter by an infinite reverseMass, which is
difficult to represent on computers.

particles with inverseMass zero have their accelerations zero at all times

*/

/*

At each frame, the engine needs to look at each object in turn, work out it's acceleration, and perform the integration.
The integrator consists of two parts: one to update the position of the object and the other to update it's velocity.
The position for the duration of a frame will depend on the velocity and acceleration (both at the start of the frame), while the velocity for the duration of the frame will depend only on the acceleration (at the start of the frame)

Integration requires a time interval over which to update the position and velocity; because we update every frame, we use the time interval between frames as the update time.
If the engine is running on a console that has a consistent frame rate, then this duration can be hard-coded into the code (although it isn't wise to do so since in the same console
, different territories can have different frame rates). If the engine is running on a PC with a variable frame-rate, then the time duration of each frame is needed.

Typically, developers will time a frame and use that to update the next frame. This can cause noticeable jolts if the frame durations are dramatically inconsistent, but the game is
unlikely to feel smooth in this case anyway, so it is a common rule of thumb.

*/

/*

The Update Equations

Position Update

To calculate position for a frame, we assume the velocity and the acceleration to be constant for the frame at the value
they had in the previous frame (or initially).

p2 = p1 + v1 * t + (1/2) * a1 * t * t

If we are updating every frame, then the time interval t will be very small (~16 milliseconds for 60 fps) and t *t further small still.
So we ignore the last term in the equation.

p2 = p1 + v1 * t

If the game regularly uses short bursts of huge accelerations, then we might be better off using the former equation. If the game does intend to use
huge accelerations, however, it is likely to get all sorts of other accuracy problems in any case: all physics engines typically
become unstable with very large accelerations.

Velocity Update

To calculate the velocity for a frame, we assume the acceleration to be constant for that frame at the value it had in the previous frame (or initially).

v2 = v1 + a1 * t

The damping parameter (d) is used to remove a bit of velocity at each frame. This is done by:

v2 = v1 * d + a1 * t

This form of the equation hides a problem, however. No matter whether we have
a long or a short time interval over which to update, the amount of velocity being
removed is the same. If our frame rate suddenly improves, then there will be more
updates per second and the object will suddenly appear to have more drag. A more
correct version of the equation solves this problem by incorporating the time into the
drag part of the equation:

v2 = v1 * (d ^ t) + a1 * t

where the damping parameter d is now the proportion of the velocity retained each
second, rather than each frame.

Calculating one floating-point number to the power of another is a slow process
on most modern hardware. If the game is simulating a huge number of objects, then it
is normally best to avoid this step. For a particle physics engine designed to simulate
thousands of sparks, for example, use the former equation, or even remove damping altogether.

Because we are heading towards an engine designed for simulating a smaller number of rigid bodies, I will use the latter form.

A different approach favoured by many engine developers is to use the former equation with a damping value very near to 1 - so small
that it will not be noticeable to the player but big enough to be able to solve the numerical instability problem. In this case a
variable frame rate will not make any visual difference. Drag forces can then be created and applied as explicit forces that will act on
each object.

Unfortunately, this simply moves the problem to another part of the code—the
part where we calculate the size of the drag force. For this reason I prefer to make the
damping parameter more flexible and allow it to be used to simulate visible levels of
drag

The time duration we use it the duration of the last frame (or some initial duration value).

*/

#ifndef PARTICLE_H
#define PARTICLE_H

#include "vector.h"
#include <assert.h>
#include <stdbool.h>

#define FORCE_LIMIT 100
#define ACC_DUE_TO_GRAV -9.8

// Forward declarations
typedef struct Particle Particle;
typedef struct ForceGenerator ForceGenerator;
typedef struct DragCoefficients DragCoefficients;

// Function pointer type for force calculations
typedef Vector (*ForceFunction)(const Particle *particle, void *parameters);

// Main particle structure
struct Particle
{
    Vector position;
    Vector velocity;
    Vector acceleration;
    Vector resultantForce;
    real inverseMass;
    real damping;
    real time;

    // Force management
    ForceGenerator *forceRegistry;
    size_t forceCount;
    size_t forceCapacity;
};

// Force generator structure
struct ForceGenerator
{
    ForceFunction function;
    void *parameters;
    real startTime;
    real endTime;
    bool isActive;
};

// Drag force coefficients
struct DragCoefficients
{
    real linear;    // k1
    real quadratic; // k2
};

// Constructor and destructor
Particle *Particle_Create(
    Vector position,
    Vector velocity,
    Vector acceleration,
    real mass,
    real damping,
    real startTime);
void Particle_Destroy(Particle *particle);

// Core physics functions
void Particle_Integrate(Particle *particle, real duration);
void Particle_AddForce(Particle *particle, ForceFunction force,
                       real startTime, real endTime, void *parameters);
void Particle_ClearForces(Particle *particle);

// Force generators
Vector Particle_GravityForce(const Particle *particle, void *parameters);
Vector Particle_DragForce(const Particle *particle, void *parameters);

// Utility functions
static inline real Particle_GetMass(const Particle *particle)
{
    return particle->inverseMass != 0.0 ? 1.0 / particle->inverseMass : INFINITY;
}

static inline void Particle_SetMass(Particle *particle, real mass)
{
    assert(mass > 0.0);
    particle->inverseMass = 1.0 / mass;
}

static inline bool Particle_IsStatic(const Particle *particle)
{
    return particle->inverseMass == 0.0;
}

// Implementation

Vector Particle_GravityForce(const Particle *particle, void *parameters)
{
    if (Particle_IsStatic(particle))
    {
        return nullVectorDef();
    }
    real mass = Particle_GetMass(particle);
    return vectorDef(0.0, ACC_DUE_TO_GRAV * mass, 0.0);
}

Vector Particle_DragForce(const Particle *particle, void *parameters)
{
    DragCoefficients *coeffs = (DragCoefficients *)parameters;
    Vector velocity = particle->velocity;
    real velocityMag = magnitude(velocity);

    if (velocityMag == 0.0)
    {
        return nullVectorDef();
    }

    normalize(&velocity);
    real dragMagnitude = coeffs->linear * velocityMag +
                         coeffs->quadratic * velocityMag * velocityMag;

    scale(&velocity, -dragMagnitude);
    return velocity;
}

Particle *Particle_Create(Vector position, Vector velocity, Vector acceleration,
                          real mass, real damping, real startTime)
{
    Particle *particle = (Particle *)malloc(sizeof(Particle));
    if (!particle)
    {
        return NULL;
    }

    particle->position = position;
    particle->velocity = velocity;
    particle->acceleration = acceleration;
    particle->resultantForce = nullVectorDef();
    particle->damping = damping;
    particle->time = startTime;

    Particle_SetMass(particle, mass);

    // Initialize force registry
    particle->forceCapacity = 8; // Start with space for 8 forces
    particle->forceCount = 0;
    particle->forceRegistry = malloc(sizeof(ForceGenerator) * particle->forceCapacity);

    if (!particle->forceRegistry)
    {
        free(particle);
        return NULL;
    }

    return particle;
}

void Particle_Destroy(Particle *particle)
{
    if (particle)
    {
        free(particle->forceRegistry);
        free(particle);
    }
}

void Particle_AddForce(Particle *particle, ForceFunction force,
                       real startTime, real endTime, void *parameters)
{
    // Resize force registry if needed
    if (particle->forceCount >= particle->forceCapacity)
    {
        size_t newCapacity = particle->forceCapacity * 2;
        ForceGenerator *newRegistry = realloc(particle->forceRegistry,
                                              sizeof(ForceGenerator) * newCapacity);
        if (!newRegistry)
        {
            return; // Failed to allocate memory
        }
        particle->forceRegistry = newRegistry;
        particle->forceCapacity = newCapacity;
    }

    ForceGenerator *generator = &particle->forceRegistry[particle->forceCount++];
    generator->function = force;
    generator->parameters = parameters;
    generator->startTime = startTime;
    generator->endTime = endTime;
    generator->isActive = true;
}

void Particle_ClearForces(Particle *particle)
{
    particle->resultantForce = nullVectorDef();
    particle->forceCount = 0;
}

void Particle_Integrate(Particle *particle, real duration)
{
    assert(duration > 0.0);

    // Update position
    addScaled(&particle->position, &particle->velocity, 1.0, duration);

    // Accumulate forces
    for (size_t i = 0; i < particle->forceCount; i++)
    {
        ForceGenerator *generator = &particle->forceRegistry[i];
        if (!generator->isActive)
            continue;

        if (particle->time >= generator->startTime &&
            particle->time <= generator->endTime)
        {
            Vector force = generator->function(particle, generator->parameters);
            vecAdd(&particle->resultantForce, &force);
        }
    }

    // Update acceleration from forces
    if (!Particle_IsStatic(particle))
    {
        Vector accFromForce = particle->resultantForce;
        scale(&accFromForce, particle->inverseMass);
        vecAdd(&particle->acceleration, &accFromForce);
    }

    // Update velocity
    real dampingFactor = pow(particle->damping, duration);
    addScaled(&particle->velocity, &particle->acceleration, dampingFactor, duration);

    // Reset forces and update time
    particle->resultantForce = nullVectorDef();
    particle->time += duration;
}

#endif