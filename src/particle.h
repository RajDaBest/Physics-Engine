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
#include <stdbool.h>

#define FORCE_LIMIT 100
#define ACC_DUE_TO_GRAV -9.81

typedef struct Particle Particle;
typedef struct ForceGenerator ForceGenerator;
typedef struct DragCoefficients DragCoefficients;
typedef struct SpringParameters SpringParameters;
typedef enum ParticleError ParticleError;
typedef enum ForceIdentifier ForceIdentifier;

typedef Vector (*ForceFunction)(const Particle *particle, void *parameters);

enum ParticleError
{
    PARTICLE_SUCCESS,
    PARTICLE_ERROR_MEMORY,
    PARTICLE_ERROR_INVALID_PARAM,
    PARTICLE_ERROR_INVALID_MASS,
    PARTICLE_ERROR_INVALID_DAMPING,
    PARTICLE_ERROR_INVALID_TIME,
    PARTICLE_ERROR_INVALID_SPRING_CONSTANT,
    PARTICLE_ERROR_INVALID_REST_LENGTH,
    PARTICLE_ERROR_INVALID_DAMPING_COEFF,
    PARTICLE_ERROR_NULL_SPRING_OTHER,
    PARTICLE_ERROR_INVALID_DRAG_COEFFS,
    PARTICLE_ERROR_INVALID_FORCE_ID,
    PARTICLE_ERROR_INVALID_DURATION
};

extern ParticleError particleErrno;
ParticleError particleErrno = PARTICLE_SUCCESS;

enum ForceIdentifier
{
    GRAV = 1,
    DRAG,
    SPRING,
    ANCHORED_SPRING,
    BUNGEE,
    TOTAL_TYPES,
};

struct SpringParameters
{
    Particle *particleA;
    Particle *particleB;
    real springConstant;
    real dampingCoeff;
    real restLength;
    Vector forceVal;
    bool notAlreadyUsed;
};

struct Particle
{
    Vector position;
    Vector velocity;
    Vector acceleration;
    Vector resultantForce;
    real inverseMass;
    real damping;
    real time;

    ForceGenerator *forceRegistry;
    size_t forceCount;
    size_t forceCapacity;

    size_t uniqueID;
};

struct ForceGenerator
{
    ForceFunction function;
    ForceIdentifier identity;
    void *parameters;
    real startTime;
    real endTime;
    bool isActive;
};

struct DragCoefficients
{
    real linear;    // k1
    real quadratic; // k2
};

Particle *Particle_Create(Vector position, Vector velocity, Vector acceleration,
                          real mass, real damping, real startTime);

void Particle_Destroy(Particle *particle);

ParticleError Particle_Integrate(Particle *particle, real duration);
ParticleError buildDragCoeffs(real linear, real quadratic, DragCoefficients *coeffs);
ParticleError buildSpringParameters(Particle *particleA, Particle *particleB, real springConstant, real restLength,
                                    real dampingCoeff, SpringParameters *params);
ParticleError Particle_AddForce(Particle *particle, ForceFunction force,
                                real startTime, real endTime, void *parameters,
                                ForceIdentifier identifier);
ParticleError Particle_AddGrav(Particle *particle);
ParticleError Particle_AddDrag(Particle *particle, DragCoefficients *dragCoefficients);
ParticleError Particle_AddSpring(Particle *particleA, Particle *particleB, SpringParameters *springParameters, real startTime, real endTime);
void Particle_ClearForces(Particle *particle);

Vector Particle_GravityForce(const Particle *particle, void *gravParameters);
Vector Particle_DragForce(const Particle *particle, void *dragParameters);
Vector Particle_SpringForce(const Particle *particle, void *springParameters);

ParticleError Particle_GetMass(const Particle *particle, real *mass);
ParticleError Particle_SetMass(Particle *particle, real mass);
ParticleError Particle_IsStatic(const Particle *particle, bool *isStatic);

ParticleError Particle_GetLastError(void);

ParticleError Particle_GetLastError(void)
{
    ParticleError lastError = particleErrno;
    particleErrno = PARTICLE_SUCCESS; // Clear the error
    return lastError;
}

Vector Particle_GravityForce(const Particle *particle, void *gravParameters)
{
    bool isStatic;
    if (Particle_IsStatic(particle, &isStatic) != PARTICLE_SUCCESS || isStatic)
    {
        particleErrno = PARTICLE_ERROR_INVALID_PARAM;
        return nullVectorDef();
    }

    real mass;
    if (Particle_GetMass(particle, &mass) != PARTICLE_SUCCESS)
    {
        particleErrno = PARTICLE_ERROR_INVALID_MASS;
        return nullVectorDef();
    }

    particleErrno = PARTICLE_SUCCESS;
    return vectorDef(0.0, ACC_DUE_TO_GRAV * mass, 0.0);
}

Vector Particle_DragForce(const Particle *particle, void *dragParameters)
{
    if (!particle || !dragParameters)
    {
        particleErrno = PARTICLE_ERROR_INVALID_PARAM;
        return nullVectorDef();
    }

    DragCoefficients *coeffs = (DragCoefficients *)dragParameters;
    Vector velocity = particle->velocity;
    real velocityMag = magnitude(velocity);

    if (velocityMag < 0.01)
    {
        return nullVectorDef();
    }

    normalize(&velocity);
    real dragMagnitude = coeffs->linear * velocityMag +
                         coeffs->quadratic * velocityMag * velocityMag;

    invert(&velocity);
    scale(&velocity, dragMagnitude);
    particleErrno = PARTICLE_SUCCESS;
    return velocity;
}

Vector Particle_SpringForce(const Particle *particle, void *springParameters)
{
    if (!particle || !springParameters)
    {
        particleErrno = PARTICLE_ERROR_INVALID_PARAM;
        return nullVectorDef();
    }

    SpringParameters *param = (SpringParameters *)springParameters;

    Vector otherPos;
    Vector otherVel;
    Vector partVel;

    if (param->particleA->uniqueID == particle->uniqueID)
    {
        otherPos = param->particleB->position;
        otherVel = param->particleB->velocity;
        partVel = particle->velocity;
    }
    else
    {
        otherPos = param->particleA->position;
        otherVel = param->particleA->velocity;
        partVel = particle->velocity;
    }

    invert(&otherPos);
    invert(&otherVel);
    vecAdd(&partVel, &otherVel);

    Vector force = particle->position;
    vecAdd(&force, &otherPos);

    real forceMagnitude = -param->springConstant * (magnitude(force) - param->restLength) - param->dampingCoeff * ((dotProduct(force, partVel) / magnitude(force)));

    normalize(&force);
    scale(&force, forceMagnitude);

    particleErrno = PARTICLE_SUCCESS;
    return force;
}

Particle *Particle_Create(Vector position, Vector velocity, Vector acceleration,
                          real mass, real damping, real startTime)
{
    if (mass <= 0.0)
    {
        particleErrno = PARTICLE_ERROR_INVALID_MASS;
        return NULL;
    }

    if (damping < 0.0 || damping > 1.0)
    {
        particleErrno = PARTICLE_ERROR_INVALID_DAMPING;
        return NULL;
    }

    if (startTime < 0.0)
    {
        particleErrno = PARTICLE_ERROR_INVALID_TIME;
        return NULL;
    }

    Particle *particle = (Particle *)malloc(sizeof(Particle));
    if (!particle)
    {
        particleErrno = PARTICLE_ERROR_MEMORY;
        return NULL;
    }

    particle->forceCapacity = 8;
    particle->forceRegistry = malloc(sizeof(ForceGenerator) * particle->forceCapacity);
    if (!particle->forceRegistry)
    {
        free(particle);
        particleErrno = PARTICLE_ERROR_MEMORY;
        return NULL;
    }

    particle->position = position;
    particle->velocity = velocity;
    particle->acceleration = acceleration;
    particle->resultantForce = nullVectorDef();
    particle->damping = damping;
    particle->time = startTime;
    particle->inverseMass = 1.0 / mass;

    particle->forceCapacity = 8;
    particle->forceCount = 0;
    particle->uniqueID = rand();

    particleErrno = PARTICLE_SUCCESS;
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

ParticleError buildDragCoeffs(real linear, real quadratic, DragCoefficients *coeffs)
{
    if (!coeffs)
    {
        return PARTICLE_ERROR_INVALID_PARAM;
    }

    if (linear < 0.0 || quadratic < 0.0)
    {
        return PARTICLE_ERROR_INVALID_DRAG_COEFFS;
    }

    coeffs->linear = linear;
    coeffs->quadratic = quadratic;
    return PARTICLE_SUCCESS;
}

ParticleError buildSpringParameters(Particle *particleA, Particle *particleB, real springConstant, real restLength,
                                    real dampingCoeff, SpringParameters *params)
{
    if (!params)
    {
        return PARTICLE_ERROR_INVALID_PARAM;
    }

    if (!particleA || !particleB)
    {
        return PARTICLE_ERROR_NULL_SPRING_OTHER;
    }

    if (springConstant < 0.0)
    {
        return PARTICLE_ERROR_INVALID_SPRING_CONSTANT;
    }

    if (restLength < 0.0)
    {
        return PARTICLE_ERROR_INVALID_REST_LENGTH;
    }

    if (dampingCoeff < 0.0)
    {
        return PARTICLE_ERROR_INVALID_DAMPING_COEFF;
    }

    params->particleA = particleA;
    params->particleB = particleB;
    params->springConstant = springConstant;
    params->restLength = restLength;
    params->dampingCoeff = dampingCoeff;
    params->notAlreadyUsed = false;
    params->forceVal = nullVectorDef();

    return PARTICLE_SUCCESS;
}

ParticleError Particle_GetMass(const Particle *particle, real *mass)
{
    if (!particle || !mass)
    {
        return PARTICLE_ERROR_INVALID_PARAM;
    }

    *mass = particle->inverseMass != 0.0 ? 1.0 / particle->inverseMass : INFINITY;
    return PARTICLE_SUCCESS;
}

ParticleError Particle_SetMass(Particle *particle, real mass)
{
    if (!particle)
    {
        return PARTICLE_ERROR_INVALID_PARAM;
    }

    if (mass <= 0.0)
    {
        return PARTICLE_ERROR_INVALID_MASS;
    }

    particle->inverseMass = 1.0 / mass;
    return PARTICLE_SUCCESS;
}

ParticleError Particle_IsStatic(const Particle *particle, bool *isStatic)
{
    if (!particle || !isStatic)
    {
        return PARTICLE_ERROR_INVALID_PARAM;
    }

    *isStatic = (particle->inverseMass == 0.0);
    return PARTICLE_SUCCESS;
}

ParticleError Particle_AddForce(Particle *particle, ForceFunction force,
                                real startTime, real endTime, void *parameters,
                                ForceIdentifier identifier)
{
    if (!particle || !force)
    {
        return PARTICLE_ERROR_INVALID_PARAM;
    }

    if (startTime < 0.0 || endTime < 0.0)
    {
        return PARTICLE_ERROR_INVALID_TIME;
    }

    if (identifier < 1 || identifier >= TOTAL_TYPES)
    {
        return PARTICLE_ERROR_INVALID_FORCE_ID;
    }

    if (particle->forceCount >= particle->forceCapacity)
    {
        size_t newCapacity = particle->forceCapacity * 2;
        ForceGenerator *newRegistry = realloc(particle->forceRegistry,
                                              sizeof(ForceGenerator) * newCapacity);
        if (!newRegistry)
        {
            return PARTICLE_ERROR_MEMORY;
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
    generator->identity = identifier;

    return PARTICLE_SUCCESS;
}

ParticleError Particle_AddGrav(Particle *particle)
{
    return Particle_AddForce(particle, Particle_GravityForce, 0.0, INFINITY, NULL, GRAV);
}

ParticleError Particle_AddDrag(Particle *particle, DragCoefficients *dragCoefficients)
{
    if (!dragCoefficients)
    {
        return PARTICLE_ERROR_INVALID_PARAM;
    }
    return Particle_AddForce(particle, Particle_DragForce, 0.0, INFINITY,
                             (void *)dragCoefficients, DRAG);
}

ParticleError Particle_AddSpring(Particle *particleA, Particle *particleB, SpringParameters *springParameters, real startTime, real endTime)
{
    Particle_AddForce(particleA, Particle_SpringForce, startTime, endTime, (void *)springParameters, SPRING);
    return (Particle_AddForce(particleB, Particle_SpringForce, startTime, endTime, (void *)springParameters, SPRING));
}

void Particle_ClearForces(Particle *particle)
{
    if (particle)
    {
        particle->resultantForce = nullVectorDef();
        particle->forceCount = 0;
    }
}

ParticleError Particle_Integrate(Particle *particle, real duration)
{
    if (!particle)
    {
        return PARTICLE_ERROR_INVALID_PARAM;
    }

    if (duration <= 0.0)
    {
        return PARTICLE_ERROR_INVALID_DURATION;
    }

    addScaled(&particle->position, &particle->velocity, 1.0, duration);

    for (size_t i = 0; i < particle->forceCount; i++)
    {
        ForceGenerator *generator = &particle->forceRegistry[i];
        if (!generator->isActive)
            continue;

        if (particle->time >= generator->startTime &&
            particle->time <= generator->endTime)
        {
            if (generator->identity == SPRING)
            {
                SpringParameters *param = (SpringParameters *)generator->parameters;
                if (param->notAlreadyUsed)
                {
                    vecAdd(&particle->resultantForce, &(param->forceVal));
                    param->notAlreadyUsed = false;
                }
                else
                {
                    Vector force = generator->function(particle, generator->parameters);
                    vecAdd(&particle->resultantForce, &force);
                    param->notAlreadyUsed = true;
                    Vector otherForce = force;
                    invert(&otherForce);
                    param->forceVal = otherForce;
                }
            }
            else
            {
                Vector force = generator->function(particle, generator->parameters);
                vecAdd(&particle->resultantForce, &force);
            }

            Vector force = generator->function(particle, generator->parameters);
            vecAdd(&particle->resultantForce, &force);
        }
    }

    bool isStatic;
    ParticleError error = Particle_IsStatic(particle, &isStatic);
    if (error != PARTICLE_SUCCESS)
    {
        return error;
    }

    if (!isStatic)
    {
        Vector accFromForce = particle->resultantForce;
        scale(&accFromForce, particle->inverseMass);
        vecAdd(&particle->acceleration, &accFromForce);
    }

    real dampingFactor = pow(particle->damping, duration);
    addScaled(&particle->velocity, &particle->acceleration, dampingFactor, duration);

    particle->resultantForce = nullVectorDef();
    particle->time += duration;
    particle->acceleration = nullVectorDef();

    return PARTICLE_SUCCESS;
}

#endif