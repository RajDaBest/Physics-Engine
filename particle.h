#ifndef _PARTICLE
#define _PARTICLE

#include "vector.h"
#include <assert.h>

Vector forceAccum;

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

typedef struct
{
    Vector position, velocity, acceleration;

    // When we come to perform the integration, we will remove a proportion of the
    // object’s velocity at each update. The damping parameter controls how velocity is left
    // after the update. If the damping is zero, then the velocity will be reduced to nothing:
    // this would mean that the object couldn’t sustain any motion without a force and
    // would look odd to the player. A value of 1 means that the object keeps all its velocity
    // (equivalent to no damping).
    // If you don’t want the object to look like it is experiencing
    // drag, then values near but less than 1 are optimal—0.995, for example.
    real damping;

    /*

    inverseMass holds the inverse of the mass of the particle. It is more useful to hold the
    inverse mass because integration is simpler and because in real-time simulation it is more
    useful to have objects with infinite mass (immovable) than zero mass (completely unstable in numerical simulation).
    The former can be represented by a zero inverseMass and the latter by an infinite reverseMass, which is
    difficult to represent on computers.

    particles with inverseMass zero have their accelerations zero at all times

    */

    real inverseMass;
} Particle;

#define setInverseMass(particle, inverseMass) (particle->inverseMass = inverseMass) // use this for setting infinite masses by passing zero
#define setMass(particle, mass) (particle->inverseMass = 1.0 / mass)                // use this for setting any mass except zero

static inline void integrate(Particle *particle, real duration);

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

static inline void integrate(Particle *particle, real duration)
{
    assert(duration > 0.0);

    // update linear position
    addScaled(&particle->position, &particle->velocity, 1.0, duration);

    // work out acceleration from the force
    addScaled(&particle->acceleration, &forceAccum, 0, particle->inverseMass);

    // update linear velocity from the resulting acceleration
    addScaled(&particle->velocity, &particle->acceleration, 1.0, duration);
}

#endif