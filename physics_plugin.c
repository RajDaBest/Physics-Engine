#include "particle.h"
#include <math.h>

// Change the export definition for Linux
#define EXPORT __attribute__((visibility("default")))

EXPORT void CreateBullet(Particle *particle)
{
    particle->position = vectorDef(0, 5, 0);
    particle->velocity = vectorDef(35, 0, 0);
    particle->acceleration = vectorDef(0, -1, 0);
    particle->forceAccum = nullVectorDef();
    particle->damping = 0.99f;
    particle->inverseMass = 1.0f / 2.0f; // 2kg mass
}

EXPORT void CreateArtillery(Particle *particle)
{
    particle->position = vectorDef(0, 5, 0);
    particle->velocity = vectorDef(30, 40, 0);
    particle->acceleration = vectorDef(0, -20, 0);
    particle->forceAccum = nullVectorDef();
    particle->damping = 0.99f;
    particle->inverseMass = 1.0f / 200.0f; // 200kg mass
}

EXPORT void CreateFireball(Particle *particle)
{
    particle->position = vectorDef(0, 5, 0);
    particle->velocity = vectorDef(10, 0, 0);
    particle->acceleration = vectorDef(0, 0.6f, 0);
    particle->forceAccum = nullVectorDef();
    particle->damping = 0.9f;
    particle->inverseMass = 1.0f; // 1kg mass
}

EXPORT void IntegrateParticle(Particle *particle, float duration)
{
    // Set force to achieve desired constant acceleration
    particle->forceAccum.x = particle->acceleration.x / particle->inverseMass;
    particle->forceAccum.y = particle->acceleration.y / particle->inverseMass;
    particle->forceAccum.z = particle->acceleration.z / particle->inverseMass;

    integrate(particle, duration);
}