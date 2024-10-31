#ifndef PART_ADDFORCES_H
#define PART_ADDFORCES_H

#include "particle_forces.h"

ParticleError Particle_AddForce(Particle *particle, ForceFunction force,
                                real startTime, real endTime, void *parameters,
                                ForceIdentifier identifier);
ParticleError Particle_AddGrav(Particle *particle);
ParticleError Particle_AddDrag(Particle *particle, DragCoefficients *dragCoefficients);
ParticleError Particle_AddSpring(Particle *particleA, Particle *particleB, SpringParameters *springParameters, real startTime, real endTime);
ParticleError Particle_AddAnchoredSpring(Particle *particle, Vector anchor, AnchoredSpringParameters *anchoredSpringParameters, real startTime, real endTime);
ParticleError Particle_AddElasticBungee(Particle *particleA, Particle *particleB, ElasticBungeeParameters *elasticBungeeParameters, real startTime, real endTime);
ParticleError Particle_AddAnchoredBungee(Particle *particle, Vector anchor, AnchoredBungeeParameters *anchoredBungeeParameters, real startTime, real endTime);

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
        ForceGenerator *newRegistry = (ForceGenerator *)realloc(particle->forceRegistry,
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
    if (!particle)
    {
        return PARTICLE_ERROR_INVALID_PARAM;
    }
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
    if (!particleA || !particleB || !springParameters)
    {
        return PARTICLE_ERROR_INVALID_PARAM;
    }
    Particle_AddForce(particleA, Particle_SpringForce, startTime, endTime, (void *)springParameters, SPRING);
    return (Particle_AddForce(particleB, Particle_SpringForce, startTime, endTime, (void *)springParameters, SPRING));
}

ParticleError Particle_AddAnchoredSpring(Particle *particle, Vector anchor, AnchoredSpringParameters *anchoredSpringParameters, real startTime, real endTime)
{
    if (!particle || anchoredSpringParameters)
    {
        return PARTICLE_ERROR_INVALID_PARAM;
    }
    return (Particle_AddForce(particle, Particle_AnchoredSpringForce, startTime, endTime, (void *)anchoredSpringParameters, ANCHORED_SPRING));
}

ParticleError Particle_AddElasticBungee(Particle *particleA, Particle *particleB, ElasticBungeeParameters *elasticBungeeParameters, real startTime, real endTime)
{
    if (!particleA || !particleB || !elasticBungeeParameters)
    {
        return PARTICLE_ERROR_INVALID_PARAM;
    }

    Particle_AddForce(particleB, Particle_ElasticBungeeForce, startTime, endTime, (void *)elasticBungeeParameters, BUNGEE);
    return (Particle_AddForce(particleA, Particle_ElasticBungeeForce, startTime, endTime, (void *)elasticBungeeParameters, BUNGEE));
}

ParticleError Particle_AddAnchoredBungee(Particle *particle, Vector anchor,
                                         AnchoredBungeeParameters *anchoredBungeeParameters,
                                         real startTime, real endTime)
{
    if (!particle || !anchoredBungeeParameters)
    {
        return PARTICLE_ERROR_INVALID_PARAM;
    }
    return Particle_AddForce(particle, Particle_AnchoredBungeeForce, startTime, endTime,
                             (void *)anchoredBungeeParameters, ANCHORED_BUNGEE);
}

#endif