#ifndef PART_FORCES_H
#define PART_FORCES_H

#include "particle_common.h"

Vector Particle_GravityForce(const Particle *particle, void *gravParameters);
Vector Particle_DragForce(const Particle *particle, void *dragParameters);
Vector Particle_SpringForce(const Particle *particle, void *springParameters);
Vector Particle_AnchoredSpringForce(const Particle *particle, void *anchoredSpringParameters);
Vector Particle_ElasticBungeeForce(const Particle *particle, void *elasticBungeeParameters);
Vector Particle_AnchoredBungeeForce(const Particle *particle, void *anchoredBungeeParameters);

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

Vector Particle_AnchoredSpringForce(const Particle *particle, void *anchoredSpringParameters)
{
    if (!particle || !anchoredSpringParameters)
    {
        particleErrno = PARTICLE_ERROR_INVALID_PARAM;
        return nullVectorDef();
    }

    AnchoredSpringParameters *params = (AnchoredSpringParameters *)anchoredSpringParameters;

    Vector anchorPos = params->anchor;
    invert(&anchorPos);

    Vector force = particle->position;
    vecAdd(&force, &anchorPos);

    real forceMagnitude = -params->springConstant * (magnitude(force) - params->restLength) - params->dampingCoeff * ((dotProduct(force, particle->velocity) / magnitude(force)));

    normalize(&force);
    scale(&force, forceMagnitude);

    particleErrno = PARTICLE_SUCCESS;
    return force;
}

Vector Particle_ElasticBungeeForce(const Particle *particle, void *elasticBungeeParameters)
{
    if (!particle || !elasticBungeeParameters)
    {
        particleErrno = PARTICLE_ERROR_INVALID_PARAM;
        return nullVectorDef();
    }

    ElasticBungeeParameters *param = (ElasticBungeeParameters *)elasticBungeeParameters;

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

    real newLen = magnitude(force) - param->restLength;

    if (newLen <= 0.0)
    {
        return nullVectorDef();
    }

    real forceMagnitude = -param->springConstant * (newLen)-param->dampingCoeff * ((dotProduct(force, partVel) / magnitude(force)));

    normalize(&force);
    scale(&force, forceMagnitude);

    particleErrno = PARTICLE_SUCCESS;
    return force;
}

Vector Particle_AnchoredBungeeForce(const Particle *particle, void *anchoredBungeeParameters)
{
    if (!particle || !anchoredBungeeParameters)
    {
        particleErrno = PARTICLE_ERROR_INVALID_PARAM;
        return nullVectorDef();
    }

    AnchoredBungeeParameters *params = (AnchoredBungeeParameters *)anchoredBungeeParameters;

    Vector anchorPos = params->anchor;
    invert(&anchorPos);

    Vector force = particle->position;
    vecAdd(&force, &anchorPos);

    real length = magnitude(force);
    real extension = length - params->restLength;

    // If the bungee is compressed, no force is applied
    if (extension <= 0.0)
    {
        return nullVectorDef();
    }

    real forceMagnitude = -params->springConstant * extension -
                          params->dampingCoeff * ((dotProduct(force, particle->velocity) / length));

    normalize(&force);
    scale(&force, forceMagnitude);

    particleErrno = PARTICLE_SUCCESS;
    return force;
}

#endif