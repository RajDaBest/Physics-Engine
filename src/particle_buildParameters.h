#ifndef PART_BUILDPARAM_H
#define PART_BUILDPARAM_H

#include "particle_common.h"

ParticleError buildDragCoeffs(real linear, real quadratic, DragCoefficients *coeffs);
ParticleError buildSpringParameters(Particle *particleA, Particle *particleB, real springConstant, real restLength,
                                    real dampingCoeff, SpringParameters *springParameters);
ParticleError buildAnchoredSpringParameters(Vector anchor, real springConstant, real restLength, real dampingCoefficient, AnchoredSpringParameters *anchoredSpringParameters);
ParticleError buildElasticBungeeParameters(Particle *particleA, Particle *particleB, real springConstant, real restLength,
                                           real dampingCoeff, ElasticBungeeParameters *elasticBungeeParameters);
ParticleError buildAnchoredBungeeParameters(Vector anchor, real springConstant, real restLength, real dampingCoeff, AnchoredBungeeParameters *anchoredBungeeParameters);

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
                                    real dampingCoeff, SpringParameters *springParameters)
{
    if (!springParameters)
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

    springParameters->particleA = particleA;
    springParameters->particleB = particleB;
    springParameters->springConstant = springConstant;
    springParameters->restLength = restLength;
    springParameters->dampingCoeff = dampingCoeff;

    return PARTICLE_SUCCESS;
}

ParticleError buildAnchoredSpringParameters(Vector anchor, real springConstant, real restLength, real dampingCoeff, AnchoredSpringParameters *anchoredSpringParameters)
{
    if (!anchoredSpringParameters)
    {
        return PARTICLE_ERROR_INVALID_PARAM;
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

    anchoredSpringParameters->dampingCoeff = dampingCoeff;
    anchoredSpringParameters->restLength = restLength;
    anchoredSpringParameters->springConstant = springConstant;
}

ParticleError buildElasticBungeeParameters(Particle *particleA, Particle *particleB, real springConstant, real restLength,
                                           real dampingCoeff, ElasticBungeeParameters *elasticBungeeParameters)
{
    return buildSpringParameters(particleA, particleB, springConstant, restLength, dampingCoeff, elasticBungeeParameters);
}

ParticleError buildAnchoredBungeeParameters(Vector anchor, real springConstant, real restLength,
                                            real dampingCoeff, AnchoredBungeeParameters *anchoredBungeeParameters)
{
    if (!anchoredBungeeParameters)
    {
        return PARTICLE_ERROR_INVALID_PARAM;
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

    anchoredBungeeParameters->anchor = anchor;
    anchoredBungeeParameters->springConstant = springConstant;
    anchoredBungeeParameters->restLength = restLength;
    anchoredBungeeParameters->dampingCoeff = dampingCoeff;

    return PARTICLE_SUCCESS;
}

#endif